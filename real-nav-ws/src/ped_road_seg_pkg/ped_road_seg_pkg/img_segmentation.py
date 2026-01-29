#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
from ament_index_python.packages import get_package_share_directory

import torch
import segmentation_models_pytorch as smp
import albumentations as albu
from ultralytics import YOLO

# --- 前処理関数 ---
def get_validation_augmentation():
    test_transform = [albu.PadIfNeeded(480, 640)]
    return albu.Compose(test_transform)

def to_tensor(x, **kwargs):
    return x.transpose(2, 0, 1).astype('float32')

def get_preprocessing(preprocessing_fn):
    _transform = [
        albu.Lambda(image=preprocessing_fn),
        albu.Lambda(image=to_tensor, mask=to_tensor),
    ]
    return albu.Compose(_transform)
# ------------------------------------

class ImgSegmentationNode(Node):
    def __init__(self):
        super().__init__('img_segmentation_node')

        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/gb_img')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('yolo_confidence', 0.7)
        self.declare_parameter('yolo_imgsz', 512)
        self.declare_parameter('out_width', 112)
        self.declare_parameter('out_height', 84)

        self.image_topic = self.get_parameter('image_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.queue_size = int(self.get_parameter('queue_size').value)
        self.confidence = float(self.get_parameter('yolo_confidence').value)
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        self.out_width = int(self.get_parameter('out_width').value)
        self.out_height = int(self.get_parameter('out_height').value)

        self.publisher_ = self.create_publisher(Image, self.output_topic, self.queue_size)
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.queue_size)
        
        self.bridge = CvBridge()
        self.segmentation_buffer = None

        try:
            share_dir = get_package_share_directory('ped_road_seg_pkg')
            default_model_path = os.path.join(share_dir, 'resource', 'best_model_house2.pth')
            default_yolo_path = os.path.join(share_dir, 'resource', 'yolo26s-seg_pedflow2cls.pt')

            self.declare_parameter('model_path', default_model_path)
            self.declare_parameter('yolo_path', default_yolo_path)

            model_path = self.get_parameter('model_path').value
            yolo_path = self.get_parameter('yolo_path').value
            
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self.get_logger().info(f"Device: {self.device}")
            
            # 走行可能領域セグメンテーションモデル
            self.model = torch.load(model_path, map_location=torch.device(self.device), weights_only=False)
            self.model.eval()
            self.get_logger().info(f"Road segmentation model loaded from {model_path} onto {self.device}")

            # YOLOセグメンテーションモデル（歩行者seg + 向き推定用）
            self.yolo_model = YOLO(yolo_path).to(self.device)
            if self.device == 'cuda':
                self.yolo_model = self.yolo_model.half()
                self.get_logger().info("YOLO half precision enabled")
            self.get_logger().info(f"YOLO model loaded from {yolo_path}")

            encoder = 'resnet50'
            encoder_weights = 'imagenet'
            self.preprocessing_fn = smp.encoders.get_preprocessing_fn(encoder, encoder_weights)
            self.augmentation = get_validation_augmentation()
            self.preprocessing = get_preprocessing(self.preprocessing_fn)

        except Exception as e:
            self.get_logger().error(f"Failed to load model or setup preprocessing: {e}")
            raise e

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        h, w = frame.shape[:2]

        if self.segmentation_buffer is None or self.segmentation_buffer.shape[:2] != (h, w):
            self.segmentation_buffer = np.empty((h, w, 3), dtype=np.uint8)

        # === 1. 走行可能領域セグメンテーション ===
        try:
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            sample = self.augmentation(image=image)
            image_aug = sample['image']
            sample = self.preprocessing(image=image_aug)
            image_preproc = sample['image']

            x_tensor = torch.from_numpy(image_preproc).to(self.device).unsqueeze(0)

            with torch.no_grad():
                pr_mask_tensor = self.model(x_tensor)

            pr_mask_np = pr_mask_tensor.squeeze().cpu().numpy()
            road_mask = (pr_mask_np > 0.5).astype(np.uint8)  # ２値化
        except Exception as e:
            self.get_logger().error(f"Road segmentation failed: {e}")
            return

        # === 2. YOLOで歩行者seg（同方向/逆方向の2クラス） ===
        same_dir_mask = np.zeros((h, w), dtype=np.uint8)
        ops_dir_mask = np.zeros((h, w), dtype=np.uint8)

        try:
            with torch.inference_mode():
                yolo_results = self.yolo_model.predict(
                    frame,
                    imgsz=self.yolo_imgsz,
                    conf=self.confidence,
                    retina_masks=True,
                    stream=False,
                )[0]

            if yolo_results.boxes is not None and yolo_results.masks is not None:
                masks = (yolo_results.masks.data > 0.5).cpu().numpy().astype(np.uint8)
                class_ids = yolo_results.boxes.cls.cpu().numpy().astype(int)

                for idx, mask_pred in enumerate(masks):
                    # マスクを元画像サイズにリサイズ
                    if mask_pred.shape[:2] != (h, w):
                        mask_pred = cv2.resize(mask_pred, (w, h), interpolation=cv2.INTER_NEAREST)

                    cls_id = class_ids[idx] if idx < len(class_ids) else None
                    if cls_id == 0:  # 同方向
                        same_dir_mask = np.maximum(same_dir_mask, mask_pred)
                    elif cls_id == 1:  # 同方向以外
                        ops_dir_mask = np.maximum(ops_dir_mask, mask_pred)

        except Exception as e:
            self.get_logger().warn(f"YOLO inference failed: {e}")

        # === 3. 四値化画像作成 ===
        segmentation = self.segmentation_buffer

        # 色定義
        road_color = (0, 255, 0)          # 緑（走行可能領域）
        same_dir_color = (255, 0, 0)      # 青（同方向）
        ops_dir_color = (0, 0, 255)       # 赤（同方向以外）
        other_color = (255, 255, 0)       # シアン（その他）

        # 優先順位: その他 -> 走行可能領域 -> 同方向 -> 逆方向（歩行者が最優先で上書き）
        segmentation[:] = other_color
        segmentation[road_mask == 1] = road_color
        segmentation[same_dir_mask == 1] = same_dir_color
        segmentation[ops_dir_mask == 1] = ops_dir_color

        # リサイズ
        img_out = cv2.resize(
            segmentation,
            (self.out_width, self.out_height),
            interpolation=cv2.INTER_NEAREST,
        )

        try:
            out_msg = self.bridge.cv2_to_imgmsg(img_out, "bgr8")
            self.publisher_.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Publish failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    img_segmentation_node = ImgSegmentationNode()
    rclpy.spin(img_segmentation_node)
    img_segmentation_node.destroy_node()
    rclpy.shutdown()


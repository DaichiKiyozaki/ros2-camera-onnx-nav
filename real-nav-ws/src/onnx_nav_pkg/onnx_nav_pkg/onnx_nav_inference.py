"""onnx_nav_inference.py

役割:
- ONNX Runtime のセッション生成を担当
- 出力テンソル名の選択ロジックを担当
- 画像/ベクトル観測からモデル入力feedを構築
- フレームスタック前処理を担当
"""

from collections import deque

import cv2
import numpy as np
import onnxruntime as ort


class FrameStackPreprocessor:
    def __init__(self, img_width: int, img_height: int, stack_size: int) -> None:
        self.img_width = img_width
        self.img_height = img_height
        self.stack_size = stack_size
        self.frame_buffer: deque[np.ndarray] = deque(maxlen=stack_size)

    def push_rgb_frame(self, img_rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        img_rgb = cv2.resize(img_rgb, (self.img_width, self.img_height), interpolation=cv2.INTER_AREA)
        img_f = img_rgb.astype(np.float32) / 255.0
        img_chw = np.transpose(img_f, (2, 0, 1))

        if len(self.frame_buffer) == 0:
            for _ in range(self.stack_size):
                self.frame_buffer.append(img_chw)
        else:
            self.frame_buffer.append(img_chw)

        stacked_chw = np.concatenate(list(self.frame_buffer), axis=0)
        img_nchw = np.expand_dims(stacked_chw, axis=0)
        img_nhwc = np.transpose(img_nchw, (0, 2, 3, 1))
        return img_nchw, img_nhwc

    def get_stacked_debug_rgb(self) -> np.ndarray:
        frames_vis = []
        for frame in self.frame_buffer:
            f_hwc = np.transpose(frame, (1, 2, 0))
            f_uint8 = (f_hwc * 255.0).astype(np.uint8)
            frames_vis.append(f_uint8)
        return np.concatenate(frames_vis, axis=1)


def create_onnx_session(model_path: str) -> ort.InferenceSession:
    return ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])


def select_action_output_name(output_names: list[str], param_action: str) -> str:
    preferred = ['continuous_actions', 'deterministic_continuous_actions']
    if param_action:
        if 'discrete' in param_action:
            raise RuntimeError(
                'discrete action outputs are not supported. '
                f"requested action_output_name='{param_action}'"
            )
        if param_action not in output_names:
            raise RuntimeError(
                f"action_output_name='{param_action}' is not in model outputs. "
                f'available_outputs={output_names}'
            )
        action_output_name = param_action
    else:
        action_output_name = next((n for n in preferred if n in output_names), None)
        if action_output_name is None:
            raise RuntimeError(
                f'action output not found. preferred={preferred}, '
                f'available_outputs={output_names}'
            )

    if 'discrete' in action_output_name:
        raise RuntimeError(
            'discrete action outputs are not supported. '
            f"selected action_output_name='{action_output_name}', "
            f'available_outputs={output_names}'
        )

    return action_output_name


def build_model_feed(
    input_names: list[str],
    input_shapes: list[list[int | str]],
    img_nchw: np.ndarray,
    img_nhwc: np.ndarray,
    vec: np.ndarray,
    stack_size: int,
) -> dict:
    feed = {}
    for name, shape in zip(input_names, input_shapes):
        if len(shape) == 4:
            ch_dim1 = shape[1]
            if isinstance(ch_dim1, int) and (ch_dim1 == 3 or ch_dim1 == 3 * stack_size):
                feed[name] = img_nchw
            else:
                feed[name] = img_nhwc
        else:
            target_dim = shape[-1] if isinstance(shape[-1], int) else vec.shape[1]
            vector = vec
            if isinstance(target_dim, int) and vector.shape[1] != target_dim:
                if vector.shape[1] < target_dim:
                    vector = np.pad(
                        vector,
                        ((0, 0), (0, target_dim - vector.shape[1])),
                        constant_values=0.0,
                    )
                else:
                    vector = vector[:, :target_dim]
            feed[name] = vector.astype(np.float32)
    return feed

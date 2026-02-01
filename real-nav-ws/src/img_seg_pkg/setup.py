from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'img_seg_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=(
        lambda model_patterns: [
            (
                'share/ament_index/resource_index/packages',
                ['resource/' + package_name],
            ),
            (
                'share/' + package_name,
                ['package.xml'],
            ),
            (
                os.path.join('share', package_name, 'models'),
                sorted(
                    {
                        model_path
                        for pattern in model_patterns
                        for model_path in glob(pattern)
                    }
                ),
            ),
        ]
    )(
        [
            'models/*.pth',
            'models/*.pt',
            'models/*.onnx',
        ]
    ),
    install_requires=[
        'setuptools',
        'torch>=2.0.0',
        'torchvision>=0.15.0',
        'opencv-python>=4.5.0',
        'numpy>=1.21.0,<2.0.0',
        'segmentation-models-pytorch==0.3.3',
        'albumentations>=1.3.0',
        'ultralytics==8.4.8',
        'ultralytics-thop==2.0.18',
    ],
    zip_safe=True,
    maintainer='daichi-kiyozaki',
    maintainer_email='kiyodai02@yahoo.co.jp',
    description='ROS2 node for pedestrian and drivable area segmentation using YOLO and semantic segmentation models',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pedflow_4cls_seg_node = img_seg_pkg.pedflow_4cls_seg_node:main',
        ],
    },
)

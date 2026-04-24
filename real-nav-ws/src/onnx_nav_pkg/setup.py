from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'onnx_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'urg'), glob('urg/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'onnxruntime', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='daichi-kiyozaki',
    maintainer_email='kiyodai02@yahoo.co.jp',
    description='ONNX navigation inference package for real robot',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'real_onnx_nav_node = onnx_nav_pkg.onnx_nav_node:main',
            'action_to_twist_node = onnx_nav_pkg.action_to_twist_node:main',
        ],
    },
)

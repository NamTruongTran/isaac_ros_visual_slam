# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0


from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from launch.substitutions import LaunchConfiguration


import os
import xacro
import tempfile


def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model_path = os.path.join(get_package_share_directory('bearcar_bringup'))
    bearcar_launch = os.path.join(model_path,'launch') 
    
    """Launch file which brings up visual slam node configured for RealSense."""
    realsense_camera_node = ComposableNode(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': False,
                'enable_depth': False,
                'depth_module.emitter_enabled': 0,
                'depth_module.profile': '640x360x30',
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'unite_imu_method': 2
        }]
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'input_base_frame': 'base_link',
                    'input_left_camera_frame': 'D455_infra1_frame',
                    'input_right_camera_frame': 'D455_infra2_frame',
                    'input_imu_frame': 'D455_gyro_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 60.00
                    }],
        remappings=[('stereo_camera/left/image', 'camera/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'camera/infra1/camera_info'),
                    ('stereo_camera/right/image', 'camera/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'camera/infra2/camera_info'),
                    ('visual_slam/imu', 'camera/imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node,
            realsense_camera_node
        ],
        output='screen'
    )
    
    xacro_path = os.path.join(get_package_share_directory('bearcar_description'), 'urdf', 'bearcar.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true'})
    model_node = Node(
        name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
        )

    return launch.LaunchDescription([
    	visual_slam_launch_container, 
    	
    	model_node
	])

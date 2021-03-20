# Copyright 2021 Carlos San Vicente
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from tracetools_launch.action import Trace
from tracetools_trace.tools.names import DEFAULT_EVENTS_ROS


def generate_launch_description():
    ros_tracing_memory_usage = Trace(
        session_name='pendulum-memory-usage',
        events_ust=[
                'lttng_ust_libc:malloc',
                'lttng_ust_libc:calloc',
                'lttng_ust_libc:realloc',
                'lttng_ust_libc:free',
                'lttng_ust_libc:memalign',
                'lttng_ust_libc:posix_memalign',
            ] + DEFAULT_EVENTS_ROS,
        events_kernel=[
            'kmem_mm_page_alloc',
            'kmem_mm_page_free',
        ]
    )

    # Get the bringup directory
    bringup_dir = FindPackageShare('pendulum_bringup').find('pendulum_bringup')
    pendulum_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir, '/launch/pendulum_bringup.launch.py'])
    )

    ld = LaunchDescription()
    ld.add_action(ros_tracing_memory_usage)
    ld.add_action(pendulum_launch)

    return ld

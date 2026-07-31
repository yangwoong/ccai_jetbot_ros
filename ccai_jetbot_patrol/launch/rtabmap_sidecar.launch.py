"""rtabmap_ros sidecar launch - runs INSIDE the separate ccai-rtabmap
container (see scripts/run_rtabmap_container.sh and
docker/rtabmap_sidecar/Dockerfile), not the main ccai-jetbot one.

UNVERIFIED (2026-07-28): never run on real hardware. Subscribes directly
to the D435i topics the main container's realsense2_camera node already
publishes (shared via --network host + matching ROS_DOMAIN_ID, no code
changes needed on the main container's side) and runs rtabmap's OWN
RGBD odometry (rgbd_odometry, package rtabmap_odom) rather than this
project's custom visual_odom_node - deliberately self-contained so this
sidecar has no dependency on that node's message format/TF frames and can
be tested/debugged in isolation. frame_id is the camera's own optical
frame (published by realsense2_camera itself, no extra TF setup needed) -
if/when this is worth wiring into the robot's actual base_link frame for
navigation, that's a deliberate follow-up, not assumed here.

Needs the main container running with the D435i active (realsense2_camera
+ CCAI_ENABLE_VISUAL_ODOM=1, so align_depth is enabled and
/camera/camera/aligned_depth_to_color/image_raw actually exists - see
config/robot.yaml's visual_odom_node section, which already assumes
these exact topic names).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


RGB_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
FRAME_ID = "camera_color_optical_frame"

REMAPPINGS = [
    ("rgb/image", RGB_TOPIC),
    ("depth/image", DEPTH_TOPIC),
    ("rgb/camera_info", CAMERA_INFO_TOPIC),
]

RTABMAP_PARAMETERS = {
    "frame_id": FRAME_ID,
    "subscribe_depth": True,
    "subscribe_rgbd": False,
    "approx_sync": True,
    # No wheel encoders/IMU on this robot - rgbd_odometry is the only
    # odometry source available to this sidecar (see module docstring for
    # why this doesn't reuse the project's own visual_odom_node here).
    "Reg/Force3DoF": "true",  # ground robot: constrain to planar motion
    "RGBD/NeighborLinkRefining": "true",
    "RGBD/ProximityBySpace": "true",
    "Grid/FromDepth": "true",
}


def generate_launch_description() -> LaunchDescription:
    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[{"frame_id": FRAME_ID, "approx_sync": True, "Reg/Force3DoF": "true"}],
        remappings=REMAPPINGS,
    )
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[RTABMAP_PARAMETERS],
        remappings=REMAPPINGS,
        arguments=["-d"],  # fresh database each run while this is still experimental
    )
    return LaunchDescription([rgbd_odometry, rtabmap])

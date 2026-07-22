from glob import glob
from setuptools import find_packages, setup

package_name = "ccai_jetbot_patrol"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "requests", "pyyaml", "fastapi", "uvicorn"],
    zip_safe=True,
    maintainer="CCAI JetBot Team",
    maintainer_email="admin@example.com",
    description="Autonomous patrol robot stack for Jetson Nano JetBot with remote VLM inference.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_node = ccai_jetbot_patrol.patrol_node:main",
            "jetbot_hardware_node = ccai_jetbot_patrol.jetbot_hardware_node:main",
            "camera_node = ccai_jetbot_patrol.camera_node:main",
            "vision_nav_node = ccai_jetbot_patrol.vision_nav_node:main",
            "vlm_client_node = ccai_jetbot_patrol.vlm_client_node:main",
            "llm_control_node = ccai_jetbot_patrol.llm_control_node:main",
            "web_chat_node = ccai_jetbot_patrol.web_chat_node:main",
            "telegram_bridge_node = ccai_jetbot_patrol.telegram_bridge_node:main",
            "ota_agent_node = ccai_jetbot_patrol.ota_agent_node:main",
        ],
    },
)

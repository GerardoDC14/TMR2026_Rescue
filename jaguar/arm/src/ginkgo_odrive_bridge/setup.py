from pathlib import Path

from setuptools import find_packages, setup

package_name = "ginkgo_odrive_bridge"


def package_files(source_dir: str, install_root: str) -> list[tuple[str, list[str]]]:
    root = Path(source_dir)
    if not root.exists():
        return []

    collected: dict[str, list[str]] = {}
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if "__pycache__" in path.parts:
            continue
        install_dir = Path(install_root) / path.parent
        collected.setdefault(str(install_dir), []).append(str(path))
    return sorted(collected.items())


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml", "README.md"]),
]
data_files.extend(package_files("config", f"share/{package_name}"))
data_files.extend(package_files("launch", f"share/{package_name}"))
data_files.extend(package_files("Python_USB_CAN_Test_64bits", f"share/{package_name}"))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Gerardo",
    maintainer_email="gerardo@example.com",
    description="ROS 2 joint-state bridge for ODrive over a Ginkgo USB-CAN adapter.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "joint_state_bridge = ginkgo_odrive_bridge.joint_state_bridge:main",
        ],
    },
)

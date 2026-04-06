from pathlib import Path

from setuptools import find_packages, setup


package_name = "bldc_can_tools"


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
data_files.extend(package_files("launch", f"share/{package_name}"))
data_files.extend(package_files("config", f"share/{package_name}"))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    scripts=["scripts/cansniffer.py"],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="gerardo",
    maintainer_email="gerardodelcid16@gmail.com",
    description="Windows-first multi-family BLDC CAN utilities built as an ament_python package.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lktech_monitor = bldc_can_tools.monitor_node:main",
            "lktech_position_test = bldc_can_tools.position_test_node:main",
            "lktech_cli_read_angle = bldc_can_tools.cli_read_angle:main",
            "lktech_cli_position_test = bldc_can_tools.cli_position_test:main",
            "lktech_can_health_check = bldc_can_tools.cli_can_health_check:main",
            "bldc_ze300_read = bldc_can_tools.cli_ze300_read:main",
        ],
    },
)


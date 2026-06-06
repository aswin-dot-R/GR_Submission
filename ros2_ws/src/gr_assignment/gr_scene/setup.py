from setuptools import find_packages, setup
from glob import glob

package_name = "gr_scene"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/urdf",   glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ashie",
    maintainer_email="aswinrajan2002@gmail.com",
    description="Planning scene for GR Assignment",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scene_loader = gr_scene.scene_loader:main",
        ],
    },
)

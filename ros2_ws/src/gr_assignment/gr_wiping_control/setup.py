from setuptools import find_packages, setup
from glob import glob

package_name = "gr_wiping_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/description", glob("description/*.xacro")),
        (f"share/{package_name}/worlds", glob("worlds/*.world") + glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ashie",
    maintainer_email="aswinrajan2002@gmail.com",
    description="Contact-aware wiping controller",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wiping_controller = gr_wiping_control.controller:main",
            "wiping_moveit = gr_wiping_control.moveit_wiping:main",
            "traj_player = gr_wiping_control.traj_player:main",
        ],
    },
)

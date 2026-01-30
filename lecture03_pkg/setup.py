from setuptools import find_packages, setup

package_name = "lecture03_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tomoaki Fujino",
    maintainer_email="fujino0728@gmail.com",
    description="Lecture03: TurtleBot3, SLAM, Navigation2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "turtlebot_controller = lecture03_pkg.turtlebot_controller:main",
            "turtlebot_stop = lecture03_pkg.turtlebot_stop:main",
            "turtlebot_exercise1 = lecture03_pkg.turtlebot_exercise1:main",
            "turtlebot_exercise2 = lecture03_pkg.turtlebot_exercise2:main",
            "show_pose = lecture03_pkg.show_pose:main",
            "navigation_sample1 = lecture03_pkg.navigation_sample1:main",
            "navigation_sample2 = lecture03_pkg.navigation_sample2:main",
        ],
    },
)

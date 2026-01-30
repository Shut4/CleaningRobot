from setuptools import find_packages, setup

package_name = "lecture01_pkg"

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
    description="Lecture01: Publisher, Subscriber",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "talker = lecture01_pkg.talker:main",
            "listener = lecture01_pkg.listener:main",
            "talker_exercise1 = lecture01_pkg.talker_exercise1:main",
            "listener_exercise1 = lecture01_pkg.listener_exercise1:main",
            "talker_exercise2 = lecture01_pkg.talker_exercise2:main",
            "listener_exercise2 = lecture01_pkg.listener_exercise2:main",
        ],
    },
)

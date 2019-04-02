from setuptools import setup

setup(
    name="CRUES",
    version="0.1",
    packages=['crues'],
    requires=['RPi.GPIO', 'rospy'],
    install_requires={
        'futures; python_version == "2.7"'
    }
)

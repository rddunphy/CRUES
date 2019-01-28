from setuptools import setup, find_packages

setup(
    name="CRUES",
    version="0.1",
    packages=['crues'],
    requires=['RPi.GPIO', 'rospy']
)

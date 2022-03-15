from setuptools import setup

from max30102 import __version__

setup(
    name="micropython-max30102",
    version=__version__,
    description="MAX30102 driver for micropython.",
    long_description=open("README.md").read(),
    
    url="https://github.com/n-elia/MAX30102-MicroPython-driver",
    license="MIT",
    keywords="micropython",

    author="Nicola Elia",
    author_email="nicolaelia94@gmail.com",
    maintainer="Nicola Elia",
    maintainer_email="nicolaelia94@gmail.com",

    packages=["max30102"],    
)

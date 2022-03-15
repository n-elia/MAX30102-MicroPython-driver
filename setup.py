from setuptools import setup, find_packages
import sdist_upip

setup(
    name="micropython-max30102",
    version="0.3.3",
    description="MAX30102 driver for micropython.",
    long_description=open("README.md").read(),
    long_description_content_type='text/markdown',

    url="https://github.com/n-elia/MAX30102-MicroPython-driver",
    license="MIT",
    keywords="micropython",

    author="Nicola Elia",
    maintainer="Nicola Elia",

    classifiers=[
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: Implementation :: MicroPython",
    ],
    cmdclass={"sdist": sdist_upip.sdist},

    packages=find_packages()
)
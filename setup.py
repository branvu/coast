from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="coast",
    version="0.0.1",
    author="Brandon Vu, Toki Migimatsu",
    author_email="vubt@cs.stanford.edu, takatoki@cs.stanford.edu",
    description="Sampling-based TAMP",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/branvu/coast",  # Replace with your actual GitHub repo URL
    packages=find_packages(include=["coast*"]),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.9",
    install_requires=[
        # Add your dependencies here
        # For example:
        "numpy==1.23.0",
        "scipy==1.8.1",
        "spatialdyn==1.5.0",
        "ctrlutils==1.4.1",
        "pybullet==3.2.5",
        "pysymbolic",
        "motion_planners@git+https://github.com/branvu/motion-planners.git@master",
    ],
    extras_require={
        "dev": [
            "mypy",
            # Add other development dependencies here
        ],
    },
    license="MIT",
    license_files=("LICENSE",),
)
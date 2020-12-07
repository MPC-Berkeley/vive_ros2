import setuptools

setuptools.setup(
    name="vive_ros2_server",
    version="0.0.1",
    author="Michael Equi",
    author_email="michaelequi@berkeley.edu",
    description="Package for running the pure python server",
    long_description="",
    long_description_content_type="text/markdown",
    url="https://github.com/MPC-Berkeley/vive_ros2",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)

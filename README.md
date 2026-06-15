# DAVE

[![Publish a Docker image (AMD64; Common X86_64 Linux Machine)](https://github.com/IOES-Lab/dave/actions/workflows/docker-amd64.yml/badge.svg)](https://github.com/IOES-Lab/dave/actions/workflows/docker-amd64.yml)
[![Publish a Docker image (ARM64; Apple Silicon)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml/badge.svg?branch=ros2)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml)

Documentation is currently at [http://dave-ros2.notion.site](http://dave-ros2.notion.site)

## 3D Sonar

This fork (`dev/sonar_3d`) adds a 3D-sonar pipeline on top of the CUDA
multibeam sonar. See
[`gazebo/dave_gz_multibeam_sonar/sonar_3d/README.md`](gazebo/dave_gz_multibeam_sonar/sonar_3d/README.md)
for the aggregator node, demo scenes, and sensor geometry.

For contribution, do `pip3 install pre-commit && pre-commit install && pre-commit run --all-files` before commit.

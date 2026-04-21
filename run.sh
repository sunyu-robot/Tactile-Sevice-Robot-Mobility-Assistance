#!/bin/bash
# Allow X11 connections from Docker
xhost +local:docker

docker compose run ros-noetic "$@"

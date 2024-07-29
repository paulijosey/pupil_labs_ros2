# pupil_labs_ros2
A ROS2 node to stream Data from a Pupillabs device. 

## Description
This repo implements a ROS2 node to interact with the Pupillabs python API. The goal is 
to stream image data and gaze data (in pixel coordinates) as well as IMU data as ROS 
topics.

## Installation
To use and develop this project a Docker environment is included (.devontainer). Before 
starting this run the install.bash script to get the right folder structure.

## Docker
To start the docker container use the provided docker-compose file (or VSCodes integrated 
devcontainer extension). 

```sh
docker compose -f .devcontainer/docker-compose.yml up -d
```

and enter the container

```
docker exec -it pupil_labs_dev bash
```

## Usage 
The ROS workspace inside the container is located at /ws.
All build commands ("colcon") should be available. 

### First build
Call colcon from /ws
```
colcon build
```

## Run the Node
Source the development workspace
```
source /ws/install/setup.bash
```

and run 
```
ros2 launch pupil_labs_ros2 pupil_labs_stream.yml
```

## Additional Info
You might have realized that the folder structure inside the devcontainer is not the same 
as in the git repo. This is on purpose. The two ROS packages "pupil_labs_ros2" and the 
additional messages package "pupil_labs_ros2_msgs" are mounted to the src location of
the workspace and everything that is related to the build process (and should be cached) 
located in the cache directory.

This readme does not include a dependency list since all required packages are added to the 
Docker environment (see .devcontainer/Dockerfile for more information).
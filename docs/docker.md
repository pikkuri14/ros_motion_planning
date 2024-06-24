# Docker Configuration

Docker is a powerful tool for managing and deploying applications in a consistent and reproducible manner, and it is also increasingly being used in robotic applications. Docker containers are designed to be reproducible, meaning that the same container image will produce the same results regardless of where it is run. This is important in robotic applications where the behavior of the robot needs to be consistent across different deployments.

first install docker, the step installation can be found at [docker-installaion](https://docs.docker.com/engine/install/ubuntu/)
make sure the docker can be run without sudo

cd to the docker directory
```sh
cd docker/<distro>
```
Then build the image using docker file
```sh
docker build -t ros-motion-planning:<distro> --no-cache -f ./Dockerfile1 ../../
```
Finally run the container using docker-compose
```sh
docker compose up
```
Here \<distro> refers to the specific ROS distributions, i.e., `kinetic`, `melodic`, `noetic`.

in order to use gui in docker, run code below in host machine terminal
```sh
xhost local:root
```
- to open up the container, personally, I use remote-window and docker plugin provided by vscode


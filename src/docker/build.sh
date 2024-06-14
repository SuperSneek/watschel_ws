#!/bin/bash

# needs docker buildx and the path to a ssh key that is authorized for github

# clone our own code and move to container. otherwise we need to either create an
# access token or copy ssh keys to the container
git clone git@github.com:Nordegraf/HumanCenteredRobotics.git

docker build --rm -t solo8 .

xhost +
docker run -i --net=host -t solo8 /bin/bash
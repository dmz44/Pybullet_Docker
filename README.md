# Pybullet_Docker

By MinHyuk Park

Pybullet_Docker for reinforcement learning

put mesh.obj from 3d scan in docker_shared folder.

xhost +local:docker   #needed for X11

docker run -it --rm \
  --gpus all \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v ~/pybullet_docker/docker_shared:/shared \
  --workdir /shared \
  --name pybullet \
  pybullet \
  bash

docker exec -it pybullet bash
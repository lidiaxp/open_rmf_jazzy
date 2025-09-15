# #!/bin/bash

# # Make the script executable
# chmod +x entrypoint.sh

# # Allow X11 forwarding (run on host machine)
# xhost +local:root

# # Build and run the container
# docker compose up --build -d

# # Execute a bash shell in the container
# docker exec -it ros2-gazebo-dev /bin/bash 

xhost +

docker run -it \
  --name my-openrmf \
  --network host \
  --privileged \
  --runtime nvidia \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
  -v $(pwd)/..:/workspace:rw \
  -v $(pwd)/laris_robot:/home/ros/rmf_ws/src/laris_robot \
  -v $(pwd)/maps:/home/ros/rmf_ws/custom_maps \
  openrmf:latest /bin/bash

# docker exec -it my-openrmf /bin/bash/
# docker rm -f my-openrmf
# ros2 run rmf_demos_tasks dispatch_patrol -p main_corridor -n 3 --use_sim_time

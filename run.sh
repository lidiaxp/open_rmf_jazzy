xhost +
docker run -it --rm \
    -e ROS_MASTER_URI=http://$(hostname -I | awk '{print $1}'):11311 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e XAUTHORITY=$XAUTH \
    -v $XAUTH:$XAUTH \
    -v $(pwd)/maps:/root/rmf_ws/maps \
    --net=host \
    --privileged \
    jazzy_open_rmf \
    /bin/bash

xhost local:root

XAUTH=/tmp/.docker.xauth

docker run --rm -it \
    --name=omnidepth_humble_image_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
    --runtime=nvidia \
    omnidepth_humble_image \
    bash

echo "Done."



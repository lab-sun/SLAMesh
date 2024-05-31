#!/bin/bash

# SLAMesh docker

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If still not working, try to run the script as root.
# Acknowledgement: this file are modified from coin-lio(https://github.com/ethz-asl/COIN-LIO?tab=readme-ov-file), by yibo wang

# Default options
DOCKER=slamesh
DOCKERFILE=Dockerfile
NAME=slamesh
BUILD=false

help()
{
    echo "Usage: run_docker.sh [ -b | --build ] [ -n | --name <docker name> ]
               [ -h | --help  ]"
    exit 2
}

SHORT=b,n:,h
LONG=build:,name:,help
OPTS=$(getopt -a -n run_docker --options $SHORT --longoptions $LONG -- "$@")
echo $OPTS

eval set -- "$OPTS"

while :
do
  case "$1" in
    -b | --build )
      BUILD="true"
      shift
      ;;
    -n | --name )
      NAME="$2"
      shift 2
      ;;
    -h | --help)
      help
      ;;
    --)
      shift;
      break
      ;;
    *)
      echo "Unexpected option: $1"
      help
      ;;
  esac
done

if [ "$BUILD" = true ]; then
    echo "Building docker: $DOCKERFILE as $DOCKER"
    docker build -f $DOCKERFILE -t $DOCKER .
fi

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ -n "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker: $DOCKER as $NAME"

#docker run -it --rm \
#    --gpus=all \
#    --runtime=nvidia \
#    -e NVIDIA_DRIVER_CAPABILITIES=all \
#    --env="DISPLAY=$DISPLAY" \
#    -e "QT_X11_NO_MITSHM=1" \
#    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#    --env="XAUTHORITY=$XAUTH" \
#    --volume="$XAUTH:$XAUTH" \
#    --volume=path_of_dataset_in_your_PC:/home/ruanjy/Dataset/ \
#    --volume=/home/$USER/slamesh_result:/home/slamesh/slamesh_ws/slamesh_result \
#    --net=host \
#    --privileged \
#    --name=$NAME \
#    ${DOCKER} \
#    bash

echo "Done."
# Gazebo Sitl Plugin #

A ROS-independent Gazebo plugin for Ardupilot's SITL.

## Requirements ##
    * Python 2.7+ (to generate mavlink headers)

## Build ##

Make sure you have initialized and updated the Mavlink submodule at least once
with:

    git submodule init
    git submodule update

Create a build folder and make using CMAKE as follows:

    mkdir ./gzsitl/build
    cd gzsitl/build
    cmake ..
    make

Export the build folder so that Gazebo finds the plugin:

    cd ./gzsitl/build
    export GAZEBO_PLUGIN_PATH=$(pwd)

Also export the models folder:
    
    cd ./models
    export GAZEBO_MODEL_PATH=$(pwd)

Gazebo automatically searches and downloads models referenced in the .world file that are
not already present in ~/.gazebo/models or in GAZEBO_MODEL_PATH. However, in some cases
the download might take a long time for no apparent reason. Download the Quadrotor model
manually with:

    curl http://models.gazebosim.org/quadrotor/model.tar.gz | tar xvz -C ~/.gazebo/models

## Run ##

Run Ardupilot:

    ARDUPILOTDIR/build/sitl/bin/arducopter --model x

Run Mavproxy:

    mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551 --streamrate 20

Run gzsitl_drone_world in Gazebo:

    gazebo ./world/gzsitl_drone_target.world

## Interaction ##

The default behavior of the drone is to follow the transparent sphere wherever it goes.

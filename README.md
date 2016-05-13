# Gazebo Sitl Plugin #

A ROS-independent Gazebo plugin for Ardupilot's SITL.

## Requirements ##
    * Python 2.7+ (to generate mavlink headers)
    * MavProxy (https://github.com/Dronecode/MAVProxy)
    * Ardupilot (https://github.com/ArduPilot/ardupilot)

## Build ##

1. Make sure you have initialized and updated the Mavlink submodule at least
once with:

    ```
    git submodule init
    git submodule update
    ```

2. Create a build folder and make using CMAKE as follows:

    ```
    mkdir ./gzsitl/build
    cd gzsitl/build
    cmake ..
    make
    cd -
    ```

3. Download Gazebo models if necessary:

    Gazebo automatically searches and downloads models referenced in the .world
    file that are not already present in ~/.gazebo/models or in
    GAZEBO_MODEL_PATH. However, in some cases the download might take a long
    time for no apparent reason. Download the Quadrotor model manually with:

    ```
    curl http://models.gazebosim.org/quadrotor/model.tar.gz | tar xvz -C ~/.gazebo/models ...
    ```

## Run ##

1. Open a second terminal and run Ardupilot:

    ```
    ${ARDUPILOTDIR}/build/sitl/bin/arducopter-quad --model x
    ```

2. In a third terminal, run Mavproxy:

    ```
    mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14556 --streamrate -1
    ```

    Load the Parameters on Mavproxy Prompt:

    ```
    param load ./parameters/gzsitl.parm
    ```

3. Set environment variables and open the Gazebo gzsitl_drone_target world file with the following command:

    ```
    GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${PWD}/gzsitl/build GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PWD}/models gazebo ./world/gzsitl_drone_target.world
    ```

## Interaction ##

The default behavior of the drone is to follow the transparent sphere wherever it goes.

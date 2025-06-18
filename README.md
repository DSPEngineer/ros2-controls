# ROS2 Control Workshop Container

ROS Control Workshop with Turtlebot3

## Dependencies
* linux
* a [recent version of docker](https://docs.docker.com/engine/install/ubuntu/)
* make (`$ sudo apt install build-essential`)
* (Optional) - [Nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian)

## Usage
1. Clone the repo and build the image.

    ```
    $ git clone git@bitbucket.org:awp-controls/ros2-control-workshop-container.git
    $ cd ros2control-workshop-container
    $ make image
    ```

2. Run the container with src directory mounted, build and source ROS packages.

    ```
    # With nvidia GPU
    $ make run-gpu

    # Without nvidia GPU
    $ make run

    $ colcon build
    $ source install/setup.bash
    ```

3. Launch the simulation

    ```
    $ ros2 launch turtlebot3_gz_bringup tb3_gz.launch.py 
    ```

4. Launch teleop-joystick for SIM control

    ```
    $ ros2 launch teleop_twist_joy teleop-launch.py publish_stamped_twist:=true joy_vel:=diff_drive_controller/reference config_filepath:=src/turtlebot3_gz_bringup/config/8bitdo.yaml
    ```
5. Launch teleop-joystick for HW control

    ```
    $ ros2 launch teleop_twist_joy teleop-launch.py publish_stamped_twist:=true joy_vel:=turtlebot_base_controller/cmd_vel config_filepath:=src/turtlebot3_gz_bringup/config/8bitdo.yaml
    ```
6. Launch teleop keyboard for HW control

    ```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=turtlebot_base_controller/cmd_vel -p stamped:=true
    ```

## TODO:

* Implement turtlebot3_hardware package (ROS2 Control - Hardware Interface)
    * Checkpoint 1 - Get the hardware interface read / write loop running. Load plugin in the URDF.
    * Checkpoint 2 - Implement read / writes from Dynamixel
        * [Dynamixel XM430-W210 Control Table](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/)
        * [Dynamixel Sync Read Write Example](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c%2B%2B/example/protocol2.0/sync_read_write/sync_read_write.cpp)

* Implement turtlebot3_bringup package - Entrypoint to the container.
    * Launch description
    * Launch controls
    * Launch joystick


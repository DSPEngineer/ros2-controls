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

4. Use keyboard teleop twist:

    ```
    $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=turtlebot_base_controller/cmd_vel -p stamped:=true
    ```

# TODO:
## TurtleBot3 Setup Notes - task list on the whiteboard

turtlebot3_hardware
- [ ] Setup package  
- [ ] Implement hardware interface C++ (framework only)  
- [ ] Setup `plugin.xml` file  

---

`turtlebot3_description`
- [ ] Add plugin to URDF  

### Launch Commands

```bash
ros2 launch turtlebot3_description description.launch.py
ros2 launch turtlebot3_control control.launch.py
```

## Checkpoint 1 reached

### Dynamixel Integration
- [ ] Add Dynamixel code to hardware interface C++
- [ ] Refer controller for XM430-W210
- [ ] Refer example for sync read/write



# Original task list from Harshil:
* Implement turtlebot3_hardware package (ROS2 Control - Hardware Interface)
    * Checkpoint 1 - Get the hardware interface read / write loop running. Load plugin in the URDF.
    * Checkpoint 2 - Implement read / writes from Dynamixel
        * [Dynamixel XM430-W210 Control Table](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/)
        * [Dynamixel Sync Read Write Example](https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/c%2B%2B/example/protocol2.0/sync_read_write/sync_read_write.cpp)

* **`turtlebot3_bringup` package - Entrypoint to the container.**
    * Launch description
    * Launch controls
    * Launch joystick

# Additional info from Cline
    * **`turtlebot3_hardware_interface` Implementation (`src/turtlebot3_hardware/src/turtlebot3_hardware_interface.cpp`):**
        *   **`on_init`:**
            *   Parse the URDF to get the Dynamixel port (`/dev/ttyUSB0`).
            *   Validate that the URDF defines two joints, each with `position` and `velocity` state interfaces and a `velocity` command interface.
        *   **`on_configure`:**
            *   Instantiate `dynamixel::PortHandler` and `dynamixel::PacketHandler`.
            *   Open the port and set the baud rate.
        *   **`on_activate`:**
            *   Enable torque for both Dynamixel motors.
            *   Initialize command interfaces with current state to prevent initial movement.
        *   **`on_deactivate`:**
            *   Disable torque on both motors.
        *   **`read`:**
            *   Use `dynamixel::GroupSyncRead` to get the present position and velocity from both motors.
            *   Update the state interfaces with the new values.
        *   **`write`:**
            *   Use `dynamixel::GroupSyncWrite` to send the commanded velocities to both motors.

    * **`turtlebot3_control` Implementation:**
        *   Create a new controller source file (e.g., `src/turtlebot3_control/src/turtlebot3_controller.cpp`).
        *   Implement a `ros2_control_node` that subscribes to `/cmd_vel` (type `geometry_msgs/msg/Twist`).
        *   In the subscriber callback, convert the `Twist` message's linear and angular velocities into left and right wheel velocities.
        *   Publish the calculated wheel velocities to the hardware interface.



# Udev Rule for Dynamixel USB Adapter

This directory contains a `udev` rule to create a consistent, persistent device name for the USB-to-Dynamixel adapter.

## Why is this needed?

When you plug a USB device into Linux, it is assigned a port like `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc. This name can change each time you plug in the device or reboot the system, which would require you to change your ROS launch files every time.

This rule identifies the adapter by its unique hardware identifiers (Vendor ID and Product ID) and creates a stable symbolic link at `/dev/ttyDXL`. The robot's URDF file is already configured to use this stable name.

## Installation on the Target Robot

1.  Copy the rule file to the `rules.d` directory on your **target robot**:
    ```bash
    sudo cp 99-dynamixel.rules /etc/udev/rules.d/
    ```

2.  Reload the `udev` rules to apply the changes:
    ```bash
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

3.  Unplug and replug your USB2DYNAMIXEL adapter.

4.  Verify that the symbolic link has been created:
    ```bash
    ls -l /dev/ttyDXL
    ```
    You should see it pointing to a `/dev/ttyUSBx` device. Once this is confirmed, the hardware interface should work without any code changes.

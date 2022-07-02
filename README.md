# Xmo
My implementation based on work from the Exomy 3D printed rover project

# Requirements
- Raspberry Pi 4 suggest at least 4GB
- 128 GB SD card
- Ubuntu 22.04 LTS
- ROS2 Galactic 
- Logitech F710 gamepad

# Rapspberry Pi Setup
1. Download [22.04 Server LTS](https://ubuntu.com/download/raspberry-pi)
1. Install ubuntu using the Raspberry Pi Imager.  Here is a link to the official Ubuntu site with a walk-through [How to install Ubuntu Server on your Raspberry Pi](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview). 
   - Get the wifi online if you wish.
      -  I edited the file /boot/firmware/network-config using the following as an example. [Getting setup with Wi-Fi](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet)
   -  I went ahead and installed the minimal desktop after getting the RPI on my local network.  Then I disabled it as it was in the way for what I wanted to accomplish.
      ```bash
         sudo apt install lxde-core
      ```
   -  Make sure the **SSH** is enabled.  You will need it later if not immediately to do the next steps. 
   - Enabled video with raspi-config
      ```bash
         sudo apt install raspi-config
         sudo mount /dev/mmcblk0p1 /boot
         sudo raspi-config
      ```
      After enabling the video, you should reboot the pi:
      ```bash
         sudo reboot
      ```
   - Enable UART 5 for the GPS serial communications.
      ```bash
         /boot/firmware/config.txt
      ```
      Add the following
      ```bash
         dtoverlay=uart5
      ```
   - Some other stuff to install
      ```bash
          apt install ffmpeg
          apt install v4l-utils
          apt install -y i2c-tools
      ```

3. Install Docker: [How to Install Docker on Raspberry Pi 4](https://linuxhint.com/install_docker_raspberry_pi-2/)
4. Login to docker: ```$ docker login ```
5. Build the base and the primary 
```
   docker build -t xmo_base:latest ~/repos/Xmo/source/docker/xmo_base
   docker build -t xmo:latest ~/repos/Xmo/source/docker/xmo
```

- These should be available as a public repo on docker hub at some point in this with a release.

## Make sure the ROS2 container executes on the RPI4
You will probably want to do the following on desktop where you have multiple terminal windows open at the same time.
1. Get a shell on the RPI
2. Execute the following command:
```
docker run --rm -it --name ros-test1 -h ros2-1 --network="host" --pid=host --privileged xmo:latest
```
- you should now have a prompt in the ROS2 contianer.
3.  Get another shell on the RPI.
4.  Execute the follwoing command:
 ```
 docker run --rm -it --name ros-test2 -h ros2-2 --network="host" --pid=host --privileged xmo:latest
 ```
 5. In one of the terminals execute the following command:
```bash
ros2 run demo_nodes_cpp talker
```
6. In the other terminal excute the following command:
```bash
ros2 run demo_nodes_py listener
```
You should now have a **talker** and the **listener** communicating with each other.  You can now exit both terminals by whichever means you desire.


## Windows Setup
Follow the install using the guide from ros.org [Installing ROS 2 on Windows](https://docs.ros.org/en/galactic/Installation/Windows-Install-Binary.html#installing-ros-2-on-windows)

I chose to put the base install in __c:\dev\ros2_galactic__.  I had some space issues and installed the __Qt__ libraries on __d:\Qt__.

I only had one major problem with the install.  It is documented in the forms [qt.qpa.plugin: Could not find the Qt platform plugin "windows" in ""](https://answers.ros.org/question/354707/qtqpaplugin-could-not-find-the-qt-platform-plugin-windows-in/?comment=356089#post-id-356089)

I solved the error with the following environment settings:

```
QT_QPA_PLATFORM_PLUGIN_PATH = C:\dev\ros2_galactic\bin\platforms
Qt5_DIR = D:\Qt\5.15.0\msvc2019_64
```

Update your environment with the following:
```
ROS_DOMAIN_ID = 27
```

# Development Environment

## Docker build
- Install WSL2
- Ubuntu
```
sudo apt update 
sudo apt dist-upgrade
```

### docker-ce
```
sudo apt 
```

Source for this: https://www.linux.com/topic/desktop/how-install-docker-ce-your-desktop/
and Preference to this link https://medium.com/@mikejohanson/running-docker-ce-in-wsl2-ed74dac32782

# Building/Dev-ing the thing.
## Joy
[Joy Node Reference](http://wiki.ros.org/joy)


# some useful links that need a shoutout when we can
### Why a parameter service ain't right with ROS2
[YAML for parameters](https://roboticsbackend.com/ros2-yaml-params/#From_a_launch_file)

[Naming Conventions](https://pep8.org/#prescriptive-naming-conventions)

[ROS2 Node documentation](https://docs.ros2.org/foxy/api/rclpy/api/node.html)

[FFMPEG Test](https://stegard.net/2021/07/capture-images-from-a-webcam-using-ffmpeg/)

[Install raspi-config to enable camera](https://chuckmails.medium.com/enable-pi-camera-with-raspberry-pi4-ubuntu-20-10-327208312f6e)

# some commands that need 'memberin' and doc'ed
- capture image on Xmo: ros2 run image_tools cam2image --ros-args -p frequency:=5.0 -p reliability:=best_effort
- show image from capture:  ros2 run image_tools showimage --ros-args -p show_image:=true -p reliability:=best_effort

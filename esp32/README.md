## Old (obsolete) example over serial USB

See ../microros_example/README.md

## New example over UDP (WiFI hotspot), uses Docker

From current directory (`esp32/`), run:

```bash
git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git
cd micro_ros_espidf_component
```

Then prepare for connecting ESP32 board:

```bash
# 4. Before you connect the ESP32 run following command in new terminal window
sudo dmesg --follow

# 5. connect the ESP32 to laptop while the command is running and it will show
# the port number the ESP32 is connected to - for me it was /dev/ttyUSB0
# you can stop the command from running by hitting CTRL+C

# 6. check the permission on that port by running
ls -l /dev/ttyUSB0

# it will show something like this, which means we don't have permissions
# to write to the port yet
crw-rw---- 1 root dialout 188, 0 Aug 28 19:41 /dev/ttyUSB0

# 7. Run following command to grant permission to write, we will need this
# to flash ESP32 with the example program we are going to run which is
# examples/int32_publisher
sudo chmod 666 /dev/ttyUSB0
```

And run (may need to run as sudo if you get permission errors):

```bash
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/micro_ros_espidf_component -v  /dev:/dev --privileged --workdir /micro_ros_espidf_component microros/esp-idf-microros:latest /bin/bash  -c "cd examples/int32_publisher; idf.py menuconfig build flash monitor"
```

Setup WiFi connection by following instructions at section 3. here: https://robofoundry.medium.com/esp32-micro-ros-actually-working-over-wifi-and-udp-transport-519a8ad52f65

Do not forget to press `CTRL + ]` at the end before disconnecting the USB cable from the ESP32 board.

Then run (while being connected on the same WiFi network as the ESP32 board):

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

## In the future

Instead of using `microros/micro-ros-agent:humble`, let's directly make our own image that runs everything on the same docker container (no need to install ROS2 on the host machine), starting from this Dockerfile ([source](https://github.com/micro-ROS/docker/blob/humble/micro-ROS-Agent/Dockerfile)):

```dockerfile
FROM microros/base:humble AS micro-ros-agent-builder

WORKDIR /uros_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  . install/local_setup.sh \
&&  apt update \
&&  ros2 run micro_ros_setup create_agent_ws.sh \
&&  ros2 run micro_ros_setup build_agent.sh \
&&  rm -rf log/ build/ src/

FROM ros:humble-ros-core

COPY --from=micro-ros-agent-builder /uros_ws /uros_ws

WORKDIR /uros_ws

# Disable shared memory
COPY disable_fastdds_shm.xml disable_fastdds_shm_localhost_only.xml /tmp/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV MICROROS_DISABLE_SHM=1

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo ". /uros_ws/install/setup.bash" >> ~/.bashrc

# setup entrypoint
COPY ./micro-ros_entrypoint.sh /
ENTRYPOINT ["/bin/sh", "/micro-ros_entrypoint.sh"]
CMD ["--help"]
```

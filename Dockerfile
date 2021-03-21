FROM ros:kinetic-ros-base
RUN apt-get update && apt-get install -y ros-kinetic-rosserial-client ros-kinetic-rosserial-arduino ros-kinetic-rosserial-server ros-kinetic-rosserial-msgs ros-kinetic-rosserial-python ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-joy ros-kinetic-geographic-msgs && apt-get clean

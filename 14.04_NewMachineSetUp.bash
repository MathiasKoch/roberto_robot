#!/usr/bin/env bash

# Some of this script inspired from Chao Qu
#
# Author: Justin Thomas

# Stop if command ends in error
set -e

############################################################################
#########################  Update and New Packages   #######################
############################################################################

sudo apt-get update -y && sudo apt-get upgrade -y
sudo apt-get install -y vim git ncurses-term curl lshw
sudo apt-get autoremove -y

# Set the Timezone
echo -n "Setup Timezone? (y/n)? " && read answer
if echo "$answer" | grep -iq "^y" ;then
  sudo dpkg-reconfigure tzdata
fi

############################################################################
#########################   SSH Keys   #####################################
############################################################################

echo 'Instead of setting up SSH keys, the preferred method is to use SSH agent'
echo 'forwarding. For details, see the guide here:'
echo 'https://developer.github.com/guides/using-ssh-agent-forwarding/'

# echo -n "Setup SSH keys? (y/n)? " && read answer
# if echo "$answer" | grep -iq "^y" ;then
#   read -e -p "Setting up SSH keys. Please enter your email address: " -i "justinthomas@jtwebs.net" email_address
#   ssh-keygen -t rsa -C ${email_address}
#   eval "$(ssh-agent -s)"
#   ssh-add ~/.ssh/id_rsa
#   
#   echo -e "Your id_rsa.pub is: \n"
#   cat ~/.ssh/id_rsa.pub
#   echo -e "\nIf you would like to add it to Github, add it now. Then, press [Enter] to resume setup..."
#   read
# fi

############################################################################
#########################   ROS Setup   ####################################
############################################################################

echo -n "Setup ROS kinetic? (y/n)? "
read answer
if echo "$answer" | grep -iq "^y" ;then

  #### Set up ROS sources list ####
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

  # Set up your keys
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

  # Update
  sudo apt-get update -y && sudo apt-get upgrade -y

  # Some needed dependencies
  sudo apt-get install -y xserver-xorg-dev-lts-utopic mesa-common-dev-lts-utopic libxatracker-dev-lts-utopic libopenvg1-mesa-dev-lts-utopic libgles2-mesa-dev-lts-utopic libgles1-mesa-dev-lts-utopic libgl1-mesa-dev-lts-utopic libgbm-dev-lts-utopic libegl1-mesa-dev-lts-utopic libeigen3-dev 

  # Install ROS base
  # sudo apt-get install ros-kinetic-desktop-full
  sudo apt-get install -y ros-kinetic-ros-base
  sudo rosdep init
  rosdep update
  
  # Environment setup
  if [[ $(grep -Fxq "/opt/ros/kinetic/setup.bash" ~/.bash_local) ]]
  then
      echo "Environment already set up"
  else
      echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
      source ~/.bashrc
  fi

  # Getting rosinstall and catkin-tools
  sudo apt-get install -y python-rosinstall python-catkin-tools

  # Set up a catkin workspace
  source /opt/ros/kinetic/setup.bash  
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  catkin init
  
else
  
  echo "Skipping installation of ROS..."

fi

############################################################################
#########################   WINE Setup for Flash Magic   ###################
############################################################################

# sudo apt-get install wine
# ln -s /dev/ttyUSB0 ~/.wine/dosdevices/com1

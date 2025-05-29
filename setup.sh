#!/bin/bash

#This scrip shold never be run exept under controlled sircumstances

# Check if running as root
if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root. Try using: sudo $0"
    exit 1
fi

Confirm_funct () {
    echo "this script will change significant part ot the opperating system."
    echo "are you sure you want to runn the settup change the behavior of the raspberrypie"
    read -p "Do you want to continue? [Y/N]: " confirm
    if [[ "$confirm" = "n" || "$confirm" = "N" ]]; then
        echo "Aborted."
        exit 1
    elif [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        Confirm_funct
    fi

}

Confirm_funct

# uppdates ubuntu and drivers
sudo apt -y update
sudo apt -y upgrade


#dissables wait online for a faster boot
sudo systemctl disable NetworkManager-wait-online.service 


read -p "Do you want to install ros2 jazzy? [Y/N]: " confirm

if [[ "$confirm" = "y" || "$confirm" = "Y" ]]; then
    # Ros2 jazzy install

    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings

    sudo apt install software-properties-common
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update && sudo apt install ros-dev-tools

    sudo apt update
    sudo apt upgrade

    sudo apt install ros-jazzy-ros-base

    source /opt/ros/jazzy/setup.bash

    # Get the username of the user who invoked sudo
    REAL_USER=$(logname)
    BASHRC_PATH="/home/$REAL_USER/.bashrc"
    LINE_TO_ADD="source /opt/ros/jazzy/setup.bash"

    # Add the line only if it's not already present
    if ! grep -Fxq "$LINE_TO_ADD" "$BASHRC_PATH"; then
        echo "$LINE_TO_ADD" >> "$BASHRC_PATH"
        echo "Added to $BASHRC_PATH: $LINE_TO_ADD"
    else
        echo "Line already exists in $BASHRC_PATH"
    fi

fi





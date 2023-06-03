#!/bin/bash -e


section(){

    echo "==========================="
    echo "==========================="

}


set_python3_as_default(){

    cd $HOME/

    section
    echo "Starting python 3 config as default"
    section

    ${SUDO} apt install -y python-is-python3

}

install_pip_dependencies(){

    cd $HOME/

    section
    echo "Starting pip installation"
    section

    ${SUDO} apt install -y python3-dev python3-numpy python3-pip 

    pip install pyserial opencv-python

}

install_gstreamer(){

    section
    echo "Starting gstreamer installation"
    section

    cd $HOME/

    ${SUDO} apt install -y gstreamer1.0-tools gstreamer1.0-nice gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libsoup2.4-dev libjson-glib-dev
    ${SUDO} apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev

}



install_build_dependencies(){

    ${SUDO} apt install -y build-essential cmake unzip pkg-config
    ${SUDO} apt install -y libjpeg-dev libpng-dev libtiff-dev
    ${SUDO} apt install -y libavcodec-dev libavformat-dev libswscale-dev
    ${SUDO} apt install -y libgtk2.0-dev libcanberra-gtk*
    ${SUDO} apt install -y libxvidcore-dev libx264-dev libgtk-3-dev
    ${SUDO} apt install -y libtbb2 libtbb-dev libdc1394-22-dev
    ${SUDO} apt install -y libv4l-dev v4l-utils
    ${SUDO} apt install -y libavresample-dev libvorbis-dev libxine2-dev
    ${SUDO} apt install -y libfaac-dev libmp3lame-dev libtheora-dev
    ${SUDO} apt install -y libopencore-amrnb-dev libopencore-amrwb-dev
    ${SUDO} apt install -y libopenblas-dev libatlas-base-dev libblas-dev
    ${SUDO} apt install -y liblapack-dev libeigen3-dev gfortran
    ${SUDO} apt install -y libhdf5-dev protobuf-compiler
    ${SUDO} apt install -y libprotobuf-dev libgoogle-glog-dev libgflags-dev
    ${SUDO} apt install -y libudev-dev
    ${SUDO} apt install -y libusb-1.0-0-dev
}


install_ros2(){

    locale  # check for UTF-8

    ${SUDO} apt update && ${SUDO} apt install locales
    ${SUDO} locale-gen fr_FR fr_FR.UTF-8
    ${SUDO} update-locale LC_ALL=fr_FR.UTF-8 LANG=fr_FR.UTF-8
    export LANG=fr_FR.UTF-8

    locale  # verify settings

    ${SUDO} apt install -y software-properties-common
    ${SUDO} add-apt-repository universe

    ${SUDO} apt update && ${SUDO} apt install -y curl
    ${SUDO} curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | ${SUDO} tee /etc/apt/sources.list.d/ros2.list > /dev/null

    ${SUDO} apt update
    ${SUDO} apt upgrade

    ${SUDO} apt install -y ros-$ros_version-ros-base
    ${SUDO} apt install -y ros-dev-tools

    ${SUDO} apt install -y python3-colcon-common-extensions

    pip install setuptools==58.2.0

    ${SUDO} apt install -y ros-$ros_version-image-tools
    ${SUDO} apt install -y ros-$ros_version-cv-bridge
    ${SUDO} apt install -y ros-$ros_version-vision-opencv

    section

}



set_ros_workspace(){

    section
    echo "Starting DRONE service script installation"
    section
    
    mkdir -p ~/manoar_ros_ws/src
    cd ~/manoar_ros_ws/src

    if [ $device_type == 'operator' ]
    then
        git clone https://github.com/man-o-ar/trafalgar_operator_v0.git
    else
        git clone https://github.com/man-o-ar/trafalgar_drone_v0.git
    fi

    cd ~/manoar_ros_ws/

    source /opt/ros/$ros_version/setup.bash

    ${SUDO} rosdep init
    ${SUDO} rosdep update

    rosdep install -i --from-path src --rosdistro $ros_version -y
    colcon build --symlink-install

    section

}

set_ros_service(){

    section
    echo "Starting DRONE service script installation"
    section
    
    cd $HOME/

    if [ $device_type == 'operator' ]
    then

        sudo cp ~/manoar_ros_ws/src/trafalgar_operator_v0/service/trafalgar_{$device_index}.service /etc/systemd/trafalgar.service
        ${SUDO} systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
    
    else
        sudo cp ~/manoar_ros_ws/src/trafalgar_drone_v0/service/trafalgar_{$device_index}.service /etc/systemd/trafalgar.service
    fi

    systemctl enable trafalgar.service
    systemctl start trafalgar.service
    #systemctl daemon-reload

    
}


set_on_automatic_login(){

    cd $HOME/

    section
    echo "please modify your settings to enable automatic login"
    section
    ${SUDO} nano /etc/gdm3/custom.conf
    
    echo "disabling screen lock"
    gsettings set org.gnome.desktop.screensaver lock-enabled false

}

restart(){

    if [ $device_type == 'operator' ]
        then
            set_on_automatic_login
    fi

    section
    echo "ending installation, reboot system after update"
    section

    cd $HOME/

    ${SUDO} apt update
    ${SUDO} apt upgrade
    ${SUDO} reboot


}



SUDO=""
if [[ $EUID -ne 0 ]]; then
  SUDO="sudo -E"
fi




while getopts r:d:i flag
    do
        case "${flag}" in
            r) ros_version=${OPTARG};;
            o) device_type=${OPTARG};;
            i) device_index=${OPTARG};;
        esac
    done



echo "ros_version_selected: $ros_version";

if [ $device_type == 'operator' ]
    then
        echo "Starting CONTROLLER Installation..."
    else
        echo "Starting DRONE Installation..."
fi
echo "device_index: $device_index";

cd $HOME/
sudo apt update

install_build_dependencies
set_python3_as_default
install_pip_dependencies
install_gstreamer

install_ros2
set_ros_workspace
set_ros_service

restart

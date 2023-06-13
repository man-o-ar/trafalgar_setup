#!/bin/bash -e


section(){

    echo "==========================="
    echo "==========================="

}


set_python3_as_default(){

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
    ${SUDO} apt install -y python3-tk
    
    sudo -u "$SUDO_USER" pip3 install pyserial opencv-python tk customtkinter Pillow
    

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

    ${SUDO} apt install -y build-essential cmake || echo "******* cmake install has failed *******"
    ${SUDO} apt install -y build-essential unzip || echo "******* unzip install has failed *******"
    ${SUDO} apt install -y build-essential pkg-config || echo "******* pkconfig install has failed *******"

    ${SUDO} apt install -y libjpeg-dev || echo "******* libjpeg-dev install has failed *******"
    ${SUDO} apt install -y libpng-dev || echo "******* libpng-dev install has failed *******"
    ${SUDO} apt install -y libtiff-dev || echo "******* libtiff-dev install has failed *******"

    ${SUDO} apt install -y libavcodec-dev || echo "******* libavcodec-dev install has failed *******"
    ${SUDO} apt install -y libavformat-dev || echo "******* libavformat-dev  install has failed *******"
    ${SUDO} apt install -y libswscale-dev || echo "******* libswscale-dev install has failed *******"

    ${SUDO} apt install -y libgtk2.0-dev || echo "******* libgtk2.0-dev install has failed *******"
    ${SUDO} apt install -y libcanberra-gtk* || echo "******* libcanberra-gtk install has failed *******"

    ${SUDO} apt install -y libxvidcore-dev || echo "******* libxvidcore-dev install has failed *******"
    ${SUDO} apt install -y libx264-dev || echo "******* libx264-dev install has failed *******"
    ${SUDO} apt install -y libgtk-3-dev || echo "******* libgtk-3-dev install has failed *******"

    ${SUDO} apt install -y libtbb2  || echo "******* libtbb2 install has failed *******"
    ${SUDO} apt install -y libtbb-dev  || echo "******* libtbb-dev install has failed *******"

    ${SUDO} apt install -y libv4l-dev || echo "******* libv4l-dev install has failed *******"
    ${SUDO} apt install -y v4l-utils || echo "******* v4l-utils install has failed *******"

    ${SUDO} apt install -y i2c-tools || echo "******* i2c-tools install has failed *******"

    ${SUDO} apt install -y libavresample-dev || echo "******* libavresample-dev install has failed *******"
    ${SUDO} apt install -y libvorbis-dev || echo "******* libvorbis-dev install has failed *******"
    ${SUDO} apt install -y libxine2-dev || echo "******* libxine2-dev install has failed *******"

    ${SUDO} apt install -y libfaac-dev || echo "******* libfaac-dev install has failed *******"
    ${SUDO} apt install -y libmp3lame-dev || echo "******* libmp3lame-dev install has failed *******"
    ${SUDO} apt install -y libtheora-dev || echo "******* libtheora-dev install has failed *******"

    ${SUDO} apt install -y libopencore-amrnb-dev || echo "******* libopencore-amrnb-dev install has failed *******"
    ${SUDO} apt install -y libopencore-amrwb-dev || echo "******* libopencore-amrwb-dev install has failed *******"

    ${SUDO} apt install -y libopenblas-dev || echo "******* libopenblas-dev install has failed *******"
    ${SUDO} apt install -y libatlas-base-dev || echo "******* libatlas-base-dev install has failed *******"
    ${SUDO} apt install -y libblas-dev || echo "******* libblas-dev install has failed *******"
    ${SUDO} apt install -y liblapack-dev || echo "******* liblapack-dev install has failed *******"
    ${SUDO} apt install -y libeigen3-dev || echo "******* libeigen3-dev install has failed *******"
    ${SUDO} apt install -y gfortran || echo "******* gfortran install has failed *******"

    ${SUDO} apt install -y libhdf5-dev || echo "******* libhdf5-dev install has failed *******"
    ${SUDO} apt install -y protobuf-compiler || echo "******* protobuf-compiler install has failed *******"

    ${SUDO} apt install -y libprotobuf-dev || echo "******* libprotobuf-dev install has failed *******"
    ${SUDO} apt install -y libgoogle-glog-dev || echo "******* libgoogle-glog-dev install has failed *******"
    ${SUDO} apt install -y libgflags-dev || echo "******* libgflags-dev install has failed *******"

    ${SUDO} apt install -y libudev-dev || echo "******* libudev-dev install has failed *******"
    ${SUDO} apt install -y libusb-1.0-0-dev || echo "******* libusb-1.0-0-dev install has failed *******"

    
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
    ${SUDO} apt upgrade -y

    ${SUDO} apt install -y ros-$ros_version-ros-base
    ${SUDO} apt install -y ros-dev-tools

    ${SUDO} apt install -y python3-colcon-common-extensions

    pip install setuptools==58.2.0

    ${SUDO} apt install -y ros-$ros_version-image-tools
    ${SUDO} apt install -y ros-$ros_version-cv-bridge
    ${SUDO} apt install -y ros-$ros_version-vision-opencv
    ${SUDO} apt install -y ros-$ros_version-rmw-cyclonedds-cpp


}


set_trafalgar_workspace(){

    section
    echo "install trafalgar ros2 workspace"
    section

    cd $HOME

    trafalgar_workspace="/home/$SUDO_USER/trafalgar_ws"

    if [ -d $trafalgar_workspace ] 
    then
        ${SUDO} rm -rf $trafalgar_workspace
        echo "previous directory has been removed"
    fi  

    sudo -u "$SUDO_USER" bash -c '
    mkdir -p "$HOME/trafalgar_ws/src"
    echo "source /opt/ros/$ros_version/setup.bash" >> ~/.bashrc
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
    echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | ${SUDO} tee /etc/sysctl.d/60-cyclonedds.conf
    '


    cd $trafalgar_workspace/src

    git clone https://github.com/man-o-ar/trafalgar_gsc_v0.git

    cd $trafalgar_workspace

    source /opt/ros/${ros_version}/setup.bash
    
    rosdep init
    sudo -u "$SUDO_USER" rosdep update

    rosdep install -i --from-path src --rosdistro ${ros_version} -y
    colcon build --symlink-install


}


restart(){


    section
    echo "ending installation, reboot system after update"
    section

    cd $HOME
    ${SUDO} apt update
    ${SUDO} apt upgrade -y

    while true; do
        read -p "Voulez-vous procéder au redémarrage ? (o/n) " answer
        case "$answer" in
            o)
                ${SUDO} reboot
                break
                ;;
            n)
                break
                ;;
            *)
            echo "Veuillez répondre par 'o' ou 'n'."
            ;;
        esac
    done

}



SUDO=""
if [[ $EUID -ne 0 ]]; then
  SUDO="sudo -E"
fi

ros_version="humble" 

while getopts r flag
    do
        case "${flag}" in
            r) ros_version=${OPTARG};;
        esac
    done

echo "Starting GSC Installation..."
echo "ros_version: $ros_version";


install_build_dependencies
set_python3_as_default
install_pip_dependencies
install_gstreamer

install_ros2
set_trafalgar_workspace
restart

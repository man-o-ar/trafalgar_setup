#!/bin/bash -e


section(){

    echo "==========================="
    echo "==========================="

}


set_python3_as_default(){

    cd $HOME

    section
    echo "Set python 3 config as default"
    section

    ${SUDO} apt install -y python-is-python3

}

install_pip_dependencies(){

    cd $HOME

    section
    echo "install pip and depedencies"
    section

    ${SUDO} apt install -y python3-dev python3-numpy python3-pip 

    sudo -u "$SUDO_USER" pip3 install pyserial opencv-python
    sudo usermod -a -G dialout $USER
    
    if [ $device_type == 'operator' ]
    then
        sudo -u "$SUDO_USER" pip3 install tk customtkinter Pillow pygame 
        ${SUDO} apt install -y python3-tk python3-pil python3-pil.imagetk
    fi
    

}

install_gstreamer(){

    section
    echo "install gstreamer"
    section
    
    cd $HOME

    ${SUDO} apt install -y gstreamer1.0-tools gstreamer1.0-nice gstreamer1.0-alsa gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libsoup2.4-dev libjson-glib-dev
    ${SUDO} apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev

}



install_build_dependencies(){

    #$SUDO apt install -y linux-headers-$(uname -r)

    ${SUDO} apt install -y dkms
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

    ${SUDO} apt install -y expect
 

}


install_ros2(){

    section
    echo "install ros2"
    section

    locale  # check for UTF-8

    ${SUDO} apt update

    if locale -a | grep ^fr_FR 
    then
        echo "fr locales already installed"
    else

        ${SUDO} apt install locales
        ${SUDO} locale-gen fr_FR fr_FR.UTF-8
        ${SUDO} update-locale LC_ALL=fr_FR.UTF-8 LANG=fr_FR.UTF-8
        export LANG=fr_FR.UTF-8
        locale  # verify settings
    fi


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



trafalgar_workspace(){

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

    sudo -u "$SUDO_USER" bash -c "
    mkdir -p \$HOME/trafalgar_ws/src
    echo 'source /opt/ros/$ros_version/setup.bash' >> ~/.bashrc
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
    echo 'net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n' | ${SUDO} tee /etc/sysctl.d/60-cyclonedds.conf
    echo 'export PEER_ID=$device_index' >> ~/.bashrc
    "

    source ~/.bashrc

    cd $trafalgar_workspace/src
        
    if [ $device_type != 'drone' ]
    then
        git clone https://github.com/man-o-ar/trafalgar_operator_v0.git
    else
        git clone https://github.com/man-o-ar/trafalgar_drone_v0.git
    fi

    #launch_index=$trafalgar_workspace/src/trafalgar_${device_type}_v0/launch/device_${device_index}_launch.py
    #launch_source=$trafalgar_workspace/src/trafalgar_${device_type}_v0/launch/device_launch.py 

    #if cmp -s $launch_index $launch_source
    #then
    #    echo "the launch files are identical, pass that step"
    #else
    #    cp $launch_index $launch_source
    #fi

    cd $trafalgar_workspace

    source /opt/ros/${ros_version}/setup.bash
    
    sources_list="/etc/ros/rosdep/sources.list.d/20-default.list"

    if [ -f "$sources_list" ]; then
        echo "rosdep init already ran"
    else
        rosdep init
    fi

    sudo -u "$SUDO_USER" rosdep update

    sudo -u "$SUDO_USER" rosdep install -i --from-path src --rosdistro ${ros_version} -y
    colcon build --symlink-install

    #${SUDO} systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

    #if [[ $device_type != 'drone' ]]
    #then
    #    ${SUDO} apt install -y unclutter || echo "******* unclutter install has failed *******"
    #    unclutter -idle 0  
        #echo 30000 | sudo tee /sys/devices/virtual/thermal/thermal_zone{0,1,2,3}/trip_point_0_temp
    #fi

    sudo chown -R $SUDO_USER $trafalgar_workspace
    sudo chmod 775 -R $trafalgar_workspace
    
    while true; do
        read -p "Voulez-vous procéder à l'installation du service script ? (o/n) " answer
        case "$answer" in
            o)
                trafalgar_service
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



set_n2l_fan_trip_point(){

    TRIP_POINT_0=30000
    TRIP_POINT_1=50000
    TRIP_POINT_2=70000
 
    echo $TRIP_POINT_0 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp
    echo $TRIP_POINT_0 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_0_temp
    echo $TRIP_POINT_0 > /sys/devices/virtual/thermal/thermal_zone2/trip_point_0_temp
    echo $TRIP_POINT_0 > /sys/devices/virtual/thermal/thermal_zone3/trip_point_0_temp
 
    echo $TRIP_POINT_1 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_1_temp
    echo $TRIP_POINT_1 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_1_temp
    echo $TRIP_POINT_1 > /sys/devices/virtual/thermal/thermal_zone2/trip_point_1_temp
    echo $TRIP_POINT_1 > /sys/devices/virtual/thermal/thermal_zone3/trip_point_1_temp
 
    echo $TRIP_POINT_2 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_2_temp
    echo $TRIP_POINT_2 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_2_temp
    echo $TRIP_POINT_2 > /sys/devices/virtual/thermal/thermal_zone2/trip_point_2_temp
    echo $TRIP_POINT_2 > /sys/devices/virtual/thermal/thermal_zone3/trip_point_2_temp

}


trafalgar_service(){

    section
    echo "install trafalgar service script"
    section

    trafalgar_workspace="/home/$SUDO_USER/trafalgar_ws"

    trafalgar_service_dir="$trafalgar_workspace/services"

    if [ -d $trafalgar_service_dir ] 
    then
        ${SUDO} rm -rf $trafalgar_service_dir
        echo "previous directory has been removed"
    fi  

    mkdir -p $trafalgar_service_dir
    
    cd $trafalgar_workspace

    sudo chown -R $SUDO_USER $trafalgar_service_dir
    sudo chmod 775 -R $trafalgar_service_dir

    service_file=$trafalgar_service_dir/trafalgar.service

    # check file existence
    if [[ -f $service_file ]]; then
        rm $service_file
        touch $service_file
    fi

    trafalgar_application="rov_app"


    if [[ $device_type != "drone" ]]
    then
       #user service file
       {
        echo "[Unit]"
        echo "Description=\"trafalgar naviscope app\""
        echo "Wants=system-user-sessions.service sound.target network.target lightdm.service display-manager.service pulseaudio.service"
        echo "After=systemd-user-sessions.service sound.target network.target lightdm.service display-manager.service pulseaudio.service"
        echo ""
        echo "[Service]"
        echo "Type=simple"
        echo "User=$SUDO_USER"
        echo "Environment=\"PEER_ID=$device_index\""
        echo "Environment=DISPLAY=:0.0"
        echo "Environment=\"XAUTHORITY=/home/odroid/.Xauthority\""
        echo "WorkingDirectory=$trafalgar_workspace"
        echo "ExecStart=/bin/bash -c \"source /opt/ros/$ros_version/setup.bash && source install/local_setup.bash && ros2 launch naviscope device_launch.py\""
        echo "Restart=on-failure"
        echo "RestartSec=30s"
        echo ""
        echo "[Install]"
        echo "WantedBy=default.target"
        } > $service_file

        # Écriture du contenu dans le fichier de service
        #echo -e $service_content > $service_file
        ${SUDO} cp $service_file /etc/systemd/user/trafalgar.service

         ${SUDO} -u $SUDO_USER systemctl --user enable trafalgar.service

        #autostart file 
        desktop_file=$trafalgar_service_dir/trafalgar.desktop
        # check file existence
        if [[ -f $desktop_file ]]; then
            rm $desktop_file
            touch $desktop_file
        fi

        {
        echo "[Desktop Entry]"
        echo "Type=Application"
        echo "Exec=systemctl --user start trafalgar.service"
        echo "Hidden=false"
        echo "NoDisplay=false"
        echo "Name[C]=Naviscope"
        echo "Name=Naviscope"
        echo "Comment[C]=naviscope ros node"
        echo "Comment=naviscope ros node"
        echo "X-MATE-Autostart-Delay=0"
        } > $desktop_file

        ${SUDO} cp $desktop_file /.config/autostart/trafalgar.desktop

    else 
       {
        echo "[Unit]"
        echo "Description=\"trafalgar $device_type app\""
        echo "After=network.target"
        echo ""
        echo "[Service]"
        echo "Type=simple"
        echo "User=$SUDO_USER"
        echo "Environment=\"PEER_ID=$device_index\""
        echo "WorkingDirectory=$trafalgar_workspace"
        echo "ExecStart=/bin/bash -c \"source /opt/ros/$ros_version/setup.bash && source install/local_setup.bash && ros2 launch rov_app device_launch.py\""
        echo "Restart=on-failure"
        echo "RestartSec=30s"
        echo ""
        echo "[Install]"
        echo "WantedBy=multi-user.target"
        } > $service_file

        
        # Écriture du contenu dans le fichier de service
        #echo -e $service_content > $service_file
        ${SUDO} cp $service_file /etc/systemd/system/trafalgar.service

        ${SUDO} systemctl enable trafalgar.service
        ${SUDO} systemctl start trafalgar.service

    fi
    


}



restart(){


    while true; do
        read -p "Voulez-vous procéder au redémarrage ? (o/n) " answer
        case "$answer" in
            o)
                section
                echo "ending installation, reboot system after update"
                section

                cd $HOME
                ${SUDO} apt update
                ${SUDO} apt upgrade -y

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



install_config(){

    case $linux_release in
    "20.04")
        ros_version="foxy"
        ;;
    "22.04")
        ros_version="humble"
        ;;
    "23.04")
        ros_version="iron"
        ;;
    *)
        echo "la distribution de linux n'est pas identifiée, le système est mal configuré ou est trop ancien"
        exit 0
        ;;
    esac


    while true; do
        read -p "installer le programme de contrôle du drone (d) ou du naviscope (n) ? " device_answer
        case $device_answer in
        d)
            device_type="drone"
            break
            ;;
        n)
            device_type="operator"
            break
            ;;
        *)
            echo "Veuillez répondre par 'd' (drone) ou 'n' (naviscope)."
            ;;
        esac
    done


    read -p "entrez l'index du $device_type : " user_index

    if ! [[ $user_index =~ ^[0-9]+$ ]]
    then 
    device_index=0
    else
    device_index=$user_index
    fi

    section
    echo "la distribution: $ros_version de ros2  sera installée"
    echo "pour l'appareil $device_type n°$device_index"
 
    section

    while true; do
        read -p "les informations enregistrées sont-elles correctes ? " install_resume
        case $install_resume in
        o)  
            break
            ;;
        n)
            echo "recommençons le paramétrage"
            install_config
            break
            ;;
        *)
            echo "Veuillez répondre par 'o' (oui) ou 'n' (non)."
            ;;
        esac
    done

}



SUDO=""
if [[ $EUID -ne 0 ]]; then
  SUDO="sudo -E"
fi

if [ "$(id -u)" -ne 0 ]; then
	echo "You must run this script with superuser (root) privileges."
	echo "Try: \"sudo ./${SCRIPT_NAME}\""
	exit 1
fi

linux_release=$(lsb_release -rs)
ros_version="humble" 
device_type="drone"
device_index=0


#while getopts i:u:' OPTIONS; do
#    case $OPTIONS in
#        i) installation_full=$OPTARG ;;
#        u) installation_update=$OPTARG ;;
#    esac
#done

install_config

install_build_dependencies
set_python3_as_default
install_pip_dependencies
install_gstreamer

install_ros2
trafalgar_workspace

restart



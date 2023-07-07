SUDO=""
if [[ $EUID -ne 0 ]]; then
  SUDO="sudo -E"
fi

if [ "$(id -u)" -ne 0 ]; then
	echo "You must run this script with superuser (root) privileges."
	echo "Try: \"sudo ./${SCRIPT_NAME}\""
	exit 1
fi


network_ssid=""
network_password=""

prepare_lwan(){

    section 
    echo "connect to "
    section

    #nmcli connection add type wifi ifname wlan0 con-name trafalgar ssid $network_ssid password $network_password

    section
    echo "install ac1300 driver"
    section

    while true; do
        read -p "Voulez-vous procéder à l'installation du driver wifi ? (o/n) " answer
        case "$answer" in
            o)
                echo "La réponse est 'oui'."
                cd $SUDO

                # check to ensure gcc is installed
                if ! command -v gcc >/dev/null 2>&1
                then
	                echo "A required package is not installed."
	                echo "the following package: gcc will be installed"
	                #echo "Once the package is installed, please run \"sudo ./${SCRIPT_NAME}\""
	                ${SUDO} apt install -y gcc
                fi

                if ! command -v bc >/dev/null 2>&1
                then
	                echo "A required package is not installed."
	                echo "the following package: bc will be installed"
	                #echo "Once the package is installed, please run \"sudo ./${SCRIPT_NAME}\""
	                ${SUDO} apt install -y bc
                fi

                if ! command -v iw >/dev/null 2>&1
                then
	                echo "A required package is not installed."
	                echo "the following package: iw will be installed"
	                #echo "Once the package is installed, please run \"sudo ./${SCRIPT_NAME}\""
	                ${SUDO} apt install -y iw
                fi

                if ! command -v rfkill >/dev/null 2>&1
                then
	                echo "A required package is not installed."
	                echo "the following package: rfkill will be installed"
	                #echo "Once the package is installed, please run \"sudo ./${SCRIPT_NAME}\""
	                ${SUDO} apt install -y rfkill
                fi
        
                git clone https://github.com/morrownr/88x2bu-20210702.git    

                cd $HOME/88x2bu-20210702

                ${SUDO} ./install-driver.sh
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

read -p "entrez le nom du réseau WIFI utilisé par ROS : " network_ssid
read -p "entrez le mot de passe pour ce réseau : " network_password

echo "sur le réseau wifi: $network_ssid avec le mot passe : $network_password"
    
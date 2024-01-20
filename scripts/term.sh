install_package()
{
    if ! dpkg -l | grep -qw $1; then
        echo "$1 is not installed. Installing..."
        sudo apt-get update
        sudo apt-get install -y $1
    else
        echo "$1 is already installed; continuing."
    fi
}

address_available()
{
    ping -c 1 $1 > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "IP is not reachable."
        return 1 #false
    else
        echo "IP is reachable."
        return 0 #true
    fi
}

create_terminal() 
{
    local name=$1
    local command=$2
    gnome-terminal --tab --title="$name" -- bash -c "sshpass -p clearpath ssh -X administrator@146.244.98.51; clear; bash"
}
###########################################################################################

main()
{
    IP_ADDRESS="146.244.98.51"
    SSH_HUSKY="sshpass -p clearpath ssh administrator@$IP_ADDRESS; clear;"
    HUSKY_DRIVER="$SSH_HUKSY; roslaunch husky_ur_bringup husky_dual_ur_bringup.launch"
    SSHPASS="sshpass"

    install_package $SSHPASS
    # if ! address_available $IP_ADDRESS; then
    #     echo "Tried to reach husky but it is unavailable, is the robot on?"
    #     return 1
    # fi
    create_terminal "ROS Driver" $HUSKY_DRIVER
    create_terminal "ROS Exploration" $SSH_HUSKY
    create_terminal "Open Terminal" $SSH_HUSKY
    create_terminal "Custom Scripts" $SSH_HUSKY
    return 0
}

main

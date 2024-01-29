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
    gnome-terminal --tab --title="$name" -- bash -c "$command; bash"
}
###########################################################################################

main()
{
    IP_ADDRESS="146.244.98.51"
    SSH_HUSKY="sshpass -p clearpath ssh -X administrator@$IP_ADDRESS"
    HUSKY_DRIVER="$SSH_HUKSY roslaunch husky_ur_bringup husky_dual_ur_bringup.launch"
    MOVE_IT_DRIVER="$SSH_HUKSY roslaunch sds04_husky_moveit_config husky_dual_ur_robotiq_2f_85_moveit_planning_execution.launch;bash"
    SSHPASS="sshpass"

    install_package $SSHPASS
    if ! address_available $IP_ADDRESS; then
        echo "Tried to reach husky but it is unavailable, is the robot on?"
        return 1
    fi
    create_terminal "ROS Exploration" "$SSH_HUSKY"
    create_terminal "Open Terminal" "$SSH_HUSKY"
    create_terminal "Custom Scripts" "$SSH_HUSKY"
    create_terminal "ROS Driver" "$HUSKY_DRIVER"
    return 0
}

main

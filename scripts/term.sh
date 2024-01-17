sudo apt-get install sshpass -y

create_husky_terminal() {
    local name=$1
    gnome-terminal --tab --title="$name" -- bash -c "sshpass -p clearpath ssh administrator@146.244.98.51; bash"
}

create_husky_terminal "ROS Driver"
create_husky_terminal "ROS Exploration"
create_husky_terminal "Open Terminal"
create_husky_terminal "Custom Scripts"

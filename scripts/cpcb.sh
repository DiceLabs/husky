#!/bin/bash

# @Zix
# Copy Codebase script

source term.sh

main()
{
    IP_ADDRESS="146.244.98.51"
    if ! address_available $IP_ADDRESS; then
        echo "Tried to reach husky but it is unavailable, is the robot on?"
        return 1
    fi

    SSH_HUSKY="sshpass -p clearpath ssh -X administrator@$IP_ADDRESS"
    REPLACE_TEMP="$SSH_HUKSY; sudo rm -r $HOME/catkin_ws/temp"
    create_terminal "Replace Temp Dir" "$SSH_HUSKY"
    scp -r $PWD administrator@146.244.98.51:/home/administrator/catkin_ws/temp
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
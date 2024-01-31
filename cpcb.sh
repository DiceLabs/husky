#!/bin/bash

# @Zix
# Copy Codebase script

source term.sh

main()
{
    if ! address_available $IP_ADDRESS; then
        echo "Tried to reach husky but it is unavailable, is the robot on?"
        return 1
    fi

    REPLACE_TEMP="$SSH_HUKSY sudo rm -r $HOME/catkin_ws/temp"
    create_terminal "Replace Temp Dir" "$REPLACE_TEMP"
    scp -r . administrator@146.244.98.51:/home/administrator/catkin_ws/temp
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
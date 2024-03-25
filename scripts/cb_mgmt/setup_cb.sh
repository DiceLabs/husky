#!/bin/bash

setup_environment()
{
    cd $HOME/husky
    catkin_make
    source source.sh
    bash  install.sh
    bash  build_urclient.sh
}

setup_cb()
{
    setup_environment
    # start_arm_servers
}

main()
{
    if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
        setup_cb "$@"
    fi
}

main
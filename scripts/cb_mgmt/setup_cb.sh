#!/bin/bash

setup_environment()
{
    cd $HOME
    git clone https://github.com/DiceLabs/Husky husky
    cd husky
    catkin_make
    source source.sh
    bash  clean_lib.sh
    bash  install.sh
    bash  see_lib.sh
    bash  build_ur.sh
}

setup_cb()
{
    setup_environment
}

main()
{
    if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
        setup_cb "$@"
    fi
}

main
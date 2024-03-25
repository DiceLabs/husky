#! /bin/bash

move_to_cmake_dir()
{
    cd $HOME
    cd husky
    cd src/components/urcap/ur_client
}

remove_redundant_build()
{
    DIRECTORY="build"
    if [ -d "$DIRECTORY" ]; then
        rm -rf "$DIRECTORY"
        echo "$DIRECTORY dir removed"
    fi
}

invoke_cmake()
{
    mkdir build && cd build && cmake .. && make
    sudo make install
}

compile_urclient()
{
    move_to_cmake_dir
    remove_redundant_build
    invoke_cmake
}

main()
{
    if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
        compile_urclient "$@"
    fi
}

main
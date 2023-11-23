#! /usr/bin/bash

function main() {
        kill $(pidof gzclient)
        kill $(pidof gzserver)
        pkill -f roslaunch
        exit 1

}

main;

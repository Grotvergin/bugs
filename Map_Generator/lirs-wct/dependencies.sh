#!/bin/bash

# Install Blender
yes Y | sudo apt-get install blender

# Install dependencies
yes Y | sudo apt-get install libeigen3-dev libassimp-dev cmake libmagick++-dev libboost-all-dev git libgl-dev libfontconfig1 libfreetype6 libxcb1 libxkbcommon* libxkbcommon-x11-*

# Check Linux Version & OS name
if [ -f /etc/os-release ]; then
    # freedesktop.org and systemd
    . /etc/os-release
    OS=$NAME
    VER=$VERSION_ID
elif type lsb_release >/dev/null 2>&1; then
    # linuxbase.org
    OS=$(lsb_release -si)
    VER=$(lsb_release -sr)
elif [ -f /etc/lsb-release ]; then
    # For some versions of Debian/Ubuntu without lsb_release command
    . /etc/lsb-release
    OS=$DISTRIB_ID
    VER=$DISTRIB_RELEASE
fi

# Check QT library version
echo "Check QT library"
QT_FOLDER=/opt/qt514
if [ "$VER" = "18.04" ] && [ "$OS" = "Ubuntu" ]; then
    echo "Ubuntu 18.04 FOUND!"
    if [ -d "$QT_FOLDER" ]; then
        echo "QT 5.14 Bionic: FOUND"
    else
        yes Y | sudo add-apt-repository ppa:beineri/opt-qt-5.14.2-bionic
        sudo apt-get update
        yes Y | sudo apt-get -y install qt514-meta-minimal
    fi   
elif [ "$VER" = "20.04" ] && [ "$OS" = "Ubuntu" ]; then
    echo "Ubuntu 20.04 FOUND!"
    if [ -d "$QT_FOLDER" ]; then
        echo "QT 5.14 Focal: FOUND"        
    else
        yes Y | sudo add-apt-repository ppa:beineri/opt-qt-5.14.2-focal
        sudo apt-get update
        yes Y | sudo apt-get -y install qt514-meta-minimal
    fi                
elif [ "$VER" = "16.04" ] && [ "$OS" = "Ubuntu" ]; then
    echo "Ubuntu 16.04 FOUND!"
    if [ -d "$QT_FOLDER" ]; then
        echo "QT 5.14 Xenial: FOUND"
    else
        yes Y | sudo add-apt-repository ppa:beineri/opt-qt-5.14.2-xenial
        sudo apt-get update
        yes Y | sudo apt-get -y install qt514-meta-minimal
    fi        
else
    echo "Unknown Linux version"
    exit 1
fi

# Add QT ENV variables
USER_FOLDER=$(eval echo ~$USER)
if [ -f "${USER_FOLDER}/.config/qtchooser/default.conf" ]; then
    echo "default.conf exists"
    # Check QT ENV variables
    if grep -Fxq "${QT_FOLDER}/lib" ~/.config/qtchooser/default.conf
    then
        echo "QT ENV variables already added!"
    else
        echo "Add QT ENV variables"
        printf "${QT_FOLDER}/bin\n${QT_FOLDER}/lib" >> ~/.config/qtchooser/default.conf
    fi
else
    mkdir -p ~/.config/qtchooser    
    touch ~/.config/qtchooser/default.conf
    printf "${QT_FOLDER}/bin\n${QT_FOLDER}/lib" >> ~/.config/qtchooser/default.conf
fi
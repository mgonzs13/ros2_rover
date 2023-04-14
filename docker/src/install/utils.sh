#!/bin/bash

wget -q -O - https://dl.google.com/linux/linux_signing_key.pub | sudo apt-key add -
apt update
apt install curl wget gnupg2 lsb-release git nano apt-utils bash-completion -y
apt install python3-pip python3-setuptools python3-wheel -y
echo "source /usr/share/bash-completion/bash_completion" >>/root/.bashrc 

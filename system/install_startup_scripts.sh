#!/bin/bash

show_usage() {
   echo -n "Installs and enables systemd services and startup scripts to autolaunch ROS wheele at startup

usage: sh install_startup_script.sh --user <username> --distro <ROS distribution>

   Options:
      -u,--user     User name
      -d,--distro   ROS distribution
"
}

prompt_confirm() {
  PROMPT=$1
  while true; do
    read -r -p "${1:-Continue?} [y/n]: " REPLY
    case $REPLY in
      [yY]) echo ; return 0 ;;
      [nN]) echo ; return 1 ;;
      *)
    esac
  done
}

if [ $# -ne 4 ]
then
   show_usage
   exit 0
fi

# Validate/process input args
while [ ! -z "$1" ];do
   case "$1" in
      -h|--help)
         show_usage
         ;;
      -u|--user)
         shift
         USER="$1"
         ;;
      -d|--distro)
         shift
         DISTRO="$1"
         ;;
      *)
         show_usage
         exit 0
   esac
   shift
done

prompt_confirm "Install for user: $USER, ROS distro: $DISTRO" || exit 0

files=("roscore.service" "roslaunch_wheele" "wheele.service")
for file in "${files[@]}"; do
  echo "Installing $file..."
  cp $file /tmp/$file
  sed -i -e "s/\$USER/$USER/g" /tmp/$file
  sed -i -e "s/\$DISTRO/$DISTRO/g" /tmp/$file
done

sudo install -D /tmp/roscore.service /etc/systemd/system
sudo install -D /tmp/wheele.service /etc/systemd/system
sudo install -D --mode=755 /tmp/roslaunch_wheele /usr/local/sbin

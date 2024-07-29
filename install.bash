#!/bin/bash

# create folder structure
echo " -----    Creating folder structure                               -----"
mkdir -p cache/build
mkdir -p cache/install
mkdir -p cache/log

# set uid and gid in env file
UID_docker=$(id -u)
GID_docker=$(id -g)
ENV_FILE=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.devcontainer/.env
if [ -f $ENV_FILE ]; then
    rm $ENV_FILE
fi
echo "uid_docker=${UID_docker}" >> $ENV_FILE
echo "gid_docker=${GID_docker}" >> $ENV_FILE

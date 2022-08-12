#! /usr/bin/env bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PKG_ROOT=$(dirname $DIR)
export GAZEBO_MODEL_PATH=${GAZEBO_PLUGIN_PATH}:$PKG_ROOT/models
VERSION=$(lsb_release -rs)
if [ "$VERSION" == "18.04" ]; then
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PKG_ROOT/plugin/gazebo9
else
  export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PKG_ROOT/plugin/gazebo7
fi

#!/usr/bin/env bash
# shellcheck disable=SC2046
# shellcheck disable=SC2006

set -ex

sudo apt-get install --no-install-recommends -qqy lsb-release gnupg2 curl
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
     | sudo tee /etc/apt/keyrings/robotpkg.asc
 echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
     | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt-get update
sudo apt-get install --no-install-recommends -qqy robotpkg-py310-pinocchio=3.3.0 robotpkg-pinocchio=3.3.0 robotpkg-eiquadprog=1.2.9 robotpkg-py310-eigenpy=3.10.1 robotpkg-py310-coal=3.0.0
echo "/opt/openrobots/lib" | sudo tee /etc/ld.so.conf.d/openrobots.conf
# shellcheck disable=SC2016
openrobots_export_cmd='export PATH="/opt/openrobots/bin:$PATH"
export PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="/opt/openrobots/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH"
export CMAKE_SYSTEM_PREFIX_PATH="/opt/openrobots/lib/cmake:$CMAKE_SYSTEM_PREFIX_PATH"
'
echo $openrobots_export_cmd | sudo tee -a /etc/profile
echo $openrobots_export_cmd | sudo tee -a /etc/profile.d/02-environment.sh
# we want to use single quotes to prevent expansion, therefore
# shellcheck disable=SC2016
echo $openrobots_export_cmd | sudo tee -a /etc/bash.bashrc

set +ex

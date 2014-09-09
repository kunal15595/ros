#!/bin/sh

set -e

# create a location to store downloaded data
mkdir -p /home/kunal15595/ros/mp/omplapp/build/Release/pyplusplus
cd /home/kunal15595/ros/mp/omplapp/build/Release/pyplusplus

# get sources
# gccxml snapshot of 7/21/2014
/usr/bin/wget --no-check-certificate -O - https://github.com/gccxml/gccxml/archive/ab651a2aa866351bdd089a4bf1d57f6a9bec2a66.tar.gz | tar xzf -
/usr/bin/wget --no-check-certificate -O - https://github.com/gccxml/pygccxml/archive/v1.6.1.tar.gz | tar xzf -
/usr/bin/wget --no-check-certificate -O - https://bitbucket.org/ompl/pyplusplus/downloads/pyplusplus-r1247.tgz | tar xzf -

# build & install gccxml
cd gccxml-ab651a2aa866351bdd089a4bf1d57f6a9bec2a66
/usr/bin/cmake  .
/usr/bin/cmake --build .
sudo /usr/bin/cmake --build . --target install

# build & install pygccxml and Py++
cd ../pygccxml-1.6.1
/usr/bin/python setup.py build
sudo /usr/bin/python setup.py install 
cd ../pyplusplus
/usr/bin/python setup.py build
sudo /usr/bin/python setup.py install 

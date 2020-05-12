#!/bin/bash

dirs=`find . -maxdepth 1 -type d -not -name '.*'`

for d in $dirs
do
    echo ===================== Processing $d Started
    mkdir -p $d/build
    pushd $d/build > /dev/null
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j
    sudo make install
    echo ===================== Processing $d Completed
    popd > /dev/null
done
    

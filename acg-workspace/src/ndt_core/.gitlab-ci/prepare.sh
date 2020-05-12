#!/bin/bash

add_aass_yaml() {
#    echo "Here you can add dependencies";
    echo "yaml https://gitsvn-nt.oru.se/software/aass_rosdep/raw/master/kinetic.yml" | tee /etc/ros/rosdep/sources.list.d/50-aass.list
    rosdep update > /dev/null
}

setup_apt() {
    #    echo "Here you can define repo with our package";
    wget -O - http://apt.aass.oru.se/repos/apt/conf/public.gpg.key | apt-key add -
    echo "deb http://apt.aass.oru.se/repos/apt/ubuntu xenial main" >> /etc/apt/sources.list
    
    apt-get update -qq #> /dev/null
}

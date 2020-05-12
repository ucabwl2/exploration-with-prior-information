#!/bin/bash

release_dependency_packages() {
    for i in ndt_fuser ndt_feature_finder ndt_localization ndt_offline; do
	create_debian_package "$i"
	dpkg -i ros-*_amd64.deb > /dev/null
    done
    move_debian_packages
}

release_meta_package() {
    
    create_debian_package "ndt_tools"
    dpkg -i ros-*_amd64.deb > /dev/null
    move_debian_packages
}

create_debian_package() {
    cd $1
    if [ -f .done ]; then
	echo "Package $1 was already build. Skipping..."
    else
	echo "generating debian package for $1"
	bloom-generate rosdebian --os-name ubuntu --os-version $UBUNTU_DISTRO --ros-distro $CI_ROS_DISTRO &&\
	    sed -i 's/dh  $@/dh  $@ --parallel/' debian/rules
	debuild -rfakeroot -us -uc -b -j8 > /dev/null
	touch .done
    fi
    cd ..
}

move_debian_packages() {
    mv *deb .build/
}

release_package() {
    release_dependency_packages
    release_meta_package
}

show_info() {
    echo $UBUNTU_DISTRO
    echo $CI_ROS_DISTRO
}



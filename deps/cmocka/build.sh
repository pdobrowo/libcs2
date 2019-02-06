#!/bin/sh
set -e

PKGNAME=cmocka-1.1.3.tar.xz

if [ -d include ]; then
    echo "cmocka: already built"
else (
    echo "cmocka: download $PKGNAME"
    rm -rf $PKGNAME
    curl -L http://www.cmocka.org/files/1.1/$PKGNAME > $PKGNAME

    echo "cmocka: build"
    rm -rf include lib .build
    xz -d -c $PKGNAME | tar xvf -
    mv `basename $PKGNAME .tar.xz` .build
    (
        cd .build;
        mkdir .cmake
        (
            cd .cmake
            cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=`pwd`/../install -DWITH_STATIC_LIB=ON
            make -j
            make install
        )
    )
    mv .build/install/include .
    mkdir lib
    if [ -f .build/install/lib64/libcmocka-static.a ]; then
        cp .build/install/lib64/libcmocka-static.a lib/libcmocka.a
    elif [ -f .build/install/lib/libcmocka-static.a ]; then
        cp .build/out/install/libcmocka-static.a lib/libcmocka.a
    else
        echo "cmocka: static library not found!"
        exit 1
    fi
    rm -rf $PKGNAME
    echo "cmocka: done"
)
fi


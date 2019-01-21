#!/bin/sh
set -e

PKGNAME=cmocka-1.1.3.tar.xz

if [ -d include ]; then
    echo "cmocka: already built"
else
(
    echo "wget cmocka $PKGNAME"
    wget http://cmocka.org/files/1.1/$PKGNAME

    echo "cmocka: build"
    rm -rf include lib build
    tar xvf $PKGNAME
    mv `basename $PKGNAME .tar.xz` build
    (
        cd build;
        mkdir tmp-build
        (
            cd tmp-build
            cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=`pwd`/../out -DWITH_STATIC_LIB=ON
            make
            make install
        )
    )
    mv build/out/include .
    mkdir lib
    if [ -f build/out/lib64/libcmocka-static.a ]; then
        cp build/out/lib64/libcmocka-static.a lib/libcmocka.a
    elif [ -f build/out/lib/libcmocka-static.a ]; then
        cp build/out/lib/libcmocka-static.a lib/libcmocka.a
    else
        echo "cmocka: static library not found!"
        exit 1
    fi
    echo "cmocka: done"
)
fi


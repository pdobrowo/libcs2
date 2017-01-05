#!/bin/sh
set -e

if [ -d include ]; then
    echo "cmocka: already built"
else
(
    echo "cmocka: build"
    rm -rf include lib build
    tar xf cmocka-1.1.0.tar.xz
    mv cmocka-1.1.0 build
    (
        cd build;
        mkdir tmp-build
        (
            cd tmp-build
            cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=`pwd`/../out
            make
            make install
        )
    )
    mv build/out/include .
    mkdir lib
    cp build/out/lib/libcmocka*.so* lib
    echo "cmocka: done"
)
fi


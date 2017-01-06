#!/bin/sh
set -e

if [ -d include ]; then
    echo "cmocka: already built"
else
(
    echo "cmocka: build"
    rm -rf include lib build
    tar zxvf cmocka-1.1.0.tar.gz
    mv cmocka-1.1.0 build
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
    cp build/out/lib/libcmocka.a lib
    echo "cmocka: done"
)
fi


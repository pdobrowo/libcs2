#!/bin/sh
set -e

if [ -d include ]; then
    echo "qhull: already built"
else (
    echo "qhull: build"
    rm -rf include lib .build
    git clone http://github.com/qhull/qhull .build
    (
        cd .build;
        git checkout tags/v8.0.2
        cmake . -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=`pwd`/install -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true
        make -j 2
        make install
    )
    mv .build/install/include .
    mkdir lib
    cp .build/lib*.a lib
    echo "qhull: done"
)
fi


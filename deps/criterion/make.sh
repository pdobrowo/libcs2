#!/bin/sh
set -e

if [ -d include ]; then
    echo "criterion: already built"
else
(
    echo "criterion: build"
    rm -rf include lib build
    git clone http://github.com/Snaipe/Criterion build
    (
        cd build;
        git checkout -b v2.3.0
        cmake . -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=`pwd`/out
        make
        make install
    )
    mv build/out/include .
    mv build/out/lib .
    echo "criterion: done"
)
fi


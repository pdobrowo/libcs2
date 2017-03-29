#!/bin/sh
set -e

GENERATOR=""
if [[ "$OSTYPE" == "msys" ]]; then
    GENERATOR=(-G "MSYS Makefiles")
fi

if [ -d include ]; then
    echo "qhull: already built"
else (
    echo "qhull: build"
    rm -rf include lib build
    git clone http://github.com/qhull/qhull build
    (
        cd build;
        cmake . "${GENERATOR[@]}" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=`pwd`/out -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=true
        make
        make install
    )
    mv build/out/include .
    mkdir lib
    cp build/lib*.a lib
    echo "qhull: done"
)
fi


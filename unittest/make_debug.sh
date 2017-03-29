#!/bin/sh

GENERATOR=""
if [[ "$OSTYPE" == "msys" ]]; then
    GENERATOR=(-G "MSYS Makefiles")
fi

cmake . "${GENERATOR[@]}" -DCMAKE_BUILD_TYPE=Debug && make

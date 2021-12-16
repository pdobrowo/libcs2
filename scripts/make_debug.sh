#!/bin/sh
mkdir -p .build
(
    cd .build
    cmake -DCMAKE_BUILD_TYPE=Debug .. -B=. && make -j 2
)

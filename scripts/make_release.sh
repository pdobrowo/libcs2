#!/bin/sh
mkdir -p .build
(
    cd .build
    cmake -DCMAKE_BUILD_TYPE=Release .. -B=. && make -j 2
)

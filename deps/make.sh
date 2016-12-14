#!/bin/sh
set -e

# criterion
(
    rm -rf criterion
    git clone http://github.com/Snaipe/Criterion criterion
    cd criterion;
    git checkout -b v2.3.0
    cmake .
    make
)


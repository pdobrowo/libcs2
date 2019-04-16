#!/bin/sh
mkdir -p .build
(
    cd .build
    cmake -DCMAKE_BUILD_TYPE=Debug .. && make -j && make test_coverage
)

GREEN='\033[0;32m'
NOCOLOR='\033[0m'

rm -rf test_coverage
cp -r .build/test_coverage test_coverage

echo -e "${GREEN}[Coverage]${NOCOLOR} Report: file://`pwd`/test_coverage/index.html"

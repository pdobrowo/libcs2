#!/bin/sh
(
    cd deps;
    (
        cd criterion;
        cmake . && make;
     )
)


#!/bin/sh
( cd decomp && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo . && make )

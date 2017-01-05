#!/bin/sh
set -e

( cd qhull && ./make.sh )
( cd cmocka && ./make.sh )

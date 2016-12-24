#!/bin/sh
set -e

# libs
( cd qhull && ./make.sh )

# test
( cd criterion && ./make.sh )


#!/bin/bash

set -e
set -o pipefail

prefix="./"
repo_dir=$PWD

for dir in $(find ./ -maxdepth 2 -type d -name 'art_*')
do
    dir=${dir#$prefix}
    if [ -f "$dir/package.xml" ]; then
        echo "Running test for package:" $dir
        cd "$dir"
        catkin run_tests --no-deps --this
        catkin_test_results ~/catkin_ws/build/${PWD##*/}
        cd $repo_dir
    fi
done
#!/usr/bin/env bash

if [ ! -d sequences ]
then
    echo "Usage: rosrun clams visualize_trajectories"
    echo "This script must be run from within a CLAMS workspace."
    exit
fi

for seq in sequences/*
do
    if [ -e slam_results/$(basename $seq)/trajectory ]
    then
        echo
        echo ==========
        echo == Displaying  $seq
        echo ==========
        echo Press ESC to advance to next map.

        rosrun clams visualize_trajectory $seq slam_results/$(basename $seq)/trajectory slam_results/$(basename $seq)/visualization_map.pcd
    fi
done


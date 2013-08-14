#!/usr/bin/env bash

if [ ! -d sequences ]
then
    echo "Usage: rosrun clams slam"
    echo "This script must be run from within a CLAMS workspace."
    exit
fi

for seq in sequences/*
do
    if [ -e slam_results/$(basename $seq)/trajectory ]
    then
        echo Trajectory for $(basename $seq) already exists.  Skipping.
        continue
    fi

    echo Running SLAM on $seq

    # -- Convert sequence to Freiburg format.
    fdir=.freiburg
    mkdir -p $fdir
    rm -rf $fdir/*
    fseq=$fdir/$(basename $seq)
    rosrun stream_sequence sseq_to_freiburg $seq $fseq
    
    # -- Run dvo_public.
    cd $fseq
    roslaunch dvo_benchmark benchmark.launch dataset:=`pwd` keep_alive:=false
    cd -
    
    # -- Convert their trajectory to CLAMS trajectory.
    mkdir -p slam_results/$(basename $seq)
    rosrun clams convert_trajectory $seq $fseq/assoc_opt_traj_final.txt slam_results/$(basename $seq)/trajectory

    # -- Cache visualization and calibration maps.
    rosrun clams generate_map --resolution 0.01 $seq slam_results/$(basename $seq)/trajectory slam_results/$(basename $seq)/calibration_map.pcd
    rosrun clams generate_map --resolution 0.03 $seq slam_results/$(basename $seq)/trajectory slam_results/$(basename $seq)/visualization_map.pcd

    # -- Clean up Freiburg format sequence.
    rm -rf $fseq
done

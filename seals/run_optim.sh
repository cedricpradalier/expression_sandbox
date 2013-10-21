#!/bin/sh

../bin/optimise_rotations \
    -num_threads 4 \
    -use_local_parameterization \
    -input preload_mat_all.txt \
    -gps gps_mat_all.txt \
    -robustify -interactive \
    -display -num_lines 500000 


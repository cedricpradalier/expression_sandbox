#!/bin/sh

../bin/optimise_rotations_tex_m \
    -num_threads 1 \
    -use_local_parameterization \
    -input preload_mat_all.txt \
    -gps gps_mat_all.txt \
    -robustify \
    -display \
    -num_lines 5000 


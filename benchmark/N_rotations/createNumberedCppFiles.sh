#!/bin/bash

mkdir -p N

for C in *.cpp; do for i in $(seq 1 20) ; do echo -e "#define ROTATIONS_N $i\n#include \"../$C\"\n" > N/$C.$i.cpp; done; done

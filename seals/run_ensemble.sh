#!/bin/sh

for I in $(seq 1 50); do
	for L in $(seq 1 15); do
		LINES=$(($L * 10000))
		DIR=stats/$LINES
		mkdir -p $DIR;
		for VAR in tex ceres tex_m ceres_m; do
			OUT="$DIR/${VAR}_${I}.out"
			if [ -e $OUT ] && grep -q 'Termination:' $OUT ; then 
				echo "skipping: $OUT"
				continue; 
			fi
			if [ $(find "$DIR" -name \*.FAILED | wc -l) -ne 0 ] ; then 
				echo "skipping FAILED $OUT"
				continue;
			fi
			echo "running: $OUT"
			taskset 01 nice -n -19 ../bin/optimise_rotations_$VAR \
			    -num_threads 1 \
			    -use_local_parameterization \
			    -input preload_mat_all.txt \
			    -gps gps_mat_all.txt \
			    -robustify \
			    -display \
			    -num_lines $LINES > $OUT
			if ! grep -q FUNCTION_TOLERANCE $OUT; then 
				echo "FAILED";
				touch $OUT.FAILED
			else
				echo "OK";
			fi
		done
	done
done

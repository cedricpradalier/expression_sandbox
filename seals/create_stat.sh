#!/bin/bash

for FILE in $(find stats -name \*.out); do 
	if [ -e $FILE.FAILED ] ; then
		echo "skipping FAILED $FILE" >&2
		continue;
	fi

	read DIR LINES VAR SEQ SUFFIX <<< $(sed -e 's/_\([0-9]\+\)/ \1/' <<< $FILE | tr "/." "   " );
	read DUMMY DUMMY RES <<< $(grep "Residual evaluation" $FILE)
	read DUMMY DUMMY JAC <<< $(grep "Jacobian evaluation" $FILE)
	read DUMMY RESP DUMMY <<< $(grep "residualsDuration" $FILE)
	read DUMMY JACP DUMMY <<< $(grep "jacobiansDuration" $FILE)
	echo $VAR,$LINES,$(printf %02d $SEQ),$RES,$JAC,$RESP,$JACP

done | sort > stat.csv

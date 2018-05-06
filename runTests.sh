#!/bin/bash

echo $0
scriptDir="${0%/*}"
#run-parts "$scriptDir"/build/ - this works too, but it does not show proper colors
list="$(find "$scriptDir"/build/ -maxdepth 1 -executable -type f)"
for file in $list; do
	echo $file
	$file
	#eval $file > /dev/null
	#if [ $? != 0  ]; then
		#$file
	#fi
done

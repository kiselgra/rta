#!/bin/bash

P="$HOME/render-data/models"
declare -a MODELS=(drache.obj sponza.noflag.obj sponza.subdiv.noflag.obj conference.obj)
declare -a EYEDIST=(1.5       0.0        0.0                         0.0)
BVHC="lbvh"
TRAV="dis cis"
LAYOUT="std 2f4"
RAYT="x -s"
AILABOX="x -A"

echo "|---------"
echo -n "| bvh construction | traversal style | node layout | ailabox | ray type |"
for m in ${MODELS[@]} ; do
	echo -n " $m |"
done
echo
echo "|---------"

IMAGES="--no-shade"
#IMAGES=""

for construction in $BVHC ; do
	for trav in $TRAV ; do
		for layout in $LAYOUT ; do
			for boxmode in "" "-A" ; do
				for raytype in "" "-s" ; do
					echo -n "| $construction | $trav | $layout | $boxmode | $raytype | "
					for ((i=0; i < ${#MODELS[@]}; i++)) ; do
						m=${MODELS[$i]}
						e=${EYEDIST[$i]}
						echo ./rta/rta -r 1920x1080 $IMAGES -m "$P/$m" --dist $e --sphere sphs/sphere0.ply bbvh-cuda -- -b lbvh -t "$trav" -l "$layout" $raytype $boxmode > bbvh-cuda-bench.out
						./rta/rta -r 1920x1080 $IMAGES -m "$P/$m" --dist $e --sphere sphs/sphere0.ply bbvh-cuda -- -b lbvh -t "$trav" -l "$layout" $raytype $boxmode 2>> bbvh-cuda-bench.error >> bbvh-cuda-bench.out
						if [ "$?" != "0" ] ; then
							echo "ERROR"
							exit 1
						fi
						echo -n "$(grep "average rps per frame" bbvh-cuda-bench.out | sed -e 's/.*frame: //' -e 's/ K//' -e 's/\..*//') |"
#echo -n "$m |"
					done
					echo 
				done
			done
		done
	done
done
echo "|---------"

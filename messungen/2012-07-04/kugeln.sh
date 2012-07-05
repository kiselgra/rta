sphs="../../sphs/sphs"
dis="2012-07-04--A--bbvh-dis.ply"  
cis="2012-07-04--B--bbvh-cis.ply"
sbvh_dis="2012-07-04--C--sbvh-bbvh-dis.ply"  
sbvh_cis="2012-07-04--D--sbvh-bbvh-cis.ply"
sbvh_po="2012-07-04--E--sbvh-po-sbvh.ply"


echo ">"
echo "> absolute versions of the bbvh traversals"
echo ">"

$sphs $dis -m absolute -o dis-absolute.ply >/dev/null
$sphs $cis -m absolute -o cis-absolute.ply >/dev/null


echo
echo ">"
echo "> comparison dis relative to cis, always try to get the slower one relative to the faster one."
echo ">"

$sphs $cis $dis -m perf -o dis-rel-to-cis.ply >/dev/null


echo
echo ">"
echo "> comparison bbvh vs sbvh-bbvh-mode"
echo ">"

$sphs $cis $sbvh_cis -m perf -o sbvh-cis-rel-to-cis.ply >/dev/null
$sphs $dis $sbvh_dis -m perf -o sbvh-dis-rel-to-dis.ply >/dev/null


echo
echo ">"
echo "> comparison bbvh vs sbvh-bbvh-mode"
echo ">"

$sphs $cis $sbvh_po -m perf -o sbvh-po-rel-to-cis.ply >/dev/null

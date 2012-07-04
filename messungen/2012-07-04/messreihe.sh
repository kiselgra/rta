p="--sphere-file sphs/sphere2.ply -b 1,1,1 -l 1,0,0"

## bbvh
./rta/rta $p -o 2012-07-04--A--bbvh-dis.ply bbvh -- --bvh-trav dis
./rta/rta $p -o 2012-07-04--B--bbvh-cis.ply bbvh -- --bvh-trav cis
## sbvh as bbvh
./rta/rta $p -o 2012-07-04--C--sbvh-bbvh-dis sbvh -- --bvh-trav bbvh-dis
./rta/rta $p -o 2012-07-04--D--sbvh-bbvh-cis sbvh -- --bvh-trav bbvh-cis
./rta/rta $p -o 2012-07-04--E--sbvh-po-sbvh sbvh -- --bvh-trav po-sbvh

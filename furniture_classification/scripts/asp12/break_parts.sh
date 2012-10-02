prev=""
category=0
for i in `ls *_segmentation.pcd`
do
  echo "file: $i"
  curr=`echo $i | awk -F "_" '{print $1}' | awk -F "s" '{print $1}'`
  if [ "$prev" != "$curr" ]; then
    prev="$curr"
    ((category++))
  fi
  nr=`echo $i | awk -F "_" '{print $1}' | awk -F "s" '{print $2}'`
  id=`echo "scale=0; $nr/31" | bc -l`
  scan=`echo "$nr%31" | bc` # NOTE: modulo does not work with -l, maybe % means something else with -l?
  echo "category: $category, id: $id, scan: $scan"
  mkdir -p "$category${id}_${curr}_id$id/scan$scan"
  
  regions=`sed '1,11d' $i | awk -F " " '{print $4}' | sort | uniq`
  echo "nr of $curr ($id) clusters: "`echo "$regions" | wc -w` # TODO: check if poisson distributed
  cluster=0
  for rgb in $regions
  do
    #echo "cluster $cluster: $rgb"
    file="$category${id}_${curr}_id$id/scan$scan/cluster$cluster.pcd"
    points=`grep $rgb $i | wc -l`
    head -n 11 $i | sed "10 c POINTS $points" | sed "7 c WIDTH $points" > $file
    grep $rgb $i >> $file
    ((cluster++))
  done
done

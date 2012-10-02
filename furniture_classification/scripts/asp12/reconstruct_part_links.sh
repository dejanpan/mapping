#!/bin/bash

for cad in `ls -d1 */` # (-d1 is the same as -d: descend only 1 level) or: tree -d -L 1 or: find . -type d -maxdepth 1 -mindepth 1 or: ls -l | grep "^d"
do
  cd $cad
  cad=`echo "${cad%?}"`
  nrscans=`ls -d1 scan*/ | wc -w`
  for scan1 in `ls -d1 scan*/`
  do
    scan1=`echo "${scan1%?}"`
    s1counter=`echo $scan1 | sed s/scan//g`
    next=$((s1counter+1))
    let "next %= $nrscans"
    nr1=`ls -1 $scan1/cluster*.pcd | wc -l`
    nr1m1=$((nr1-1))
    for scan2 in `ls -d1 scan*/`
    do
      scan2=`echo "${scan2%?}"`
      s2counter=`echo $scan2 | sed s/scan//g`
      #if [[ "$scan1" < "$scan2" ]]
      #if [ "$s1counter" -lt "$s2counter" ]
      if [ "$next" -eq "$s2counter" ]
      then
#        echo "$cad: $scan1 vs $scan2"
        nr2=`ls -1 $scan2/cluster*.pcd | wc -l`
        nr2m1=$((nr2-1))
#        echo "$nr1m1 $nr2m1"
        for i in `seq 0 $nr1m1`
        do
          nrp1=`grep POINTS $scan1/cluster$i.pcd | awk -F " " '{print $2}'`
          for j in `seq 0 $nr2m1`
          do
#            echo "intersecting clusters: $i vs $j"
            nrp2=`grep POINTS $scan2/cluster$j.pcd | awk -F " " '{print $2}'`
            /home/marton/work/_Clouds/src/CloudEdit/build/CloudIntersect -nr_n 300 -max_dist 0.01 -precision 8 $scan1/cluster$i.pcd $scan2/cluster$j.pcd tmp1.pcd tmp2.pcd 2> tmp.txt
            tmp=`grep -v '#' tmp1.pcd`
            if [ "$tmp" != "" ]
            then
              tmp=`grep -v '#' tmp2.pcd`
              if [ "$tmp" != "" ]
              then
                nrp1match=`grep POINTS tmp1.pcd | awk -F " " '{print $2}'`
                nrp2match=`grep POINTS tmp2.pcd | awk -F " " '{print $2}'`
#                echo "matches: $nrp1match vs $nrp2match"
                overlap1=`echo "scale=0; 100*$nrp1match/$nrp1" | bc`
                overlap2=`echo "scale=0; 100*$nrp2match/$nrp2" | bc`
#                echo "overlaps: $overlap1 vs $overlap2"
                if [ "$overlap1" -ge "$1" ]
                then
                  if [ "$overlap2" -ge "$1" ]
                  then
                    #echo "$i $j"
                    echo "${cad}_${scan1}_cluster$i.pcd ${cad}_${scan2}_cluster$j.pcd"
                  fi
                fi
              fi
            fi
          done
        done
        break
      fi
    done
  done
  cd ..
done

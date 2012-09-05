#!/bin/bash

# Arguments, within funtions, are treated in the same manner as arguments given to the script
function reconstruct()
{
  cad=$1
  scan=$2
  p=$3
  list=$4
  list=`echo "$list" | tr "," " "`
  prefix=$5
  
            echo "$cad$scan: "`echo $list`
            echo -n "" > ../tmp.pcd
            #head -n 11 cluster0.pcd | sed "10 c POINTS 0" | sed "7 c WIDTH 0" > output.pcd
            for c in `echo $list`
            do
              sed '1,11d' cluster$c.pcd >> ../tmp.pcd
              #ConcatenatePointsPCD output.pcd cluster$c.pcd -binary 0 -precision 8 -params 0
            done
            #sed '1,29d' output.pcd > ../tmp.pcd
            rm -f output.pcd
            points=`cat ../tmp.pcd | wc -l`
            head -n 11 cluster0.pcd | sed "10 c POINTS $points" | sed "7 c WIDTH $points" > ../tmp_header.pcd
            name="$prefix${p}_"`echo $list | tr " " "-"`
            cat ../tmp_header.pcd ../tmp.pcd > ../${name}_${scan}_subset.pcd
}

for cad in `ls -d1 */` # (-d1 is the same as -d: descend only 1 level) or: tree -d -L 1 or: find . -type d -maxdepth 1 -mindepth 1 or: ls -l | grep "^d"
do
  cd $cad
  rm -f random*.pcd
  rm -f all*.pcd
  for scan in `ls -d1 scan*/`
  do
    cd $scan
    scan=`echo $scan | awk -F "/" '{print $1}'`
    nr=`ls -1 cluster*.pcd | wc -l`
    if [ "$1" == "random" ]
    then
      # number of clusters to concatenate: 1-5
      for p in `seq 1 3`
      do
        # number of trials: 100
        for i in `seq 1 100`
        do
          # select $p random clusters and concatenate them
          list=""
          for c in `seq 1 $p`
          do
            r=$RANDOM
            let "r %= $nr"
            list="$list $r"
          done
          list=`echo "$list" | tr " " "\n" | sort | uniq | tail -n $p` # tail for getting rid of first blank
          if [ $p -eq `echo $list | wc -w` ]
          then
            list=`echo "$list" | tr " " ","`
            reconstruct $cad $scan $p $list "random"
          fi
        done
      done
    else
      echo "$nr"
      nrm1=$((nr-1))
      for i in `seq 0 $nrm1`
      do
        list="$i"
        reconstruct $cad $scan 1 $list "all"
        ip1=$((i+1))
        for j in `seq $ip1 $nrm1`
        do
          list="$i,$j"
          reconstruct $cad $scan 2 $list "all"
          jp1=$((j+1))
          for k in `seq $jp1 $nrm1`
          do
            list="$i,$j,$k"
            reconstruct $cad $scan 3 $list "all"
          done
        done
      done
    fi
    cd ..
  done
  cd ..
done

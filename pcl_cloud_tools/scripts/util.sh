#!/bin/bash
# Example directory containing .pcd files
CURRENT=`pwd`
Dirlist=$(find $CURRENT  -type d -maxdepth 1)

BASENAME=`basename $CURRENT`

echo $CURRENT
for direc in $Dirlist ; 
do
###########################################
#for finding analogous names 
#     filespng=`ls $direc/*.png`
#     #echo $file
     filespcd=`ls $direc/*.pcd`
#     array1=($filespng)
#     array2=($filespcd)
#     count=${#array1[@]}
#     for i in `seq 1 $count`
#     do
# 	stamppng=`echo ${array1[$i-1]} | awk -F '_' {'print $2'}`
# 	for j in `seq 1 $count`
# 	do
# 	    stamppcd=`echo ${array2[$j-1]} | awk -F '_' {'print $2'}`
# 	    if [ $stamppng == $stamppcd ]; then
# 		#echo "stamppcd" $stamppcd
# 		#echo "stamppng" $stamppng
# 		tmpfull=${array1[$i-1]}
# 		tmpbase=${tmpfull%.*}
# 		echo "moving to: " $tmpbase.pcd
# 		mv ${array2[$j-1]} $tmpbase.pcd
# 	    fi
# 	done
#     done

    # for i in $filespcd ;
    # do
    # 	echo `echo $i | awk -F '_' {'print $2'}`
    # done
    # rm bla
    # cd $direc
    # mkdir 1
    # mkdir 2
    # mkdir 3
    # mkdir 4
    # mkdir 5
    # mkdir 6
    #rm -r 1 2 3 4 5 6

#########################################3
#copy for fridge
     # for i in $filespcd;
     # do
     # 	 echo "copying " $i "to: " $(dirname $i)/3
     # 	 cp $i $(dirname $i)/3
     # done

#########################################3
#copy for island
     # for i in $filespcd;
     # do
     # 	 name=`echo $i | awk -F '/' {'print $10'}`
     # 	 name1=`echo $name | awk -F '_' {'print $1'}`
     # 	 echo $name1
     # 	 tmp="LANCASTERTEA"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/4
     # 	     cp $i $(dirname $i)/4
     # 	 fi
	 
     # 	 tmp="ALPENMILCH"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/3
     # 	     cp $i $(dirname $i)/3
     # 	 fi

     # 	 tmp="COFFEEFILTER"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/2
     # 	     cp $i $(dirname $i)/2
     # 	 fi

     # 	 tmp="ICETEASTRAWBERRY"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/3
     # 	     cp $i $(dirname $i)/3
     # 	 fi
     # done

#########################################3
#copy for oven
     # for i in $filespcd;
     # do
     # 	 name=`echo $i | awk -F '/' {'print $10'}`
     # 	 name1=`echo $name | awk -F '_' {'print $1'}`
     # 	 echo $name1
     # 	 tmp="cereals"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/2
     # 	     cp $i $(dirname $i)/2
     # 	 fi
	 
     # 	 tmp="chips"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/5
     # 	     cp $i $(dirname $i)/5
     # 	 fi
     # done

#########################################3
#copy for sink
     # for i in $filespcd;
     # do
     # 	 name=`echo $i | awk -F '/' {'print $10'}`
     # 	 name1=`echo $name | awk -F '_' {'print $1'}`
     # 	 echo $name1
     # 	 tmp="coffejacobs"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/4
     # 	     cp $i $(dirname $i)/4
     # 	 fi
	 
     # 	 tmp="iceteastrawberry"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/3
     # 	     cp $i $(dirname $i)/3
     # 	 fi

     # 	 tmp="opencvbook"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/2
     # 	     cp $i $(dirname $i)/2
     # 	 fi

     # 	 tmp="pasulj"
     # 	 if [ $name1 == $tmp ] ;
     # 	 then
     # 	     echo "copying " $i "to: " $(dirname $i)/1
     # 	     cp $i $(dirname $i)/1
     # 	 fi
     # done
done
# array1=($list1)
# array2=($list2)

# count=${#array1[@]}
# for i in `seq 1 $count`
# do
#     echo ${array1[$i-1]} ${array2[$i-1]}
# done

#!/bin/bash

echo "ACQ=$1"
echo "KERN=$2"
echo "LIMBO_ALPHA=$3"
echo "LIMBO_L=$4"
# then make a binary for the search
for dim in 3 ; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    rm -rf *
    if [ $dim -gt 3 ]; then

      cmake -DLARGE=ON -DBO_ACQ=$1 -DBO_KERN=$2 -DLIMBO_ALPHA=$3 -DLIMBO_L=$4 -DHETEROGENEOUS=ON -DNN_INPUT_TYPE=1 -DCMAKE_BUILD_TYPE=Debug -DCVT_USAGE=ON -DBD=${dim}  ..
    else
      cmake -DLARGE=ON -DBO_ACQ=$1 -DBO_KERN=$2 -DLIMBO_ALPHA=$3 -DLIMBO_L=$4 -DHETEROGENEOUS=ON -DNN_INPUT_TYPE=1 -DCMAKE_BUILD_TYPE=Debug -DBD=${dim} ..
    fi

   make -j 2

done 




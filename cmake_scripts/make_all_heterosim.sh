
# first make a binary for the recording of fitness
for dim in 3 ; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    rm -rf *
    if [ $dim -gt 3 ]; then
    
      cmake -DRECORD_FITNESS=ON -DHETEROGENEOUS=ON -DNN_INPUT_TYPE=1 -DCMAKE_BUILD_TYPE=Debug -DCVT_USAGE=ON -DBD=${dim}  ..
    else
      cmake -DRECORD_FITNESS=ON -DHETEROGENEOUS=ON -DNN_INPUT_TYPE=1 -DCMAKE_BUILD_TYPE=Debug -DBD=${dim} ..
    fi
   make -j 2


done 


# then make a binary for the search
for dim in 3 ; do   # 10 14 21 400 
    cd ~/argos-sferes
    mkdir build_${dim}D
    cd build_${dim}D
    rm -rf *
    if [ $dim -gt 3 ]; then
    
      cmake -DHETEROGENEOUS=ON -DNN_INPUT_TYPE=1 -DCMAKE_BUILD_TYPE=Debug -DCVT_USAGE=ON -DBD=${dim}  ..
    else
      cmake -DHETEROGENEOUS=ON -DNN_INPUT_TYPE=1 -DCMAKE_BUILD_TYPE=Debug -DBD=${dim} ..
    fi
   make -j 2


done 




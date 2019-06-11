#!/bin/bash


DATAPATH=$1
FILE_PATTERN=${DATAPATH}/gen_*
for file in ${FILE_PATTERN}; do
  newnumber='00000'${file#${DATAPATH}/gen_}      # get number, pack with zeros
  newnumber=${newnumber:(-5)}       # the last five characters
  mv $file ${DATAPATH}/gen_${newnumber}            # rename
done

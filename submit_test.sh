#!/bin/bash


FILE=./bin/obsavoid_evol${VORONOI}${BD}D
if [ -f "$FILE" ]; then
    echo "$FILE exist"
else 
    echo "$FILE does not exist"
    exit 1;
fi
generationfile="${OUTPUTDIR}/gen_${FINALGEN}"
archivefile="${OUTPUTDIR}/archive_${FINALGEN}.dat"

jobtocome="${FILE} ${CONFIG} -d ${OUTPUTDIR} --load ${generationfile} --o outputfile"
echo "Starting the following command: "${jobtocome}" for all individuals"
echo "Looking for individuals at: "${archivefile}

python /home/david/PycharmProjects/Visualisation/BD_plots/process_archive_data.py -c "${jobtocome}" -p "${archivefile}"

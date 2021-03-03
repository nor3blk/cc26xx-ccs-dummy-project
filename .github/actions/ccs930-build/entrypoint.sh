#!/bin/bash

DATA=/github
PJ=workspace
PJ_HOME=${DATA}/${PJ}
OUT_DIR=${PJ_HOME}/artifact

config_list=$1
echo $config_list
configs=(${config_list//,/ })
echo ${configs[@]}

for config in ${configs[@]}; do
    eclipse -noSplash -data ${DATA} -application com.ti.ccstudio.apps.projectBuild -ccs.autoImport -ccs.projects ${PJ} -ccs.configuration ${config}
    mkdir -p ${OUT_DIR}/${config}
    cp -p ${PJ_HOME}/${config}/${PJ}.{out,map} ${OUT_DIR}/${config}/
    head -16 ${PJ_HOME}/${config}/${PJ}.map | tail -7 > ${OUT_DIR}/${config}/${PJ}.txt
done

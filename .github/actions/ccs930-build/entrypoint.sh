#!/bin/bash

config_list=$1
echo $config_list
configs=(${config_list//,/ })
echo ${configs[@]}

for config in ${configs[@]}; do
    eclipse -noSplash -data /github -application com.ti.ccstudio.apps.projectBuild -ccs.autoImport -ccs.projects workspace -ccs.configuration ${config}
done

#/bin/bash

configurations=$1
echo $configurations
configurations=list(${configurations//,/ })
echo ${configurations[@]}

for config in ${configurations[@]}; do
    eclipse -noSplash -data /github -application com.ti.ccstudio.apps.projectBuild -ccs.autoImport -ccs.projects workspace -ccs.configuration ${config}
done

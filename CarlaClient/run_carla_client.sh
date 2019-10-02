#!/bin/bash
echo "run carla clients controlled by SUMO Server"
if [ $# == 1 ]
then
    run_nums=$1
    # location=$(cd `dirname $0`; pwd)
    # echo $location
    echo "run nums: $1"
    while (( $run_nums>0 ))
    do
        python_command="source activate carla; python automatic_control.py"
        # command="gnome-terminal -- bash -c \"${python_command}\""
        {
        gnome-terminal -x bash -c "source activate carla; python automatic_control.py"
        }&
        sleep 1s
        # echo ${command}
        # $command
        # $python_command
        let "run_nums--"
    done
else
    echo "shell file usage: $0 [run_nums]"
fi

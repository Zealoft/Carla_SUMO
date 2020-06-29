#!/bin/bash
# generate 100 vehicles and trips by default 
# change num by change the value of vehicles_num
# author: Zhang Wenhao
echo "generate simulation files for sumo"
echo "This shell script can automatically support *.osm file and *.xodr file."
vehicles_num=100
end_time=1000
if [ $# == 2 ]
then
    file_path=$1
    typemap_path=$2
    echo "filename: $1, typemap: $2"
    file_path_len=${#file_path}
    last_four=${file_path:file_path_len-4:4}
    last_three=${file_path:file_path_len-3:3}
    if [ $last_four == "xodr" ]
    then
        echo "xodr file detected."
        # deal with file path
        single_file_name=${file_path##*/}
        # echo $single_file_name
        relative_path=${file_path%${single_file_name}}
        # echo $relative_path
        single_file_name=${single_file_name%.xodr}
        
        file_path=$1
        
        # get the .net.xml file
        net_xml_file="${single_file_name}.net.xml"
        net_xml_command="netconvert --opendrive ${file_path} -o ${relative_path}${net_xml_file}"
        echo $net_xml_command
        $net_xml_command
        # get the .poly.xml file
        poly_file="${single_file_name}.poly.xml"
        polyconvert_command="polyconvert --net-file ${relative_path}${net_xml_file} --type-file $typemap_path -o ${relative_path}${poly_file}"
        echo $polyconvert_command
        $polyconvert_command
        # generate random trip using python
        trip_file="${single_file_name}.trip.xml"
        generate_randomtrip_command="python ${SUMO_HOME}/tools/randomTrips.py -n ${relative_path}${net_xml_file} -e ${vehicles_num} -l -o ${relative_path}${trip_file}"
        echo $generate_randomtrip_command
        $generate_randomtrip_command
        # generate route file using python & duarouter
        route_file="${single_file_name}.rou.xml"
        duarouter_command="python ${SUMO_HOME}/tools/randomTrips.py -n ${relative_path}${net_xml_file} -e ${vehicles_num} -l -r ${relative_path}${route_file}"
        echo $duarouter_command
        $duarouter_command
        # write the sumocfg file
        sumocfg_file="${relative_path}${single_file_name}.sumocfg"
        dump_file="${single_file_name}.sumo.tr"
        tripinfo_file="${single_file_name}.tripinfo.xml"
        vehroutes_file="${single_file_name}.vehroutes.xml"
        # echo $sumocfg_file
        printf "<configuration>
    <input>
        <net-file value=\"${net_xml_file}\"/>
        <route-files value=\"${route_file}\"/>
        <additional-files value=\"${poly_file}\"/>
        <junction-files value=\"\"/>
    </input>
    <output>
        <netstate-dump value=\"${dump_file}\"/>
        <tripinfo-output value=\"${tripinfo_file}\"/>
        <vehroute-output value=\"${vehroutes_file}\"/>
    </output>
        <time>
        <begin value=\"0\"/>
        <end value=\"${end_time}\"/>
        </time>
</configuration>"  > $sumocfg_file
        echo "write sumocfg file done!"
    elif [ $last_three == "osm" ]
    then
        echo "osm file detected."
        # deal with file path
        single_file_name=${file_path##*/}
        # echo $single_file_name
        relative_path=${file_path%${single_file_name}}
        # echo $relative_path
        single_file_name=${single_file_name%.xodr}
        file_path=$1
        # get the .net.xml file
        net_xml_file="${single_file_name}.net.xml"
        net_xml_command="netconvert --osm-files ${file_path} -o ${relative_path}${net_xml_file}"
        echo $net_xml_command
        $net_xml_command
        # get the .poly.xml file
        poly_file="${single_file_name}.poly.xml"
        polyconvert_command="polyconvert --net-file ${relative_path}${net_xml_file} --type-file $typemap_path -o ${relative_path}${poly_file}"
        echo $polyconvert_command
        $polyconvert_command
        # generate random trip using python
        trip_file="${single_file_name}.trip.xml"
        generate_randomtrip_command="python ${SUMO_HOME}/tools/randomTrips.py -n ${relative_path}${net_xml_file} -e ${vehicles_num} -l -o ${relative_path}${trip_file}"
        echo $generate_randomtrip_command
        $generate_randomtrip_command
        # generate route file using python & duarouter
        route_file="${single_file_name}.rou.xml"
        duarouter_command="python ${SUMO_HOME}/tools/randomTrips.py -n ${relative_path}${net_xml_file} -e ${vehicles_num} -l -r ${relative_path}${route_file}"
        echo $duarouter_command
        $duarouter_command
        # write the sumocfg file
        sumocfg_file="${relative_path}${single_file_name}.sumocfg"
        dump_file="${single_file_name}.sumo.tr"
        tripinfo_file="${single_file_name}.tripinfo.xml"
        vehroutes_file="${single_file_name}.vehroutes.xml"
        # echo $sumocfg_file
        printf "<configuration>
    <input>
        <net-file value=\"${net_xml_file}\"/>
        <route-files value=\"${route_file}\"/>
        <additional-files value=\"${poly_file}\"/>
        <junction-files value=\"\"/>
    </input>
    <output>
        <netstate-dump value=\"${dump_file}\"/>
        <tripinfo-output value=\"${tripinfo_file}\"/>
        <vehroute-output value=\"${vehroutes_file}\"/>
    </output>
        <time>
        <begin value=\"0\"/>
        <end value=\"${end_time}\"/>
        </time>
</configuration>"  > $sumocfg_file
        echo "write sumocfg file done!"
    else
        echo "file type error! It should be xodr(Opendrive) or OSM(openstreet map) file. program aborted."
    fi
else
    echo "shell file usage: $0 [xodr_file] or [osm_file] [typemap_path]"
fi

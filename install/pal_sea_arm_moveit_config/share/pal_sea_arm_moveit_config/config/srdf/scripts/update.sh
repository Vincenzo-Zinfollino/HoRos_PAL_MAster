#!/bin/bash
set -e
set -o pipefail
ulimit -m 8048000
pal_moveit_config_generator=$(ros2 pkg prefix pal_moveit_config_generator)
source "$pal_moveit_config_generator/share/pal_moveit_config_generator/srdf_utils.sh" "$(dirname "${BASH_SOURCE[0]}")/../pal_sea_arm.srdf.xacro"

# crawl all end effectors and generate the corresponding subtree SRDF
for end_effector_file in "$srdf_folder"/end_effectors/*.srdf.xacro; do
    end_effector=$(basename "$end_effector_file" .srdf.xacro)
    echo $end_effector
    if [ "$end_effector" = "no-end-effector" ]; then
        end_effector_value="no-ee"
    else
        end_effector_value="$end_effector"
    fi
    echo $end_effector_value


    for ft_sensor in no-ft-sensor rokubi; do
        args=(arm_type:='pal-sea-arm-standalone' "ft_sensor:=$ft_sensor" end_effector:="$end_effector")
        if [ "$ft_sensor" != 'no-ft-sensor' ]; then
            generate_disable_collisions_subtree arm_tool_link "${end_effector_value}_${ft_sensor}" "${end_effector_value}" "${args[@]}"
        else
            generate_disable_collisions_subtree arm_tool_link "${end_effector_value}"  "" "${args[@]}"
        fi
    done
done

prefix="pal_sea_arm"

generate_disable_collisions "${prefix}_no-ee" "${prefix}_no-arm" ft_sensor:="no-ft-sensor" end_effector:="no-end-effector" # plus arm
generate_disable_collisions "${prefix}_no-ee_rokubi" "${prefix}_no-ee" ft_sensor:=rokubi end_effector:="no-end-effector" # plus FT sensor

# crawl all end effectors and generate the corresponding SRDF
for end_effector_file in "$srdf_folder"/end_effectors/*.srdf.xacro; do
    end_effector=$(basename "$end_effector_file" .srdf.xacro)
    if [ "$end_effector" = "no-end-effector" ]; then
        end_effector_value="no-ee"
    else
        end_effector_value="$end_effector"
    fi
    for ft_sensor in no-ft-sensor rokubi; do
        echo $end_effector
        args=("ft_sensor:=$ft_sensor" end_effector:="$end_effector")
        if [ "$ft_sensor" != 'no-ft-sensor' ]; then
            echo "${prefix}_${end_effector_value}_${ft_sensor}" "${prefix}_no-ee_${ft_sensor}:${end_effector_value}_${ft_sensor}" "${args[@]}"
            generate_srdf "${prefix}_${end_effector_value}_${ft_sensor}" "${prefix}_no-ee_${ft_sensor}:${end_effector_value}_${ft_sensor}" "${args[@]}"
        else
            echo "${prefix}_${end_effector_value}" "${prefix}_no-ee:${end_effector_value}" "${args[@]}"
            generate_srdf "${prefix}_${end_effector_value}" "${prefix}_no-ee:${end_effector_value}" "${args[@]}"
        fi
    done
done


#!/bin/bash
set -e
pal_moveit_config_generator=$(rospack find pal_moveit_config_generator)
source "$pal_moveit_config_generator/srdf_utils.sh" "$(dirname "${BASH_SOURCE[0]}")/../pal_sea_arm.srdf.xacro"

ref=${1:-HEAD}

for f in "$srdf_folder"/end_effectors/*.srdf.xacro; do
    end_effector=$(basename "$f" .srdf.xacro)
    end_effector_name=$(grep -Po '<xacro:property name="end_effector_name" value="\K[^"]+' "$f" || echo "gripper")
    add_diff_matrix_to_xacro_from_ref "$ref" "pal_sea_arm_${end_effector}.srdf" "$end_effector_name" "disable_collisions/$end_effector.srdf.xacro"
done

for f in "$srdf_folder"/disable_collisions/*.srdf.xacro; do
    p=${f#"$srdf_folder"/}
    add_diff_matrix_to_xacro_from_ref "$ref" "$p" "" "$p"
done

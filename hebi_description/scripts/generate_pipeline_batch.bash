#! /usr/bin/env bash

# NOTE: If the ROS environment is not sourced, this script should be run from hebi_description/tools,
# otherwise paths break.

PKG_DIR=$(rospack find hebi_description)
HRDF_LOCATION=$(rospack find hebi_cpp_api_examples)/config
if [ -z "$PKG_DIR" ]
then
  PKG_DIR=".."
fi
# Call script to generate xacro/sdf from hrdf
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2084-01.hrdf"                  --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-04.hrdf"                  --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-05.hrdf"                  --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-06.hrdf"                  --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2 J6_wrist3
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2084-01-parallel-gripper.hrdf" --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-04-parallel-gripper.hrdf" --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-05-parallel-gripper.hrdf" --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2
python3 "$PKG_DIR/scripts/urdf_generator.py" "$HRDF_LOCATION/hrdf/A-2085-06-parallel-gripper.hrdf" --urdfdir "$PKG_DIR/urdf/kits" --sdfdir "$PKG_DIR/models" --family Arm --actuators J1_base J2_shoulder J3_elbow J4_wrist1 J5_wrist2 J6_wrist3

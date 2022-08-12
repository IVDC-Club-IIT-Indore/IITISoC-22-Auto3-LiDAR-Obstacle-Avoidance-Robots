# hebi_description

This repository is a collection of `xacro` macros for creating URDF files for HEBI components and systems.

Along with the macros, several macros for full systems are defined.

When using these macros, note that you may have to run `xacro --xacro-ns <filename>` in order to suppress legacy interpretation of xacro tags without namespaces (see http://wiki.ros.org/xacro).

## Dependencies

The scripts for converting between hrdf, xacro, and urdf depend on lxml, which can be installed by running `pip3 install --user lxml`

## xacro macros

We provide several `xacro` macros for HEBI components that can be used to create robots for simulation or visualization:

### `<xacro:actuator/>`

This represents a HEBI actuator.  Required attributes are:

* `name` The name of this actuator.  This is used to reference this actuator from other HEBI components.  Note that when HEBI components are used with the HEBI gazebo plugins, this should be in the format `<family>/<name>`, where these are the human-readable and settable names of the actuators on the network (e.g., "HEBI\Elbow" for the elbow joint on a standard HEBI arm).
* `child` The element that is attached to the output of this actuator. This should be the "name" of the HEBI component; for standard URDF components, this will attempt to connect to `<name>/INPUT_INTERFACE`.
* `type` The actuator type - `X5_1`, `X5_4`, `X5_9`, `X8_3`, `X8_9`, or `X8_16`.

Optional attributes are:

* `limits` The position limits of the actuator.  If not defined, assumes a continuously rotatable revolute joint.  If defined, both must be finite and the lower limit must be below the upper limit.  The format is `${[<low>,<high>]}`, and is in radians.  For example, one full revolution centered at 0 would be `${[-${pi}, ${pi}]}`.

### `<xacro:link/>`

* `name` A unique name that can be used to reference this link from other HEBI and URDF components.
* `child` The element that is attached to the output of this link. This should be the "name" of the HEBI component; for standard URDF components, this will attempt to connect to `<name>/INPUT_INTERFACE`.
* `extension` The extension of the link, in meters, per documented convention on docs.hebi.us. For example, a `300` mm tube would have an extension of `0.325`.
* `twist` The twist of the link input and output frames, in radians, per documented convention on docs.hebi.us.  A `0` radian twist would mean that the input and output z-axis are the same direction.

### `<xacro:bracket/>`

* `name` A unique name that can be used to reference this bracket from other HEBI and URDF components.
* `child` The element that is attached to the output of this bracket. This should be the "name" of the HEBI component; for standard URDF components, this will attempt to connect to `<name>/INPUT_INTERFACE`.
* `type` The bracket type - `X5LightLeft`, `X5LightRight`, `X5HeavyLeftInside`, `X5HeavyLeftOutside`, `X5HeavyRightInside`, or `X5HeavyRightOutside`

### `<xacro:gripper/>`

This represents a HEBI actuator (currently only the parallel jaws style)

* `name` A unique name that can be used to reference this bracket from other HEBI and URDF components; the base link that will be created is called `<name>/INPUT_INTERFACE`
* `type` The gripper type; currently `parallel` is the only supported value.

### `<xacro:null_end_effector/>`

Represents the end of a robot, with no particular other information (this allows the final joint in another `actuator`, `link`, or `bracket` to be completed)

* `name` A unique name that can be used to reference this bracket from other HEBI and URDF components; the link that will be created is called `<name>/INPUT_INTERFACE`.

## Conventions

The naming convention we use for elements is that the xacro actuator/link/bracket/etc macros automatically define two specifically named subelements:

* `<element name>/INPUT_INTERFACE` is the input body of a module
* `<element name>/OUTPUT_INTERFACE` is the output joint of a module

This allows custom components to reference HEBI library components, as seen below.

Additionally, when naming actuators (`xacro:x5` or `xacro:x8` types), then convention is `<family>/<name>`.  This allows the HEBI Gazebo simulation plugins to properly load and simulate the actuators.

Here is a segment of a XACRO file which connects a 3-DOF hebi arm to a "base" link in the world, and adds an "end effector" link at the end.  Note -- HEBI xacro components currently must always define a child element for their joint to connect to!

```
  ...

  <!-- connect to a "base" link in the world -->
  <joint name="base_robot_base" type="fixed">
    <parent link="base"/>
    <child link="HEBI/base/INPUT_INTERFACE"/>
  </joint>

  <!-- define the HEBI robot -->
  <xacro:actuator name="Arm/J1_base" child="shoulder_bracket" type="X5_4"/>
  <xacro:bracket name="shoulder_bracket" child="Arm/J2_shoulder" type="X5HeavyRightInside"/>
  <xacro:actuator name="Arm/J2_shoulder" child="shoulder_elbow" type="X5_4"/>
  <xacro:link name="shoulder_elbow" child="Arm/J3_elbow" extension="0.33" twist="${PI}"/>
  <xacro:actuator name="Arm/J4_elbow" child="elbow_end" type="X5_4"/>
  <xacro:link name="elbow_end" child="end_effector" extension="0.325" twist="${PI}"/>

  <!-- end link; necessary to provide output for HEBI components to attach to. -->
  <link name="end_effector/INPUT_INTERFACE">
  </link>

  <gazebo reference="base">
    <selfCollide>true</selfCollide>

  ...
```


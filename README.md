# Spring Joint Plugin

A Gazebo Sim system plugin that creates virtual linear springs between arbitrary links in a URDF model.

The plugin is loaded through the `<gazebo><plugin>` tag and applies spring forces between user-defined attachment points on two links.

## Motivation

URDF represents robots as tree structures, making it impossible to directly model closed-chain mechanisms such as parallel grippers, four-bar linkages, and other overconstrained assemblies.

This plugin provides a spring-based constraint that connects arbitrary links without modifying the robot's kinematic tree, allowing such mechanisms to be simulated while remaining compatible with standard URDF models.

## Features

- Pure linear spring constraints
- Multiple springs supported in a single plugin
- User-defined local attachment points
- Works with existing URDF models
- Uses Gazebo Sim's native physics API (`AddWorldWrench`)

## Plugin Configuration

```xml
<gazebo>
  <plugin
      filename="libspring_joint_plugin.so"
      name="spring_joint::SpringPlugin">

    <spring>

      <parent_link>link1</parent_link>
      <child_link>link2</child_link>

      <parent_point>0 0 0.05</parent_point>
      <child_point>0 0 0</child_point>

      <k_linear>1000</k_linear>

    </spring>

  </plugin>
</gazebo>
```

## How it Works

During startup, the plugin:

- Parses every `<spring>` element
- Finds the corresponding parent and child links
- Stores their local attachment points
- Stores the spring stiffness

During every simulation step, the plugin:

- Computes the world coordinates of both attachment points
- Calculates the linear displacement between them
- Computes the spring force using Hooke's law
- Applies equal and opposite forces to both links

## Build

```bash
mkdir build
cd build

cmake ..
make
```

### Register the Plugin Path

Gazebo needs to know where to discover the compiled library binary. Export the build location to your environment paths (add this to your `~/.bashrc` for persistence):

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/spring_joint_plugin/build
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/spring_joint_plugin/build

```

## Implementation

The plugin is intended to complement existing URDF models rather than replace their joint definitions.

For mechanisms containing closed kinematic loops, the recommended workflow is:

1. Model the mechanism normally in CAD.
2. Export it to URDF.
3. Remove one or more joints that close the kinematic loop, leaving the mechanism as an open tree.
4. Add one or more virtual springs between the disconnected links using this plugin.

The virtual springs restore the missing physical relationship during simulation while keeping the robot compatible with URDF's tree-based structure.

This approach is particularly useful for mechanisms such as:

- Parallel grippers
- Four-bar linkages
- Parallel manipulators
- Closed-chain robotic mechanisms

Since the plugin simply applies forces between arbitrary attachment points, it is not limited to replacing missing joints. It can also be used wherever compliant or spring-like interactions are desired between links, such as elastic couplings, compliant mechanisms, suspension elements, or other custom force-based constraints.

## Current Status

This project is currently under active development.

Implemented:

- Linear spring forces
- Multiple spring support
- URDF plugin configuration
- Local attachment points

Planned:

- Angular springs
- Damping
- Joint limits
- Additional constraint types

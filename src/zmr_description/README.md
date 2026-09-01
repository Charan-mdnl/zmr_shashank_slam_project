# ZMR Description - final CAD-matched package

This package is based on the last accepted ZMR caster/mesh/joint model. The visual transforms and joint transforms were preserved. The remaining work was to make the collision geometry and ROS package structure consistent.

## Corrected

- All mesh references use `package://zmr_description/...` paths.
- Base collision uses the exact chassis STL instead of an offset-independent box.
- Drive-wheel collision uses the same mesh, origin and rotation as the accepted visual geometry.
- Each caster swivel link has BOTH Component4 and Component5 collision geometry, matching its two visual meshes.
- Each caster wheel collision uses its accepted wheel mesh and transform.
- LiDAR collision uses its accepted mesh.
- Base inertial reference is placed at the CAD body center (0.355, 0.525, 0.105 m in `cad_base_link`).
- The accepted caster kinematic structure is unchanged: one Z-axis continuous yaw/swivel joint plus one continuous wheel-rotation joint per caster.
- The accepted drive-wheel geometry/joints are unchanged.

## Important physical-data note

`cad_base_link` uses a 10.0 kg mass because that is the value in the accepted model. Replace it with the measured/validated chassis mass before using the model for quantitative dynamics simulation.

## Collision note

The collision folder intentionally mirrors the accepted visual STL geometry so that the collision model cannot silently drift away from the visual model. Detailed mesh collision is accurate but can be more computationally expensive than primitive/convex collision in a physics simulator.

## ROS 2 diff-drive parameters

`config/diff_drive_controller.yaml` contains the verified wheel separation (0.526 m), wheel radius (0.080 m), joint names and `base_link` frame. A hardware/simulation interface is still required separately when using `ros2_control`.

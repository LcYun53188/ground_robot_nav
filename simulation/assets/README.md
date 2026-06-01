# Isaac Sim Assets

Place optional local robot and scene assets here.

Large mesh/USD/STL files should stay out of git unless they are intentionally
small test fixtures. Point `simulation/config/isaac_sim_nav.yaml` at local files:

- `scene.scene_mesh_stl_path`: static environment mesh.
- `scene.robot_mesh_stl_path`: visual robot body mesh.
- `scene.world_usd_path`: complete Isaac Sim USD stage to open instead of the
  built-in primitive test scene.

If these paths are empty, the Isaac Sim script creates a simple ground plane,
box obstacles, and a primitive robot body so the ROS2 navigation interface can
be validated without external assets.

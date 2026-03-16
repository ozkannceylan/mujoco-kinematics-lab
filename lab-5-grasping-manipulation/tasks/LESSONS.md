# Lab 5: Lessons Learned

## Bugs & Fixes

<!-- Log bugs here as they occur during implementation -->

## Debug Strategies

### Verify gripper geometry with MuJoCo viewer
Run `python -c "import mujoco; import mujoco.viewer; m=mujoco.MjModel.from_xml_path('models/scene_grasp.xml'); d=mujoco.MjData(m); mujoco.viewer.launch(m, d)"` from models/ to visually inspect the scene before running any control code.

### Print contact pairs for slipping diagnosis
When the box slips: `for i in range(data.ncon): c=data.contact[i]; print(model.geom(c.geom1).name, model.geom(c.geom2).name, c.dist)` to see which geoms are in contact and their penetration depth.

## Key Insights

<!-- Insights go here as discovered -->

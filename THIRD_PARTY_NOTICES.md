# Third-Party Notices

This repository includes third-party code and robot assets. Those files are
not relicensed by the root [LICENSE](LICENSE). Their original license files and
copyright notices must be preserved.

## Summary

| Path | Upstream | License | Notes |
|---|---|---|---|
| `lab-2-Ur5e-robotics-lab/models/mujoco_menagerie/` | Google DeepMind MuJoCo Menagerie | See bundled `LICENSE` and per-model licenses | Upstream repository contains multiple models with mixed permissive licenses. |
| `lab-2-Ur5e-robotics-lab/models/mujoco_menagerie/universal_robots_ur5e/` | Google DeepMind / Universal Robots UR5e model | BSD-3-Clause | Includes the MuJoCo UR5e model used in Lab 2. |
| `lab-2-Ur5e-robotics-lab/models/Universal_Robots_ROS2_Description/` | Universal Robots ROS 2 description package | BSD-3-Clause for most files | The package ships additional mesh directories with separate vendor terms. |
| `lab-2-Ur5e-robotics-lab/models/Universal_Robots_ROS2_Description/meshes/ur8long/` | Universal Robots | Universal Robots Terms and Conditions for Use of Graphical Documentation | Not OSI-open-source. |
| `lab-2-Ur5e-robotics-lab/models/Universal_Robots_ROS2_Description/meshes/ur15/` | Universal Robots | Universal Robots Terms and Conditions for Use of Graphical Documentation | Not OSI-open-source. |
| `lab-2-Ur5e-robotics-lab/models/Universal_Robots_ROS2_Description/meshes/ur18/` | Universal Robots | Universal Robots Terms and Conditions for Use of Graphical Documentation | Not OSI-open-source. |
| `lab-2-Ur5e-robotics-lab/models/Universal_Robots_ROS2_Description/meshes/ur20/` | Universal Robots | Universal Robots Terms and Conditions for Use of Graphical Documentation | Not OSI-open-source. |
| `lab-2-Ur5e-robotics-lab/models/Universal_Robots_ROS2_Description/meshes/ur30/` | Universal Robots | Universal Robots Terms and Conditions for Use of Graphical Documentation | Not OSI-open-source. |

## Redistribution Rules

- Original repository content authored for this project is licensed under Apache-2.0.
- Bundled third-party directories keep their upstream licenses.
- If you redistribute this repository, keep the upstream `LICENSE`, `README`, and
  notice files inside those third-party directories.
- If you want the repository to be strictly OSI-open-source only, remove or
  replace the Universal Robots graphical-documentation mesh directories listed
  above before redistributing.

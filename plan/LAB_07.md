# Lab 7: Locomotion Fundamentals

> **Status:** Not Started  
> **Prerequisites:** Lab 3 (dynamics & force control)  
> **Platform:** Unitree G1 on MuJoCo  
> **Capstone Demo:** Stable bipedal walking on flat ground (10+ steps)

---

## Objectives

1. Understand the fundamental differences between fixed-base and floating-base dynamics
2. Implement standing balance using center of mass (CoM) control
3. Generate a basic walking gait using Zero Moment Point (ZMP) planning
4. Achieve stable bipedal walking on the Unitree G1 in MuJoCo

---

## Why This Lab Matters

This is the biggest paradigm shift in the series. Labs 1–6 dealt with fixed-base robots: the base is bolted to the ground, and the only moving parts are the joints. The G1 humanoid has a floating base — the entire robot can fall over. Gravity is no longer just a torque to compensate; it's an existential threat. Every concept you learned about dynamics (Lab 3) still applies, but the problem is fundamentally harder: you must simultaneously control the joints AND keep the robot from falling.

---

## Theory Scope

- Floating base dynamics: 6-DOF unactuated base + actuated joints
- Center of Mass (CoM) and support polygon
- Zero Moment Point (ZMP): the point where ground reaction forces produce zero moment
- ZMP stability criterion: ZMP must stay inside the support polygon
- Simple gait generation: pre-planned footstep sequence + ZMP trajectory
- Centroidal dynamics: linear and angular momentum at the CoM

---

## Architecture

```
Footstep Plan (predefined step positions)
        │
        ▼
┌──────────────────────┐
│  ZMP Trajectory       │
│  Planner              │
│  Footsteps → desired  │
│  CoM trajectory       │
└──────────┬───────────┘
           │ desired CoM(t)
           ▼
┌──────────────────────┐
│  Inverse Kinematics   │
│  CoM + foot poses →   │
│  joint angles         │
│  (Pinocchio IK)       │
└──────────┬───────────┘
           │ joint targets q_d(t)
           ▼
┌──────────────────────┐
│  Joint PD + Gravity   │
│  Compensation         │
│  (adapted from Lab 3) │
└──────────┬───────────┘
           │ joint torques
           ▼
┌──────────────────────┐
│  MuJoCo Simulation    │
│  G1 humanoid model    │
│  Flat ground contact  │
└──────────────────────┘
```

---

## Implementation Phases

### Phase 1 — G1 Setup & Standing Balance
- Load Unitree G1 from MuJoCo Menagerie
- Understand the G1 joint structure (DOF layout, actuator limits)
- Implement standing controller: keep CoM above support polygon
- Validate: push the robot (apply perturbation force), it recovers

### Phase 2 — ZMP Planning
- Implement Linear Inverted Pendulum Model (LIPM) for CoM trajectory
- Define a simple footstep plan: alternating left-right steps, fixed stride
- Compute ZMP-stable CoM trajectory using preview control
- Visualize: CoM trajectory + ZMP trajectory + support polygon over time

### Phase 3 — Walking Gait Execution
- Implement whole-body IK: given CoM trajectory + foot trajectories → joint angles
- Use Pinocchio's IK with task prioritization (CoM > feet > posture)
- Execute the gait on the G1 in MuJoCo
- Iterate: tune PD gains, step timing, step height until 10+ stable steps

### Phase 4 — Documentation & Blog
- Write LAB_07.md: floating-base dynamics, ZMP theory, LIPM, results
- Write blog post: "making a humanoid walk — the ZMP approach"
- Record walking demo video

---

## Key Design Decisions for Claude Code

- **Start with standing, not walking.** Standing balance is a prerequisite. If the robot can't stand still under perturbation, walking will fail immediately.
- **Use LIPM, not full nonlinear optimization.** The Linear Inverted Pendulum Model is the classic entry point. Full centroidal MPC (Lab 8 territory) is overkill here.
- **Pre-planned footsteps.** Don't implement footstep planning. Hard-code a sequence of alternating steps on flat ground. The goal is gait execution, not gait planning.
- **G1 has many DOFs you don't need.** Focus on the legs + waist. Lock the arms in a neutral pose (they'll be used in Lab 8). This reduces the problem to ~12 actuated DOFs.
- **Expect falls.** Lots of them. Implement a "reset to standing" function for fast iteration.

---

## Success Criteria

- [ ] G1 stands stably on flat ground under perturbation (5N push)
- [ ] ZMP trajectory stays inside support polygon during planned gait
- [ ] G1 walks 10+ steps on flat ground without falling
- [ ] Gait is visually smooth (no excessive wobble or jerking)
- [ ] LAB_07.md complete
- [ ] Blog post published

---

## References

- Kajita et al., "Biped Walking Pattern Generation using Preview Control of ZMP" (2003)
- Vukobratović & Borovac, "Zero-Moment Point — Thirty Five Years of Its Life" (2004)
- MuJoCo Menagerie: Unitree G1 model documentation
- Pinocchio: centroidal dynamics, computeCenterOfMass

# Lab 4: TODO

## Canonical UR5e + Robotiq Path
- [x] Step R1: Replace the old Lab 4 robot/collision baseline with Menagerie UR5e + mounted Robotiq 2F-85
- [x] Step R2: Align planning-time collision truth with the executed MuJoCo geometry
- [x] Step R3: Re-validate RRT / RRT* on the canonical stack
- [x] Step R4: Re-validate path shortcutting and time parameterization
- [x] Step R5: Re-validate executed tracking on the canonical stack
- [x] Step R6: Record a Lab 4 validation video on the canonical stack
- [x] Step R7: Update README and task docs to the signed-off final state

## Video Production
- [x] Step V1: Shared `tools/video_producer.py` three-phase pipeline
- [x] Step V2: Slalom demo with multi-waypoint RRT* planner, metrics, and video
- [x] Step V3: Consolidated all output into `media/`

## Slalom Redesign (2026-03-24)
- [x] Step S1: Replace obstacles with 4 staggered tabletop boxes (10x10x20 cm)
- [x] Step S2: Add 9-waypoint slalom path at z=0.56 with gap-midpoints
- [x] Step S3: Rewrite capstone_demo.py as multi-segment RRT* weaving demo
- [x] Step S4: Rewrite record_lab4_demo.py and record_lab4_validation.py for slalom
- [x] Step S5: Delete slalom_demo.py and generate_lab4_demo.py (absorbed into capstone)
- [x] Step S6: Update tests — all 44 pass with new obstacle layout
- [x] Step S7: Update architecture and task docs

## Current Focus
> Lab 4 complete. Slalom capstone validated — arm weaves through 4 obstacles.

## Blockers
> None.

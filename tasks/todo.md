# Todo

## Lab 1: 2-Link Planar Arm

- [x] `A1` MuJoCo setup ve actuator denemeleri eklendi
- [x] `A2` FK ve workspace analizi eklendi
- [x] `A3` Jacobian ve singularity analizi eklendi
- [x] `A4` analitik/nümerik IK eklendi
- [x] `A5` dynamics gözlemi ve `qfrc_bias`/`qM` erişimi eklendi
- [x] `B1` trajectory generation eklendi
- [x] `B2` PD controller ve gravity compensation karşılaştırması eklendi
- [x] `B3` full pipeline demo'ları eklendi
- [x] `B4` opsiyonel ROS2 bridge iskeleti eklendi
- [x] `C1` Cartesian kare çizim — computed torque control + MuJoCo viewer trail
- [x] README, docs (EN + TR), video ve GIF kaydı tamamlandı

## Lab 2: UR5e 6-DOF Arm

- [x] `A1` Ortam kurulumu — MuJoCo + Pinocchio FK cross-validation (0.000 mm, 6 config)
- [x] `A2` Forward kinematics — DH parametreleri + Pinocchio + MuJoCo karşılaştırma
- [x] `A3` Jacobian — geometric (gerçek eksen tespiti), Pinocchio, numerical + singularity analizi
- [x] `A4` Inverse kinematics — pseudo-inverse + adaptive DLS, FK roundtrip validation
- [x] `A5` Dynamics — M(q) CRBA, g(q), C(q,qd) + RNEA/ABA + MuJoCo cross-validation
- [x] `B1` Trajectory — cubic, quintic, trapezoidal, min-jerk, multi-segment
- [x] `B2` Control — PD+g, computed torque, task-space impedance, OSC
- [x] `B3` Constraints — joint limits, velocity scaling, torque saturation, self-collision
- [x] `C1` Full pipeline — pick-and-place + circle tracking demo'ları
- [x] `C2` ROS2 bridge — scaffold + standalone demo mode
- [x] `C3` 3D küp çizimi — DLS IK, quintic traj, GC+VFF position control (RMS 0.088 mm)
- [x] Unit tests — 34 test, 5 dosya, hepsi geçiyor
- [x] Docs (EN + TR) — A1, A3–C3 tamamlandı (A2 kullanıcı tarafından yönetiliyor)
- [x] Video + GIF kaydı — c3_draw_cube.mp4 (707 frame, 24s) + c3_draw_cube.gif
- [x] README.md — lab içi README + main README güncellendi
- [x] CLAUDE.md — yeni klasör yapısı + lab template eklendi

## Genel

- [x] Klasör yapısı yeniden düzenlendi: lab-1-2link-arm/ ve lab-2-Ur5e-robotics-lab/
- [x] Main README — her iki labın GIF'i, sonuç tabloları, proje yapısı
- [x] CLAUDE.md — yeni lab ekleme şablonu tanımlandı
- [ ] Lab 3 (TBD) — gelecekte eklenecek

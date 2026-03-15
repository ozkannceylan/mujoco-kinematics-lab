# Interview Cheatsheet

## FK vs IK'yi Acikla

Forward kinematics, joint acilarindan end-effector pose'unu uretir. Inverse kinematics bunun tersidir ve ayni pose icin birden fazla gecerli joint cozumu olabilir.

## Jacobian Nedir?

Jacobian, joint velocity'yi end-effector twist'ine map eder. Numerical IK, manipulability analizi ve task-space control icin kullanilan temel local linearization budur.

## Singularity'leri Nasil Yonetirsin?

Dusuk manipulability veya coken Jacobian determinant'i ile tespit ederim; damped least squares, branch selection, trajectory shaping ve constraint-aware motion ile yönetirim.

## Computed Torque Control Nedir?

Computed torque, robot dynamics modelini kullanip nonlinear davranisin buyuk kismini iptal eder. Boylece closed-loop hata dinamigi daha cok lineer ikinci dereceden sisteme benzer.

## OSC'yi Acikla

Operational Space Control dogrudan Cartesian space'te calisir. Once IK cozmek yerine, task-space error'u Jacobian ve task-space inertia uzerinden joint torque'lara cevirir.

## Humanoid veya VLA Ile Baglantisi Ne?

Ust seviye policy'ler genelde Cartesian niyet cikarir. Alt seviye stack ise bu niyeti guvenli ve smooth sekilde uygulamak icin kinematics, dynamics, constraints ve evaluation disiplinine ihtiyac duyar.

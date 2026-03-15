# Lab 1 Notlar Indeksi

Bu klasor Lab 1 icin Turkce calisma notlarini icerir.

## Onerilen Calisma Sirasi

1. [A1: MuJoCo Temelleri](a1_mujoco_temelleri.md)
2. [A2: Forward Kinematics](a2_forward_kinematics.md)
3. [A3: Jacobian](a3_jacobian.md)
4. [A4: Inverse Kinematics](a4_inverse_kinematics.md)
5. [A5: Dynamics Temelleri](a5_dynamics.md)
6. [B1: Trajectory Generation](b1_trajectory_generation.md)
7. [B2: PD Controller](b2_pd_controller.md)
8. [B3: Full Pipeline](b3_full_pipeline.md)
9. [B4: ROS2 Bridge](b4_ros2_bridge.md)
10. [C1: Cartesian Kare Cizimi](c1_draw_square.md)

## Bu Notlar Nasil Kullanilmali

Her modul icin:

1. once bu klasordeki notu oku,
2. sonra eslesen kaynak dosyayi `../src/` altinda ac,
3. notta gecen CSV veya gorsel artifact'i incele,
4. daha uzun anlatim istersen `../blog/` altindaki Ingilizce yaziya gec.

## Onemli Kapsam Notu

Lab 1'de bazi ciktilar kosulludur:

- CSV loglari Python scriptleri dogrudan uretir.
- Plot ve GIF dosyalari `matplotlib` veya Pillow gibi opsiyonel kutuphanelere baglidir.
- MuJoCo'ya ozgu ciktilar icin MuJoCo Python paketi gerekir.

Bu nedenle bir not, scriptin uretebildigi bir artifact'i anlatiyor olabilir; dosya mevcut repo snapshot'inda commit edilmis olmayabilir.

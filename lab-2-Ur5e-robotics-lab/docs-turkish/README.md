# Lab 2 Dokuman Indeksi

Bu klasor, UR5e laboratuvarinin sirasiyla takip edilmesi icin hazirlanmis calisma rotasidir. Notlar, kaynak kodla birlikte okunmak uzere yazildi; koddan kopuk teori ozetleri degiller.

## Baslamadan Once

- Bu notlarin varsayimi, `lab-2-Ur5e-robotics-lab/` klasoru icinde oldugundur.
- Lab artik bagimlilik-hafif degil. Tum kaynak agacinda `numpy` gerekiyor; modullerin cogu ek olarak `mujoco` ve `pinocchio` ister.
- `c2_ros2_bridge.py` ROS 2 yoksa scaffold modunda calisabilir, ancak gercek bridge akisi icin ROS 2 ortami ve ilgili mesaj paketleri yine gerekir.

## Onerilen Calisma Sirasi

| Adim | Not | Ana Script | Ana Cikti |
|---|---|---|---|
| 1 | `a1_environment_setup.md` | `src/a1_model_setup.py` | `a1_environment_status.csv` |
| 2 | `a2_forward_kinematics.md` | `src/a2_forward_kinematics.py` | `a2_fk_validation.csv` |
| 3 | `a3_jacobian.md` | `src/a3_jacobian.py` | `a3_jacobian_validation.csv`, `a3_manipulability_heatmap.csv`, `a3_singularity_cases.csv` |
| 4 | `a4_inverse_kinematics.md` | `src/a4_inverse_kinematics.py` | `a4_ik_benchmark.csv` |
| 5 | `a5_dynamics.md` | `src/a5_dynamics.py` | `a5_dynamics_snapshot.csv` |
| 6 | `b1_trajectory_generation.md` | `src/b1_trajectory_generation.py` | `b1_*.csv` |
| 7 | `b2_control_hierarchy.md` | `src/b2_control_hierarchy.py` | `b2_*.csv` |
| 8 | `b3_constraints.md` | `src/b3_constraints.py` | `b3_*.csv` |
| 9 | `c1_full_pipeline.md` | `src/c1_pick_and_place.py` | `c1_pick_place_log.csv`, `c1_circle_log.csv`, `c1_metrics.csv` |
| 10 | `c2_ros2_bridge.md` | `src/c2_ros2_bridge.py` | sadece calisma zamani ciktisi |
| 11 | `c3_draw_cube.md` | `src/c3_draw_cube.py`, `src/c3_record_video.py` | `media/c3_draw_cube.gif`, `media/c3_draw_cube.mp4` |
| 12 | `d1_vla_bridge.md` | mimari not | yok |
| 13 | `interview_cheatsheet.md` | tekrar notu | yok |
| 14 | `portfolio_package.md` | paketleme notu | yok |

## Kod Haritasi

- Scriptler: `src/`
- Modeller: `models/`
- Medya: `media/`
- Testler: `tests/`
- Uzun blog yazilari: `blog/`

## Cikti Notlari

- Bu klasordeki CSV dosyalari onceki calistirmalardan versionlanmis snapshot'lardir.
- Ana A1-C3 scriptleri yukarida listelenen ciktilari uretir.
- `c1_multi_waypoint_log.csv` veya `c1_singularity_log.csv` gibi ek dosyalar gorebilirsin. Bunlari ana rota degil, yan deneyler olarak dusun.

## Bu Klasor Nasil Kullanilir

1. Modul notunu oku.
2. Eslesen scripti `src/` altinda ac.
3. Ilgili CSV veya medya ciktisini incele.
4. Daha uzun muhendislik anlatimi icin `blog/` altindaki yaziyi oku.
5. Bagimliliklar kurulduktan sonra scripti yeniden calistir.

Bu akis, matematik niyeti, implementasyon detaylari ve kaydedilmis sonuclari ayni cizgide tutar.

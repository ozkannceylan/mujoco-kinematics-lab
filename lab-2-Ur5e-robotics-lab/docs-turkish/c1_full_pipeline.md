# C1: Entegre Pick-and-Place Pipeline

## Hedef

Onceki modulleri tek bir manipulasyon akisinda birlestirmek: Kartezyen waypointleri IK ile cozmek, bunlari duzgun eklem yorungeleriyle baglamak, MuJoCo icinde model-tabanli kontrolle calistirmak ve hem takip hem guvenlik metriklerini loglamak.

## Dosyalar

- Script: `src/c1_pick_and_place.py`
- Ana bagimliliklar: `src/a4_inverse_kinematics.py`, `src/b1_trajectory_generation.py`, `src/b2_control_hierarchy.py`, `src/b3_constraints.py`, `src/mujoco_sim.py`
- Scriptin yazdigi ana ciktIlar:
  - `docs-turkish/c1_pick_place_log.csv`
  - `docs-turkish/c1_circle_log.csv`
  - `docs-turkish/c1_metrics.csv`

Bu klasorde daha eski deneme loglari da olabilir, ancak guncel cekirdek script bu uc dosyayi uretir.

## Implementasyon

### Ana veri yapilari

- `WaypointPose`, isimlendirilmis bir Kartezyen hedefi tutar: pozisyon ve rotasyon.
- `PipelineLog`, zaman, faz adi, end-effector hatasi, tork normu, joint-limit margini ve self-collision skorunu kaydeder.

### Pipeline Adimlari

`run_pipeline()` her demo icin ayni dort adimlik akisi calistirir:

1. **Her waypoint icin IK coz**
   `solve_waypoints()`, `ik_damped_least_squares()` cagirir ve her hedefi onceki cozumle warm-start eder.
2. **Eklem-uzayi yorunge uret**
   `generate_pipeline_trajectory()`, cozulmus eklem waypointleri arasinda quintic segmentler zincirler.
3. **MuJoCo icinde calistir**
   Yorunge `computed_torque_control()` ile takip edilir, sonra `saturate_torques()` ile torklar sinirlanir.
4. **Metrikleri logla**
   Dongu her adimda hem takip kalitesini hem de guvenlik marjlarini kaydeder.

## Demo 1: Pick and Place

Ilk senaryo, home oryantasyonunu sabit tutup alti isimli fazdan gecer:

1. `approach`
2. `pick`
3. `lift`
4. `move`
5. `place`
6. `retreat`

Her segment 1.5 saniyede calistirilir.

## Demo 2: Daire Takibi

Ikinci senaryo Kartezyen bir daire uzerinde sekiz nokta uretir ve baslangica geri doner:

- Merkez: `[0.4, 0.0, 0.50]`
- Yaricap: `0.08 m`
- Segment suresi: `1.0 s`

Bu demo ozellikle faydalidir, cunku IK cozumunun sadece birkac secilmis poza degil, birbirine yakin bircok hedefe karsi da tutarli davranmasini zorlar.

## Neler Loglanir

Adim-bazli loglarda su alanlar vardir:

- `time`
- `phase`
- `ee_error`
- `torque_norm`
- `joint_margin`
- `collision_score`

Ozet CSV ise her demo icin su degerleri saklar:

- RMS end-effector hatasi
- maksimum tork normu
- minimum joint-limit margini
- minimum self-collision mesafesi

## Bu Modul Neden Onemli

C1, labin yalnizca ayrik robotik konular toplulugu olmaktan ciktigi noktadir. Kinematik, dinamik, yorunge uretimi, kontrol ve guvenlik kodu ayni gercek dongude birlikte calismak zorundadir.

Bu nedenle dikkatle incelenmeye deger:

- zincirlenmis IK, cozum surekliligi problemlerini ortaya cikarir,
- quintic yorunge, zamanlama ve interpolasyon varsayimlarini gorunur yapar,
- computed torque, dinamik modelin yeterince iyi olup olmadigini gosterir,
- guvenlik loglari, hareketin sadece dogru mu yoksa kullanilabilir mi oldugunu ayirt eder.

## Nasil Calistirilir

```bash
python3 src/c1_pick_and_place.py
```

## Neye Bakmali

1. Once waypoint listesinden basla ve Kartezyen niyetin eklem-uzayi harekete nasil donustugunu izle.
2. Computed-torque dongusunu satir satir takip et ve A4, A5, B1, B2, B3'un ayni kontrolde nerede birlestigini bul.
3. `ee_error` zaman serisini ciz ve segment gecislerinde sivrilme olup olmadigina bak.
4. `c1_metrics.csv` icindeki `pick_place` ve `circle` satirlarini karsilastir; genelde solver surekliligini daha iyi zorlayan gorev dairedir.
5. Joint marjinlerinin pozitif kaldigini ve collision score'un sifira yaklasmadigini kontrol et.

## Sonraki Adim

C2'ye gecip ayni MuJoCo tabanli labin ROS 2 topic'lerine nasil acildigina bak; ardindan C3 ile final tam-pipeline cizim demosuna gec.

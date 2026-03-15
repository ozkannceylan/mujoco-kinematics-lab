# B2: Kontrol Hiyerarsisi

## Hedef

Ayni UR5e yolu uzerinde dort farkli kontrolor katmanini karsilastirmak ve her birinin ne kazandirdigini anlamak. Tum kontrolorler MuJoCo simUlasyonunda test edilir.

## Dosyalar

- Script: `src/b2_control_hierarchy.py`
- Ciktilar:
  - `b2_controller_summary.csv`
  - `b2_pd_gravity_tracking.csv`
  - `b2_computed_torque_tracking.csv`
  - `b2_task_impedance_tracking.csv`
  - `b2_osc_tracking.csv`

## Implementasyon

### 1. PD + Yercekimi Kompanzasyonu (`pd_gravity_control`)

En basit ve en guvenilir baseline:

```
tau = Kp * (q_des - q) + Kd * (qd_des - qd) + g(q)
```

- `g(q)` Pinocchio ile hesaplanir: `pin.computeGeneralizedGravity()`
- Yercekimi kompanzasyonu olmadan robot kendi agirligina karsi mucadele eder
- Basit ama nonlineer dinamikleri iptal edemez

### 2. Hesaplanmis Tork Kontrolu (`computed_torque_control`)

Ters dinamik tabanli tam model iptal:

```
tau = M(q) * (qdd_des + Kp*(q_des-q) + Kd*(qd_des-qd)) + C*qd + g
```

- `pin.rnea()` ile hesaplanir: RNEA'ya istenen ivme (`a_cmd`) verilir
- Robotun nonlineer davranisinin buyuk kismini iptal eder
- Model dogrulugu ile performans orantili: iyi model = iyi izleme

### 3. Gorev Uzayi Empedans Kontrolu (`task_space_impedance_control`)

Kartezyen uzayda yay-sOnumleyici davranis:

```
F = [Kp_pos * e_pos - Kd_pos * xd_pos; Kp_rot * e_rot - Kd_rot * xd_rot]
tau = J.T @ F + g(q)
```

- Pozisyon hatasi: `e_pos = x_des - x_cur`
- Oryantasyon hatasi: `e_rot = log3(R_des @ R_cur.T)` (log haritasi)
- Jacobian `LOCAL_WORLD_ALIGNED` cercevesinde hesaplanir
- Odak sadece dogruluk degil, uyumlu Kartezyen davranis
- Varsayilan kazanclar: `Kp_pos=200`, `Kd_pos=40`, `Kp_rot=50`, `Kd_rot=10`

### 4. Operasyonel Uzay Kontrolu — OSC (`osc_control`)

Khatib 1987 formUlasyonu ile tam gorev uzayi dinamikleri:

```
Lambda = (J * M^{-1} * J.T)^{-1}   (gorev uzayi ataletI)
F = Lambda * (xdd_des + Kp*e - Kd*xd)
tau = J.T @ F + h(q, qd)            (h = bias kuvvetleri)
```

- `Lambda`, gorev uzayindaki etkin kutle matrisidir
- `h = RNEA(q, qd, 0)` ile Coriolis + yercekimi bias kuvvetleri hesaplanir
- Regularizasyon: `JMinvJt + 1e-6 * I` ile tekillik korunmasi
- Humanoid ve VLA uygulamalarinda en kritik kontrolor tipi

### Simulasyon Dongusu (`run_controller`)

Her kontrolor ayni senaryo ile test edilir:
- Quintic yol: Q_HOME'dan hedef konfigurasyona, 3 saniye, 2 ms adim
- MuJoCo simulasyonu `UR5eSimulator` ile calistirilir
- Tork limiti: `[-150, 150] Nm` clipping
- Loglama: zaman, eklem hatasi normu, tork normu, EE pozisyon hatasi

### Kazanclar

Buyuk eklemler (1-3) icin `Kp=2000, Kd=400`; kucuk eklemler (4-6) icin `Kp=500, Kd=100`.

## Nasil Calistirilir

```bash
python3 src/b2_control_hierarchy.py
```

## Neye Bakmali

1. Sonuc tablosunda dort kontroloru karsilastir: RMS eklem hatasi, RMS EE hatasi, ortalama ve maksimum tork.
2. Hesaplanmis torkunun neden en dusuk izleme hatasini verdIgini anla: tam model iptali.
3. PD + yercekiminin basit ama guvenilir oldugunu gor: temel bir baseline.
4. Empedans kontrolunun dogruluk yerine uyumluluga odaklandigini anla.
5. OSC'nin gorev uzayi ataletini nasil kullandigini incele: `Lambda` matrisi ne ise yariyor?

## Bu Karsilastirmadan Ne Ogrenilir

- **PD + yercekimi**: en basit stabil baseline. Model bilgisi gerektirmez (yercekimi haric).
- **Hesaplanmis tork**: model yardimiyla robotun nonlineer davranisinin buyuk kismini iptal eder. En iyi izleme performansi.
- **Empedans kontrolu**: Kartezyen uzayda tanImlanan uyumlu davranis. Cevre ile etkilesimde onemli.
- **OSC**: ust seviye Kartezyen politikalar ve humanoid/VLA tarafinda en kritik yon. Gorev uzayi dinamiklerini hesaba katar.

## Sonraki Adim

B3'te eklem kisitlamalari ve guvenlik katmani (limit kontrolu, hiz olceklendirme, tork doyumu, kendine carpisma tespiti) eklenir.

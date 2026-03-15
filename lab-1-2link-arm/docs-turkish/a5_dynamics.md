# A5: Dynamics Temelleri — Öğrenme Notu

## Temel Denklem

Robot dynamics denklemi:

```text
tau = M(q) qdd + C(q, qd) qd + g(q)
```

- `tau`: joint torque
- `M(q)`: inertia matrix
- `C(q, qd) qd`: Coriolis + centrifugal etkiler
- `g(q)`: gravity kaynaklı joint torque

MuJoCo bu terimleri adım adım içeride hesaplar. Bu adımın amacı tam türetim yapmak değil, bu büyüklükleri API üzerinden görmektir.

## MuJoCo'da Ne Okuyoruz?

- `data.qfrc_bias`
  - pratikte `C(q,qd) qd + g(q)` terimlerini birlikte verir
- `data.qM`
  - packed inertia matrix verisidir
  - `mj_fullM(...)` ile tam `M(q)` matrisi açılır

## Bu Modelde Gravity Notu

Bu repo içindeki 2-link kol XY düzleminde hareket ediyor ve joint eksenleri `z` yönünde. Bunun sonucu:

- gravity `z` yönünde verilirse joint eksenleri etrafında anlamlı gravity torque üretmeyebilir
- gravity etkisini öğretici biçimde görmek için gravity'yi düzlem içi bir yöne vermek daha uygundur

Bu yüzden `src/a5_dynamics_basics.py` üç durumu karşılaştırır:

1. gravity kapalı
2. `z` yönünde gravity
3. `y` yönünde gravity

## Konfigürasyona Bağlılık

`M(q)` sabit değildir. Linklerin kütle dağılımı ve inertia eksenleri poz değiştikçe farklı görünür. Aynı ivmeyi üretmek için gereken torque bu yüzden poza bağlıdır.

Script iki poz için tam inertia matrix'i karşılaştırır:

- `q = [0°, 0°]`
- `q = [90°, -90°]`

## Calistirma

Lab klasorunden:

```bash
cd lab-1-2link-arm
python3 src/a5_dynamics_basics.py
```

## Cikti Davranisi

Iki farkli mod vardir:

1. `mujoco` ve `numpy` kuruluysa:
   - `docs/a5_bias_cases.csv` uretilir
   - `docs/a5_mass_matrix_cases.csv` uretilir
   - terminale kucuk bir dynamics raporu yazilir
2. Bu bagimliliklar yoksa:
   - script yalnizca kavramsal notlari basar
   - CSV dosyasi uretilmez

Bu ayrim onemli, cunku repo icindeki dokuman sabit kalirken mevcut ortam her zaman MuJoCo tarafini yeniden uretemeyebilir.
Bu nedenle mevcut repo snapshot'inda bu CSV dosyalari bulunmasa bile script bunlari uygun ortamda uretebilir.

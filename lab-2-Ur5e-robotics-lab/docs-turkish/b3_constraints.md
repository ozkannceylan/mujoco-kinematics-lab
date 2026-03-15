# B3: Eklem Kisitlamalari ve Guvenlik

## Hedef

Kontrol komutlarina guvenlik katmani eklemek: eklem limitleri, hiz olceklendirmesi, tork doyumu, kendine carpisma tespiti ve butunlesik guvenli komut filtresi.

## Dosyalar

- Script: `src/b3_constraints.py`
- Ciktilar:
  - `b3_limit_guard.csv`
  - `b3_collision_cases.csv`
  - `b3_stress_test.csv`

## Implementasyon

### 1. Eklem Limiti Kontrolu ve Clamping

**`check_joint_limits(q)`**: Her eklemin alt ve ust limite olan mesafesini hesaplar. Pozitif margin = limit icinde, negatif = ihlal.

**`clamp_joint_positions(q)`**: `np.clip` ile eklem acIlarini limit araligina keser.

**`joint_limit_repulsion(q)`**: Buffer bolgesinde (varsayilan 0.1 rad) yay benzeri itme torku uretir:
- Alt limite yaklasirsa pozitif tork (yukari iter)
- Ust limite yaklasirsa negatif tork (asagi iter)
- `gain * (buffer - mesafe)` formulUyle orantili kuvvet

Bu yaklasim sert kesmeden daha puruzsuzdur: robot limitlere yaklastikca yumusak bir sekilde itilir.

### 2. Hiz Limiti Olceklendirmesi

**`scale_velocity(qd)`**: Herhangi bir eklemin hiz limitini astiginda, tum hiz vektoru orantili olarak kucultulur. Boylece hareket yonu korunur ama buyukluk limitlere uyar.

**`scale_delta_q(dq, dt)`**: Pozisyon artimini (`dq`) zaman adimiyla bolerek ima edilen hizi hesaplar, sonra `scale_velocity` uygular.

### 3. Tork Doyumu

**`saturate_torques(tau)`**: Komutu her eklemin tork limitine (`TORQUE_LIMITS`) gore keser. UR5e icin: buyuk eklemler 150 Nm, kucuk eklemler 28 Nm.

### 4. Kendine Carpisma Tespiti

**`check_self_collision(q)`**: Pinocchio FK ile komsul olmayan link ciftleri arasindaki mesafeleri hesaplar. `min_dist` (varsayilan 5 cm) altinda olan ciftler carpisma olarak raporlanir.

**`self_collision_score(q)`**: Komsul olmayan linkler arasindaki minimum mesafeyi doner. Dusuk deger = carpismaya yakin.

Bu geometrik sezgisel yaklasimdir — tam collision checking icin HPP-FCL veya MuJoCo contact engine tercih edilmelidir.

### 5. Guvenli Komut Filtresi (`safe_command`)

Tum kisitlamalari tek bir fonksiyonda birlestirir:

1. Hiz olceklendirmesi uygula
2. Tork doyumu uygula
3. Eklem limiti itme torkunu ekle
4. Itme torku eklendikten sonra tekrar tork doyumu uygula
5. Sonraki pozisyonu hesapla ve limit ihlali varsa clamping yap

Donus: `(safe_qd, safe_tau, info_dict)` — info sozlugu hangi kisitlarin aktif oldugunu bildirir.

## Demo

Script calistirildiginda alti test bolumu gorulur:
1. Eklem limiti kontrolu: home, limite yakin, limit otesi
2. Hiz olceklendirmesi: limitler icinde ve limitler disinda
3. Tork doyumu: buyuk tork komutu ornegi
4. Kendine carpisma: home, katlanmis, acik konfigurasyonlar
5. Eklem limiti itme torku: limite yakin konfigurasyon
6. Guvenli komut filtresi: tum kisitlamalar birlesik

## Nasil Calistirilir

```bash
python3 src/b3_constraints.py
```

## Neye Bakmali

1. Sert clamping ile yumusak repulsion arasindaki farki anla: ikisi birlikte kullanilir.
2. Hiz olceklemesinin yon koruyan dogasini incele: `max_ratio > 1` durumunda tum vektoru boler.
3. Tork doyum degerlerini UR5e veri sayfasiyla karsilastir: buyuk eklemler 150 Nm, kucuk eklemler 28 Nm.
4. Kendine carpisma tespitinin sinirlarini bil: sadece frame mesafesi kontrolu, gercek geometri degil.
5. `safe_command` fonksiyonundaki kisitlama sIrasini incele: sira onemli (once hiz, sonra tork, sonra itme, sonra pozisyon).

## Yukseltme Yolu

Nihai versiyon icin:
- Kendine carpisma tespiti: Pinocchio + HPP-FCL veya MuJoCo tabanli gercek geometri kontrolune gecilmeli
- Cevre cisimleriyle carpisma kontrolu eklenmeli
- Kisitlamalar optimizasyon tabanli IK icine entegre edilmeli (null-space projection)

## Sonraki Adim

C1'de tum moduller (IK, yol planlama, kontrol, kisitlamalar) entegre bir pick-and-place pipeline'inda birlestirilir.

# C3: 3D Küp Çizimi — Tam Pipeline Demo

## Hedef

UR5e uç işlemcisi ile MuJoCo'da 3 boyutlu bir küpün 12 kenarını çizerek, laboratuvardaki tüm modülleri tek bir demoda birleştirmek. Alt milimetre hassasiyetinde takip performansı hedeflenir.

## Dosyalar

- Etkileşimli demo: `src/c3_draw_cube.py`
- Video kaydedici: `src/c3_record_video.py`
- Çıktı video: `media/c3_draw_cube.mp4`
- Çıktı GIF: `media/c3_draw_cube.gif`

## Pipeline

Demo altı lab modülünü tek bir pipeline'da birleştirir:

```
Küp Geometri → IK (A4) → Quintic Yörünge (B1) → Pozisyon Kontrol + GC + VFF → MuJoCo Sim
                                                          ↑
                                                   FK (A2) + Dinamik (A5)
```

### Aşama 1: Küp Geometrisi

- `cube_vertices()`: Merkez nokta etrafında 8 köşe noktası üretir
- `cube_edges()`: 12 kenarı indeks çiftleri olarak tanımlar
- `cube_drawing_path()`: Tüm 12 kenarı kalemi kaldırmadan sürekli çizen 13 waypointli Euler-benzeri yol

Yapılandırma: merkez = [0.3, 0.2, 0.55], kenar = 10 cm. Y ekseninde öteleme, tüm IK çözümlerinde kolun masayla çarpışmasını önler.

### Aşama 2: Ters Kinematik (A4)

`solve_cube_ik()` her köşe noktasında Sönümlü En Küçük Kareler (DLS) IK kullanır:
- Sabit uç işlemci yönelimi (home yönelimi)
- Sıralı tohum: her çözüm bir sonrakini tohumlar, düzgün eklem-uzayı geçişleri sağlar
- Tolerans: 1e-4 (her noktada < 0.1 mm IK hatası)

### Aşama 3: Yörünge Üretimi (B1)

`build_cube_trajectory()` 12 quintic polinom segmentini zincirler (her biri 2.0 s, toplam 24 s). Quintic polinomlar segment sınırlarında sıfır hız ve ivme garanti eder.

### Aşama 4: İleri Besleme ile Pozisyon Kontrolü

MuJoCo Menagerie UR5e modeli dahili PD pozisyon servolarına sahiptir:

```
tau = Kp * (ctrl - qpos) - Kd * qvel
```

Naif pozisyon kontrolü (`ctrl = q_desired`) iki sorundan muzdariptir:
1. **Yerçekimi düşmesi**: Servo yerçekimini pozisyon hatası üzerinden yenmelidir → kalıcı offset
2. **Takip gecikmesi**: Servo hareketli referansı hıza orantılı gecikmeyle takip eder

Çözüm: ctrl sinyaline iki ileri besleme terimi eklemek:

```python
ctrl = q_des + g(q) / Kp + Kd * qd_des / Kp
```

- **Yerçekimi telafisi (GC)**: `g(q) / Kp` — servonun pozisyon hatası yerçekimini tam olarak karşılayacak torku üretir. `data.qfrc_bias`'tan okunur.
- **Hız ileri beslemesi (VFF)**: `Kd * qd_des / Kp` — sönümleme teriminin istenen hıza karşı direncini iptal eder.

Efektif kontrol yasası:
```
tau ≈ Kp * (q_des - qpos) - Kd * (qvel - qd_des) + g(q)
```

### Yerleşme

Yörünge başlamadan önce robot ilk waypointte 3 saniye GC-telafili ctrl ile bekler.

## Sonuçlar

| Metrik | Değer |
|---|---|
| RMS Kartezyen hata | 0.088 mm |
| Maks Kartezyen hata | 0.234 mm |
| Maks aktüatör torku | 16.50 N·m |
| Toplam süre | 24.0 s |
| Video kare sayısı | 707 |

## Önemli Dersler

1. **Masa çarpışması önemli**: Orijinal küp [0.4, 0.0, 0.50] konumunda üst kol ve ön kol linklerinin masa yüzeyiyle (z ≈ 0.41 m) çarpışmasına neden oldu. Temas kuvvetleri robotu istenen yörüngeden itti (133 mm RMS hata). Küpü [0.3, 0.2, 0.55]'e taşımak tüm çarpışmaları ortadan kaldırdı.

2. **GC + VFF pozisyon servo için şart**: İleri besleme olmadan hata çarpışma olmasa bile 13+ mm idi. Yalnızca GC ile biraz azaldı. Hız ileri beslemesi ekleyince RMS 0.1 mm'nin altına düştü — orijinalden 1500x iyileşme.

3. **Aktüatör doyması düşman**: PD servo kuvvet aralığının izin verdiğinden fazla tork talep ettiğinde (eklem 1-3 için 150 Nm), robot takip otoritesini kaybeder. Doğru ileri besleme aktüatör kuvvetlerini sınırlar içinde tutar (16.5 Nm tepe).

## Nasıl Çalıştırılır

```bash
# Etkileşimli görüntüleyici (ekran gerektirir)
python3 src/c3_draw_cube.py

# Video kaydet (ekransız)
python3 src/c3_record_video.py
```

## Neye Bakmalı

1. `cube_drawing_path()` fonksiyonunu okuyun ve Euler geçişini kağıt üzerinde takip edin.
2. `ctrl = q_des` (naif) ile `ctrl = q_des + g/Kp + Kd*qd/Kp` (GC+VFF) karşılaştırın — kayıt scriptini değiştirip her ikisini deneyin ve RMS'leri karşılaştırın.
3. `CUBE_CENTER`'ı [0.4, 0.0, 0.50] yapıp masa çarpışmasının takibi nasıl bozduğunu gözlemleyin.
4. Farklı `SEGMENT_DURATION` değerleri deneyerek yörünge hızının ileri besleme ile ve olmadan takibi nasıl etkilediğini görün.

## Sonraki Adım

Bu demo UR5e Robotik Laboratuvarını tamamlar. A1'den C3'e tüm modülleri gözden geçirin ve bunları daha ileri projeler (VLA entegrasyonu, MoveIt2 planlama, vb.) için yapı taşları olarak kullanın.

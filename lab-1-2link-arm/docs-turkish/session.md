# Lab 1 Durum Ozet Notu

Bu dosya, Lab 1'in mevcut durumunu kisa ve guncel bir not olarak tutar.

## Saglam Olan Kisimlar

- `C1` kare cizim gorevi implement edildi ve dokumante edildi.
- Nihai takip gorevinin ana sayisal sonucu:
  - RMS Cartesian hata: **0.008 mm**
  - Max Cartesian hata: **0.013 mm**
- Lab su alanlari birlikte iceriyor:
  - `src/` altinda kaynak kod
  - `docs/` altinda Ingilizce notlar
  - `docs-turkish/` altinda Turkce notlar
  - `blog/` altinda uzun anlatim yazilari
  - `media/` altinda final medya ciktilari

## Onemli Teknik Notlar

- MuJoCo modellerinde `compiler angle="radian"` kullaniliyor; bu sayede joint-limit birim hatasi engelleniyor.
- Final demo su parcalara dayaniyor:
  - quintic Cartesian trajectories
  - analytic IK
  - Jacobian tabanli velocity mapping
  - computed torque control
- Bazi artifact'ler kosulludur:
  - MuJoCo'ya bagli CSV'ler icin `mujoco` gerekir
  - plot ve GIF icin `matplotlib` veya uygun writer gerekebilir

## Sonraki Mantikli Adimlar

- daire veya cokgen gibi ek cizim gorevleri eklemek
- daha zor trajectory'lerde PD ile computed torque'u karsilastirmak
- ROS2 entegrasyonu oncelik kazanirsa bridge yapisini daha resmi paketlemek

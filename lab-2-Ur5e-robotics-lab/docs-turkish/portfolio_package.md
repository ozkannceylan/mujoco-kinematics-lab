# Portfolio Package Notlari

## Guncel Guclu Yanlar

- tam lab-2 klasor yapisi mevcut
- yerel URDF ve MJCF varliklari repo icinde
- her faz scripti somut CSV artefact'lari uretiyor
- Ingilizce ve Turkce calisma notlari paralel hazir

## Guncel Sayisal Ozet

- A4 IK benchmark: `16 / 20` basarili reachable-pose cozum
- B2 computed torque RMS EE error: `0.00358264`
- C1 singularity stress RMS EE error: `0.00369054`
- C1 demolari mevcut log'larda sifir collision sample gosteriyor

## Public Showcase Icin Hala Eksik Olanlar

1. MuJoCo ve Pinocchio'yu kurup tum cross-check metriklerini yeniden uret.
2. Branch-seeded IK scaffold'ini exact analytical UR5e IK ile degistir.
3. Video kaydindan once C1 genel hareket tracking kalitesini artir.
4. `media/` altina render ciktilarini ekle.
5. ROS 2 bridge'i gercek bir ROS 2 runtime'a bagla.

## Daha Sonraki README Yapisi Icin Oneri

1. Overview
2. Why UR5e + MuJoCo + Pinocchio
3. Architecture diagram
4. Key metrics table
5. Demo clips
6. How to run
7. Folder structure
8. VLA / humanoid connection

## Bu Not Nasil Kullanilmali

Bu dosyayi mevcut study-first repo durumuyla son portfolio-ready surum arasindaki gap listesi gibi dusun.

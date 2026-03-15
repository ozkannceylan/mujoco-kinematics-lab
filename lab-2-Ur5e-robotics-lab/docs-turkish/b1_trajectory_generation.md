# B1: Yol Planlama

## Hedef

UR5e icin eklem uzayinda ve Kartezyen uzayda durumlastirma referans yollari uretmek. Kontrolore fiziksel olarak uygulanabilir referanslar saglamak.

## Dosyalar

- Script: `src/b1_trajectory_generation.py`
- Ciktilar:
  - `b1_cubic_joint_traj.csv`
  - `b1_quintic_joint_traj.csv`
  - `b1_trapezoidal_joint_traj.csv`
  - `b1_cartesian_traj.csv`
  - `b1_waypoint_traj.csv`

## Implementasyon

Bes yol planlama yontemi implement edilir:

### 1. Kubik Polinom (`cubic_trajectory`)

Uc noktalarda sifir hiz kosulu ile 3. derece polinom:
- `q(t) = a0 + a1*t + a2*t^2 + a3*t^3`
- Sinir kosullari: `qd(0) = 0`, `qd(T) = 0`
- Katsayilar: `a2 = 3*dq/T^2`, `a3 = -2*dq/T^3`
- Cikti: pozisyon, hiz ve ivme

### 2. Quintic Polinom (`quintic_trajectory`)

Uc noktalarda sifir hiz VE sifir ivme kosulu ile 5. derece polinom:
- `q(t) = a0 + a3*t^3 + a4*t^4 + a5*t^5`
- Sinir kosullari: `qd(0) = qd(T) = 0`, `qdd(0) = qdd(T) = 0`
- Katsayilar: `a3 = 10*dq/T^3`, `a4 = -15*dq/T^4`, `a5 = 6*dq/T^5`
- Kubikten daha puruzsuz: ivme sureksizligi yok

### 3. Trapez Hiz Profili (`trapezoidal_trajectory`)

Uc fazli profil: ivmelenme, sabit hiz, yavasalama.
- Her eklem icin `v_max` ve `a_max` limitleri alinir
- Eger mesafe kisa ise ucgen profil olusur (sabit hiz fazsi olmaz)
- **Senkronizasyon**: tum eklemler ayni anda biter — en yavas eklemin suresi referans alinir
- Gercekci: fiziksel hiz ve ivme sinirlarini direkt uygular

### 4. Minimum Jerk Kartezyen Yol (`minimum_jerk_trajectory`)

Kartezyen uzayda (sadece pozisyon) minimum jerk profili:
- `s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5`
- Baslangic ve bitis noktalarini duz cizgide birlestirir
- s(t) parametrizasyonu puruzsuzdur: hiz, ivme ve jerk baslangic/bitiste sifir

### 5. Coklu Segment Via-Point Yolu (`multi_segment_trajectory`)

Birden fazla ara noktayi (waypoint) quintic segmentlerle zincirler:
- Her segment icin ayri quintic polinom hesaplanir
- Zaman kaymalari (`t_offset`) dogru sekilde toplanir
- Coklu segment pipeline (C1) icin temel yapi tasini olusturur

Her yol noktasi `JointTrajectoryPoint` dataclass'i ile ifade edilir: `t`, `q` (pozisyon), `qd` (hiz), `qdd` (ivme).

## Demo

Script calistirildiginda:
- Home'dan hedef konfigurasyona bes yontemle yol uretilir
- Her yol icin nokta sayisi, sure, baslangic/bitis hiz/ivme degerleri yazdirilir
- UR5e hiz limitlerine karsi uygunluk kontrolu yapilir (kubik, quintic, trapez)

## Nasil Calistirilir

```bash
python3 src/b1_trajectory_generation.py
```

## Neye Bakmali

1. Kubik yolda baslangic ve bitis hizlarinin sifir oldugunu dogrula.
2. Quintic yolda hem hiz hem ivmenin uc noktalarda sifir oldugunu dogrula — bu onu kubikten ustun kilar.
3. Trapez profilinde maksimum hizin `v_max` limitini asmadigini kontrol et.
4. Uygunluk kontrolunda hicbir yolun UR5e hiz limitlerini ihlal etmedigini dogrula.
5. Kubik ve quintic katsayi formullerini elle turet ve koda karsilastir.

## Neden Kontrolden Once Geliyor

Kotu bir kontrolor, kotu bir referans yolunu kurtaramaz. Hiz sureksizligi olan bir referansta kontrolor sonsuza yakin torklar uretmeye calisir. B1, kontrolore fiziksel olarak mantikli, puruzsuz referanslar vererek B2'nin basarisini mumkun kilar.

## Sonraki Adim

B2'de kontrol hiyerarsisi (PD + yercekimi, hesaplanmis tork, empedans, OSC) ile bu yollari MuJoCo'da takip etmeye gecilir.

# A3: Jacobian Analizi

## Hedef

Joint hizlarini end-effector twist'ine baglayan Jacobian matrisini uc farkli yontemle hesaplamak, manipulabilite analizi yapmak ve UR5e'nin tekillik bolgelerini tespit etmek.

## Dosyalar

- Script: `src/a3_jacobian.py`
- Ciktilar:
  - `a3_jacobian_validation.csv`
  - `a3_manipulability_heatmap.csv`
  - `a3_singularity_cases.csv`

## Implementasyon

Script dort ana bolumden olusur:

### 1. Jacobian Capraz Dogrulama (Geometrik vs Pinocchio vs Sayisal)

Uc farkli yontemle 6x6 Jacobian hesaplanir ve birbirleriyle karsilastirilir:

**Geometrik Jacobian** (`geometric_jacobian`): Pinocchio FK frame verisinden kurulur. Her revolute eklem icin:
- `J_linear[:, i] = z_i x (p_ee - p_i)` (dogrusal kisim)
- `J_angular[:, i] = z_i` (acisal kisim)

Burada `z_i`, eklemin dunya cercevesindeki donus eksenidir. Pinocchio modelindeki gercek eksen tipi (`RX`, `RY`, `RZ`) kontrol edilir — her eklem icin z ekseni varsayilmaz.

**Pinocchio yerlesik Jacobian** (`pinocchio_jacobian`): `pin.computeFrameJacobian()` fonksiyonu ile `LOCAL_WORLD_ALIGNED` referans cercevesinde hesaplanir.

**Sayisal Jacobian** (`numerical_jacobian`): Merkezi sonlu farklar ile hesaplanir (`eps=1e-6`). Her eklem icin `q[i] +/- eps` perturbasyonu uygulanir. Pozisyon turevi basit fark bolumu ile, acisal turevi ise `pin.log3(R_plus @ R_minus.T) / (2*eps)` ile hesaplanir.

Bes konfigurasyonda (zeros, home, elbow_open, wrist_flip, random) uc yontem arasindaki Frobenius hata normu `1e-4` altinda olmalidir.

### 2. Manipulabilite Analizi

**Yoshikawa manipulabilite indeksi**: `w = sqrt(det(J @ J.T))`. Bu skaler deger, robotun bir konfigurasyonda ne kadar becerikli oldugunu olcer. Yuksek degerler daha iyi dexterity demektir.

`jacobian_determinant()` fonksiyonu da dogrudan `det(J)` hesaplar; kare Jacobian icin `det(J) = 0` tekillik noktasini gosterir.

Home konfigurasyonunda tekil deger ayristirmasi (SVD) yapilir ve kosul sayisi raporlanir.

### 3. Tekillik Tespiti

`detect_singularity()` fonksiyonu UR5e'nin uc klasik tekillik tipini kontrol eder:

- **Bilek tekilligi**: `q5 ~ 0` (bilek eksenleri 4 ve 6 hizalanir, ~2.9 derece esik)
- **Omuz tekilligi**: EE'nin xy mesafesi taban z-ekseninden `< 0.02 m`
- **Dirsek tekilligi**: `q3 ~ 0` veya `q3 ~ pi` (kol tamamen acik)

Ayrica genel manipulabilite esigi kontrolu yapilir (`w < 1e-3`).

### 4. Manipulabilite Isi Haritasi

`q2` ve `q3` eksenleri uzerinde 25x25'lik bir grid taranir, diger eklemler `Q_HOME` degerinde sabit tutulur. Her noktada manipulabilite ve determinant hesaplanir. Sonuclar `a3_manipulability_heatmap.csv` dosyasina kaydedilir. En iyi/en kotu dexterity noktalari ve tekilligi yakin noktlarin yuzdesi raporlanir.

## Nasil Calistirilir

```bash
python3 src/a3_jacobian.py
```

## Neye Bakmali

1. Jacobian karsilastirma tablosunda tum satirlarin `[OK]` oldugunu dogrula (Frobenius hata `< 1e-4`).
2. Home konfigurasyonunda tam 6x6 Jacobian matrisini incele: hangi eklem hangi Kartezyen yone en cok katkida bulunuyor?
3. Tekil deger ayristirmasinda `sigma_6`'nin sifira ne kadar yakin oldugunu kontrol et — bu en zayif yonu gosterir.
4. Tekillik tespiti tablosunda `wrist_singularity` ve `elbow_extended` satirlarinin dogru algsilandigini dogrula.
5. Isi haritasinda (`a3_manipulability_heatmap.csv`) tekilligi yakin bolgelerinin oranini incele.

## Mulakat Ozeti

Tekillik, robotun en az bir Kartezyen yonde hareket kabiliyetini kaybettigi konfigurasyondur. Jacobian'in rank dusur, `det(J) -> 0` olur. Pratikte bu, istenen Kartezyen hiz icin joint hizlarinin sonsuza yaklasmasina ve kontrol sinyallerinin patlamasina yol acar. Dampeli En Kucuk Kareler (DLS) veya tekillik-robust IK yontemleri bu sorunu hafifletir.

## Sonraki Adim

Jacobian ve tekillik anlayisi ile birlikte A4'te Ters Kinematik (IK) asamasina gec.

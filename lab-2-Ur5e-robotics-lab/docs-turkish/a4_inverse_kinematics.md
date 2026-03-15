# A4: Ters Kinematik

## Hedef

Hedef EE pozisyon ve oryantasyonunu UR5e eklem konfigurasyonlarina cevirmek. Iki sayisal IK yontemi implement edilir, FK geri donus dogrulamasi yapilir ve 50 rastgele hedef uzerinde benchmark kosuturulur.

## Dosyalar

- Script: `src/a4_inverse_kinematics.py`
- Cikti: `a4_ik_benchmark.csv`

## Implementasyon

### 1. Jacobian Pseudo-Inverse IK (`ik_pseudoinverse`)

Tam 6-DOF IK: hem pozisyon hem oryantasyon hedeflenir.

- 6B hata vektoru: `e = [p_target - p_current; log3(R_target @ R_current.T)]`
- Adim: `dq = pinv(J) @ e`, sonra `q = q + alpha * dq`
- Eklem limitleri her adimda `np.clip` ile uygulanir
- Varsayilan parametreler: `max_iter=200`, `tol=1e-4`, `alpha=0.5`

### 2. Dampeli En Kucuk Kareler (DLS) IK (`ik_damped_least_squares`)

Tekilliklere karsi dayanikli versiyon. Uyarlanabilir damping faktoru `lambda` kullanir:

- `dq = J.T @ (J @ J.T + lambda^2 * I)^{-1} @ e`
- Hata azalirsa `lambda` yarilir (daha agresif adimlar)
- Hata artarsa `lambda` ikiye katlanir (daha muhafazakar)
- `lambda` araligi: `[1e-4, 1.0]`, baslangic: `0.1`
- Varsayilan: `max_iter=300`, `tol=1e-4`, `alpha=0.5`

### 3. FK Geri Donus Dogrulamasi (`validate_ik_solution`)

IK cozumunu Pinocchio FK ile dogrular:
- Bulunan `q_solution` ile FK hesaplanir
- Pozisyon hatasi ve oryantasyon hatasi olculur
- Pozisyon toleransi: 1 mm, oryantasyon toleransi: 0.01 rad

### 4. MuJoCo Capraz Dogrulamasi (`apply_ik_in_mujoco`)

Pinocchio ile cozulen IK sonucu MuJoCo'ya aktarilir:
- Bulunan eklem acilari MuJoCo'da ayarlanir, `mj_forward` calistirilir
- MuJoCo'dan okunan EE pozisyonu Pinocchio sonucu ile karsilastirilir
- 1.0 mm altinda fark beklenir

### 5. Benchmark (50 Rastgele Hedef)

`generate_random_targets()` fonksiyonu rastgele eklem konfigurasyonlarindan FK ile erislebilir hedefler uretir (`seed=42`). Her hedef icin:
- Perturbe edilmis baslangic tahmini kullanilir (gercek cozum degil)
- Hem pseudo-inverse hem DLS ile cozulur
- Basari orani, ortalama iterasyon, ortalama sure, ortalama pozisyon/oryantasyon hatasi raporlanir
- DLS cozumleri icin FK geri donus dogrulamasi yapilir

Sonuclar `IKResult` dataclass'inda doner: `success`, `q`, `iterations`, `final_error`, `position_error`, `orientation_error`, `elapsed_s`.

## Nasil Calistirilir

```bash
python3 src/a4_inverse_kinematics.py
```

## Neye Bakmali

1. Tekli hedef demosunda pseudo-inverse ve DLS sonuclarini karsilastir: DLS genelde daha gucludur.
2. MuJoCo dogrulamasinda Pinocchio/MuJoCo EE uyumsuzlugunun 1 mm altinda oldugunu dogrula.
3. Benchmark tablosunda iki yontemin basari oranini ve ortalama iterasyon sayisini karsilastir.
4. FK geri donus dogrulamasinda gecerli cozum yuzdesi %90 uzerinde olmali.
5. `alpha` adim buyuklugunu ve `lambda` uyarlama mekanizmasini anla: bunlar convergence hizi ile kararlilik arasindaki dengeyi belirler.

## Sonraki Adim

IK ile hedef pose'lari eklem konfigurasyonlarina cevirebiliyoruz. A5'te dinamik (kutle matrisi, yercekimi, Coriolis, RNEA, ABA) asamasina gec.

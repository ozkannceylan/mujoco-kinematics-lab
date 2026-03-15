# A5: Dinamik

## Hedef

Pinocchio'nun analitik rigid-body algoritmalari ile UR5e'nin tam dinamik modelini kurmak ve MuJoCo ile capraz dogrulama yapmak.

## Dosyalar

- Script: `src/a5_dynamics.py`
- Wrapper: `src/mujoco_sim.py` (`UR5eSimulator`)
- Cikti: `a5_dynamics_snapshot.csv`

## Implementasyon

Script yedi analiz bolumu icerir:

### 1. Kutle Matrisi M(q)

`pin.crba(model, data, q)` ile hesaplanir (Composite Rigid Body Algorithm). CRBA sadece ust ucgeni doldurdugu icin simetrize edilir: `M = triu(M) + triu(M, k=1).T`.

Her konfigurasyonda analiz edilir:
- Simetrilik kontrolu: `M == M.T` (atol=1e-10)
- Pozitif tanimlilik: tum ozdegerler > 0
- Kosul sayisi: `cond(M)` — yuksek degerler numerik zorlugu gosterir
- Diyagonal elemanlar: her eklemin etkin ataleti

### 2. Yercekimi Vektoru g(q)

`pin.computeGeneralizedGravity(model, data, q)` ile hesaplanir. Sonuc, robotun o konfigurasyonda hareketsiz kalmasi icin gereken torktur. En buyuk bileseni ve hangi ekleme karsilik geldigi raporlanir.

### 3. Coriolis Matrisi C(q, v)

`pin.computeCoriolisMatrix(model, data, q, v)` ile hesaplanir. Sifirdan farkli bir hiz vektoru (`v_test`) ile test edilir.

**Carpiklik-simetri kontrolu**: `M_dot - 2C` matrisinin anti-simetrik olmasi gerekir (`N + N.T ~ 0`). Bu, enerji korunum ozelliginin saglandigini kanitlar. `M_dot`, sonlu farkla yaklastirilir.

### 4. RNEA — Ters Dinamik

`pin.rnea(model, data, q, v, a)` ile hesaplanir: `tau = M*a + C*v + g`.

`v=0, a=0` durumunda RNEA'nin yercekimi vektorune esit oldugu dogrulanir: `RNEA(q, 0, 0) == g(q)`.

### 5. ABA — Ileri Dinamik

`pin.aba(model, data, q, v, tau)` ile hesaplanir (Articulated Body Algorithm): `qdd = M^{-1} * (tau - C*v - g)`.

Sifir tork altinda yercekimi ivmesi hesaplanir: hangi eklemler ne kadar hizla ivmeleniyor? En buyuk ivmelenme genelde joint 2 ve 3'te gorulur.

Dogrulama: `ABA == M^{-1} * (tau - g)` esitligi (v=0 durumunda) `atol=1e-8` ile kontrol edilir.

### 6. Capraz Dogrulama: Pinocchio RNEA vs MuJoCo qfrc_bias

`v=0, a=0` durumunda Pinocchio RNEA yercekimi torklerini verir. Ayni konfigurasyonda MuJoCo'dan `qfrc_bias` okunur (v=0'da bu da yercekimine esittir). Bes konfigurasyonda karsilastirilir.

Beklenen sonuc: `< 0.1 Nm` maksimum hata. Bu, iki motorun ayni fiziksel modeli temsil ettigini kanitlar.

### 7. Kosul Sayisi Taramasi

Tum test konfigurasyonlarinda `M(q)` kosul sayisi, minimum ve maksimum ozdeger raporlanir. Yuksek kosul sayisi, o konfigurasyonda dinamik kontrolun numerik olarak daha zor oldugunu gosterir.

## Test Konfigurasyonlari

- `zeros`: tum eklemler sifir
- `home`: UR5e home pozisyonu
- `elbow_up`: dirsek yukari
- `shoulder_only`: sadece omuz donmus
- `random`: rastgele bir konfigurasyon

## Nasil Calistirilir

```bash
python3 src/a5_dynamics.py
```

## Neye Bakmali

1. Kutle matrisinin her konfigurasyonda simetrik ve pozitif tanimli oldugunu dogrula.
2. Yercekimi vektorunun en buyuk bilesenleri joint 2 ve 3'te olmali (en agir linkler).
3. Carpiklik-simetri hatasinin `~0` oldugunu kontrol et (`||N + N.T||_F`).
4. RNEA ile g(q) esitligini dogrula: `Match: True`.
5. ABA dogrulamasinda `Match: True` oldugundan emin ol.
6. Capraz dogrulamada Pinocchio ve MuJoCo arasindaki maksimum hatayi kontrol et: `< 0.1 Nm` olmali.

## Sonraki Adim

Dinamik model tamamlandiginda B1'de yol planlama (trajectory generation) asamasina gec. Dinamik bilgi kontrolor tasariminda (B2) kritik rol oynayacak: hesaplanmis tork kontrolu M, C ve g'ye dogrudan baglidir.

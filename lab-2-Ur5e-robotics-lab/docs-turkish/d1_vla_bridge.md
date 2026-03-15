# D1: VLA Kopru Notlari

## Neden Bu Dokuman Var

UR5e lab'i sadece manipulator egzersizi degil. Humanoid ve VLA sistemlerinde tekrar kritik olan katmanlari kompakt sekilde guclendiren bir calisma:

- kinematics tutarliligi,
- stabil Cartesian control,
- constraint handling,
- logging ve evaluation disiplini.

## VLA Calismasina Dogrudan Baglantilar

### 1. Policy Output Katmani

Bir VLA policy genelde niyeti task space'te ifade eder:

- target pose,
- target waypoint,
- target contact region,
- target sub-goal.

Bu dogrudan A4, B1 ve B2'ye baglanir.

### 2. Controller Katmani

Ogrenilmis policy tek basina yeterli degil.

Hala ihtiyacin olan seyler:

- IK veya OSC,
- limit handling,
- collision check,
- stabil low-level execution.

Bu da dogrudan B2 ve B3'e baglanir.

### 3. Veri ve Degerlendirme Katmani

Lab seni su metrikleri loglamaya zorluyor:

- pose error,
- torque magnitude,
- manipulability,
- constraint margin.

Ayni disiplin policy evaluation tarafina da tasinmali.

## Bu Lab'in VLA Tarafina Onerdigi Uc Iyilestirme

1. Her learned policy output icin daha guclu analytical baseline tut.
2. Sadece task success degil, constraint margin'leri de acikca logla.
3. Singularity ve low-dexterity maruziyetini birinci sinif metrik yap.

## Mevcut Repo Durumundan Cikan Dersler

- computed torque, mevcut modelde basit PD baseline'ini geciyor
- pipeline collision olmadan uc uca calisiyor
- IK kullanisli ama polished demo seviyesine gelmeden once ana kalite darbozazi olmaya devam ediyor

## Sonra Ne Eklenmeli

UR5e stack'inin geri kalani kuruldugunda bu notu su bilgilerle guncelle:

- exact Pinocchio-vs-MuJoCo karsilastirmalari,
- render edilmis demolar,
- daha genis humanoid/VLA projene daha net baglanti.

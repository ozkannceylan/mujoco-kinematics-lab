# A2: Forward Kinematics

## Hedef

Bu adımın amacı, **UR5e robotunun 6 eklem açısını** alıp bunları **3B uzayda end-effector pose’una** nasıl çevirdiğimizi net biçimde anlamaktır.

Buradaki en kritik nokta şudur:

* 2-link düzlemsel robotta yalnızca `(x, y)` düşünüyorduk.
* UR5e’de artık **3 boyutlu uzay**, **6 dönel eklem**, ve **pozisyon + yönelim** birlikte düşünülmek zorunda.

Yani artık yalnızca robot kolunun ucunun **nereye geldiğini** değil, aynı zamanda **hangi doğrultuya baktığını** da hesaplıyoruz.

Bu yüzden Forward Kinematics (FK), robotikte “eklem uzayından Kartezyen uzaya geçiş”in temel taşıdır.

---

## Neden Basit Geometri Yetmiyor?

2-link bir robotta trigonometri ile ilerlemek mümkündü:

* birinci link için `cos(theta1), sin(theta1)`
* ikinci link için `cos(theta1+theta2), sin(theta1+theta2)`

Bu yaklaşım küçük örneklerde çok öğreticidir. Ancak UR5e gibi 6-DOF bir robotta:

* omuz döner,
* dirsek bükülür,
* bilek kendi eksenlerinde yeniden yönlenir,
* bütün bunlar 3B uzayda üst üste biner.

Burada tek tek geometri kurmaya çalışmak çok hızlı şekilde kaotik hale gelir. Denklem sayısı büyür, eksenler sürekli değişir, hangi dönüşün hangi sırada uygulandığını takip etmek zorlaşır.

Bu problemi çözmek için robotikte standart bir çerçeve kullanılır:

**Denavit-Hartenberg (DH) parametreleri**.

DH yaklaşımının büyük gücü şudur:

> Robot ne kadar karmaşık olursa olsun, her eklemi bir öncekine göre standart birkaç parametre ile tanımlayıp bütün zinciri sistematik şekilde kurabiliriz.

---

## Ana Dosyalar

* Script: `src/a2_forward_kinematics.py`
* Ortak matematik: `src/ur5e_common.py`
* Çıktı: `a2_fk_validation.csv`

Bu lab’da asıl iş bölümü şöyledir:

* `ur5e_common.py` robotun ortak geometrik/matematiksel altyapısını taşır.
* `a2_forward_kinematics.py` bu altyapıyı kullanarak FK zincirini kurar ve test eder.
* `a2_fk_validation.csv` ise farklı joint konfigürasyonları için çıkan sonuçları görmeyi sağlar.

---

## Forward Kinematics Tam Olarak Ne Üretiyor?

Forward kinematics’in girdisi şudur:

```text
q = [q1, q2, q3, q4, q5, q6]
```

Buradaki her `qi`, ilgili eklemin açısını temsil eder.

FK’in çıktısı ise tipik olarak bir **4x4 homojen dönüşüm matrisi**dir:

```text
T_base_to_ee
```

Bu matris bize tek seferde iki şeyi verir:

1. **Pozisyon**

   * End-effector’un taban koordinat sistemine göre `(x, y, z)` konumu
2. **Yönelim**

   * End-effector’un uzayda nasıl döndüğü

Matris yapısı şöyledir:

```text
[ R  p ]
[ 0  1 ]
```

Burada:

* `R` = `3x3` rotasyon matrisi
* `p` = `3x1` pozisyon vektörü

Yani bu tek matris, robot ucunun hem **nerede** olduğunu hem de **nasıl durduğunu** söyler.

---

## DH Parametreleri: Her Eklem İçin 4 Temel Tanım

DH yaklaşımında her eklem bir öncekine göre dört parametre ile tanımlanır:

### 1. `theta` — Joint Angle

Eklemin kendi dönüş açısıdır.

* Revolute joint için değişken olan parametre budur.
* Motor encoder’ından okunan ana değer budur.
* Tipik olarak `z` ekseni etrafındaki dönüşü temsil eder.

### 2. `d` — Link Offset

Bir çerçeveden bir sonrakine, `z` ekseni boyunca olan ötelenmedir.

* Dikey ya da eksen doğrultusundaki mesafe gibi düşünülebilir.
* Fiziksel montaj offset’lerini temsil eder.

### 3. `a` — Link Length

`x` ekseni boyunca olan mesafedir.

* Bir eklemden sonrakine uzanan efektif kol boyu gibi düşünülebilir.
* Robot link geometrisinin önemli bir parçasıdır.

### 4. `alpha` — Twist Angle

Bir sonraki eklemin `z` ekseninin, mevcut `z` eksenine göre ne kadar eğik olduğunu söyler.

* `x` ekseni etrafındaki dönüş olarak tanımlanır.
* Özellikle 3B robotlarda eksenler birbirine paralel değilse kritik hale gelir.

Bu dört parametre sayesinde, karmaşık bir uzaysal robot bile disiplinli bir zincir yapısına dönüştürülür.

---

## Bu Repo’daki Kritik Detay: Düz Standard DH Değil

Teoride birçok kaynak “standard DH” veya “modified DH” diye anlatır. Ama pratikte projeler bunların birebir ders kitabı sürümünü değil, bazen **uygulamaya daha uygun hibrit ya da sabit dönüşüm temelli** versiyonlarını kullanır.

Bu repo’daki implementasyon notu özellikle önemli:

```text
T_fixed = Rot_x(alpha) * Trans_x(a) * Trans_z(d)
T_joint = T_fixed * Rot_z(theta)
```

Yani burada zincir şu mantıkla kuruluyor:

1. Önce linkin sabit geometrik kısmı uygulanıyor:

   * `Rot_x(alpha)`
   * `Trans_x(a)`
   * `Trans_z(d)`
2. Sonra eklemin değişken açısı uygulanıyor:

   * `Rot_z(theta)`

Bu, klasik DH tablosunun ruhunu koruyan ama implementasyon açısından daha açık bir ayrımdır:

* **sabit geometri** ayrı,
* **eklem hareketi** ayrı.

Bu ayrım özellikle kodu okurken çok faydalıdır çünkü sana şu soruyu net cevaplatır:

* Bu transform robotun fiziksel gövde geometrisinden mi geliyor?
* Yoksa o anda eklem açısı değiştiği için mi geliyor?

Robotik yazılımında bu ayrımı görebilmek ciddi bir mühendislik refleksidir.

---

## `ur5e_common.py` İçinde Ne Var?

Bu dosya, UR5e için ortak kinematik yapı taşlarını taşır. Genellikle burada şu tip içerikler olur:

* DH satırları veya eşdeğer link parametreleri
* rotasyon ve translasyon matris fonksiyonları
* zincir kurulumunda kullanılan yardımcı fonksiyonlar
* orientation dönüşümleri (`rotation_to_rpy` gibi)
* tool offset veya flange offset tanımları

Bu dosya aslında robotun “kinematik sözlüğü” gibidir.

Bir URDF olmasa bile, robotun matematiksel iskeletini bu tip ortak dosyada taşırız.

---

## Zinciri Kurmak: `forward_chain()` Ne Yapıyor?

Forward kinematics’in özü zinciri sırayla çarpmaktır.

Her eklem için bir transform matrisi üretiriz:

```text
T_1, T_2, T_3, T_4, T_5, T_6
```

Sonra bunları sırayla birleştiririz:

```text
T_base_to_ee = T_1 * T_2 * T_3 * T_4 * T_5 * T_6
```

Eğer en sonda tool offset varsa onu da ekleriz:

```text
T_base_to_tool = T_1 * T_2 * T_3 * T_4 * T_5 * T_6 * T_tool
```

Buradaki mantık çok önemlidir:

* Her yeni eklem, koordinat sistemini yeniden tanımlar.
* Sonraki dönüşüm artık öncekinin oluşturduğu yeni frame üzerinde uygulanır.
* Bu yüzden çarpım sırası kritiktir.

Matris çarpımı burada yalnızca cebir değil, fiziksel anlam taşır:

> “Önce omuz dönsün, sonra dirsek kendi yeni eksenine göre dönsün, sonra bilek onun üstüne binsin.”

Yanlış sırada çarparsan, fiziksel olarak bambaşka bir robot tanımlamış olursun.

---

## Homojen Dönüşüm Matrisinin Fiziksel Yorumu

FK sonucu olan 4x4 matrisin parçalarını fiziksel olarak yorumlayabilmek gerekir.

### Pozisyon

Matrisin sağ üst kısmındaki sütun vektörü:

```text
[x, y, z]^T
```

Bu, end-effector’un tabana göre uzaydaki konumudur.

Örneğin home pose için mevcut repo’da elde edilen sonuç:

```text
(-0.8172, -0.3679, 0.0628) m
```

Bu sayı sana şunu söyler:

* robot ucu tabana göre x ekseninde ne kadar ileride/geride,
* y ekseninde ne kadar yanda,
* z ekseninde ne kadar yukarıda/aşağıda.

### Yönelim

Matrisin sol üst `3x3` kısmı rotasyon matrisidir.

Bu matris, end-effector frame’inin eksenlerinin base frame’e göre nasıl döndüğünü söyler.

Bu çok kritik çünkü robotik uygulamalarda çoğu zaman yalnızca konum yetmez.

Örnek:

* bir parçayı yukarıdan alacaksan tool aşağı bakmalı,
* vidalama yapacaksan eksen hizası doğru olmalı,
* kaynak veya lehim yapacaksan belirli bir açı korunmalı.

Yani aynı `(x, y, z)` pozisyonunda olup tamamen yanlış yöne bakıyor olabilirsin. FK bunu da hesaplar.

---

## Roll–Pitch–Yaw Neden Çıkarılıyor?

Kod çıktılarında genellikle yalnızca matris değil, daha okunabilir orientation formatları da bulunur. Bunlardan en yaygını:

* roll
* pitch
* yaw

Yani `rotation_to_rpy` benzeri bir fonksiyon, `3x3` rotasyon matrisini daha insan okunur üç açıya çevirir.

Bu faydalıdır çünkü:

* CSV’de sonuçları hızlı yorumlarsın,
* konfigürasyonlar arasında karşılaştırma yaparsın,
* orientation değişimini sezgisel takip edersin.

Ama burada önemli bir mühendislik notu var:

RPY yalnızca yorum kolaylığı sağlar. Matematiksel olarak asıl sağlam temsil:

* rotasyon matrisi,
* quaternion,
* veya eksen-açı gibi daha güvenli formatlardır.

Çünkü Euler/RPY gösterimleri belirli açılarda singularity ve yorum zorluğu doğurabilir.

Yine de lab seviyesinde ve hızlı doğrulamada RPY çok faydalıdır.

---

## Tool Offset Neden En Son Ekleniyor?

Robotun son mekanik frame’i çoğu zaman doğrudan fiziksel tool ucu değildir.

Genelde şu yapı vardır:

* son eklem frame’i,
* flange frame’i,
* tool center point (TCP)

Bu yüzden FK zincirinin sonunda ek bir sabit dönüşüm uygulanır:

```text
T_tool
```

Bu transform, son link frame’inden gerçek tool merkezine geçişi temsil eder.

Pratikte bu çok önemlidir çünkü:

* simülasyonda gördüğün mesh ucu ile
* kontrol etmek istediğin gerçek operasyon noktası
  aynı şey olmayabilir.

TCP yanlış tanımlanırsa robot doğru görünür ama işi yanlış noktada yapar.

---

## Güncel Sonuç

Yerel FK implementasyonundan elde edilen home pose end-effector pozisyonu:

```text
(-0.8172, -0.3679, 0.0628) m
```

Bu sonuç, mevcut kinematik zincirin en azından sayısal olarak bir pose ürettiğini ve zincirin tutarlı çalıştığını gösterir.

Ama burada önemli bir uyarı var:

Bu tek başına “model kesin doğru” demek değildir.

Çünkü bir FK implementasyonu:

* tutarlı olabilir,
* ama eksen konvansiyonu hatalı olabilir,
* link offset’i yanlış olabilir,
* tool offset’i yanlış tanımlanmış olabilir,
* base frame yönü simülatörle uyuşmuyor olabilir.

Bu yüzden bir sonraki aşama her zaman **harici doğrulama**dır.

---

## `a2_fk_validation.csv` Neyi Tutar?

CSV şu alanları içerir:

* test edilen joint konfigürasyonu,
* Cartesian pozisyon,
* roll, pitch, yaw,
* opsiyonel MuJoCo ve Pinocchio pozisyon hata kolonları.

Bu dosya çok değerlidir çünkü FK yalnızca “bir fonksiyon çalıştı” meselesi değildir. Farklı pozlarda ne olduğunu gözle görmek gerekir.

CSV sayesinde şu tür sorulara bakabilirsin:

* eklem 2 değişince uç nokta nasıl hareket ediyor?
* bilek eklemleri orientation’ı mı değiştiriyor, pozisyonu mu daha çok etkiliyor?
* home, ready ve over_table gibi pozlar mantıklı mı?
* bazı konfigürasyonlarda beklenmedik sıçrama var mı?

Bu tür tablolar robotik yazılımında hata ayıklamanın temel araçlarından biridir.

---

## MuJoCo ve Pinocchio Kolonları Neden Önemli?

CSV’de opsiyonel olarak MuJoCo ve Pinocchio ile karşılaştırma kolonları düşünülmüş.

Mevcut repo durumunda bu kolonlar boş kalıyor çünkü ilgili harici kütüphaneler kurulu değil.

Ama bu tasarım çok doğru bir mühendislik yaklaşımı gösteriyor.

Çünkü manuel FK yazmak bir şeydir, onu endüstri-standardı bir referansa doğrulatmak başka bir şeydir.

### Neden Pinocchio?

Pinocchio, robotik kinematik ve dinamik için çok güçlü bir kütüphanedir. Özellikle:

* URDF tabanlı modelleme,
* hızlı kinematik hesaplar,
* Jacobian ve dynamics işlemleri

konusunda ciddi bir referanstır.

### Neden MuJoCo?

MuJoCo ise simülasyon motoru olarak yalnızca görselleştirme yapmaz; modelin geometrik ve fiziksel tutarlılığı için de iyi bir karşılaştırma zemini sunar.

Eğer manuel FK çıktın ile MuJoCo/Pinocchio çıktısı yakınsa, şu güven oluşur:

* frame tanımların büyük oranda doğru,
* link uzunlukların mantıklı,
* eksen sıralaman simülasyon modeliyle uyumlu.

Bu yüzden ideal durumda hedef, manuel FK ile referans model arasındaki farkın çok küçük olmasıdır.

Milimetre seviyesinde hata görmek genelde iyi bir işarettir.

---

## DH vs URDF: Endüstride Hangisi Gerçekten Kullanılıyor?

Bu nokta çok önemli.

Öğrenme açısından DH parametreleri çok güçlüdür çünkü sana robotun matematiksel zincirini öğretir. Ama modern robotik yazılım ekosisteminde çoğu zaman doğrudan DH ile çalışılmaz.

Bunun yerine genellikle:

* **URDF**
* ve onun üzerinde çalışan kütüphaneler
  kullanılır.

### DH’nin avantajı

* öğreticidir,
* sistematik zincir kurdurur,
* matris mantığını oturtur,
* türev alırken Jacobian’a geçişi kolaylaştırır.

### DH’nin sınırı

* eklem eksenleri ve frame yerleşimi daha kısıtlıdır,
* bazen fiziksel robotu doğrudan ifade etmek yerine yapay frame’ler kurmak gerekir,
* karmaşık mekanik düzeneklerde doğal ifade gücü sınırlanır.

### URDF’nin avantajı

* daha serbest ifade sunar,
* gerçek link/joint yapısını daha doğal tarif eder,
* ROS2, MoveIt2, Pinocchio, MuJoCo gibi modern araçlarla uyumludur.

Bu yüzden doğru zihinsel model şudur:

> DH öğrenmek, robotik matematiğin temelini kurar. URDF öğrenmek ise bu matematiği gerçek yazılım ekosistemine bağlar.

Senin mevcut çalışmanda manuel FK ile modern araçları kıyaslama fikri bu yüzden çok değerlidir. Bu, akademik anlayış ile endüstriyel doğrulamayı aynı yerde buluşturur.

---

## `forward_chain()` Fonksiyonunu Nasıl Okumalısın?

Bu kodu anlamak için satır satır şu gözle ilerle:

### 1. Başlangıç frame’i nedir?

Genelde zincir identity matris ile başlar:

```text
T = I
```

Bu, “henüz hiçbir dönüşüm uygulanmadı” demektir.

### 2. Her eklem için hangi sabit parametreler geliyor?

İlgili `alpha`, `a`, `d` değerlerini bul.

Bunlar robotun geometrik iskeletidir.

### 3. Hangi değişken joint açısı ekleniyor?

O satırda kullanılan `theta`, o anki joint state’ten gelir.

### 4. Birikimli matris nasıl güncelleniyor?

Tipik mantık:

```text
T = T @ T_joint
```

Burada `@` matris çarpımıdır.

### 5. Tool transform var mı?

Varsa en son eklenir.

### 6. En sonda hangi parçalar dışarı veriliyor?

* pozisyon
* rotasyon matrisi
* RPY
* validation satırı

Bu bakış açısıyla fonksiyonu okuduğunda kod yalnızca “numpy işlemleri” olmaktan çıkar; fiziksel robot hareketine dönüşür.

---

## Home, Ready, Over-Table Gibi Pozları Neden Çizmelisin?

Dokümandaki öneri çok doğru:

1. `ur5e_common.py` içindeki DH satırlarını oku.
2. `forward_chain()` fonksiyonunu adım adım izle.
3. `a2_fk_validation.csv` dosyasında konfigürasyon değiştikçe pose’un nasıl değiştiğini incele.
4. `home`, `ready` ve `over_table` durumlarını çiz.

Bu son madde özellikle önemli. Çünkü FK’i gerçekten anlamanın yolu sadece formülü görmek değil, konfigürasyon uzayı ile fiziksel uzay arasındaki ilişkiyi kafada canlandırmaktır.

Örneğin:

* `home` daha nötr bir başlangıç pozu olabilir,
* `ready` operasyon öncesi güvenli bir bekleme pozu olabilir,
* `over_table` çalışma alanına uzanmış bir görev pozu olabilir.

Bu pozları çizersen şunları sezgisel olarak anlarsın:

* hangi eklemler reach üzerinde baskın,
* hangi eklemler orientation için daha kritik,
* bilek eklemleri pozisyonu ne kadar değiştiriyor,
* robotun çalışma hacmi nasıl şekilleniyor.

Bu sezgi Jacobian ve singularity konularına geçerken çok işine yarar.

---

## Mühendislik Açısından Bu Lab’ın En Büyük Kazancı

Bu çalışma sana üç temel refleks kazandırır:

### 1. Joint space ile task space arasındaki ilişkiyi görmek

Robot controller eklem açılarıyla çalışır.

Ama kullanıcı çoğu zaman şunu ister:

* “ucu masanın üstüne götür”
* “gripper aşağı baksın”
* “şu noktaya şu açıyla yaklaş”

FK, bu iki dünya arasındaki ilk köprüdür.

### 2. Frame düşünmeyi öğrenmek

Robotikte her şey frame’dir:

* base frame
* joint frame
* tool frame
* world frame

FK çalışırken aslında koordinat sistemlerini birbirine zincirliyorsun.

Bunu kavrayan biri robotik yazılımı gerçekten okumaya başlar.

### 3. Simülasyon ile matematik arasında çapraz doğrulama yapmak

Kendi elinle yazdığın FK ile MuJoCo/Pinocchio gibi araçları karşılaştırmak, seni yalnızca kod kullanan biri olmaktan çıkarır.

Bu seni:

* model doğrulayan,
* frame hatası yakalayan,
* eksen konvansiyonu sorgulayan,
* mühendisçe güven oluşturan
  bir profile taşır.

---

## Önemli Sınır

Bu modül, **lab 2 için repo başlangıç noktasıdır**; henüz nihai, endüstri seviyesi, tam doğrulanmış model değildir.

Bu yüzden şu sınır açıkça kabul edilmelidir:

* Buradaki FK yapısı öğretici ve işlevseldir.
* Ama kesin referans model muamelesi görmemelidir.
* İlk ciddi doğrulama adımı, MuJoCo ve Pinocchio kurulduğunda bu modülün sayısal karşılaştırmasını yapmak olmalıdır.

Yani bu aşama “temel doğru mu?” sorusunun başlangıcıdır; son cevabı değil.

---

## Özet

Bu adımda yaptığımız şey, UR5e’nin 6 joint açısını sistematik bir kinematik zincir üzerinden end-effector pose’una çevirmekti.

Ana fikirler:

* 2D geometri artık yeterli değil, 3D matris temelli düşünmek gerekiyor.
* Her eklem, DH benzeri parametrelerle tanımlanıyor.
* Repo’da sabit geometri ve eklem dönüşü ayrıştırılmış bir transform yapısı kullanılıyor.
* Tüm eklem matrisleri sırayla çarpılarak base’ten end-effector’a gidiliyor.
* Sonuç yalnızca pozisyon değil, yönelimi de içeriyor.
* CSV çıktısı, farklı joint konfigürasyonlarının pose üzerindeki etkisini gözlemlemeyi sağlıyor.
* MuJoCo ve Pinocchio ile yapılacak karşılaştırma, bu manuel kinematiğin ilk ciddi doğrulama adımı olacak.

Kısacası:

> Forward kinematics, robot kolunun “eklem açıları” dilini, fiziksel dünyanın “konum ve yönelim” diline çeviren mekanizmadır.

Bu temel sağlam olmadan Jacobian, inverse kinematics, singularity, velocity control ve task-space control gibi ileri başlıklara güvenli geçiş yapılamaz.

---

## Sonraki Mantıklı Adım

Buradan sonraki doğal adım:

* **Jacobian (6x6)**
* hız kinematiği
* singularity analizi

Çünkü FK bize “robot şu açılarda nerededir?” sorusunu cevaplar.

Jacobian ise bir üst seviyeye geçer ve şunu sorar:

> “Joint’leri çok az oynatırsam end-effector hızı ve yönelimi nasıl değişir?”

İşte bu geçiş, robotu yalnızca pozlayan değil, gerçekten kontrol etmeye başlayan mühendislik eşiğidir.

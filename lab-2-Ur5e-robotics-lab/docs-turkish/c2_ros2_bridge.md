# C2: ROS 2 Koprusu

## Hedef

MuJoCo UR5e simulasyonunu ROS 2 ekosistemIne baglamak. Joint durumlarini yayinlamak, EE pozunu yayinlamak ve dis sistemlerden eklem komutlari almak.

## Dosyalar

- Script: `src/c2_ros2_bridge.py`

## Implementasyon

### Topic Yapisi

| Topic | Mesaj Tipi | Yon | Aciklama |
|-------|-----------|-----|----------|
| `/joint_states` | `sensor_msgs/JointState` | Yayinla | Eklem pozisyonlari ve hizlari |
| `/ee_pose` | `geometry_msgs/Pose` | Yayinla | End-effector pozisyon ve oryantasyon |
| `/joint_commands` | `std_msgs/Float64MultiArray` | Dinle | Tork komutlari (6 eleman) |

### ROS 2 Node: `UR5eBridgeNode`

- `UR5eSimulator` wrapper'ini kullanir (ayni MuJoCo backend)
- 500 Hz zamanlayici ile calisir
- Her dongude:
  1. Gelen komutu aktUatore uygular (`set_ctrl`)
  2. Simulasyonu bir adim ilerletir (`step`)
  3. Durumu okur (`get_state`)
  4. `/joint_states` mesaji olusturup yayinlar (isim, pozisyon, hiz)
  5. `/ee_pose` mesaji olusturup yayinlar (pozisyon + quaternion)

### Quaternion Donusumu

EE oryantasyonu `scipy.spatial.transform.Rotation` ile rotasyon matrisinden quaternion'a cevrilerek `geometry_msgs/Pose` icine yazilir.

### Demo Modu (ROS 2 Olmadan)

`rclpy` bulunamazsa script temiz sekilde cikar ve bridge tasarimini aciklar. Ardindan bagimsiz bir simulasyon dongusu calistirir:
- `UR5eSimulator` olusturulur, `Q_HOME` ayarlanir
- 1000 adim simule edilir
- Baslangic ve bitis EE pozisyonlari ve simulasyon zamani yazdirilir

Bu sayede ROS 2 kurulu olmayan ortamda bile scriptler calisabilir ve bridge API'si gorunur kalir.

### Konfigurasyon

`BridgeConfig` dataclass'i ile ayarlanir:
- `joint_state_topic`: `/joint_states`
- `ee_pose_topic`: `/ee_pose`
- `joint_command_topic`: `/joint_commands`
- `rate_hz`: `500.0`

## Nasil Calistirilir

ROS 2 kurulu ise:
```bash
source /opt/ros/humble/setup.bash
python3 src/c2_ros2_bridge.py
```

ROS 2 kurulu degilse demo modu otomatik calisir:
```bash
python3 src/c2_ros2_bridge.py
```

## Neye Bakmali

1. Topic isimlerini ve mesaj tiplerini incele: bunlar ROS 2 standart konvansiyonlari.
2. 500 Hz zamanlayicinin MuJoCo adim suresi ile nasil eslestIgini anla.
3. Komut dinleme mekanizmasini oku: `_on_command` fonksiyonu gelen veriyi nasil islIyor?
4. Quaternion donusumune dikkat et: `scipy` kullaniliyor, Pinocchio/MuJoCo konvansiyonu degil.
5. Demo modunun bos ortamda bile calisabilir olmasini dogrula.

## ROS 2 ile Test

ROS 2 kurulu ortamda:

```bash
# Terminal 1: Bridge'i baslat
source /opt/ros/humble/setup.bash
python3 src/c2_ros2_bridge.py

# Terminal 2: Joint state'leri dinle
ros2 topic echo /joint_states

# Terminal 3: Basit komut gonder
ros2 topic pub /joint_commands std_msgs/Float64MultiArray "{data: [0,0,0,0,0,0]}"
```

## Sonraki Adim

Bridge calisiyor oldugunda MoveIt2 entegrasyonu ile motion planning eklenir. Uzun vadede bu yapiyi gercek UR5e donanimi ile de ayni interface uzerinden kullanmak mumkun.

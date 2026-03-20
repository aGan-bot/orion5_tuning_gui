# orion5_tuning_gui

Orion5 icin ROS 2 tabanli tuning GUI araci.

Bu paket, `gravity_comp_relay_node` parametrelerini (q_ref, kp, kd, gain_vector)
canli olarak degistirmek ve ayni anda joint durumlari ile torklari izlemek icin
hazirlanmistir.

## Ozellikler

- 6 eksen icin slider tabanli tuning
- Parametre yazma: `q_ref`, `kp`, `kd`, `gain_vector`
- Parametre okuma: `Refresh From Node`
- Home preset: `0, 0, -90, 0, 90, 0` (deg)
- Canli izleme:
  - `q(rad)`
  - `qd(rad/s)`
  - `bias_tau` (`/mujoco/bias_torque`)
  - `cmd_tau` (`/effort_controller/commands`)
- Local profil kaydetme/yukleme:
  - Dosya: `~/.ros/orion5_tuning_last.json`

## Bagimliliklar

- ROS 2 Humble
- `python3-pyqt5`
- `rclpy`, `rcl_interfaces`, `sensor_msgs`, `std_msgs`

Ornek kurulum:

```bash
sudo apt update
sudo apt install -y ros-humble-rclpy ros-humble-rcl-interfaces \
  ros-humble-sensor-msgs ros-humble-std-msgs python3-pyqt5
```

## Build

Workspace kokunden:

```bash
colcon build --symlink-install --packages-select orion5_tuning_gui
source install/setup.bash
```

## Calistirma

Sadece GUI node:

```bash
ros2 run orion5_tuning_gui orion5_tuning_gui
```

Simulasyon + kontrol + GUI birlikte:

```bash
ros2 launch orion5_tuning_gui orion5_tuning_gui.launch.py
```

`orion5_tuning_gui.launch.py`, altta `mujoco_pendulum/launch/orion5_gravity_comp.launch.py`
launch dosyasini include eder.

## Kullanim Akisi

1. `Refresh From Node` ile mevcut parametreleri cek.
2. Slider'lari degistir.
3. `Apply Arrays` ile node'a gonder.
4. Son ayarlar otomatik local profile kaydedilir.
5. Sonraki acilista `Load Last Local` ile geri yuklenebilir.

## Beklenen Konular

- GUI'nin parametre yazmasi icin `/gravity_comp_relay_node` calisir durumda olmali.
- Topic isimleri sabittir:
  - `/joint_states`
  - `/effort_controller/commands`
  - `/mujoco/bias_torque`

## Dosya Yapisi

- `orion5_tuning_gui/tuning_gui.py`: Ana GUI ve ROS bridge
- `launch/orion5_tuning_gui.launch.py`: Sim + GUI ortak launch


# ackermann_car

## Модуль с моделированием мобильного робота с Аккерманн кинематикой
### Ссылки на источники (литература и репо)
```
https://workabotic.com/2025/ackermann-steering-vehicle-simulation/
https://github.com/lucasmazzetto/gazebo_ackermann_steering_vehicle?tab=readme-ov-file
```

### Для сборки и запуска пакета нужно ROS2 (Humble or Jazzy) + Gazebo (Ignition or Harmonic). Я только на этих попробовал запустить.
```bash
sudo apt update
sudo apt install -y \
     ros-jazzy-ros2-controllers \
     ros-jazzy-gz-ros2-control \
     ros-jazzy-ros-gz \
     ros-jazzy-ros-gz-bridge \
     ros-jazzy-joint-state-publisher \
     ros-jazzy-robot-state-publisher \
     ros-jazzy-xacro
# Заменить "jazzy" на "humble" если надо

cd ~/your_ws/src
git clone https://github.com/Edgar-dotcom02/ackermann_car.git
cd ..
colcon build --packages-select car_interfaces
source install/setup.bash
colcon build --symlink-install --packages-select ackermann_car\
source install/setup.bash
```
### В одном терминале
```bash
ros2 launch ackermann_car vehicle.launch.py
```
### В другом терминале, для запуска миссии
```bash
ros2 action send_goal /follow_path car_interfaces/action/FollowPath "{}"
```
### ВАЖНО! Не забыть изменить параметры для сохранения или загрузки csv файлы траектории.

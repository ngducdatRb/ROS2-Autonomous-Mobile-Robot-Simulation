# Giải thích
## Nội dung

- [Tổng quan](#tổng-quan)
- [World](#world)
- [Scene](#scece-hiển-thị)
- [Physics](#physics-vật-lý)
- [ Plugin Physics](#plugin-physic)
- [Plugin User Commands ](#plugin-user-commands)
- [Plugin Sensors](#plugin-sensors)
- [Plugin Scene Broadcaster](#plugin-scene-broadcaster)
- [Plugin IMU](#plugin-imu)
- [Light](#light-mặt-trời)
- [Ground](#ground-plane-mặt-đất)

## Tổng quan
File này là 1 file SDF (Simulation Description Format) dùng trong Gazebo/Ignition Gazebo để mô tả 1 world (môi trường mô phỏng)

```xml
<sdf version="1.9">
```

- Sử dụng chuẩn SDF version 1.9
- Đây là format XML dùng để mô tả vật lý, mô hình, sensor, plugin, ...

## World
```xml
<world name="demo">
```

- Định nghĩa một thế giới mô phỏng tên là `demo`
- Mọi thứ (physics, light, model, plugin, ...) đều nằm ở đây

## Scece (hiển thị)
```xml
<scene>
    <shadows>0</shadows>
</scene>
```
- `shadows = 0`: tắt đổ bóng → giúp tăng hiệu năng

## Physics (vật lý)
```xml
<physics name="1ms" type="ignored">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
</physics>
```
| Tham số | Ý nghĩa |
| :--- | :--- |
| `max_step_size` | bước thời gian mô phỏng (0.001s = 1ms) |
| `real_time_factor` | tốc độ chạy (1.0 = realtime) |
| `type="ignored"` | Ignition sẽ override (ghi đè) bằng plugin Physics |

## Plugin Physic 
```xml
<plugin filename="ignition-gazebo-physics-system"
        name="gz::sim::systems::Physics">
</plugin>
```
- Plugin này thực thi engine vật lý
- Thay thế cho `physics type` ở trên

## Plugin User Commands
```xml
<plugin filename="ignition-gazebo-user-commands-system"
        name="gz::sim::systems::UserCommands">
</plugin>
```
- Cho phép spawn model
- Cho phép delete model
- Cho phép điều khiển world runtime

## Plugin Sensors
```xml
<plugin filename="libignition-gazebo-sensors-system.so"
        name="ignition::gazebo::systems::Sensors">
    <render_engine>ogre2</render_engine>
</plugin>
```
Vai trò:
- Kích hoạt hệ thống sensor (camera, lidar, imu, ...)
- Dùng render engine: `orge2`

Nếu không có plugin này:
- Camera / Lidar sẽ không hoạt động

## Plugin Scene Broadcaster
```xml
<plugin filename="ignition-gazebo-scene-broadcaster-system"
        name="gz::sim::systems::SceneBroadcaster">
</plugin>
```
- Broadcast trạng thái world ra ngoài
- Dùng cho:
    - GUI
    - visualization tool

## Plugin IMU
```xml
<plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu">
</plugin>
```
- Xử lý sensor IMU
    - Gia tốc
    - Vận tốc góc

## Light (mặt trời)
```xml
<light name="sun" type="directional">
```
| Tag | Ý nghĩa |
| :--- | :--- |
| `type="directional"` | ánh sáng song song (giống mặt trời) |
| `pose` | vị trí nguồn sáng |
| `diffuse` | màu ánh sáng |
| `specular` | phản xạ |
| `direction` | hướng chiếu |

## Ground Plane (mặt đất)
```xml
<model name="ground_plane">
    <static>true</static>
```
- Model tĩnh (không bị ảnh hưởng vật lý)

### Collision
```xml
<collision name="collision">
```
- Dùng cho vật lý (va chạm)

```xml
<plane>
    <normal>0 0 1</normal>
    <size>100 100</size>
</plane>
```
- Mặt phẳng 100m x 100m

### Visual
```xml
<visual name="visual">
```
- Dùng để render (hiển thị)

```xml
<material>
    <ambient>0.8 0.8 0.8 1</ambient>
```
- Màu xám sáng
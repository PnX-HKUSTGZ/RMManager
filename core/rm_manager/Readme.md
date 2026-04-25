# rm_manager

这是一个与裁判系统同通信的节点，可以通过这个节点向裁判系统发送消息，并且从裁判系统接受消息

> 无雷达链路（没写）

## 资料

[RMU 2026 通信协议](https://bbs.robomaster.com/wiki/20204847/811363?source=7)

## 参数设置

所有参数均为可选项，如果不指定则使用默认值。

### 图传链路参数
- **`image_port`** (string, 默认值: `/dev/ttyImage`)
  - 图传链路使用的串口设备名称
  - 示例: `/dev/ttyUSB0` 或 `/dev/ttyUSB1`
  - 如果不使用图传链路，设置为 `None` 或 `""` (空字符串)

### 裁判系统链路参数
- **`referee_port`** (string, 默认值: `/dev/ttyRef`)
  - 裁判系统使用的串口设备名称
  - 示例: `/dev/ttyUSB0` 或 `/dev/ttyUSB1`
  - 如果不使用裁判系统链路，设置为 `None` 或 `""` (空字符串)

### 启动示例

```bash
# 使用默认串口
ros2 run rm_manager rm_manager

# 指定自定义串口
ros2 run rm_manager rm_manager --ros-args -p image_port:=/dev/ttyUSB0 -p referee_port:=/dev/ttyUSB1

# 仅使用图传链路
ros2 run rm_manager rm_manager --ros-args -p referee_port:=None

# 仅使用裁判系统链路
ros2 run rm_manager rm_manager --ros-args -p image_port:=None
```

### 参数配置文件示例

在 `params.yaml` 中配置参数：

```yaml
rm_manager:
  ros__parameters:
    image_port: "/dev/ttyUSB0"
    referee_port: "/dev/ttyUSB1"
```

然后启动：

```bash
ros2 run rm_manager rm_manager --ros-args --params-file params.yaml
```

## 消息传递

`rm_manager` 当前只对“机器人入站”协议做 typed topic 解析，其他合法但不属于机器人入站的数据会继续发布到原始观测 topic。

### 原始观测

- `/<node_name>/all_receive_data`
  - 串口每次 `read()` 收到的原始字节块，`cmd_id = 0xFFFF`
- `/<node_name>/all_messages`
  - 所有通过标准 `0xA5` 帧校验的数据，按 `GeneralMessage` 转发
- `/<node_name>/unknown_command`
  - 合法标准帧，但当前链路下不属于机器人入站 typed 协议的数据

### 裁判系统串口 typed topics

- `game_status`, `game_result`, `robot_hp`
- `field_events`, `referee_warning`, `dart_info`
- `robot_status`, `buffer_heat`, `robot_position`, `robot_buffs`
- `hurt_event`, `shoot_data`, `ammo_allowance`, `rfid_status`, `dart_cmd`
- `ground_positions`, `radar_mark`, `sentry_decision`, `radar_decision`
- `robot_interaction`, `map_downlink`

### 图传串口 typed topics

- `remote_control`
  - legacy/private 兼容输入，来源于旧图传私有遥控帧 `0xA9 0x53`
  - 单独发布，不进入 `all_messages`
- `custom_controller`
- `client_custom_command`
- `set_vtm_channel`
- `query_vtm_channel`

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

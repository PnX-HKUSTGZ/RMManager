# rm_custom_controller_imu

用于接收和处理裁判系统自定义控制器 IMU 数据的 ROS 2 包。

## 功能概述

该包从裁判系统接收自定义控制器消息（30字节数据），根据数据包头识别并处理两种不同类型的数据：

### ControlData1 (0xA1) - 四元数 + 位置数据
- **数据结构**：包含四元数 (qw, qx, qy, qz) 和位置坐标 (pos_x, pos_y, pos_z)
- **功能**：
  - 发布 TF2 变换，将位置和姿态信息广播到 TF 树
  - 同时发布对应的 PoseStamped 消息
- **特性**：支持位置坐标的独立缩放（X/Y/Z轴）

### ControlData2 (0xA2) - 通道 + GPIO 数据
- **数据结构**：包含4个通道 (channel_0~3, int8类型) 和 GPIO 状态字节
- **功能**：
  - 发布 Twist 命令（geometry_msgs/TwistStamped）
  - 发布 GPIO 状态（8个独立的 std_msgs/Bool topic）
- **通道映射**：可配置每个通道映射到 twist 的不同轴（linear_x, linear_y, angular_z）
- **归一化**：通道值自动从 int8 [-128, 127] 归一化到 [-1.0, 1.0]，然后乘以配置的最大值

## 数据协议

### ControlData1 结构 (30字节)
```c
typedef struct {
    uint8_t header;   // 0xA1 (1字节)
    float qw;         // 四元数实部 (4字节)
    float qx;         // 四元数 i 分量 (4字节)
    float qy;         // 四元数 j 分量 (4字节)
    float qz;         // 四元数 k 分量 (4字节)
    float pos_x;      // X坐标 (4字节)
    float pos_y;      // Y坐标 (4字节)
    float pos_z;      // Z坐标 (4字节)
    uint8_t reserved; // 保留 (1字节)
} ControlData1;
```

### ControlData2 结构 (30字节)
```c
typedef struct {
    uint8_t header;      // 0xA2 (1字节)
    int8_t channel_0;    // 通道0 - 左摇杆水平 (1字节)
    int8_t channel_1;    // 通道1 - 左摇杆垂直 (1字节)
    int8_t channel_2;    // 通道2 - 右摇杆水平 (1字节)
    int8_t channel_3;    // 通道3 - 右摇杆垂直 (1字节)
    uint8_t gpio_state;  // GPIO状态 - bit0和bit1对应左右按钮 (1字节)
    uint8_t reserved[24]; // 填充 (24字节)
} ControlData2;
```

## 参数配置

### 通用参数
- `ref_topic` (string)：裁判系统自定义控制器消息的 topic
- `watchdog_timeout` (double, 默认: 1.0)：看门狗超时时间（秒）

### ControlData1 (TF2 与 Pose) 参数
- `parent_frame` (string)：TF2 父坐标系名称 / Pose 消息的 frame_id
- `child_frame` (string)：TF2 子坐标系名称
- `pose_topic` (string, 默认: "~/pose")：Pose 消息发布的 topic
- `position_scale_x` (double, 默认: 1.0)：X轴位置缩放系数
- `position_scale_y` (double, 默认: 1.0)：Y轴位置缩放系数
- `position_scale_z` (double, 默认: 1.0)：Z轴位置缩放系数

### ControlData2 (Twist) 参数
- `enable_twist_cmd` (bool, 默认: true)：是否启用 Twist 命令发布
- `twist_cmd_topic` (string, 默认: "~/twist_cmd")：Twist 命令 topic
- `channel_mapping` (string_array, 长度: 4)：通道映射配置
  - 可选值：`"linear_x"`, `"linear_y"`, `"angular_z"`, `"none"`
  - 默认：`["linear_x", "linear_y", "angular_z", "none"]`
- `channel_max` (double_array, 长度: 4)：每个通道的最大值（归一化后的缩放系数）
  - 默认：`[1.0, 1.0, 1.0, 1.0]`

## 发布的 Topics

### ControlData1 发布
- **TF2 变换**：从 `parent_frame` 到 `child_frame` 的变换
- `~/pose` (geometry_msgs/PoseStamped)：位置和姿态信息（可配置 topic 名称）

### ControlData2 发布
- `~/twist_cmd` (geometry_msgs/TwistStamped)：底盘速度命令（可配置）
- `~/gpio0_state` (std_msgs/Bool)：GPIO bit 0 状态
- `~/gpio1_state` (std_msgs/Bool)：GPIO bit 1 状态
- `~/gpio2_state` ~ `~/gpio7_state`：其他 GPIO 状态

## 订阅的 Topics

- `ref_topic` (rm_message/msg/CustomController)：裁判系统自定义控制器数据

## 使用示例

### 1. 构建包
```bash
cd /home/pnx/code/RMManager
colcon build --packages-select rm_custom_controller_imu
source install/setup.bash
```

### 2. 配置参数文件
编辑 `test/config/params.yaml` 或创建自定义参数文件：

```yaml
rm_custom_controller_imu:
  ros__parameters:
    ref_topic: "/rm_manager/custom_controller"
    watchdog_timeout: 1.0
    
    # TF2 与 Pose 配置
    parent_frame: "world"
    child_frame: "controller_imu"
    pose_topic: "~/pose"
    position_scale_x: 1.0
    position_scale_y: 1.0
    position_scale_z: 1.0
    
    # Twist 配置
    enable_twist_cmd: true
    twist_cmd_topic: "~/twist_cmd"
    channel_mapping: ["linear_x", "linear_y", "angular_z", "none"]
    channel_max: [2.0, 2.0, 1.5, 1.0]
```

### 3. 运行节点

**方式1：直接运行**
```bash
ros2 run rm_custom_controller_imu rm_custom_controller_imu --ros-args --params-file test/config/params.yaml
```

**方式2：使用 launch 文件**
```bash
ros2 launch rm_custom_controller_imu test.launch.py
```

### 4. 验证功能

**查看 TF2 变换**
```bash
ros2 run tf2_ros tf2_echo world controller_imu
```

**查看 Pose 消息**
```bash
ros2 topic echo /rm_custom_controller_imu/pose
```

**查看 Twist 命令**
```bash
ros2 topic echo /rm_custom_controller_imu/twist_cmd
```

**查看 GPIO 状态**
```bash
ros2 topic echo /rm_custom_controller_imu/gpio0_state
ros2 topic echo /rm_custom_controller_imu/gpio1_state
```

## 通道映射说明

通道值从 int8 [-128, 127] 归一化到 [-1.0, 1.0]，然后乘以对应的 `channel_max` 值。

**示例**：
- `channel_0 = 127` (最大正值)
- `channel_max[0] = 2.0`
- 归一化：`127 / 127.0 = 1.0`
- 最终值：`1.0 * 2.0 = 2.0` → `twist.linear.x = 2.0`

## 看门狗功能

节点内置看门狗功能，如果在 `watchdog_timeout` 时间内没有接收到数据，会发出警告。当重新接收到数据后，会自动恢复并输出日志。

## GPIO 状态

gpio_state 字节的每个 bit 对应一个 GPIO，按位解析：
- bit 0 → gpio0_state
- bit 1 → gpio1_state
- ...
- bit 7 → gpio7_state

按下时为 1，松开时为 0。

## 依赖

- rclcpp
- rclcpp_components
- std_msgs
- geometry_msgs
- tf2_ros
- rm_message
- generate_parameter_library
- pluginlib

## 注意事项

1. 确保裁判系统消息正确发布在 `ref_topic` 上
2. 数据包必须是 30 字节长度
3. 数据包头必须是 0xA1 或 0xA2
4. TF2 父子坐标系名称必须正确配置
5. 通道映射数组和最大值数组的长度必须为 4

## 开发时间

2026年3月

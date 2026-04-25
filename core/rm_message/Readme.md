# RMMessage

`rm_message` 提供 `rm_manager` 当前会按 typed topic 发布的“机器人入站协议”消息定义。

## 串口帧格式

所有标准串口帧均使用统一头尾格式：

| 字段 | 大小 | 说明 |
| :--- | :--- | :--- |
| `SOF` | 1 byte | 固定为 `0xA5` |
| `data_length` | 2 byte | `data` 字段长度 |
| `seq` | 1 byte | 包序号 |
| `CRC8` | 1 byte | 帧头校验 |
| `cmd_id` | 2 byte | 命令字 |
| `data` | n byte | 负载 |
| `CRC16` | 2 byte | 整帧校验 |

## 当前支持的机器人入站 typed 命令

### 裁判系统串口

- `0x0001` `GameState`
- `0x0002` `GameResult`
- `0x0003` `RobotHP`
- `0x0101` `FieldEvents`
- `0x0104` `RefereeWarning`
- `0x0105` `DartInfo`
- `0x0201` `RobotStatus`
- `0x0202` `BufferAndHeat`
- `0x0203` `RobotPosition`
- `0x0204` `RobotBuffs`
- `0x0206` `HurtEvent`
- `0x0207` `ShootData`
- `0x0208` `AmmoAllowance`
- `0x0209` `RFIDStatus`
- `0x020A` `DartCmd`
- `0x020B` `GroundPositions`
- `0x020C` `RadarMark`
- `0x020D` `SentryDecision`
- `0x020E` `RadarDecision`
- `0x0301` `RobotInteraction`
- `0x0303` `MapDownlink`

### 图传串口

- `0x0302` `CustomController`
- `0x0311` `ClientCustomCommand`
- `0x0F01` `SetVTMChannel`
- `0x0F02` `QueryVTMChannel`

## V1.3.0 对齐变更

- `0x0304` 已从协议中删除，`rm_manager` 不再发布对应 typed topic。
- 图传旧私有遥控帧 `0xA9 0x53` 仅作为 `rm_manager` 的 legacy/private 兼容输入保留，单独发布 `/remote_control`，不作为标准 typed cmd 支持协议。
- `RobotBuffs.attack_buff` 已改为 `uint16`，能量反馈位来自最后 1 字节。
- `RFIDStatus` 已按 40 bit 完整展开，补齐装配增益点和 12 个隧道位置位。
- `RadarMark` 已补齐空中机器人特殊标识位。
- `SentryDecision` 已补齐 `is_out_of_combat` 和 `remaining_exchangeable_17mm`。
- 新增 `ClientCustomCommand`，对应图传入站命令 `0x0311`。

#### 25. 小地图路径数据 (Path Uplink) - `0x0307`
*   **发送方：** 哨兵/半自动机器人 $\rightarrow$ 对应选手端
*   **Data段长度：** 103-byte
*   **详细定义：** 意图 (1:攻, 2:守, 3:移)；起点X, Y；X轴增量数组 (49字节)；Y轴增量数组 (49字节)。

#### 26. 小地图自定义信息 (Custom String) - `0x0308`
*   **发送方：** 己方机器人 $\rightarrow$ 己方选手端
*   **Data段长度：** 34-byte
*   **详细定义：** 发送者 ID (2-byte)；接收者 ID (2-byte)；30字节 UTF-16 字符串。

#### 27. 机器人发送自定义数据 - `0x0309 / 0x0310`
*   **0x0309:**
    *   **发送方：** 己方机器人 $\rightarrow$ 对应操作手连接的自定义控制器
    *   **Data段长度：** 30-byte
*   **0x0310:**
    *   **发送方：** 己方机器人 $\rightarrow$ 图传链路 $\rightarrow$ 自定义客户端
    *   **Data段长度：** 150-byte

---

### 六、 系统信道管理类 (VTM Config)

#### 28. 图传出图信道设置 (Set VTM Channel) - `0x0F01`
*   **发送方：** 机器人 $\rightarrow$ 图传发送端
*   **Data段定义：**
    *   **发送段:** 1-byte, 设置值 1-6 代表信道 1-6。
    *   **接收反馈:** 0: 成功, 1: 启动中无法设置, 2: 设置有误。

#### 29. 查询图传信道 (Query VTM Channel) - `0x0F02`
*   **发送方：** 机器人 $\rightarrow$ 图传发送端
*   **反馈定义：** 0: 未设置, 1-6: 当前运行信道。

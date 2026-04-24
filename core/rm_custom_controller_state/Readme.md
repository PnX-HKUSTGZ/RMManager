# rm_custom_controller_state

协议包定义

```cpp
typedef struct {

    /**
     * \brief 从 j0-j6四个关节的角度，将 [-2pi,2pi] 压缩为 uint16 
     */
    uint16_t rotor_angles[7];
    uint8_t channel_0;
    uint8_t channel_1;
    uint8_t channel_2;
    uint8_t channel_3;

    /**
     * \brief 8个gpio状态,每个bit标识一个
     */
    uint8_t gpio_state;
    uint8_t reserved[11];

} ControlData;
```
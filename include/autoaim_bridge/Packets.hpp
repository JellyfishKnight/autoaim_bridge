// created by liuhan on 2023/11/18
// Submodule of HeliosCV
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 *
 */
#pragma once

#include <cstdint>


namespace helios_cv {

typedef struct SendPacket {
    uint8_t TOF = 0xA5;    
    uint8_t tracking;      // tracking  = 1或0是指当前上位机的predictor的整车观测器当前是否观测到了车辆
                        // 或者打符是否在观测中
    uint8_t id;          // outpost = 8  guard =  7  base = 9   energy = 10
    uint8_t armors_num;   // 2-balance 3-outpost 4-normal  5-energy
    float x;   // 车辆中心x  (能量机关装甲板x) 
    float y;   // 车辆中心y  (能量机关装甲板y)
    float z;   // 车辆中心z  (能量机关装甲板z)
    float yaw;  // 面对我们的装甲板的yaw值 
    float vx;  // 类推
    float vy;  
    float vz;  
    float v_yaw;  
    float r1;   // 车辆第一个半径
    float r2;   // 车辆第二个半径
    float dz;  //两对装甲板之间的高低差值
    uint16_t checksum; // 前48字节相加和
    uint8_t SOF = 0xA6;
}SendPacket;

typedef struct ReceivePacket {
    uint8_t TOF = 0x5A;
    uint8_t target_color; // 敌方颜色
    uint8_t autoaim_mode; // 0自瞄 1 小符 2 大符
    float bullet_speed; // 弹速
    float yaw;   // total yaw
    float pitch;   // 直接转发陀螺仪pitch即可
    uint16_t checksum;  // 前16字节之和
    uint8_t SOF = 0x6A;
}ReceivePacket;

} // namespace helios_cv
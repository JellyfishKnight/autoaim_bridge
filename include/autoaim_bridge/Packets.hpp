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
#include <cstring>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

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

    static void convert_send_packet_to_write_buffer(const SendPacket& packet, uint8_t* write_buffer) {
        write_buffer[0] = packet.TOF;
        write_buffer[1] = packet.tracking;
        write_buffer[2] = packet.id;
        write_buffer[3] = packet.armors_num;
        std::memcpy(write_buffer + 4, &packet.x, 4);
        std::memcpy(write_buffer + 8, &packet.y, 4);
        std::memcpy(write_buffer + 12, &packet.z, 4);
        std::memcpy(write_buffer + 16, &packet.yaw, 4);
        std::memcpy(write_buffer + 20, &packet.vx, 4);
        std::memcpy(write_buffer + 24, &packet.vy, 4);
        std::memcpy(write_buffer + 28, &packet.vz, 4);
        std::memcpy(write_buffer + 32, &packet.v_yaw, 4);
        std::memcpy(write_buffer + 36, &packet.r1, 4);
        std::memcpy(write_buffer + 40, &packet.r2, 4);
        std::memcpy(write_buffer + 44, &packet.dz, 4);
        // caculate checksum
        uint16_t checksum = 0;
        for (int i = 0; i < 48; i++) {
            checksum += write_buffer[i];
        }
        std::memcpy(write_buffer + 48, &checksum, 2);
        write_buffer[50] = packet.SOF;
    }
    
}SendPacket;

typedef struct ReceivePacket {
    uint8_t TOF = 0x5A;
    uint8_t target_color; // 敌方颜色
    uint8_t autoaim_mode; // 0自瞄 1 小符 2 大符
    float bullet_speed; // 弹速
    float yaw;   // total yaw
    float pitch;   // 直接转发陀螺仪pitch即可
    uint16_t checksum;  // 前15字节之和
    uint8_t SOF = 0x6A;

    static bool verify_check_sum(uint8_t* read_buffer) {
        uint16_t sum = 0;
        for (int i = 0; i < 15; i++) {
            sum += read_buffer[i];
        }
        // get checksum
        uint16_t checksum = 0;
        checksum = read_buffer[16];
        checksum = (checksum << 8) | read_buffer[15];
        return sum == checksum;
    }

    static void convert_read_buffer_to_recv_packet(uint8_t* read_buffer, ReceivePacket& packet) {
        packet.TOF = read_buffer[0];
        packet.target_color = read_buffer[1];
        packet.autoaim_mode = read_buffer[2];
        std::memcpy(&packet.bullet_speed, read_buffer + 3, 4);
        std::memcpy(&packet.yaw, read_buffer + 7, 4);
        std::memcpy(&packet.pitch, read_buffer + 11, 4);
        std::memcpy(&packet.checksum, read_buffer + 15, 2);
        packet.SOF = read_buffer[17];
    }

}ReceivePacket;

} // namespace helios_cv
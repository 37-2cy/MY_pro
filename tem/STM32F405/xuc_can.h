#pragma once

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "imu.h"   // ✅ 复用 CanRxMsg_t 定义

// XUC CAN 队列
extern QueueHandle_t g_xuc_can_queue;

class XUC
{
public:
    typedef struct { float yaw, pitch; } TargetAngle;

    void InitCAN(QueueHandle_t canQueue, uint16_t can_id = 0x98, float scale = 100.0f);
    void Decode();

    // 只负责打包，不发送
    void Encode(uint16_t robot_id);

    float GetTargetYaw() { return target.yaw; }
    float GetTargetPitch() { return target.pitch; }
    uint8_t fire_auto = 0;
public:
    // 发送数组，外部可直接取用
    uint8_t tx_data[8] = { 0 };

private:
    TargetAngle target{};
    QueueHandle_t m_canQueue = NULL;
    uint16_t m_can_id = 0x98;
    float m_scale = 100.0f;

private:
    static inline int16_t rd_i16_le(const uint8_t* p)
    {
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    }
};

extern XUC xuc;
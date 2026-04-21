#include "xuc_can.h"
#include <string.h>

void XUC::InitCAN(QueueHandle_t canQueue, uint16_t can_id, float scale)
{
    m_canQueue = canQueue;
    m_can_id = can_id;
    m_scale = scale;
}

void XUC::Decode()
{
    if (m_canQueue == NULL) return;

    CanRxMsg_t msg{};
    CanRxMsg_t last{};
    bool got = false;

    while (xQueueReceive(m_canQueue, &msg, 0) == pdTRUE) {
        if (msg.id == m_can_id && msg.dlc == 8) {
            last = msg;
            got = true;
        }
    }

    if (!got) return;

    int16_t y_q = rd_i16_le(&last.data[0]);
    int16_t p_q = rd_i16_le(&last.data[2]);
    fire_auto   = last.data[4];
    target.yaw = (float)y_q / m_scale;
    target.pitch = (float)p_q / m_scale;
}

void XUC::Encode(uint16_t robot_id)
{
    memset(tx_data, 0, sizeof(tx_data));

    // 先把 robot_id 放进去，低字节在前
    tx_data[0] = (uint8_t)(robot_id & 0xFF);
    tx_data[1] = (uint8_t)((robot_id >> 8) & 0xFF);

    // 以后你要加别的信息，直接继续往后填
    // 例如：
    // tx_data[2] = xxx;
    // tx_data[3] = xxx;
    // tx_data[4] = xxx;
}
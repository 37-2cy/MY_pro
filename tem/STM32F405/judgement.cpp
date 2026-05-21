// judgement.cpp
#include "judgement.h"
#include "label.h"
#include <cstdarg>
#include "imu.h"
#include "control.h"
#include "HTmotor.h"
#include "xuc_can.h"
#include "RC.h"
#include "supercap.h"
extern uint8_t flag_shoot;

//void Judgement::BuffData()
//{
//    //if (m_uart->updateFlag)
//    //{
//    //    m_uart->updateFlag = false;
//
//    if (queueHandler == NULL || *queueHandler == NULL)
//        return;  // 或者报错
//
//    // ★ 必须判断有没有真的收到队列数据
//    if (xQueueReceive(*queueHandler, m_uartrx, 0) != pdPASS)
//        return;
//
//    m_readnum = m_uart->dataDmaNum;
//
//    if ((m_whand + m_readnum) < (m_FIFO + BUFSIZE))
//    {
//        memcpy(m_whand, m_uartrx, m_readnum);
//        m_whand = m_whand + m_readnum;
//    }
//    else if ((m_whand + m_readnum) == (m_FIFO + BUFSIZE))
//    {
//        memcpy(m_whand, m_uartrx, m_readnum);
//        m_whand = m_FIFO;
//    }
//    else
//    {
//        const uint8_t left_size = m_FIFO + BUFSIZE - m_whand;
//        memcpy(m_whand, m_uartrx, left_size);
//        m_whand = m_FIFO;
//        memcpy(m_whand, m_uartrx + left_size, m_readnum - left_size);
//        m_whand = m_FIFO + m_readnum - left_size;
//    }
//    m_leftsize = m_leftsize + m_readnum;
//
//    supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
//    supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
//    //}
//}

// 在 judgement.cpp 顶部（和其他调试变量一起放）加上：
uint16_t judge_debug_len = 0;
uint8_t  judge_debug_b0 = 0;
uint8_t  judge_debug_b1 = 0;
uint8_t  judge_debug_b2 = 0;



void Judgement::BuffData()
{
    if (queueHandler == NULL || *queueHandler == NULL)
        return;

    // ★ 从队列里收一包原始数据到 m_uartrx
    if (xQueueReceive(*queueHandler, m_uartrx, 0) != pdPASS)
        return;

    // ★ 先不要再信 m_uart->dataDmaNum 了
    //   先用队列元素大小（这里假设就是 DMA_RX_SIZE）
    m_readnum = DMA_RX_SIZE;

    // 调试看看队列里来的东西是不是对的
    judge_debug_len = m_readnum;
    judge_debug_b0 = m_uartrx[0];
    judge_debug_b1 = m_uartrx[1];
    judge_debug_b2 = m_uartrx[2];


    supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
    supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
}



void Judgement::Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base)
{
    huart->Init(uart_base, baud).DMARxInit();
    m_uart = huart;
    queueHandler = &huart->UartQueueHandler;
}

// 根据协议自己定义一个最大 data 长度，防止异常长度把缓冲区撑爆
#define JUDGEMENT_MAX_DATA_LEN  200u

// 加几个调试变量（放在 Judgement 类里或者 cpp 顶部加 static/全局）:
uint8_t judge_debug_step = 0;
uint16_t judge_debug_dlen = 0;
uint16_t judge_debug_frame_len = 0;


void Judgement::GetData(void)
{
    // ★ 一进函数立刻标记一下
    judge_debug_step = 1;

    uint16_t len = (uint16_t)m_readnum;
    if (len < 9)
    {
        // 帮你再看一眼当前长度
        judge_debug_dlen = len;
        // 这里就先返回
        return;
    }

    judge_debug_dlen = len;  // 保存一下本帧总长度

    uint16_t idx = 0;

    while (idx + 9 <= len)
    {
        // 1. 找 SOF
        if (m_uartrx[idx] != 0xA5)
        {
            idx++;
            continue;
        }

        judge_debug_step = 2;

        // 2. CRC8
        if (idx + 5 > len)
            break;

        if (!VerifyCRC8CheckSum(&m_uartrx[idx], 5))
        {
            judge_debug_step = 3;
            idx++;
            continue;
        }

        judge_debug_step = 4;

        // 3. data_length
        const uint16_t data_length =
            (uint16_t)m_uartrx[idx + 1] |
            (uint16_t)(m_uartrx[idx + 2] << 8);

        judge_debug_dlen = data_length;

        if (data_length == 0 || data_length > JUDGEMENT_MAX_DATA_LEN)
        {
            judge_debug_step = 5;
            idx++;
            continue;
        }

        const uint16_t frame_length = (uint16_t)(data_length + 9);
        judge_debug_frame_len = frame_length;

        if (idx + frame_length > len)
        {
            judge_debug_step = 6;
            break;
        }

        judge_debug_step = 7;

        if (!VerifyCRC16CheckSum(&m_uartrx[idx], frame_length))
        {
            judge_debug_step = 8;
            idx++;
            continue;
        }

        // 一帧完整 OK
        judge_debug_step = 9;
        Decode(&m_uartrx[idx]);

        idx += frame_length;
    }
}




void Judgement::DisplayStaticUI()
{
    string_data_struct_t staticStringUI;
    switch (count % 100)
    {
    case 0:
    {
        char UIPitchData[5] = {};

        UIPitchData[0] = 'P';
        UIPitchData[1] = 'I';
        UIPitchData[2] = 'T';
        UIPitchData[3] = 'C';
        UIPitchData[4] = 'H';
        Char_Draw(&staticStringUI, "PTH", UI_Graph_ADD, 7, UI_Color_Green, 30, 5, 3, 60, 820, UIPitchData);//PITCH

        Char_ReFresh(&staticStringUI);
        break;
    }
    case 10:
    {
        char UIStateData[5] = {};
        UIStateData[0] = 'S';
        UIStateData[1] = 'T';
        UIStateData[2] = 'A';
        UIStateData[3] = 'T';
        UIStateData[4] = 'E';
        Char_Draw(&staticStringUI, "STATE", UI_Graph_ADD, 8, UI_Color_Green, 30, 5, 3, 60, 760, UIStateData);//CAP

        Char_ReFresh(&staticStringUI);
        break;


    }
    case 20:
    {
        char UIModeData[3] = {};
        UIModeData[0] = 'M';
        UIModeData[1] = 'O';
        UIModeData[2] = 'D';

        Char_Draw(&staticStringUI, "mod", UI_Graph_ADD, 9, UI_Color_Green, 30, 4, 3, 60, 700, UIModeData);//MODE
        Char_ReFresh(&staticStringUI);
        break;
    }
    case 30:
    {
        char UIfly[3] = {};

        UIfly[0] = 'F';
        UIfly[1] = 'L';
        UIfly[2] = 'Y';

        Char_Draw(&staticStringUI, "F1y", UI_Graph_ADD, 6, UI_Color_Green, 30, 3, 3, 60, 640, UIfly);//飞坡
        Char_ReFresh(&staticStringUI);
        break;
    }
    case 40:
    {
        char UIOpenRubData[3] = {};

        UIOpenRubData[0] = 'R';
        UIOpenRubData[1] = 'U';
        UIOpenRubData[2] = 'B';

        Char_Draw(&staticStringUI, "RUB", UI_Graph_ADD, 5, UI_Color_Green, 30, 3, 3, 60, 580, UIOpenRubData);//RUB

        Char_ReFresh(&staticStringUI);
        break;


    }
    case 50:
    {
        graphic_data_struct_t staticGraohUI4[7] = {};

        LineDraw(&staticGraohUI4[0], "R1", UI_Graph_ADD, 1, UI_Color_Main, 2, 960 + 70, 540 - 48, 960 + 70, 540 - 120);
        LineDraw(&staticGraohUI4[1], "L1", UI_Graph_ADD, 1, UI_Color_Main, 2, 960 - 70, 540 - 48, 960 - 70, 540 - 120);
        LineDraw(&staticGraohUI4[2], "R2", UI_Graph_ADD, 1, UI_Color_Main, 2, 1061, 480, 1061, 420);
        LineDraw(&staticGraohUI4[3], "L2", UI_Graph_ADD, 1, UI_Color_Main, 2, 960 - 100, 480, 960 - 100, 420);
        /*LineDraw(&staticGraohUI4[4], "B3", UI_Graph_ADD, 1, UI_Color_Main, 2, 960 - 30, 400, 960 + 30, 400);
        LineDraw(&staticGraohUI4[5], "B4", UI_Graph_ADD, 1, UI_Color_Main, 2, 960 - 20, 380, 960 + 20, 380);*/
        UI_ReFresh(7, staticGraohUI4);
        break;
        //char UICapDisplayData1[3] = {};
        //UICapDisplayData1[0] = '1';
        //UICapDisplayData1[1] = '3';
        //UICapDisplayData1[2] = 'V';

        //Char_Draw(&staticStringUI, "C1", UI_Graph_ADD, 4, UI_Color_Purplish_red, 20, 3, 2, 580, 70, UICapDisplayData1);//13V

        //Char_ReFresh(&staticStringUI);
    }
    case 60:
    {
        //char UICapDisplayData2[3] = {};
        //UICapDisplayData2[0] = '1';
        //UICapDisplayData2[1] = '9';
        //UICapDisplayData2[2] = 'V';

        //Char_Draw(&staticStringUI, "C2", UI_Graph_ADD, 4, UI_Color_Yellow, 20, 3, 2, 1012, 70, UICapDisplayData2);//19V

        //Char_ReFresh(&staticStringUI);
        break;
    }
    case 70:
    {
        //char UICapDisplayData3[3] = {};
        //UICapDisplayData3[0] = '2';
        //UICapDisplayData3[1] = '3';
        //UICapDisplayData3[2] = 'V';

        //Char_Draw(&staticStringUI, "C2", UI_Graph_ADD, 3, UI_Color_Green, 20, 3, 2, 1300, 70, UICapDisplayData3);//23V

        //Char_ReFresh(&staticStringUI);
        break;
    }
    case 80:
    {
        graphic_data_struct_t staticGraohUI3[5] = {};
        /*LineDraw(&staticGraohUI3[1], "B5", UI_Graph_ADD, 3, UI_Color_Main, 2, 960-30, 340, 960 + 30, 340);
        LineDraw(&staticGraohUI3[2], "B6", UI_Graph_ADD, 3, UI_Color_Main, 2, 960-20, 320, 960 + 20, 320);*/
        LineDraw(&staticGraohUI3[4], "MIDLINE", UI_Graph_ADD, 3, UI_Color_Main, 2, 960 + 100, 450, 960 - 100, 450);
        LineDraw(&staticGraohUI3[3], "RIGHT_FRONT", UI_Graph_ADD, 3, UI_Color_Main, 2, 1370, 100, 1150, 400);

        UI_ReFresh(5, staticGraohUI3);
        break;
    }
    case 90:
    {
        graphic_data_struct_t staticGraohUI2[7] = {};

        //LineDraw(&staticGraohUI2[0], "MID", UI_Graph_ADD, 2, UI_Color_Main, 2, 960, 540 + 70, 960, 280);
        //LineDraw(&staticGraohUI2[1], "B1", UI_Graph_ADD, 2, UI_Color_Main, 2, 930, 440, 960 + 30, 440); //倒数第三根
        //LineDraw(&staticGraohUI2[2], "B2", UI_Graph_ADD, 2, UI_Color_Main, 2, 930, 420, 960 + 30, 420);
        Circle_Draw(&staticGraohUI2[1], "aim", UI_Graph_ADD, 2, UI_Color_Main, 2, 960, 450, 20);
        LineDraw(&staticGraohUI2[5], "LEFT_FRONT", UI_Graph_ADD, 2, UI_Color_Main, 2, 550, 100, 770, 400);
        LineDraw(&staticGraohUI2[3], "aml", UI_Graph_ADD, 2, UI_Color_Main, 2, 960, 450 + 20, 960, 450 - 20);
        UI_ReFresh(7, staticGraohUI2);
        break;
    }
    default:
        break;
    }
}

void Judgement::DisplayRP(int flag)
{
    char RP[2] = {};
    string_data_struct_t RPData;

    switch (flag)
    {
    case 0:
        RP[0] = 'D';
        RP[1] = '0';
        break;
    case 1:
        RP[0] = 'D';
        RP[1] = '1';
        break;
    case 2:
        RP[0] = 'D';
        RP[1] = '2';
        break;
    default:
        break;
    }

    if (!graphInit)
    {
        Char_Draw(&RPData, (char*)"aa", UI_Graph_ADD, 5, UI_Color_Green,
            30, 2, 3, 280, 820, RP);
    }
    else
    {
        Char_Draw(&RPData, (char*)"aa", UI_Graph_Change, 5, UI_Color_Green,
            30, 2, 3, 280, 820, RP);
    }

    Char_ReFresh(&RPData);
}

void Judgement::DisplayCapState(uint8_t capState)
{
    char capStateChar[5] = {};
    string_data_struct_t capStateData;

    if (capState == WORKING)
    {
        capStateChar[0] = 'W';
        capStateChar[1] = 'O';
        capStateChar[2] = 'R';
        capStateChar[3] = 'K';
    }
    else if (capState == DISCHARGE)
    {
        capStateChar[0] = 'D';
        capStateChar[1] = 'I';
        capStateChar[2] = 'S';
        capStateChar[3] = 'C';
        capStateChar[4] = 'H';
    }
    else if (capState == SHUT)
    {
        capStateChar[0] = 'S';
        capStateChar[1] = 'H';
        capStateChar[2] = 'U';
        capStateChar[3] = 'T';
    }

    if (!graphInit)
    {
        Char_Draw(&capStateData, (char*)"CO1", UI_Graph_ADD, 7, UI_Color_Green,
            30, 5, 3, 280, 760, capStateChar);
    }
    else
    {
        Char_Draw(&capStateData, (char*)"CO1", UI_Graph_Change, 7, UI_Color_Green,
            30, 5, 3, 280, 760, capStateChar);
    }

    Char_ReFresh(&capStateData);
}
void Judgement::DisplayState(bool isUp)
{
    char stateChar[5] = {};
    string_data_struct_t stateData = {};
    const uint32_t graphOperate = graphInit ? UI_Graph_Change : UI_Graph_ADD;
    const uint32_t color = isUp ? UI_Color_Yellow : UI_Color_Cyan;

    if (isUp)
    {
        stateChar[0] = 'U';
        stateChar[1] = 'P';
        stateChar[2] = ' ';
        stateChar[3] = ' ';
    }
    else
    {
        stateChar[0] = 'D';
        stateChar[1] = 'O';
        stateChar[2] = 'W';
        stateChar[3] = 'N';
    }

    Char_Draw(&stateData, (char*)"ST1", graphOperate, 8, color,
        30, 4, 3, 280, 760, stateChar);
    Char_ReFresh(&stateData);
}
void Judgement::DisplpayMode(uint8_t mode)
{
    char modeChar[7] = {};
    string_data_struct_t modeData = {};
    const uint32_t graphOperate = graphInit ? UI_Graph_Change : UI_Graph_ADD;

    if (mode == 1)
    {
        modeChar[0] = 'S';
        modeChar[1] = 'U';
        modeChar[2] = 'P';
        modeChar[3] = 'E';
        modeChar[4] = 'R';
        modeChar[5] = ' ';
    }
    else
    {
        modeChar[0] = 'N';
        modeChar[1] = 'O';
        modeChar[2] = 'R';
        modeChar[3] = 'M';
        modeChar[4] = 'A';
        modeChar[5] = 'L';
    }

    Char_Draw(&modeData, (char*)"MD1", graphOperate, 6, UI_Color_Green,
        30, 6, 3, 280, 700, modeChar);
    Char_ReFresh(&modeData);
}
void Judgement::DisplaySpinSquare(void)
{
    static uint8_t spinStep = 0;
    const bool spinning = (rc.c_toggle != 0) && (ctrl.chassis.speedz != 0);
    const uint32_t graphOperate = graphInit ? UI_Graph_Change : UI_Graph_ADD;
    const uint32_t color = spinning ? UI_Color_Yellow : UI_Color_Cyan;
    const int16_t centerX = 1015;
    const int16_t centerY = 450;
    static const int16_t vertices[4][4][2] = {
        {{-18, -18}, { 18, -18}, { 18,  18}, {-18,  18}},
        {{-10, -24}, { 24, -10}, { 10,  24}, {-24,  10}},
        {{  0, -26}, { 26,   0}, {  0,  26}, {-26,   0}},
        {{ 10, -24}, { 24,  10}, {-10,  24}, {-24, -10}},
    };
    char names[4][3] = {
        {'S', '0', 0},
        {'S', '1', 0},
        {'S', '2', 0},
        {'S', '3', 0},
    };
    graphic_data_struct_t square[4] = {};

    if (spinning)
    {
        spinStep = static_cast<uint8_t>((spinStep + 1U) & 0x03U);
    }
    else
    {
        spinStep = 0;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        const uint8_t next = static_cast<uint8_t>((i + 1U) & 0x03U);
        LineDraw(&square[i], names[i], graphOperate, 4, color, 2,
            centerX + vertices[spinStep][i][0],
            centerY + vertices[spinStep][i][1],
            centerX + vertices[spinStep][next][0],
            centerY + vertices[spinStep][next][1]);
    }

    UI_ReFresh(2, &square[0]);
    UI_ReFresh(2, &square[2]);
}
void Judgement::DisplayLegPose(void)
{
    const bool leftExtended = (leg_state == 1) && (DMmotor[0].setPos < -0.40f);
    const bool rightExtended = (leg_state == 1) && (DMmotor[1].setPos > 0.40f);
    const uint32_t graphOperate = graphInit ? UI_Graph_Change : UI_Graph_ADD;
    const uint32_t leftColor = leftExtended ? UI_Color_Yellow : UI_Color_Cyan;
    const uint32_t rightColor = rightExtended ? UI_Color_Yellow : UI_Color_Cyan;
    graphic_data_struct_t leg[4] = {};
    char names[4][3] = {
        {'L', 'G', '0'},
        {'L', 'G', '1'},
        {'R', 'G', '0'},
        {'R', 'G', '1'},
    };

    int16_t leftHipX = 910;
    int16_t leftHipY = 425;
    int16_t leftKneeX = leftExtended ? 885 : 892;
    int16_t leftKneeY = leftExtended ? 455 : 450;
    int16_t leftFootX = leftExtended ? 858 : 910;
    int16_t leftFootY = leftExtended ? 490 : 475;

    int16_t rightHipX = 1010;
    int16_t rightHipY = 425;
    int16_t rightKneeX = rightExtended ? 1035 : 1028;
    int16_t rightKneeY = rightExtended ? 455 : 450;
    int16_t rightFootX = rightExtended ? 1062 : 1010;
    int16_t rightFootY = rightExtended ? 490 : 475;

    LineDraw(&leg[0], names[0], graphOperate, 4, leftColor, 3,
        leftHipX, leftHipY, leftKneeX, leftKneeY);
    LineDraw(&leg[1], names[1], graphOperate, 4, leftColor, 3,
        leftKneeX, leftKneeY, leftFootX, leftFootY);
    LineDraw(&leg[2], names[2], graphOperate, 4, rightColor, 3,
        rightHipX, rightHipY, rightKneeX, rightKneeY);
    LineDraw(&leg[3], names[3], graphOperate, 4, rightColor, 3,
        rightKneeX, rightKneeY, rightFootX, rightFootY);

    UI_ReFresh(2, &leg[0]);
    UI_ReFresh(2, &leg[2]);
}
void Judgement::DisplayCapture(bool isCapture)
{
    char captureChar[5] = {};
    string_data_struct_t captureData;

    if (isCapture)
    {
        captureChar[0] = 'T';
        captureChar[1] = 'R';
        captureChar[2] = 'U';
        captureChar[3] = 'E';
    }
    else
    {
        captureChar[0] = 'F';
        captureChar[1] = 'A';
        captureChar[2] = 'L';
        captureChar[3] = 'S';
        captureChar[4] = 'E';
    }

    if (!graphInit)
    {
        Char_Draw(&captureData, (char*)"CO", UI_Graph_ADD, 5, UI_Color_Green,
            30, 5, 3, 280, 640, captureChar);
    }
    else
    {
        Char_Draw(&captureData, (char*)"CO", UI_Graph_Change, 5, UI_Color_Green,
            30, 5, 3, 280, 640, captureChar);
    }

    Char_ReFresh(&captureData);
}

void Judgement::DisplayCapVoltage(float capVoltage)
{
    constexpr uint32_t kBarLeft = 604;
    constexpr uint32_t kBarRight = 1316;
    constexpr uint32_t kBarCenterY = 125;
    constexpr uint32_t kBarWidth = 42;
    constexpr float kCapEnergyMax = 2000.0f;

    graphic_data_struct_t voltageData{};
    float energy = capVoltage;

    if (energy < 0.0f)
        energy = 0.0f;
    else if (energy > kCapEnergyMax)
        energy = kCapEnergyMax;

    const uint32_t voltagePos = kBarLeft +
        static_cast<uint32_t>(energy * (kBarRight - kBarLeft) / kCapEnergyMax);

    if (!graphInit)
    {
        LineDraw(&voltageData, (char*)"VD1", UI_Graph_ADD, 3, UI_Color_Yellow,
            kBarWidth, kBarLeft, kBarCenterY, voltagePos, kBarCenterY);
    }
    else
    {
        LineDraw(&voltageData, (char*)"VD1", UI_Graph_Change, 3, UI_Color_Yellow,
            kBarWidth, kBarLeft, kBarCenterY, voltagePos, kBarCenterY);
    }

    UI_ReFresh(1, &voltageData);
}

//void Judgement::SendData(void)
//{
//    robotId = data.robot_status_t.robot_id;
//    clientId = robotId | 0x100;
//
//    if (count < 200)
//    {
//        DisplayStaticUI();
//    }
//    else
//    {
//        switch (count % 20)
//        {
//        case 0:
//            rc.gear++;
//            DisplayRP(rc.gear);
//            break;
//        case 4:
//            DisplayCapState(supercap.Txsuper.state);
//            break;
//        case 8:
//            DisplayCapVoltage(supercap.Rxsuper.cap_energy);
//            break;
//        case 12:
//            DisplayCapture(ctrl.shooter.displayOpenRub);
//            break;
//        default:
//            break;
//        }
//    }
//
//    const uint8_t modeFlag = (flag_shoot == 1) ? 1 : 0;
//    DisplpayMode(modeFlag);
//
//    if (count > 220)
//    {
//        graphInit = true;
//    }
//
//    count++;
//}

void Judgement::SendData(void)
{

    robotId = data.robot_status_t.robot_id;
    clientId = robotId | 0x100;
    static uint8_t b_last = 0;
    if (rc.pc.B == 1 && b_last == 0) {
        count = 0;
        graphInit = false;
    }
    b_last = rc.pc.B;
    if (count < 200)
    {
        DisplayStaticUI();
    }
    else
    {
        switch (count % 10)
        {
        case 0:
        {
            //DisplayImuPitch(imu_pantile.GetAnglePitch());
            DisplayOpenRub(ctrl.shooter.displayOpenRub);
            break;
        }
        case 1:
        {
            //Displayload();
            DisplayOpenRub(ctrl.shooter.displayOpenRub);
            break;
        }
        case 2:
        {
            //Displayfly(power.fly_flag);
            break;
        }
        case 3:
        {
            DisplayState(leg_state == 1);
            break;
        }
        case 4:
        {
            //DisplayCapVoltage(supercap.Rxsuper.cap_energy);
           // DisplayYawDirection(can2_motor[3].angle[now]);
            break;
        }
        case 5:
        {
            DisplayOpenRub(ctrl.shooter.displayOpenRub);
            break;
        }
        case 6:
        {
            //DisplayCapEnergyBar(supercap.Rxsuper.cap_energy);
            //DisplayBulletSpeed();
            DisplayLegPose();
            break;
        }
        case 7:
        {
            //DisplpayHit();
            break;
        }
        case 8:
        {
            DisplpayMode(flag_shoot);
            break;
        }
        case 9:
        {
            DisplaySpinSquare();
            break;
        }
        default:
            break;
        }
    }
    if (count > 500)
    {
        graphInit = true;
    }
    count++;

}

void Judgement::DisplayOpenRub(bool openrub)
{
    char rubChar[3] = {};
    int color;
    string_data_struct_t rubData;
    if (openrub)
    {
        rubChar[0] = 'O';
        rubChar[1] = 'N';
        color = UI_Color_Green;
    }
    else
    {
        rubChar[0] = 'O';
        rubChar[1] = 'F';
        rubChar[2] = 'F';
        color = UI_Color_Pink;
    }
    if (!graphInit)
    {
        Char_Draw(&rubData, "OpenRub", UI_Graph_ADD, 5, color, 30, 5, 3, 170, 580, rubChar);
    }
    else
    {
        Char_Draw(&rubData, "OpenRub", UI_Graph_Change, 5, color, 30, 5, 3, 170, 580, rubChar);
    }

    Char_ReFresh(&rubData);
}

void Judgement::Decode(uint8_t* m_frame)
{
    const uint16_t cmdID =
        static_cast<uint16_t>(m_frame[5] | (m_frame[6] << 8));
    data.CmdID = cmdID;
    uint8_t* rawdata = &m_frame[7];

    switch (cmdID)
    {
        // 0x0001 比赛状态
    case 0x0001:
        data.game_status_t.game_type =
            static_cast<uint8_t>(rawdata[0] & 0x0F);
        data.game_status_t.game_progress =
            static_cast<uint8_t>(rawdata[0] >> 4);
        data.game_status_t.stage_remain_time =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        {
            uint64_t ts = 0;
            for (int i = 0; i < 8; ++i)
            {
                ts |= (static_cast<uint64_t>(rawdata[3 + i]) << (8 * i));
            }
            data.game_status_t.SyncTimeStamp = ts;
        }
        break;

        // 0x0002 比赛结果
    case 0x0002:
        data.game_result_t.winner = rawdata[0];
        break;

        // 0x0003 己方血量
    case 0x0003:
        data.game_robot_HP_t.ally_1_robot_HP =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.game_robot_HP_t.ally_2_robot_HP =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.game_robot_HP_t.ally_3_robot_HP =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.game_robot_HP_t.ally_4_robot_HP =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        data.game_robot_HP_t.reserved =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.game_robot_HP_t.ally_7_robot_HP =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.game_robot_HP_t.ally_outpost_HP =
            static_cast<uint16_t>(rawdata[12] | (rawdata[13] << 8));
        data.game_robot_HP_t.ally_base_HP =
            static_cast<uint16_t>(rawdata[14] | (rawdata[15] << 8));
        break;

        // 0x0101 场地事件
    case 0x0101:
        data.event_data_t.event_data =
            static_cast<uint32_t>(rawdata[0]) |
            (static_cast<uint32_t>(rawdata[1]) << 8) |
            (static_cast<uint32_t>(rawdata[2]) << 16) |
            (static_cast<uint32_t>(rawdata[3]) << 24);
        break;

        // 0x0102 补给站动作
    case 0x0102:
        data.ext_supply_projectile_action_t.reserved = rawdata[0];
        data.ext_supply_projectile_action_t.supply_robot_id = rawdata[1];
        data.ext_supply_projectile_action_t.supply_projectile_step = rawdata[2];
        data.ext_supply_projectile_action_t.supply_projectile_num = rawdata[3];
        break;

        // 0x0104 裁判警告
    case 0x0104:
        data.referee_warning_t.level = rawdata[0];
        data.referee_warning_t.foul_robot_id = rawdata[1];
        data.referee_warning_t.count = rawdata[2];
        break;

        // 0x0105 飞镖倒计时
    case 0x0105:
        data.dart_dart_info_t.dart_remaining_time = rawdata[0];
        data.dart_dart_info_t.dart_info =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        break;

        // 0x0201 机器人状态
    case 0x0201:
        data.robot_status_t.robot_id = rawdata[0];
        judgementready = true;
        data.robot_status_t.robot_level =
            rawdata[1];
        data.robot_status_t.current_HP =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.robot_status_t.maximum_HP =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.robot_status_t.shooter_barrel_cooling_value =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        data.robot_status_t.shooter_barrel_heat_limit =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.robot_status_t.chassis_power_limit =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.robot_status_t.power_management_gimbal_output =
            static_cast<uint16_t>(rawdata[12] & 0x01);
        data.robot_status_t.power_management_chassis_output =
            static_cast<uint16_t>((rawdata[12] & 0x02) >> 1);
        data.robot_status_t.power_management_shooter_output =
            static_cast<uint16_t>((rawdata[12] & 0x04) >> 2);
        break;

        // 0x0202 缓冲能量 & 热量
    case 0x0202:
        powerheatready = true;

        data.power_heat_data_t.reserved0 =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.power_heat_data_t.reserved1 =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.power_heat_data_t.reserved2 = u32_to_float(&rawdata[4]);
        data.power_heat_data_t.buffer_energy =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.power_heat_data_t.shooter_17mm_1_barrel_heat =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.power_heat_data_t.shooter_42mm_barrel_heat =
            static_cast<uint16_t>(rawdata[12] | (rawdata[13] << 8));

        supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
        supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
        break;

        // 0x0203 位置
    case 0x0203:
        data.robot_pos_t.x = u32_to_float(&rawdata[0]);
        data.robot_pos_t.y = u32_to_float(&rawdata[4]);
        data.robot_pos_t.angle = u32_to_float(&rawdata[8]);
        break;

        // 0x0204 增益
    case 0x0204:
        data.buff_t.recovery_buff = rawdata[0];
        data.buff_t.cooling_buff =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        data.buff_t.defence_buff = rawdata[3];
        data.buff_t.vulnerability_buff = rawdata[4];
        data.buff_t.attack_buff =
            static_cast<uint16_t>(rawdata[5] | (rawdata[6] << 8));
        data.buff_t.remaining_energy = rawdata[7];
        break;

        // 0x0205 空中能量
    case 0x0205:
        data.air_support_data_t.airforce_status = rawdata[0];
        data.air_support_data_t.time_remain = rawdata[1];
        break;

        // 0x0206 伤害
    case 0x0206:
        data.hurt_data_t.armor_id =
            static_cast<uint8_t>(rawdata[0] & 0x0F);
        data.hurt_data_t.HP_deduction_reason =
            static_cast<uint8_t>(rawdata[0] >> 4);
        break;

        // 0x0207 射击
    case 0x0207:
        data.shoot_data_t.bullet_type = rawdata[0];
        data.shoot_data_t.shooter_number = rawdata[1];
        data.shoot_data_t.bullet_freq = rawdata[2];
        data.shoot_data_t.bullet_speed = u32_to_float(&rawdata[3]);

        if (prebulletspd != data.shoot_data_t.bullet_speed)
        {
            nBullet++;
            prebulletspd = data.shoot_data_t.bullet_speed;
        }
        break;

        // 0x0208 发弹量
    case 0x0208:
        data.projectile_allowance_t.projectile_allowance_17mm =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.projectile_allowance_t.projectile_allowance_42mm =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.projectile_allowance_t.remaining_gold_coin =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.projectile_allowance_t.projectile_allowance_fortress =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        break;

        // 0x0209 RFID
    case 0x0209:
    {
        uint32_t rfid =
            static_cast<uint32_t>(rawdata[0]) |
            (static_cast<uint32_t>(rawdata[1]) << 8) |
            (static_cast<uint32_t>(rawdata[2]) << 16) |
            (static_cast<uint32_t>(rawdata[3]) << 24);

        data.rfid_status_t.rfid_status = rfid;
        data.rfid_status_t.rfid_status_2 = rawdata[4];

        baseRFID =
            static_cast<uint8_t>((rfid & (1u << 0)) ? 1 : 0);
        highlandRFID =
            static_cast<uint8_t>(((rfid & (1u << 1)) || (rfid & (1u << 2))) ? 1 : 0);
        feipoRFID =
            static_cast<uint8_t>(((rfid & (1u << 5)) || (rfid & (1u << 6)) ||
                (rfid & (1u << 7)) || (rfid & (1u << 8))) ? 1 : 0);
        outpostRFID =
            static_cast<uint8_t>((rfid & (1u << 18)) ? 1 : 0);
        resourseRFID =
            static_cast<uint8_t>(((rfid & (1u << 19)) || (rfid & (1u << 20))) ? 1 : 0);

        energyRFID = 0; // 能量机关可从 event_data_t 等其它地方推
        break;
    }

    // 0x020A 飞镖客户端指令
    case 0x020A:
        data.dart_client_cmd_t.dart_launch_opening_status = rawdata[0];
        data.dart_client_cmd_t.reserved = rawdata[1];
        data.dart_client_cmd_t.target_change_time =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.dart_client_cmd_t.latest_launch_cmd_time =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        break;

        // 0x020B 地面机器人位置
    case 0x020B:
        data.ground_robot_position_t.hero_x = u32_to_float(&rawdata[0]);
        data.ground_robot_position_t.hero_y = u32_to_float(&rawdata[4]);
        data.ground_robot_position_t.engineer_x = u32_to_float(&rawdata[8]);
        data.ground_robot_position_t.engineer_y = u32_to_float(&rawdata[12]);
        data.ground_robot_position_t.standard_3_x = u32_to_float(&rawdata[16]);
        data.ground_robot_position_t.standard_3_y = u32_to_float(&rawdata[20]);
        data.ground_robot_position_t.standard_4_x = u32_to_float(&rawdata[24]);
        data.ground_robot_position_t.standard_4_y = u32_to_float(&rawdata[28]);
        data.ground_robot_position_t.standard_5_x = u32_to_float(&rawdata[32]);
        data.ground_robot_position_t.standard_5_y = u32_to_float(&rawdata[36]);
        break;

    default:
        break;
    }
}

bool Judgement::Transmit(uint32_t read_size, uint8_t* plate)
{
    if (m_leftsize < read_size) return false;

    if ((m_rhand + read_size) < (m_FIFO + BUFSIZE))
    {
        memcpy(plate, m_rhand, read_size);
        m_rhand = m_rhand + read_size;
    }
    else if ((m_rhand + read_size) == (m_FIFO + BUFSIZE))
    {
        memcpy(plate, m_rhand, read_size);
        m_rhand = m_FIFO;
    }
    else
    {
        const uint8_t left_size = m_FIFO + BUFSIZE - m_rhand;
        memcpy(plate, m_rhand, left_size);
        memcpy(plate + left_size, m_rhand = m_FIFO,
            read_size - left_size);
        m_rhand = m_FIFO + read_size - left_size;
    }

    m_leftsize = m_leftsize - read_size;
    return true;
}

/************************************************绘制直线*************************************************/
void Judgement::LineDraw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t End_x, uint32_t End_y)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Line;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制矩形*************************************************/
void Judgement::Rectangle_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t End_x, uint32_t End_y)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制整圆*************************************************/
void Judgement::Circle_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t Graph_Radius)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************/
void Judgement::Arc_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_StartAngle,
    uint32_t Graph_EndAngle, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t x_Length, uint32_t y_Length)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_StartAngle;
    image->end_angle = Graph_EndAngle;
    image->end_x = x_Length;
    image->end_y = y_Length;
}

/************************************************绘制浮点型数据*************************************************/
void Judgement::Float_Draw(float_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Size,
    uint32_t Graph_Digit, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    float Graph_Float)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_Size;
    image->end_angle = Graph_Digit;

    int32_t temp1 = static_cast<int32_t>(Graph_Float * 1000.0f);
    int32_t temp2 = temp1 / 1024;
    image->end_x = temp2;
    image->radius = temp1 - temp2 * 1024; // 1 -> 1.024e-3
}

/************************************************绘制字符型数据*************************************************/
void Judgement::Char_Draw(string_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Size,
    uint32_t Graph_Digit, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    char* Char_Data)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->Graph_Control.figure_name[2 - i] = imagename[i];

    image->Graph_Control.figure_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.width = Graph_Width;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;
    image->Graph_Control.start_angle = Graph_Size;
    image->Graph_Control.end_angle = Graph_Digit;

    for (i = 0; i < static_cast<int>(Graph_Digit); i++)
    {
        image->show_Data[i] = *Char_Data;
        Char_Data++;
    }
}

/************************************************UI删除函数*************************************************/
void Judgement::UIDelete(uint8_t deleteOperator, uint8_t deleteLayer)
{
    uint16_t dataLength;
    CommunatianData_graphic_t UIDeleteData;

    UIDeleteData.txFrameHeader.sof = 0xA5;
    UIDeleteData.txFrameHeader.data_length = 8;
    UIDeleteData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &UIDeleteData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    UIDeleteData.CMD = UI_CMD_Robo_Exchange;
    UIDeleteData.txID.data_cmd_id = UI_Data_ID_Del;
    UIDeleteData.txID.receiver_ID = clientId;
    UIDeleteData.txID.sender_ID = robotId;

    memcpy(m_uarttx + 5, (uint8_t*)&UIDeleteData.CMD, 8);

    m_uarttx[13] = deleteOperator;
    m_uarttx[14] = deleteLayer;

    dataLength = sizeof(CommunatianData_graphic_t) + 2;

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

/************************************************UI推送图形*************************************************/
void Judgement::UI_ReFresh(int cnt, graphic_data_struct_t* imageData)
{
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length = 6 + cnt * 15;
    graphicData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &graphicData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    graphicData.CMD = UI_CMD_Robo_Exchange;
    switch (cnt)
    {
    case 1:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
        break;
    case 2:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
        break;
    case 5:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
        break;
    case 7:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
        break;
    default:
        break;
    }

    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, imageData,
        cnt * sizeof(graphic_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) +
        cnt * sizeof(graphic_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

/************************************************UI推送浮点*************************************************/
void Judgement::UI_ReFresh(int cnt, float_data_struct_t* floatdata)
{
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length =
        6 + cnt * sizeof(graphic_data_struct_t);
    graphicData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &graphicData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    graphicData.CMD = UI_CMD_Robo_Exchange;
    switch (cnt)
    {
    case 1:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
        break;
    case 2:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
        break;
    case 5:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
        break;
    case 7:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
        break;
    default:
        break;
    }

    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, floatdata,
        cnt * sizeof(graphic_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) +
        cnt * sizeof(graphic_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}


void Judgement::Char_ReFresh(string_data_struct_t* string_Data)
{
    int i, n;
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length = 51;
    graphicData.txFrameHeader.seq = UI_seq;
    memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

    graphicData.CMD = UI_CMD_Robo_Exchange;

    graphicData.txID.data_cmd_id = UI_Data_ID_DrawChar;
    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;                          //填充操作数据

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, string_Data, sizeof(string_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) + sizeof(string_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;                                                         //包序号+1
}
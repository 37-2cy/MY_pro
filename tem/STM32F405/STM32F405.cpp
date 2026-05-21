#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "Power_limit.h"
#include "supercap.h"
#include "xuc_can.h"
// ========= 你的全局对象不改 =========
Motor can1_motor[CAN1_MOTOR_NUM] = {
    Motor(M3508,SPD,chassis, ID1, PID(10.f, 0.0f, 1.5f,0.f)),
    Motor(M3508,SPD,chassis, ID2, PID(10.f, 0.0f, 1.5f,0.f)),
    Motor(M3508,SPD,chassis, ID3, PID(10.f, 0.0f, 1.5f,0.f)),
    Motor(M3508,SPD,chassis, ID4, PID(10.f, 0.0f, 1.5f,0.f)),
    Motor(M2006,SPD,chassis, ID8, PID(5.f, 0.0f, 1.f,0.f)),//-
    Motor(M2006,SPD,chassis, ID5, PID(5.f, 0.0f, 1.f,0.f)),//+
    Motor(M2006,SPD,supply, ID7, PID(5.f, 0.0f, 1.5f,0.f)),
    Motor(M6020,POS_IMU,pantile, ID6, PID(1100.f, 0.1f, 50.f,0.f),PID(1.8f, 0.01f, 12.f,0.f))//PID(2000.f, 0.5f, 50.f,0.f),PID(3.f, 0.01f, 10.0f,0.f)
};
Motor can2_motor[CAN2_MOTOR_NUM] = {
    Motor(M3508,SPD_SHOOT,shooter, ID1, PID(5.f, 0.0f, 1.5f,0.f)),//-
    Motor(M3508,SPD_SHOOT,shooter, ID2, PID(5.f, 0.0f, 1.5f,0.f)),//+
    Motor(M6020,POS,pantile, ID3, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
    Motor(M6020,POS,pantile, ID4, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
    Motor(M6020,POS,pantile, ID7, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
    Motor(M6020,SPD,chassis, ID8, PID(10.f, 0.0f, 1.5f,0.f))
};
DMMOTOR DMmotor[5] = {
    DMMOTOR(0x01, MIT),
    DMMOTOR(0x02, MIT),
    DMMOTOR(0x03, POS_DM),
    DMMOTOR(0x04, POS_DM),
    DMMOTOR(0x05, POS_DM),
};

CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile;
DELAY delay;
RC rc;
POWER power;
LED led1, led2, led3, led4;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;
SUPERCAP supercap;
XUC xuc;

namespace
{
Motor* g_powerMotors[4] = {
    &can1_motor[0],
    &can1_motor[1],
    &can1_motor[2],
    &can1_motor[3],
};
}
// ========= ✅ 新增：IMU CAN Queue 定义 =========
QueueHandle_t g_imu_can_queue = NULL;
QueueHandle_t g_xuc_can_queue = NULL;
static void IMU_CanQueueInit()
{
    g_imu_can_queue = xQueueCreate(16, sizeof(CanRxMsg_t));
    g_xuc_can_queue = xQueueCreate(16, sizeof(CanRxMsg_t));
}


int main(void)
{
    SystemClockConfig();
    delay.Init(168);
    HAL_Init();

    // ✅ 先创建 IMU CAN 队列（在开中断收包前准备好）
    IMU_CanQueueInit();

    can1.Init(CAN1);
    can2.Init(CAN2);

    timer.Init(BASE, TIM3, 1000).BaseInit();

    // ✅ IMU 不再走串口：改成 CAN（StdID=0x99，SCALE=100）
    imu_pantile.InitCAN(g_imu_can_queue, 0x99, 100.0f, CH010);
    xuc.InitCAN(g_xuc_can_queue, 0x98, 100.0f);
    rc.Init(&uart6, USART6, 921600);
    //power.Init(&uart1, USART1, 9600);
    judgement.Init(&uart1, 115200, USART1);
    supercap.Init(&uart4, 115200, UART4);

    para.Init();

    ctrl.Init(std::vector<Motor*>{
        &can2_motor[0],
            & can2_motor[1],
            & can2_motor[2],
            & can2_motor[3],
            & can2_motor[4],
            & can2_motor[5]
    });
    ctrl.Init(std::vector<Motor*>{
        &can1_motor[0],
             & can1_motor[1],
            & can1_motor[2],
            & can1_motor[3],
            & can1_motor[4],
            & can1_motor[5],
            & can1_motor[6]
    });

    powerLimiter.Init(g_powerMotors, 60.0f);
    task.Init();

    for (;;);
}
    

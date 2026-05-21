#include "label.h"
#include "RC.h"
#include "control.h"
#include "HTmotor.h"

namespace
{
float GetChassisSpeedScale(uint8_t modeFlag)
{
    switch (modeFlag)
    {
    case 1:
        return 1.5f;   // aggressive
    case 2:
        return 0.9f;  // conservative
    default:
        return 0.9f;   // normal
    }
}

constexpr float kFacePantileKp = 35.0f;
constexpr float kFacePantileMinSpeed = 500.0f;
constexpr float kFacePantileDeadbandDeg = 2.0f;
constexpr float kFacePantileDir = 1.0f;

float LimitFloat(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

float GetFacePantileSpeed(float speedLimit)
{
    const float yawErrorDeg = ctrl.GetDelta(
        mechanicalToDegree(can1_motor[7].angle[now]) - mechanicalToDegree(para.initial_yaw));

    if (fabsf(yawErrorDeg) <= kFacePantileDeadbandDeg)
    {
        return 0.0f;
    }

    float speed = yawErrorDeg * kFacePantileKp * kFacePantileDir;
    speed = LimitFloat(speed, speedLimit);

    const float minSpeed = (speedLimit < kFacePantileMinSpeed) ? speedLimit : kFacePantileMinSpeed;
    if (fabsf(speed) < minSpeed)
    {
        speed = (speed > 0.0f) ? minSpeed : -minSpeed;
    }

    return speed;
}
}
void RC::Decode()
{
	//if (queueHandler == NULL || *queueHandler == NULL) {
	//	return;  // ���߱���
	//}
	//else {
	//	pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	//}

	//if (sizeof(m_frame) < 18) return;
	//if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	//rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	//rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	//rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	//rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	//if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	//if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	//if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	//if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	//pre_rc.s[0] = rc.s[0];
	//pre_rc.s[1] = rc.s[1];

	//rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	//rc.s[1] = ((m_frame[5] >> 4) & 0x03);

	//pc.x = m_frame[6] | (m_frame[7] << 8);
	//pc.y = m_frame[8] | (m_frame[9] << 8);
	//pc.z = m_frame[10] | (m_frame[11] << 8);
	//pc.press_l = m_frame[12];
	//pc.press_r = m_frame[13];

	//pc.key_h = m_frame[15];//�����ĸ�λ����R F G Z X C 
	//pc.key_l = m_frame[14];//�����ĵ�8λ W S A D SHIFT CTRL Q E

	//
}

int start_time = 0;
uint8_t go_up_lock = 0;
uint8_t step_state = 0;
extern uint8_t flag_shoot;
void RC::OnRC()
{
	if (rc.s[0] == 1)
	{
		ctrl.mode = CONTROL::NORMAL;
	}
	else if (rc.s[0] == 0)
	{
		ctrl.mode = CONTROL::ROTATION;
	}
	else if (rc.s[0] == 2)
	{
		ctrl.mode = CONTROL::AUTOAIM;
	}
	if (rc.go_up == 1)
	{
		can2_motor[0].setspeed = -7000;
		can2_motor[1].setspeed = 7000;
	}
	else {
		can2_motor[0].setspeed = 0;
		can2_motor[1].setspeed = 0;
	}

	//if (rc.go_up == 1 && go_up_lock == 0)
	//{
	//	go_up_lock = 1;
	//	start_time = CNT_1;   // ��¼��ʼʱ��
	//	step_state = 0;       // ��step1ǰ�ĵȴ���ʼ
	//}

	//// ����״̬�£�ÿ��2������ִ�� step1 -> step2 -> step3 -> step4 -> step5����ɺ����
	//if (go_up_lock == 1)
	//{
	//	switch (step_state)
	//	{
	//	case 0: // �ȴ�2s -> step_1
	//		if (CNT_1 - start_time >= 2000)
	//		{
	//			// step_1();
	//			//DMmotor[0].setPos = -0.77f;
	//			//DMmotor[1].setPos = 0.75f;
	//			start_time = CNT_1;
	//			step_state = 1;
	//		}
	//		break;

	//	case 1: // �ٵ�2s -> step_2
	//		if (CNT_1 - start_time >= 200)
	//		{
	//			// step_2();
	//			DMmotor[1].setPos = 0.f;

	//			start_time = CNT_1;
	//			step_state = 2;
	//		}
	//		break;

	//	case 2: // �ٵ�2s -> step_3
	//		if (CNT_1 - start_time >= 1000)
	//		{
	//			// step_3();
	//			// ��������ĵ���������
	//			DMmotor[1].setPos = 0.f;
	//			ctrl.chassis.speedz = -50 * para.max_speed / 660.f;
	//			ctrl.chassis.speedx = 60 * para.max_speed / 660.f;
	//			//ctrl.chassis.speedy = -80 * para.max_speed / 660.f;
	//			start_time = CNT_1;
	//			step_state = 3;
	//		}
	//		break;

	//	case 3: // �ٵ�2s -> step_4
	//		if (CNT_1 - start_time >= 1000)
	//		{
	//			// step_4();
	//			// ��������ĵ��Ĳ�����
	//			DMmotor[1].setPos = 0.f;
	//			ctrl.chassis.speedz = -40 * para.max_speed / 660.f;
	//			ctrl.chassis.speedx = 20 * para.max_speed / 660.f;
	//			ctrl.chassis.speedy = -80 * para.max_speed / 660.f;
	//			start_time = CNT_1;
	//			step_state = 4;
	//		}
	//		break;

	//	case 4: // �ٵ�2s -> step_5
	//		if (CNT_1 - start_time >= 1000)
	//		{
	//			// step_5();
	//			// ��������ĵ��岽����
	//			DMmotor[0].setPos = 0.f;
	//			DMmotor[1].setPos = 0.f;
	//			// �岽ִ����� -> �������
	//			go_up_lock = 0;
	//			step_state = 0;
	//		}
	//		break;

	//	default:
	//		// �������쳣״ֱ̬�Ӹ�λ
	//		go_up_lock = 0;
	//		step_state = 0;
	//		break;
	//	}
	//}
	
	if (ctrl.mode == CONTROL::NORMAL)
	{
		//if (step_state != 3 && step_state != 4) {
		//	
		//}
		//if (step_state != 4 && step_state != 3) {
		//	ctrl.chassis.speedy = rc.ch[3] * para.max_speed / 660.f;
		//}
		//if (step_state != 4 && step_state != 3) {
		//	ctrl.chassis.speedz = -1.f * rc.wheel * para.max_speed / 660.f;
		//}
		//ctrl.Control_Pantile(rc.ch[0] * para.yaw_speed / 660.f, rc.ch[1] * para.pitch_speed / 660.f);
		//can2_motor[0].setspeed = 0;
		//can2_motor[1].setspeed = 0;
		//can1_motor[6].setspeed = 0;
		//ctrl.Control_Pantile_IMU();
		ctrl.chassis.speedx = rc.ch[2] * para.max_speed / 330.f;
		ctrl.chassis.speedy = rc.ch[3] * para.max_speed / 330.f;
		ctrl.chassis.speedz = -1.f * rc.wheel * para.max_speed / 330.f;

		ctrl.chassis.Keep_Direction();
		ctrl.Control_Pantile_IMU();
	}
	else if (ctrl.mode == CONTROL::ROTATION && rc.pause_key != 1)
	{
		//ctrl.chassis.speedz = para.rota_speed;
		//ctrl.chassis.Keep_Direction();
		//can2_motor[0].setspeed =0;
		//can2_motor[1].setspeed =0;

		//ctrl.Control_Pantile_IMU();
		//can1_motor[6].setspeed = 0;
	}
	else if (ctrl.mode == CONTROL::AUTOAIM && rc.pause_key != 1)
	{
		//ctrl.Control_Pantile_IMU();
		//can2_motor[0].setspeed = -7000;
		//can2_motor[1].setspeed = 7000;

	}

	if (rc.go_up == 1)  //����
	{


	}
	if (rc.pause_key == 1)  //��ͣ
	{
		can1_motor[0].setspeed = 0;
		can1_motor[1].setspeed = 0;
		can1_motor[2].setspeed = 0;
		can1_motor[3].setspeed = 0;
		can1_motor[4].setspeed = 0;
		can1_motor[5].setspeed = 0;
		can1_motor[6].setspeed = 0;
		can1_motor[7].setspeed = 0;
		can2_motor[0].setspeed = 0;
		can2_motor[1].setspeed = 0;
		can2_motor[2].setspeed = 0;
		can2_motor[3].setspeed = 0;
		can2_motor[4].setspeed = 0;
		can2_motor[5].setspeed = 0;
		DMmotor[0].setSpeed = 0;
		DMmotor[1].setSpeed = 0;
		DMmotor[2].setSpeed = 0;
	}
}
extern uint8_t flag_shoot;
void RC::OnPC()
{
	if (rc.s[0] != 1) {
		const float chassis_speed_scale = GetChassisSpeedScale(flag_shoot);
		const float chassis_speed_limit = para.max_speed * chassis_speed_scale;
		const float spin_speed_limit = para.rota_speed * chassis_speed_scale;
		const float accel_step = chassis_speed_limit * 0.1f;   // mode-aware ramp
		float target_speedx = 0.0f;
		float target_speedy = 0.0f;
		float target_speedz = 0.0f;


		// ǰ��Ŀ���ٶ�
		if (pc.W)
			target_speedx = chassis_speed_limit;
		else if (pc.S)
			target_speedx = -chassis_speed_limit;

		// ����Ŀ���ٶ�
		if (pc.A)
			target_speedy = -chassis_speed_limit;
		else if (pc.D)
			target_speedy = chassis_speed_limit;

		// ��תĿ���ٶȣ�Z/C����������
		if (pc.Z)
			target_speedz = spin_speed_limit;
		else if (pc.C)
			target_speedz = -spin_speed_limit;

		// ---------------- X���� ----------------
		if (target_speedx == 0.0f)
		{
			ctrl.chassis.speedx = 0.0f;
		}
		else if ((ctrl.chassis.speedx > 0.0f && target_speedx < 0.0f) ||
			(ctrl.chassis.speedx < 0.0f && target_speedx > 0.0f))
		{
			ctrl.chassis.speedx = 0.0f;
		}
		else
		{
			if (ctrl.chassis.speedx < target_speedx)
			{
				ctrl.chassis.speedx += accel_step;
				if (ctrl.chassis.speedx > target_speedx)
					ctrl.chassis.speedx = target_speedx;
			}
			else if (ctrl.chassis.speedx > target_speedx)
			{
				ctrl.chassis.speedx -= accel_step;
				if (ctrl.chassis.speedx < target_speedx)
					ctrl.chassis.speedx = target_speedx;
			}
		}

		// ---------------- Y���� ----------------
		if (target_speedy == 0.0f)
		{
			ctrl.chassis.speedy = 0.0f;
		}
		else if ((ctrl.chassis.speedy > 0.0f && target_speedy < 0.0f) ||
			(ctrl.chassis.speedy < 0.0f && target_speedy > 0.0f))
		{
			ctrl.chassis.speedy = 0.0f;
		}
		else
		{
			if (ctrl.chassis.speedy < target_speedy)
			{
				ctrl.chassis.speedy += accel_step;
				if (ctrl.chassis.speedy > target_speedy)
					ctrl.chassis.speedy = target_speedy;
			}
			else if (ctrl.chassis.speedy > target_speedy)
			{
				ctrl.chassis.speedy -= accel_step;
				if (ctrl.chassis.speedy < target_speedy)
					ctrl.chassis.speedy = target_speedy;
			}
		}

		// ---------------- C���л� speedz ----------------
		if (pc.C == 1 && c_last == 0)
		{
			c_toggle = !c_toggle;
		}
		c_last = pc.C;

		if (pc.V)
		{
			ctrl.chassis.speedz = GetFacePantileSpeed(spin_speed_limit);
		}
		else if (c_toggle)
		{
			ctrl.chassis.speedz = spin_speed_limit;
		}
		else
		{
			ctrl.chassis.speedz = 0.0f;
		}

		////// ---------------- R���л� can1_motor[4][5] ----------------
		//if (pc.R == 1 && r_last == 0)
		//{
		//	r_toggle = !r_toggle;
		//}
		//r_last = pc.R;

		//if (r_toggle)
		//{
		//	leg_state = 1;
		//	can1_motor[4].setspeed = 3000;
		//	can1_motor[5].setspeed = -3000;
		//}
		//else
		//{
		//	leg_state = 0;
		//	can1_motor[4].setspeed = 0;
		//	can1_motor[5].setspeed = 0;
		//}

		// ---------------- F���л� can2_motor[0][1] ----------------
		if (pc.F == 1 && f_last == 0)
		{
			f_toggle = !f_toggle;
		}
		f_last = pc.F;

		if (f_toggle)
		{
			ctrl.shooter.displayOpenRub = true;
			can2_motor[0].setspeed = -6500;
			can2_motor[1].setspeed = 6500;
		}
		else
		{
			ctrl.shooter.displayOpenRub = false;
			can2_motor[0].setspeed = 0;
			can2_motor[1].setspeed = 0;
		}
		// ---------------- G键切换 flag_shoot (1 <-> 2) ----------------
		if (pc.G == 1 && g_last == 0)
		{
			g_toggle = !g_toggle;

			if (g_toggle)
				flag_shoot = 1;
			else
				flag_shoot = 2;
		}
		g_last = pc.G;
		ctrl.Control_Pantile_IMU();

	}
}

void RC::Update()
{
	OnRC();
	OnPC();
}


void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}
void RC::Decode_NEW()
{
	// 0~1: ֡ͷ(0xA9 0x53)
	// 2~18: ������
	// 19~20: CRC16
	constexpr size_t FRAME_LEN = 21;

	if (queueHandler == NULL || *queueHandler == NULL) return;

	if (xQueueReceive(*queueHandler, m_frame, 0) != pdTRUE) return;

	// ֡ͷУ��
	if (m_frame[0] != 0xA9 || m_frame[1] != 0x53) return;

	// CRC16 У��
	if (!RC::verify_crc16_check_sum(m_frame, FRAME_LEN)) return;

	// ---------------- ͨ������ ----------------
	rc.ch[0] = (int16_t)(((m_frame[2] | ((uint16_t)m_frame[3] << 8)) & 0x07FF) - 1024);
	rc.ch[1] = (int16_t)((((m_frame[3] >> 3) | ((uint16_t)m_frame[4] << 5)) & 0x07FF) - 1024);
	rc.ch[2] = (int16_t)((((m_frame[4] >> 6) | ((uint16_t)m_frame[5] << 2) | ((uint16_t)m_frame[6] << 10)) & 0x07FF) - 1024);
	rc.ch[3] = (int16_t)((((m_frame[6] >> 1) | ((uint16_t)m_frame[7] << 7)) & 0x07FF) - 1024);

	// ��������
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8) rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8) rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8) rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8) rc.ch[3] = 0;

	// ---------------- ����/����/���� ----------------
	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = (uint8_t)((m_frame[7] >> 4) & 0x03);   // C:0 N:1 S:2
	rc.s[1] = 0;                                     // ��Э����û�еڶ������˵Ļ�����0

	rc.pause_key = (uint8_t)((m_frame[7] >> 6) & 0x01);
	rc.custom_key_l = (uint8_t)((m_frame[7] >> 7) & 0x01);
	rc.custom_key_r = (uint8_t)((m_frame[8] >> 0) & 0x01);
	rc.go_up = (uint8_t)((m_frame[9] >> 4) & 0x01);

	const uint16_t wheel_raw = (uint16_t)((((uint16_t)m_frame[8] >> 1) | ((uint16_t)m_frame[9] << 7)) & 0x07FF);
	rc.wheel = (int16_t)wheel_raw - 1024;
	if (rc.wheel <= 8 && rc.wheel >= -8) rc.wheel = 0;

	// ---------------- ������ ----------------
	pc.x = (int16_t)(m_frame[10] | ((uint16_t)m_frame[11] << 8));
	pc.y = (int16_t)(m_frame[12] | ((uint16_t)m_frame[13] << 8));
	pc.z = (int16_t)(m_frame[14] | ((uint16_t)m_frame[15] << 8));

	const uint8_t mb = m_frame[16];
	const uint8_t m_left = (uint8_t)((mb >> 0) & 0x03);
	const uint8_t m_right = (uint8_t)((mb >> 2) & 0x03);
	const uint8_t m_mid = (uint8_t)((mb >> 4) & 0x03);

	pc.press_l = (m_left == 1) ? 1 : 0;
	pc.press_r = (m_right == 1) ? 1 : 0;
	pc.press_m = (m_mid == 1) ? 1 : 0;

	// ---------------- ���̽��� ----------------
	const uint16_t key = (uint16_t)(m_frame[17] | ((uint16_t)m_frame[18] << 8));

	// ����ԭʼֵ
	//pc.key_l = (uint8_t)(key & 0xFF);
	//pc.key_h = (uint8_t)((key >> 8) & 0xFF);

	// ���ӳ�䣺����=1��δ��=0
	pc.W = (key >> 0) & 0x01;
	pc.S = (key >> 1) & 0x01;
	pc.A = (key >> 2) & 0x01;
	pc.D = (key >> 3) & 0x01;
	pc.SHIFT = (key >> 4) & 0x01;
	pc.CTRL = (key >> 5) & 0x01;
	pc.Q = (key >> 6) & 0x01;
	pc.E = (key >> 7) & 0x01;

	pc.R = (key >> 8) & 0x01;
	pc.F = (key >> 9) & 0x01;
	pc.G = (key >> 10) & 0x01;
	pc.Z = (key >> 11) & 0x01;
	pc.X = (key >> 12) & 0x01;
	pc.C = (key >> 13) & 0x01;
	pc.V = (key >> 14) & 0x01;
	pc.B = (key >> 15) & 0x01;
}

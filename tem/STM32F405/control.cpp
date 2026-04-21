#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "RC.h"
#include "xuc_can.h"

volatile uint8_t leg_state = 0;

namespace
{
	constexpr float kLegLeftHome = -0.12f;
	constexpr float kLegRightHome = 0.13f;

	constexpr float kLegLeftMin = -0.77f;
	constexpr float kLegRightMax = 0.75f;

	constexpr float kLegSuspensionStage1 = 0.10f;
	constexpr float kLegSuspensionStage2 = 0.20f;
	constexpr float kLegSuspensionMaxTorque = 4.0f;

	// ================= 手动单腿控制位置参数 =================
	// 左腿：负方向抬起�? 为收�?
	constexpr float kLegLeftLiftPos = -0.75f;
	constexpr float kLegLeftRetractPos = 0.0f;

	// 右腿：正方向抬起�? 为收�?
	constexpr float kLegRightLiftPos = 0.75f;
	constexpr float kLegRightRetractPos = 0.0f;

	// ================= 手动单腿控制参数 =================
	// 左腿抬起参数
	constexpr float kLegLeftLiftKp = 480.0f;
	constexpr float kLegLeftLiftKd = 2.65f;

	// 左腿收回参数
	constexpr float kLegLeftRetractKp = 180.0f;
	constexpr float kLegLeftRetractKd = 0.45f;

	// 右腿抬起参数
	constexpr float kLegRightLiftKp = 480.0f;
	constexpr float kLegRightLiftKd = 2.65f;

	// 右腿收回参数
	constexpr float kLegRightRetractKp = 180.0f;
	constexpr float kLegRightRetractKd = 0.45f;

	// 手动模式下，不按键时用于保持当前位置的参�?
	constexpr float kLegHoldKp = 480.0f;
	constexpr float kLegHoldKd = 0.50f;

	enum LegControlState
	{
		LEG_STATE_SUSPENSION = 0,
		LEG_STATE_STEP = 1,
	};
}
void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			//supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
	}
	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;
}


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)

{
	ch_pitch *= (-1.f);
	ch_yaw *= (1.f);//方向相反修改这里正负
	float adjangle = this->pantile.sensitivity * 2;
	ctrl.pantile.mark_pitch -= (float)(adjangle * ch_pitch);

	if (can1_motor[7].mode == POS) {
		ctrl.pantile.mark_yaw -= (float)(adjangle * ch_yaw);

		if (ctrl.pantile.mark_yaw > 8192.0)ctrl.pantile.mark_yaw -= 8192.0;
		if (ctrl.pantile.mark_yaw < 0.0)ctrl.pantile.mark_yaw += 8192.0;
		can1_motor[7].setangle = ctrl.pantile.mark_yaw;
	}
	DMmotor[2].setSpeed = 1.5;
	//DMmotor[2].setPos += (rc.rc.ch[1] / 660.f) * (PI / 360.f);
	if (rc.rc.ch[1] > 330) {
		can1_motor[6].setspeed = 1000;
	}
	else if (rc.rc.ch[1] < -330)
	{
		can1_motor[6].setspeed = -800;
	}
	else {
		can1_motor[6].setspeed = 0;
	}
	if (DMmotor[2].setPos >= 0.40f) DMmotor[2].setPos = 0.40f;
	if (DMmotor[2].setPos <= -0.15f) DMmotor[2].setPos = -0.15f;
}

void CONTROL::Control_Pantile_IMU()

{
	if (can1_motor[7].mode == POS_IMU && imu_pantile.IsDataValid()) {
		can1_motor[7].setangle -= (rc.pc.x / 660.f);
		if (can1_motor[7].setangle > 180.f) {
			can1_motor[7].setangle -= 360.f;                    //陀螺仪控制角度�?180~180
		}
		else if (can1_motor[7].setangle < -180.f) {
			can1_motor[7].setangle += 360.f;
		}
	}
	if (rc.pc.press_r == 1) {
		can1_motor[7].setangle = xuc.GetTargetYaw();
		DMmotor[2].setPos -= xuc.GetTargetPitch() * (PI / 360.f) * 0.01;
	}
	DMmotor[2].setSpeed = 1.5;
	DMmotor[2].setPos += (rc.pc.y / 66.f) * (PI / 360.f);

	if (can1_motor[7].mode == POS_IMU && imu_pantile.IsDataValid()&& ctrl.mode == CONTROL::NORMAL) {
		can1_motor[7].setangle -= (rc.rc.ch[0] / 660.f);
		DMmotor[2].setPos += (rc.rc.ch[1] / 660.f) * (PI / 360.f);
		if (can1_motor[7].setangle > 180.f) {
			can1_motor[7].setangle -= 360.f;                    //陀螺仪控制角度�?180~180
		}
		else if (can1_motor[7].setangle < -180.f) {
			can1_motor[7].setangle += 360.f;
		}
	}
	//if (ctrl.mode == CONTROL::AUTOAIM && xuc.GetTargetYaw() != 0) {
	//	can1_motor[7].setangle = xuc.GetTargetYaw();
	//	DMmotor[2].setPos -= xuc.GetTargetPitch() * (PI / 360.f) * 0.01;
	//}
	//if (rc.rc.ch[1] > 300) {
	//	can1_motor[6].setspeed = 4000;
	//}
	//else if (rc.rc.ch[1] < -300) {
	//	can1_motor[6].setspeed = -500;
	//}
	//else {
	//	can1_motor[6].setspeed = 0;
	//}
	//if (rc.rc.ch[1] > 300) {
	//	can1_motor[6].setspeed = 800;
	//}
	//else if (rc.rc.ch[1] < -300) {
	//	can1_motor[6].setspeed = -800;
	//}
	//else {
	//	can1_motor[6].setspeed = 0;
	//}
	if (DMmotor[2].setPos >= 0.40f) DMmotor[2].setPos = 0.40f;
	if (DMmotor[2].setPos <= -0.15f) DMmotor[2].setPos = -0.15f;


}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference)
{
	float delta = 0, adjust = sensitivity;
	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8291.f;
		if (abs(delta) >= 10.f)
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta);
	}
	else if (type == PITCH)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;

		if (abs(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);
		}
	}
}

void CONTROL::CHASSIS::Keep_Direction()
{
	double s_x = speedx, s_y = speedy;
	double theat = ctrl.GetDelta(mechanicalToDegree(can1_motor[7].angle[now])
		- mechanicalToDegree(para.initial_yaw)) / 180.f;
	double st = sin(theat*PI);
	double ct = cos(theat*PI);
	speedx = s_x * ct + s_y * st;
	speedy = -s_x * st + s_y * ct;
}

void CONTROL::CHASSIS::Update()
{
	double s_x = speedx;
	double s_y = speedy;

	double theta = ctrl.GetDelta(
		mechanicalToDegree(can1_motor[7].angle[now]) - mechanicalToDegree(para.initial_yaw)
	) / 180.0 * PI;

	double st = sin(theta);
	double ct = cos(theta);

	double vx = s_x * ct + s_y * st;
	double vy = -s_x * st + s_y * ct;

	ctrl.chassis_motor[0]->setspeed = +vy + vx - speedz;
	ctrl.chassis_motor[1]->setspeed = -vy + vx - speedz;
	ctrl.chassis_motor[3]->setspeed = -vy - vx - speedz;
	ctrl.chassis_motor[2]->setspeed = +vy - vx - speedz;

	DMmotor[0].setSpeed = 0.0f;
	DMmotor[1].setSpeed = 0.0f;
	//DMmotor[0].setTorque = 0.0f;
	//DMmotor[1].setTorque = 0.0f;
	
	auto applySuspensionProfile = [](DMMOTOR& motor, float home_pos, float torque_sign)
	{
		const float compression = fabsf(motor.pos - home_pos);

		if (compression <= kLegSuspensionStage1)
		{
			motor.Kp = 800.0f;
			motor.Kd = 2.8f;
			//motor.setTorque = torque_sign * (1.2f + 14.0f * compression);
		}
		else if (compression <= kLegSuspensionStage2)
		{
			motor.Kp = 300.0f;
			motor.Kd = 2.1f;
			//motor.setTorque = torque_sign * (2.6f + 4.0f * (compression - kLegSuspensionStage1));
		}
		else
		{
			motor.Kp = 305.0f;
			motor.Kd = 1.5f;
			//motor.setTorque = torque_sign * fminf(3.0f + 2.0f * (compression - kLegSuspensionStage2),
			//	kLegSuspensionMaxTorque);
		}
	};

	switch (leg_state)
	{
	case LEG_STATE_STEP:
	{
		// 手动单腿控制模式
		// Q        -> 左腿抬起
		// SHIFT+Q  -> 左腿收回
		// E        -> 右腿抬起
		// SHIFT+E  -> 右腿收回

		// 默认给保持参数，不按键时保持当前目标�?
		DMmotor[0].Kp = kLegHoldKp;
		DMmotor[0].Kd = kLegHoldKd;
		DMmotor[1].Kp = kLegHoldKp;
		DMmotor[1].Kd = kLegHoldKd;

		// 左腿控制
		if (rc.pc.Q)
		{
			if (rc.pc.SHIFT)
			{
				// 左腿收回
				DMmotor[0].setPos = kLegLeftRetractPos;
				DMmotor[0].Kp = kLegLeftRetractKp;
				DMmotor[0].Kd = kLegLeftRetractKd;
			}
			else
			{
				// 左腿抬起
				DMmotor[0].setPos = kLegLeftLiftPos;
				DMmotor[0].Kp = kLegLeftLiftKp;
				DMmotor[0].Kd = kLegLeftLiftKd;
			}
		}

		// 右腿控制
		if (rc.pc.E)
		{
			if (rc.pc.SHIFT)
			{
				// 右腿收回
				DMmotor[1].setPos = kLegRightRetractPos;
				DMmotor[1].Kp = kLegRightRetractKp;
				DMmotor[1].Kd = kLegRightRetractKd;
			}
			else
			{
				// 右腿抬起
				DMmotor[1].setPos = kLegRightLiftPos;
				DMmotor[1].Kp = kLegRightLiftKp;
				DMmotor[1].Kd = kLegRightLiftKd;
			}
		}
		break;
	}

	case LEG_STATE_SUSPENSION:
	default:
	{
		DMmotor[0].setPos = kLegLeftHome;
		DMmotor[1].setPos = kLegRightHome;
		applySuspensionProfile(DMmotor[0], kLegLeftHome, 1.0f);
		applySuspensionProfile(DMmotor[1], kLegRightHome, -1.0f);
		break;
	}
	}

	// ================= 限位保护 =================
	if (DMmotor[0].setPos > 0.0f)
	{
		DMmotor[0].setPos = 0.0f;
	}
	if (DMmotor[1].setPos < 0.0f)
	{
		DMmotor[1].setPos = 0.0f;
	}
	if (DMmotor[0].setPos < -0.77f)
	{
		DMmotor[0].setPos = kLegLeftMin;
	}
	if (DMmotor[1].setPos > 0.75f)
	{
		DMmotor[1].setPos = kLegRightMax;
	}
}

void CONTROL::PANTILE::Update()
{

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
	if (mark_yaw < 0.0)mark_yaw += 8192.0;

	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

}

void CONTROL::SHOOTER::Update()
{
	//now_bullet_speed = judgement.data.ext_shoot_data_t.bullet_speed;
	if (ctrl.mode == RESET)
	{
		openRub = false;
		supply_bullet = false;
		auto_shoot = false;
	}
	if (openRub)
	{
		ctrl.shooter_motor[0]->setspeed = -speed;
		ctrl.shooter_motor[1]->setspeed = speed;
	}
	else
	{
		ctrl.shooter_motor[0]->setspeed = 0;
		ctrl.shooter_motor[1]->setspeed = 0;
	}

	if (supply_bullet && openRub)
	{
		if (auto_shoot)
		{
			//ctrl.supply_motor[0]->setspeed = 2160;
			//ctrl.supply_motor[0]->spinning = true;
		}
		else
		{
			//ctrl.supply_motor[0]->setspeed = 2160;
			//ctrl.supply_motor[0]->spinning = true;
		}
	}
	else
	{
		//ctrl.supply_motor[0]->spinning = false;
		//ctrl.supply_motor[1]->spinning = false;
	}
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

extern uint8_t Power_stsRx[];

#include "can_receive.h"
//#include "detect_task.h"

//声明电机变量
static motor_measure_t motor_yaw, motor_pit, motor_trigger, motor_chassis[4],motor_shoot[2];
static supercap_measure_t supercap;
//需要Ecd闭环的电机
static motor_ecd_measure_t motorRolling;



static void ecd_value_calculate(motor_ecd_measure_t *motor);

CAN_RxHeaderTypeDef  Rx1Message;
CAN_TxHeaderTypeDef  Tx1Message;


uint32_t pTxMailbox;

void CAN1_Init()						
{
	CAN_FilterTypeDef canfilter;

	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	
	//filtrate any ID you want here
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;
  
	canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;
	//use different filter for can1&can2
	canfilter.FilterBank=0;

	HAL_CAN_ConfigFilter(&hcan1,&canfilter);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan1);
}

void CAN2_Init()						
{
	CAN_FilterTypeDef canfilter;
	
	//canfilter.FilterNumber = 14;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	
	//filtrate any ID you want here
	canfilter.FilterIdHigh = 0x0000;
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = 0x0000;
	canfilter.FilterMaskIdLow = 0x0000;

	canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
	canfilter.FilterActivation = ENABLE;
	canfilter.SlaveStartFilterBank = 14;
	//use different filter for can1&can2
	canfilter.FilterBank=14;

	HAL_CAN_ConfigFilter(&hcan2,&canfilter);
	
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_CAN_Start(&hcan2);
}

//返回yaw电机变量地址，通过指针方式获得原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获得原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//返回trigger电机变量地址，通过指针方式获得原始数据
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//返回底盘电机变量地址，通过指针方式获得原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

const motor_measure_t *get_Shoot_Motor_Measure_Point(uint8_t i)
{
	  return &motor_shoot[(i & 0x03)];
}

const motor_ecd_measure_t *get_Rolling_Motor_Measure_Point(void)
{
    return &motorRolling;
}

const supercap_measure_t *get_SuperCap_Measure_Point(void)
{
    return &supercap;
}

//统一处理can中断函数，并记录发送数据的时间，作为离线判断的依据
void CAN1_Getdata(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[])
{
	switch (pHeader->StdId)
	{
		case CAN_YAW_MOTOR_ID:
		{
			//处理云台电机数据		
			get_motor_measure(&motor_yaw, aData);
			//记录时间
			//etectHook(YawGimbalMotorTOE);
			
		}break;
		case CAN_PIT_MOTOR_ID:
		{
			//处理云台电机数据
			get_motor_measure(&motor_pit, aData);
			//记录时间
			//DetectHook(YawGimbalMotorTOE);
			
		}break;
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		{
			static uint8_t i = 0;
			//处理电机ID号
			i = pHeader->StdId - CAN_3508_M1_ID;
			//处理电机数据
			get_motor_measure(&motor_chassis[i], aData);
			//记录时间
			//DetectHook(ChassisMotor1TOE + i);
			
		}break;
        case CAN_SUPERCAP_ID:
        {
            get_supercap_measure(&supercap,aData);
        }break;
		default:
		{
			
		}break;
	}
}

void CAN2_Getdata(CAN_RxHeaderTypeDef *pHeader,uint8_t aData[])
{
    switch (pHeader->StdId)
	{
		case CAN_FRICTION_LEFT_MOTOR_ID:
		{
			//得到右摩擦轮电机数据
			get_motor_measure(&motor_shoot[0],aData);
			
		}break;
		case CAN_FRICTION_RIGHT_MOTOR_ID:
		{
			//得到左摩擦轮电机数据
			get_motor_measure(&motor_shoot[1],aData);
			
		}break;
		case CAN_RAMMER_ID:
		{
			get_motor_measure(&(motorRolling.motor_measure),aData);
            ecd_value_calculate(&motorRolling);
		}break;
		
		default:
		{
			
		}break;
	}
}

void get_motor_measure(motor_measure_t *motor,uint8_t aData[])
{
	motor->last_ecd = motor->ecd;
	motor->ecd = aData[0]<<8|aData[1];
	motor->speed_rpm = aData[2]<<8|aData[3];
	motor->given_current = aData[4]<<8|aData[5];
	motor->temperate = aData[6];	
}

void get_supercap_measure(supercap_measure_t *cap, uint8_t aData[])
{
    cap->inputvolt    = aData[1]<<8|aData[0];
    cap->capvolt      = aData[3]<<8|aData[2];
    cap->inputcurrent = aData[5]<<8|aData[4];
    cap->setpower     = aData[7]<<8|aData[6];
}

void CAN_CMD_Chassis(int16_t iq1,int16_t iq2,int16_t iq3,int16_t iq4)
{
	static uint8_t TxData[8];
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;
	TxData[6] = iq4 >> 8;
	TxData[7] = iq4;
	
	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}

void CAN_CMD_Gimbal(int16_t pitch,int16_t yaw)
{
	static uint8_t TxData[8];
	Tx1Message.StdId = 0x1ff;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
	TxData[0] = pitch >> 8;
	TxData[1] = pitch;
	TxData[2] = yaw >> 8;
	TxData[3] = yaw;

	HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}


void CAN_CMD_Shoot(int16_t iq1,int16_t iq2,int16_t iq3)
{
	static uint8_t TxData[8];
	Tx1Message.StdId = 0x200;
	Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
	
    //摩擦轮
	TxData[0] = iq1 >> 8;
	TxData[1] = iq1;
	TxData[2] = iq2 >> 8;
	TxData[3] = iq2;
    //拨弹轮
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;

	
	HAL_CAN_AddTxMessage(&hcan2, &Tx1Message,  TxData, &pTxMailbox);
}

void CAN_CMD_SUPERCAP(int16_t dataset)
{
    static uint8_t TxData[2];
    Tx1Message.StdId = 0x210;
    Tx1Message.IDE = CAN_ID_STD;
	Tx1Message.RTR = CAN_RTR_DATA;
	Tx1Message.DLC = 0x08;
    
    TxData[0] = dataset >> 8;
	TxData[1] = dataset;
    
    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,  TxData, &pTxMailbox);
}


/** 
* 电机Ecd值闭环计算
* @param[in]   	有Ecd参数的电机使用. 
* @param[out]  	无.  
* @par 修改日志 
*      		zd于2020-12-03创建 
*/
static void ecd_value_calculate(motor_ecd_measure_t *motor)
{
	int16_t Ecd_Dvalue = motor->motor_measure.ecd - motor->motor_measure.last_ecd;
	
	if (Ecd_Dvalue > ECDDVALE_MAX)
	{
		Ecd_Dvalue = -FULLECD + Ecd_Dvalue;
	}
	else if (Ecd_Dvalue < -ECDDVALE_MAX)
	{
		Ecd_Dvalue = Ecd_Dvalue + FULLECD; 
	}
	
	motor->EcdPosition += ROLLINGDIRECTION * Ecd_Dvalue;
	
	if (motor->EcdPosition >= 0x7FFFFFFFFFF00000) 
	{
		motor->EcdPosition -= 0x7FFFFFFFFFF00000;
	}
	else if (motor->EcdPosition <= -0x7FFFFFFFFFF00000)
	{
		motor->EcdPosition -= 0x7FFFFFFFFFF00000;
	}
}

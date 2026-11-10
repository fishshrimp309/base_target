#include "bsp_can.h"
#include "Motor.h"

motor_info_t motor_info[9] = {0};

static void CAN1_Rx0Callback(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data);
static void CAN2_Rx0Callback(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data);

void can_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0;
  can_filter.FilterIdLow  = 0;
  can_filter.FilterMaskIdHigh = 0; 
  can_filter.FilterMaskIdLow  = 0; // Accept all IDs
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank  = 14;
  
  if (hcan->Instance == CAN1)
  {
    can_filter.FilterBank = 0;
  }
  else //CAN2
  {
    can_filter.FilterBank = 14;
  }

  HAL_CAN_ConfigFilter(hcan, &can_filter);
  HAL_CAN_Start(hcan);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  if (hcan->Instance == CAN1)
	{
			CAN1_Rx0Callback(&rx_header, rx_data);
	}
  else if (hcan->Instance == CAN2)
	{
			CAN2_Rx0Callback(&rx_header, rx_data);
	}
}

void CAN1_Rx0Callback(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data)
{
  switch(rx_header->StdId)
  {
    case 0x201:
      motor_info[0].rotor_pulse = (rx_data[0] << 8) | rx_data[1];
      motor_info[0].rotor_rpm = (rx_data[2] << 8) | rx_data[3];
      motor_info[0].torque_current = (rx_data[4] << 8) | rx_data[5];
      motor_3508.angle = (float)motor_info[0].rotor_pulse * ANGLE_RAW_TO_RAD;
      motor_3508.speed = (float)motor_info[0].rotor_rpm * SPEED_RPM_TO_RAD_S;
      motor_3508.current = (float)motor_info[0].torque_current * CURRENT_TO_A;
      break;
    default:
      break;
  }
}

void CAN2_Rx0Callback(CAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data)
{
  switch(rx_header->StdId)
  {
	default:
      break;
  }
}

void set_motor_current(CAN_HandleTypeDef* hcan, uint32_t std_id, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
  uint32_t            tx_mailbox;

  tx_header.StdId = std_id;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] = (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] = (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] = (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] = (v4)&0xff;

  HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);  // ¡û 2. ´«µØÖ·
}

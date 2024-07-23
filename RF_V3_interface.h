/*
 * RF_V3_interface.h
 *
 *  Created on: Nov 17, 2023
 *      Author: Norana
 */

#ifndef HAL_RF_V3_RF_V3_INTERFACE_H_
#define HAL_RF_V3_RF_V3_INTERFACE_H_


void H_RF_ChipSelect(void);
void H_RF_ChipUnSelect(void);

void H_RF_RxTxEnable(void);
void H_RF_RxTxDisable(void);


void H_RF_ReadRegister(uint8 Register, uint8 *ReadRegister);
void H_RF_DebugRegisters_UART (void);
void H_RF_WriteToRegister(uint8 Register, uint8 Copy_uint8Data);
void H_RF_WriteCommand(uint8 Copy_uint8Command);
void H_RF_WriteToRegister_NBytes(uint8 Register, uint8 *Copy_uint8Data,uint8 Size);

void H_RF_Activate_cmd(void);
void H_RF_FLUSH_RX(void);

void H_RF_Reset(void);

void H_RF_PowerDown(void);
void H_RF_PowerUp(void);



void H_RF_Receiver_Init (uint8 Copy_uint8PipeNumber);
uint8 H_RF_RX_IsDataAvailable (uint8 Copy_uint8PipeNumber);
void H_RF_Receive (uint8 Copy_uint8PipeNumber,uint8 *ReceivedData);


#endif /* HAL_RF_V3_RF_V3_INTERFACE_H_ */

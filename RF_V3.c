/*
 * RF_V3.c
 *
 *  Created on: Nov 17, 2023
 *      Author: Norana
 */


/*			Replace with your libraries 			*/
/*
#include "../Inc/LIB/BIT_MATH.h"
#include "../Inc/LIB/STD_TYPES.h"

#include "../Inc/MCAL/RCC/RCC_interface.h"
#include "../Inc/MCAL/DIO/DIO_interface.h"
#include "../Inc/MCAL/SPI/SPI_interface.h"
#include "../Inc/MCAL/USART/UART_interface.h"

#include "../Inc/HAL/RF_V3/RF_V3_interface.h"
#include "../Inc/HAL/RF_V3/RF_V3_private.h"
#include "../Inc/HAL/RF_V3/RF_V3_config.h"
*/

/*****************************/
static uint32 Iterator = 0;

/*****************************/
/*********************************************************************/
void H_RF_ChipSelect (void){
    MDIO_voidWrite_D_pin(CSN_PORT,CSN_PIN, LOW); //SPI chip select //active low
}
/*********************************************************************/
void H_RF_ChipUnSelect(void){
    MDIO_voidWrite_D_pin(CSN_PORT,CSN_PIN, HIGH);  //SPI chip unselect
}
/*********************************************************************/
void H_RF_RxTxEnable(void){
    MDIO_voidWrite_D_pin(CE_PORT,CE_PIN, HIGH); //enables the RX and TX modes
}
/*********************************************************************/
void H_RF_RxTxDisable(void){
    MDIO_voidWrite_D_pin(CE_PORT,CE_PIN, LOW); //Disables the RX and TX modes
}
/*********************************************************************/
void H_RF_ReadRegister(uint8 Register, uint8 *ReadRegister){
	uint8 buffer[2];
	buffer[0] = Register & 0x1f;
	buffer[1] = NOP;
    // select the slave (rf module)
    H_RF_ChipSelect();

    SPI_transmit(RF_SPI_NUMBER ,buffer, 2);

    uint16 temp ;
    temp = MSPI_uint16EnableTranceive (RF_SPI_NUMBER,temp);
    *ReadRegister = (uint8)(temp &0x00ff);
    //for (Iterator = 0; Iterator < 100; Iterator++);
    // release the slave (rf module)
    //*ReadRegister = SPI_transmit(RF_SPI_NUMBER ,&(buffer[1]), 1);

    H_RF_ChipUnSelect();
}
/*********************************************************************/
void H_RF_DebugRegisters_UART (void){
	/************* UART 1 init ***********/
	MRCC_voidEnableClock(RCC_APB2,RF_DebugUART_EN);
	MDIO_INIT(DebugUART_PORT);
	MDIO_voidSetDirection(DebugUART_PORT,DebugUART_TX,AF_2MHZ_PUSH_PULL); //TX
	MDIO_voidSetDirection(DebugUART_PORT,DebugUART_RX,INPUT_FLOATING); //RX
	MUSART_voidInit(RF_DebugUART);
	/**************************************/
	uint8 Temp=0,reg;
	//1-
	MUSART_voidSendWord(RF_DebugUART,"CONFIG : ");
	 H_RF_ReadRegister(CONFIG_REG, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');
	 //2-
	 MUSART_voidSendWord(RF_DebugUART,"EN_AA : ");
	 H_RF_ReadRegister(EN_AA, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');

	 //3-
	 MUSART_voidSendWord(RF_DebugUART,"EN_RXADDR : ");
	 H_RF_ReadRegister(EN_RXADDR, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');


	 //5-
	 MUSART_voidSendWord(RF_DebugUART,"SETUP_AW : ");
	 H_RF_ReadRegister(SETUP_AW, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');

	 //6-
	 MUSART_voidSendWord(RF_DebugUART,"SETUP_RETR : ");
	 H_RF_ReadRegister(SETUP_RETR, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');

	 //7-
	 MUSART_voidSendWord(RF_DebugUART,"RF_CH : ");
	 H_RF_ReadRegister(RF_CH, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');

	 //8-
	 MUSART_voidSendWord(RF_DebugUART,"RF_SETUP : ");
	 H_RF_ReadRegister(RF_SETUP, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');

	 //9-
	 MUSART_voidSendWord(RF_DebugUART,"STATUS : ");
	 H_RF_ReadRegister(STATUS, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');
	 //10-
	 MUSART_voidSendWord(RF_DebugUART,"FIFO_STATUS : ");
	 H_RF_ReadRegister(FIFO_STATUS, &reg);
	 //reg=0xaf;
	 Temp = reg & 0xf0;
	 Temp >>=4;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);
	 Temp = reg & 0x0f;
	 Temp;
	 if (Temp >9){
		 Temp +=0x37;
	 }
	 else{
		 Temp+='0';
	 }
	 MUSART_voidSendByte(RF_DebugUART,Temp);

	 MUSART_voidSendByte(RF_DebugUART,'\n');
}
/*********************************************************************/
void H_RF_WriteToRegister(uint8 Register, uint8 Copy_uint8Data){
	uint8 buffer [2] ;
	Register &=0x1f; //mask
	buffer [0] = Register|0x20; //to force one in the fifth bit 001A AAAA
	buffer [1] = Copy_uint8Data;

    // select the slave (rf module)
    H_RF_ChipSelect();

    SPI_transmit(RF_SPI_NUMBER, buffer,2);

    // release the slave (rf module)
    H_RF_ChipUnSelect();
}
/*********************************************************************/
void H_RF_WriteCommand(uint8 Copy_uint8Command){
    // select the slave (rf module)
    H_RF_ChipSelect();

    SPI_transmit(RF_SPI_NUMBER , &Copy_uint8Command,1);

    // release the slave (rf module)
    H_RF_ChipUnSelect();
}
/*********************************************************************/
void H_RF_WriteToRegister_NBytes(uint8 Register, uint8 *Copy_uint8Data,uint8 Size){
	// select the slave (rf module)
	H_RF_ChipSelect();

	Register = (Register & 0x1f)|0x20;
	SPI_transmit(RF_SPI_NUMBER ,&Register,1);

	SPI_transmit(RF_SPI_NUMBER ,Copy_uint8Data,Size);// LSB in index zero

	// release the slave (rf module)
	H_RF_ChipUnSelect();

}
/*********************************************************************/
//for DYNPD and FEATURE registers accessing
void H_RF_Activate_cmd(void){
	H_RF_WriteCommand(ACTIVATE_COMMAND);
	H_RF_WriteCommand(ACTIVATE_DATA);
}
/*********************************************************************/
void H_RF_FLUSH_RX(void){
	H_RF_WriteCommand(FLUSH_RX);
}
/*********************************************************************/
void H_RF_Reset(void){
	/*************************************/
	H_RF_WriteToRegister(CONFIG_REG, 0x08);
	H_RF_WriteToRegister(EN_AA, 	0x3f);
	H_RF_WriteToRegister(EN_RXADDR, 0x03);
	H_RF_WriteToRegister(SETUP_AW, 	0x03);
	H_RF_WriteToRegister(SETUP_RETR,0x03);
	H_RF_WriteToRegister(RF_CH, 	0x02);
	H_RF_WriteToRegister(RF_SETUP, 	0x0f);/// 0x0e in viedo ,, 0x0f in data sheet and old code
	H_RF_WriteToRegister(STATUS, 	0x0e);/// 0x00 in viedo , 0x0e in data sheet and old code
	H_RF_WriteToRegister(OBSERVE_TX,0x00);
	H_RF_WriteToRegister(CD, 		0x00);
	/************** pipes addresses *****************/
	uint8 rx_addr_p0 [5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	H_RF_WriteToRegister_NBytes(RX_ADDR_P0,rx_addr_p0,5);

	uint8 rx_addr_p1 [5] = {0xC2,0xC2,0xC2,0xC2,0xC2};
	H_RF_WriteToRegister_NBytes(RX_ADDR_P1,rx_addr_p1,5);

	H_RF_WriteToRegister(RX_ADDR_P2,0xC3);
	H_RF_WriteToRegister(RX_ADDR_P3,0xC4);
	H_RF_WriteToRegister(RX_ADDR_P4,0xC5);
	H_RF_WriteToRegister(RX_ADDR_P5,0xC6);
	/**************** Tx Address *******************/
	H_RF_WriteToRegister_NBytes(TX_ADDR,rx_addr_p0,5);
	/****************************************/
	H_RF_WriteToRegister(RX_PW_P0, 		0x00);
	H_RF_WriteToRegister(RX_PW_P1, 		0x00);
	H_RF_WriteToRegister(RX_PW_P2, 		0x00);
	H_RF_WriteToRegister(RX_PW_P3, 		0x00);
	H_RF_WriteToRegister(RX_PW_P4, 		0x00);
	H_RF_WriteToRegister(RX_PW_P5, 		0x00);
	/****************************************/
	H_RF_WriteToRegister(FIFO_STATUS, 	0x11);
	/************** *Features *******************/
	H_RF_Activate_cmd();
	H_RF_WriteToRegister(DYNPD, 		0x00);
	H_RF_WriteToRegister(FEATURE, 		0x00);
}
/*********************************************************************/
void H_RF_PowerDown(void){
	uint8 Temp = 0;

	H_RF_RxTxDisable(); // CE = 0

	H_RF_ReadRegister(CONFIG_REG, &Temp); // PWR_UP = 0
	CLR_BIT(Temp,1);
	H_RF_WriteToRegister(CONFIG_REG,Temp);
}
/*********************************************************************/
void H_RF_PowerUp(void){
	uint8 Temp = 0;

	H_RF_ReadRegister(CONFIG_REG, &Temp); // PWR_UP = 0
	SET_BIT(Temp,1);
	H_RF_WriteToRegister(CONFIG_REG,Temp);

	for (Iterator = 0; Iterator < 16000; Iterator++); //2 ms >> power_Down to Standby-I mode
}
/*********************************************************************/
void H_RF_Receiver_Init (uint8 Copy_uint8PipeNumber){
	// init the spi
	MRCC_voidEnableClock(RCC_APB2,RF_SPI_EN);
	MSPI_voidInit(RF_SPI_NUMBER);
	// init PORTA
	MDIO_INIT(RF_SPI_PORT);
	MRCC_voidEnableClock(RCC_APB2,RF_SPI_PORT_EN);//////// nagham
	/****************************************/
	// set the CE and CSN pins as output
	MDIO_voidSetDirection(CSN_PORT,CSN_PIN,OUTPUT_2MHZ_PUSH_PULL);
	MDIO_voidWrite_D_pin(CSN_PORT,CSN_PIN, HIGH); // initially high >>> unselect
	MDIO_voidSetDirection(CE_PORT,CE_PIN, OUTPUT_2MHZ_PUSH_PULL);
	MDIO_voidWrite_D_pin(CE_PORT,CE_PIN, LOW); // initially low >>> unenable
	MDIO_voidSetDirection(CLK_PORT,CLK_PIN, AF_2MHZ_PUSH_PULL);
	MDIO_voidSetDirection(MISO_PORT,MISO_PIN, INPUT_FLOATING);
	MDIO_voidSetDirection(MOSI_PORT,MOSI_PIN, AF_2MHZ_PUSH_PULL);
	/****************************************/
	uint32 Iterator =0;
	 for (Iterator = 0; Iterator < 41600; Iterator++);
	/****************************************/
    //1-but the device in the power down mode by clear bit PWR_UP in the config register and disabel TX_RX operations
	H_RF_PowerDown();
	//2-reset all registers
	H_RF_Reset();

    //3- enable auto ack
    uint8 Temp = 0;
    H_RF_ReadRegister(EN_AA,&Temp);
    SET_BIT(Temp,0);
    H_RF_WriteToRegister(EN_AA,Temp);

    //4- set air data rate to 1Mbps
    H_RF_ReadRegister(RF_SETUP,&Temp);
    CLR_BIT(Temp,3);
    H_RF_WriteToRegister(RF_SETUP,Temp);

    //5- set the payload length >>> one byte
    H_RF_WriteToRegister(RX_PW_P0,1);

    //6-Flush rx buffer and tx buffers
   	H_RF_WriteCommand(FLUSH_RX);
   	H_RF_WriteCommand(FLUSH_TX);

    //7-Clear RX_DR , TX_DS , MAX_RT flags in status register
    H_RF_ReadRegister(STATUS,&Temp);
	Temp |=0x70;
	H_RF_WriteToRegister(STATUS,Temp);

	//8- enable CRC with 1 byte and enable RX mode
	H_RF_ReadRegister(CONFIG_REG, &Temp);
	SET_BIT(Temp,0);// PRX=1
	H_RF_WriteToRegister(CONFIG_REG,Temp);

    //10-Power up
	H_RF_PowerUp();

    //12-enable the device in rx mode
    H_RF_RxTxEnable();

    //13- wait for 130 usec
    for (Iterator = 0; Iterator < 1040; Iterator++); //130 us >>  Standby-I mode to RX mode

}
/*********************************************************************/
uint8 H_RF_RX_IsDataAvailable (uint8 Copy_uint8PipeNumber){
	uint8 Temp = 0;
	H_RF_ReadRegister(STATUS,&Temp);//////
	if (GET_BIT(Temp,6)){
		Temp = 1;
		H_RF_WriteToRegister(STATUS,(1<<6));//clear flag
	}
	else{
		Temp = 0;
	}
	return Temp;
}
/*********************************************************************/
void H_RF_Receive (uint8 Copy_uint8PipeNumber,uint8 *ReceivedData){
    //1- rf select
    H_RF_ChipSelect();

    //2-read rx fifo
    uint8 buffer [2];
    buffer [0] = R_RX_PAYLOAD;
    buffer [1] = NOP; // dumy data to receive the register from the rf

    SPI_transmit(RF_SPI_NUMBER ,buffer, 2);

    uint16 temp ;
	temp = MSPI_uint16EnableTranceive (RF_SPI_NUMBER,temp);
	*ReceivedData = (uint8)(temp &0x00ff);

    // delay >>>>>> problem
    //4-flush the rx fifo
    //H_RF_WriteCommand(FLUSH_RX);


    //5-rf unselect
    H_RF_ChipUnSelect();

}
/*********************************************************************/


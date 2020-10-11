#include "CTR_nrf24l01.h"
//---------dia chi slave 1------------//
//unsigned char  CTR_TX_ADDRESS[CTR_TX_ADR_WIDTH]= {0x11,0x11,0x11,0x11,0x01};   //5 bytes
//unsigned char  CTR_RX_ADDRESS[CTR_RX_ADR_WIDTH]= {0x11,0x11,0x11,0x11,0x01};  //5 bytes
//---------dia chi slave 1------------//

//---------dia chi slave 2------------//
unsigned char  CTR_TX_ADDRESS[CTR_TX_ADR_WIDTH]= {0x11,0x11,0x11,0x11,0x02};   //5 bytes
unsigned char  CTR_RX_ADDRESS[CTR_RX_ADR_WIDTH]= {0x11,0x11,0x11,0x11,0x02};  //5 bytes
//---------dia chi slave 2------------//

void CTR_nrfPinConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = NRF_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin NRF_SCK_Pin NRF_MOSI_Pin 
                           LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin|NRF_SCK_Pin|NRF_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);// CHU Y CHAN GPIO GI?
}
//Lenh gui data den NRF
unsigned char CTR_spiRW(unsigned char Buff)//reg=buff=0x20 + 0x10
{
	unsigned char c;
	for(c=0;c<8;c++)//output 8-bit
	{
		if( (Buff&0x80) == 0x80)
			HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port,NRF_MOSI_Pin,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(NRF_MOSI_GPIO_Port,NRF_MOSI_Pin,GPIO_PIN_RESET);
		HAL_Delay(5);
		Buff =  Buff<<1;
		HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_SET);
		HAL_Delay(5);
		Buff |= HAL_GPIO_ReadPin(NRF_MISO_GPIO_Port,NRF_MISO_Pin);//read data from NRF
		HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_RESET);		
	}
	return (Buff);// tra ve ket qua buff kieu unsigned char
}
//Doc gia tri tra ve tu cac thanh ghi NRF
unsigned char CTR_spiRead(unsigned char reg)
{
	unsigned char reg_value=0;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);	//spi enable	CSN=0
	CTR_spiRW(reg);
	reg_value = CTR_spiRW(0x00);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	//spi disable	CSN=1
	return reg_value;// tra ve gia tri doc tren thanh ghi
}

unsigned char CTR_spiRWreg(unsigned char reg, unsigned char value)
{
	unsigned char status;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);	
	status = CTR_spiRW(reg);
	CTR_spiRW(value);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	
	return status;
}

unsigned char CTR_spiReadBuff(unsigned char reg, unsigned char *buff, unsigned char uchars)
{
	unsigned char status, c;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);//spi enable	CSN=0
	status = CTR_spiRW(reg);// Select register to write to and read status uchar
	for(c=0;c<uchars;c++)//luu chuoi doc duoc tu chip
	{
		buff[c] = CTR_spiRW(0x00);
	}
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	//spi disable	CSN=1
	return status;
}

unsigned char CTR_spiWriteBuff(unsigned char reg, unsigned char *buff, unsigned char uchars)//reg: thanh ghi // *buff: dia chi // uchars: kich thuoc dia chi
{
	unsigned char status,c;
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_RESET);//spi enable	CSN=0
	status = CTR_spiRW(reg);//reg=0x20 + 0x10 // Select register to write to and read status uchar
	//printf("%d",status);
	for(c=0;c<uchars;c++)// viet chuoi len chip
	{
		CTR_spiRW(*buff++);
	}
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);	//spi disable	CSN=1
	return status;
}

void CTR_nrfInit(void)
{
	HAL_Delay(100);
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);//chip enable CE=0
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port,NRF_CSN_Pin,GPIO_PIN_SET);//Spi disable CSN=1
	HAL_GPIO_WritePin(NRF_SCK_GPIO_Port,NRF_SCK_Pin,GPIO_PIN_RESET);//Spi clock line init high SCK=0
	
	CTR_spiWriteBuff(CTR_WRITE_RE + CTR_TX_ADDR, CTR_TX_ADDRESS, CTR_TX_ADR_WIDTH);    // 0x20 + 0x10, cau hinh dia chi truyen 3-5 bytes
  //--openWritingPipe tren ca 2 luon 0 va 1--//
	CTR_spiWriteBuff(CTR_WRITE_RE + CTR_RX_ADDR_P0, CTR_RX_ADDRESS, CTR_RX_ADR_WIDTH); // 0x20 + 0x0A,	cau hinh dia chi nhan len luong P0
  CTR_spiRWreg(CTR_WRITE_RE + CTR_RX_PW_P0, CTR_RX_PLOAD_WIDTH);  // Do rong data NHAN 32 byte luong P0
	//CTR_spiWriteBuff(CTR_WRITE_RE + CTR_RX_ADDR_P1, CTR_RX_ADDRESS, CTR_RX_ADR_WIDTH); // 0x20 + 0x0A,	cau hinh dia chi nhan len luong P0
  //CTR_spiRWreg(CTR_WRITE_RE + CTR_RX_PW_P1, CTR_RX_PLOAD_WIDTH);  // Do rong data truyen 32 byte
	//--openWritingPipe--//
	CTR_spiRWreg(CTR_WRITE_RE + CTR_EN_AA, 0x01);//Enable auto acknowledgement data pipe 0      
	/*cho phep dung che do ACK o luong nao
	Reserved 	7:6 	00 	R/W Only '00' allowed
	ENAA_P5 	5 		1 	R/W Enable auto acknowledgement data pipe 5
	ENAA_P4 	4 		1 	R/W Enable auto acknowledgement data pipe 4
	ENAA_P3 	3 		1 	R/W Enable auto acknowledgement data pipe 3
	ENAA_P2 	2 		1 	R/W Enable auto acknowledgement data pipe 2
	ENAA_P1 	1 		1 	R/W Enable auto acknowledgement data pipe 1
	ENAA_P0 	0 		1 	R/W Enable auto acknowledgement data pipe 0
	*/
  CTR_spiRWreg(CTR_WRITE_RE + CTR_EN_RXADDR, 0x01);  //chon luong nhan la P0, chon luong nhan du lieu co cac option nhu ack
  CTR_spiRWreg(CTR_WRITE_RE + CTR_RF_CH, 4);        // Chanel 1 RF = 2400 + RF_CH* (1 or 2 M) chon kenh 1	 
  CTR_spiRWreg(CTR_WRITE_RE + CTR_RF_SETUP, 0x07);        // 1M, 0dbm
																													//M?ch Thu Phát RF NRF24L01 + PA LNA 2.4Ghz Anten R?i.||-18dBm,-12dBm,-6dBm,0dBm,20dBm
																													//M?ch Thu Phát RF NRF24L01 +                         ||-18dBm,-12dBm,-6dBm,0dBm
	
  CTR_spiRWreg(CTR_WRITE_RE + CTR_CONFIG, 0x0e);         // Enable CRC, 2 byte CRC,power up, Send (cau hinh truyen)
	//CTR_spiRWreg(CTR_WRITE_RE + CTR_STATUS, 0xFF);//XOA BIT tren thanh Status(nen xoa truoc khi vao che do nhan moi kiem tra xem tren thanh bao co du lieu moi hay khong)
}

void CTR_nrfSetRX(void)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);//enable chip
	CTR_spiRWreg(CTR_WRITE_RE + CTR_CONFIG, 0x0f);//Enable CRC, 2 byte CRC,power up, Receive (cau hinh nhan)
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);//disable chip
	HAL_Delay(130);
}

void CTR_nrfSetTX(void)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);//enable chip
	CTR_spiRWreg(CTR_WRITE_RE + CTR_CONFIG, 0x0e);//Enable CRC, 2 byte CRC,power up, Transmit (cau hinh truyen)
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);//disable chip
	HAL_Delay(130);
}

unsigned char CTR_nrfGetPacket(unsigned char*rx_buf)
{
	unsigned char value=0,sta;
	sta = CTR_spiRead(CTR_STATUS);//doc tren thanh ghi STATUS:0x07 //=radio.available() ben arduino 
	if((sta&0x40) != 0)// kiem tra tai bit thu 6 la 0(khong co chuoi) hay 1(co chuoi moi)
	{
		HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);//enable chip
		CTR_spiReadBuff(CTR_RD_RX_PLOAD,rx_buf,CTR_TX_PLOAD_WIDTH);//doc tren thanh ghi 0x61 va ghi vao chuoi doc ve rx_buf gia tri nhan duoc
		value = 1;
	}
	CTR_spiRWreg(CTR_WRITE_RE+CTR_STATUS,sta);//ghi len thanh ghi STATUS gia tri sta
	return value;//nhac lai CTR_nrfGetPacket = 1 hay bang 0
}

void CTR_nrfSendPacket(unsigned char*tx_buf)
{
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_RESET);//enable chip
	CTR_spiWriteBuff(CTR_WRITE_RE + CTR_RX_ADDR_P0, CTR_TX_ADDRESS, CTR_TX_ADR_WIDTH);// Send Address
	CTR_spiWriteBuff(CTR_WR_TX_PLOAD, tx_buf, CTR_TX_PLOAD_WIDTH);//send data bang cach ghi data len thanh ghi 0xA0
	CTR_spiRWreg(CTR_WRITE_RE + CTR_CONFIG, 0x0e);// Send Out //Enable CRC, 2 byte CRC,power up, Transmit (cau hinh truyen)
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port,NRF_CE_Pin,GPIO_PIN_SET);//disable chip
	CTR_spiRWreg(CTR_WRITE_RE + CTR_STATUS, 0xFF);//XOA BIT tren thanh Status(nen xoa truoc khi vao che do nhan moi kiem tra xem tren thanh bao co du lieu moi hay khong)
}

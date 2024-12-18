#include <rfid.h>
#include "oled.h"
enum rfid_rwcommand_frames
{
    COMMAND_LENGTH = 0,
    RFID_ADDR,
    COMMAND_CODE,
    CHANNEL,
    BLOCK,
    BLOCKNUM,
    CARD_TYPE,
    SECTION_PASSWORD_INDEX1,
    SECTION_PASSWORD_INDEX2,
    SECTION_PASSWORD_INDEX3,
    SECTION_PASSWORD_INDEX4,
    SECTION_PASSWORD_INDEX5,
    SECTION_PASSWORD_INDEX6,  
};


uint8_t wirte_state = 1;
uint8_t IsReadDataFlag = 0;// 1 读卡成功，0 读卡失败
uint8_t read_status = 0; // 读IC卡的内存一次 ，返回为 0 读取成功， 返回为 1 读取失败。

uint8_t count = sizeof(Material_Data) / 16;
//{0x4D, 0x49, 0x4E, 0x47, 0x44, 0x41};//mingda

uint8_t plaintext[16];
uint8_t key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
uint8_t section_key[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};//mingda

uint8_t Config_ModuleAddr[6] 	   = {0x06, 0x01, 0x48, 0x01, 0xb1, 0x03};
uint8_t Config_ICType[6] 	 	   = {0x06, 0x01, 0x42, 0x41, 0xfb, 0x03};
uint8_t Config_AutoReadID[6] 	   = {0x06, 0x01, 0x45, 0x00, 0xbd, 0x03};
uint8_t Config_FilterCopyCard[6]   = {0x06, 0x01, 0x46, 0x01, 0xbf, 0x03};
uint8_t Config_ChannelNum[6] 	   = {0x06, 0x01, 0x50, 0x06, 0xae, 0x03};
uint8_t Config_SectionPassward[14] = {0x0E, 0x01, 0x47, 0x01, 0x01, 0x60, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0Xd7, 0x03};
//{0x0E, 0x01, 0x47, 0x01, 0x01, 0x60, 0x4d, 0x49, 0x4e, 0x47, 0x44, 0x41, 0XDf, 0x03};
uint8_t Command_WriteData[31] = {0x1f,0x01,0x4d,0X01,0X01,0X01,0X60, 0XFF,0xff,0xff,0xff,0xff,0xff,
								0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00, 0x00, 0x00,0x00, 0x00,0x00, 0x00,0x00, 0x00, 0x00, 0x03};
uint8_t Command_ReadData[15]  = {0x0f, 0x01,0x4c,0X01,0X01,0X01,0x60,0xff,0xff,0xff,0xff,0xff,0xff,0xdc,0x03};

uint8_t	config_respond[6];
uint8_t WirteData_Respond[8];
uint8_t ReadData_HaveICRespond[29];
uint8_t ReadData_NOICRespond[7];

/**
 * 获取校验码的值
 */
uint8_t Get_Bcc_Value(uint8_t* arr, uint8_t size) {
    uint8_t result = arr[0];

    for (int i = 1; i < size; i++) {
        result ^= arr[i];
    }

    // 异或取反
    result = ~result;
    return result;
}

/**
 * @brief 向写数据的命令中插入数据
 */
void Insert_Data(uint8_t* USART_SendArray, uint8_t* Data)
{
	uint8_t bcc = 0;
	memcpy(USART_SendArray + 13,Data,16);
	bcc = Get_Bcc_Value(USART_SendArray,29);
	USART_SendArray[29] = bcc;
}

/**
 * @brief 插入扇区密钥到指定命令写入数据中
 * @param command_write_data 指向命令写入数据缓冲区的指针，该缓冲区应有足够的空间容纳插入的分区密钥
 * @param section_key 指向分区密钥的指针，该密钥将被插入到命令写入数据中
 * @return null
 */
void insert_section_key(uint8_t* command_write_data, uint8_t* section_key)
{
    memcpy(command_write_data + 7, section_key, 6);
}

void USART_SendArray(uint8_t* array,uint8_t size)
{
	HAL_UART_Transmit(&huart1, array, size, 500);
}

void USART_ReceiveArray(uint8_t* array, uint8_t size)
{
	memset(array,0,size);
	uint8_t count = 0;
    while(count < size)
    {
        HAL_UART_Receive(&huart1 , array + count, 1, 500);
        count++;
    }
}

void USART_ReceiveData(void)
{

    uint8_t size = 0;
    HAL_UART_Receive(&huart1,&size, 1, 100);
	if (size == 0x07)
    {
        ReadData_NOICRespond[0] = size;
        USART_ReceiveArray(ReadData_NOICRespond + 1, 6);
    }
    else
    {
        ReadData_HaveICRespond[0] = size;
        USART_ReceiveArray(ReadData_HaveICRespond + 1, 28);
        IsReadDataFlag = 1;
    }
}

uint8_t RFID_Rc523_Config(void)
{
    uint8_t Config_State = 0;

    USART_SendArray(Config_ModuleAddr,6);
    OLED_ShowHexArray(Config_ModuleAddr, 6, 1);
    USART_ReceiveArray(config_respond,6);
    OLED_ShowHexArray(config_respond, 6, 2);
    Config_State |= config_respond[3];

    USART_SendArray(Config_ICType,6);
    OLED_ShowHexArray(Config_ICType, 6, 3);
    USART_ReceiveArray(config_respond,6);
    OLED_ShowHexArray(config_respond, 6, 4);
    Config_State |= config_respond[3];
    HAL_Delay(3000);

    USART_SendArray(Config_AutoReadID,6);
    OLED_ShowHexArray(Config_AutoReadID, 6, 1);
    USART_ReceiveArray(config_respond,6);
    OLED_ShowHexArray(config_respond, 6, 2);
    Config_State |= config_respond[3];


    USART_SendArray(Config_FilterCopyCard,6);
    OLED_ShowHexArray(Config_FilterCopyCard, 6, 3);
    USART_ReceiveArray(config_respond,6);
    OLED_ShowHexArray(config_respond, 6, 4);
    Config_State |= config_respond[3];
    HAL_Delay(3000);

    USART_SendArray(Config_ChannelNum,6);
    OLED_ShowHexArray(Config_ChannelNum, 6, 1);
    USART_ReceiveArray(config_respond,6);
    Config_State |= config_respond[3];
    OLED_ShowHexArray(config_respond, 6, 2);

    USART_SendArray(Config_SectionPassward,14);
    OLED_ShowHexArray(Config_SectionPassward, 6, 3);
    USART_ReceiveArray(config_respond,6);
    OLED_ShowHexArray(config_respond, 6, 4);
    Config_State |= config_respond[3];
    HAL_Delay(3000);
    OLED_Clear();
    return Config_State;
}

uint8_t RFID_Rc523_Wirte_Data(uint8_t* Data)
{
	insert_section_key(Command_WriteData, section_key);
	Insert_Data(Command_WriteData, Data);
	USART_SendArray(Command_WriteData,31);
	USART_ReceiveArray(WirteData_Respond,8);
	// 校验值
	if(0 == WirteData_Respond[3] && 0 == WirteData_Respond[3])
	{
		wirte_state = 0;
	}
	OLED_ShowHexArray(Data, 8, 1);
	OLED_ShowHexArray(Data + 8, 8, 2);
	OLED_ShowHexArray(WirteData_Respond, 8, 3);
	return wirte_state;
}

uint8_t RFID_Rc523_Read_Data(uint8_t* Data)
{
	uint8_t bcc = 0;
	insert_section_key(Command_ReadData, section_key);
	bcc = Get_Bcc_Value(Command_ReadData, 13);
	Command_ReadData[13] = bcc;
	USART_SendArray(Command_ReadData,15);
    USART_ReceiveData();
    OLED_ShowHexArray(Command_ReadData, 8, 1);
    OLED_ShowHexArray(Command_ReadData + 8, 8, 2);
    HAL_Delay(3000);
    OLED_Clear();
    OLED_ShowHexArray(Data, 8, 1);
	OLED_ShowHexArray(Data + 8, 8, 2);

    if (IsReadDataFlag)
    {
		memcpy(Data,ReadData_HaveICRespond + 11, 16);
		IsReadDataFlag = 0;
        OLED_ShowHexArray(Data, 8, 3);
	    OLED_ShowHexArray(Data + 8, 8, 4);
    }
    else
    {
        OLED_ShowHexArray(ReadData_NOICRespond, 8, 4);   
    }
    return IsReadDataFlag;
}



/**
 *  @brief  指定信道指定、块地址，并写入数据块写入数据
 *  @param  Channel:信道号
 *  @param  BlockAddress:块地址
 *  @return null
 */
void RFID_Rc523_Write_Block(uint8_t Channel, uint8_t  blockaddr, uint8_t* Data)
{
    Command_WriteData[CHANNEL] = Channel;
    Command_WriteData[BLOCK] = blockaddr;
    insert_section_key(Command_WriteData, section_key);
    Insert_Data(Command_WriteData, Data);
    memset(WirteData_Respond, 0, sizeof(WirteData_Respond));
    USART_SendArray(Command_WriteData,31);
    USART_ReceiveArray(WirteData_Respond,8);

//  OLED_ShowHexArray(Command_WriteData, 8, 1);
//	OLED_ShowHexArray(Command_WriteData + 8, 8, 2);
//	OLED_ShowHexArray(Command_WriteData + 16, 8, 3);
//  OLED_ShowHexArray(Command_WriteData + 24, 8, 4);
//  HAL_Delay(3000);
//  OLED_Clear();

//  OLED_ShowHexArray(Data, 8, 1);
//	OLED_ShowHexArray(Data + 8, 8, 2);
//	HAL_Delay(30000);

	OLED_ShowHexArray(WirteData_Respond, 8, 4);
	HAL_Delay(3000);
	OLED_Clear();
}

/**
 * @brief 读取RFID ic贴纸芯片上的指定数据块 
 * @param Channel 通信通道号，用于指定通信信道
 * @param blockaddr 数据块地址，用于指定要读取的数据块
 * @param Data 数据缓冲区指针，用于存储读取到的数据块内容
 */
void RFID_Rc523_Read_Block(uint8_t Channel, uint8_t blockaddr, uint8_t* Data)
{
    uint8_t bcc = 0;
    Command_ReadData[CHANNEL] = Channel;
    Command_ReadData[BLOCK] = blockaddr;
    insert_section_key(Command_ReadData, section_key);
    bcc = Get_Bcc_Value(Command_ReadData,13);
    Command_ReadData[13] = bcc;

    USART_SendArray(Command_ReadData,15);
    USART_ReceiveData();

    if (IsReadDataFlag)
    {
		memcpy(Data,ReadData_HaveICRespond + 11, 16);
//	    OLED_ShowHexArray(Data, 8, 1);
//	    OLED_ShowHexArray(Data + 8, 8, 2);
//	    HAL_Delay(5000);
//	    OLED_Clear();
//		IsReadDataFlag = 0;
    }
    else
    {
    	read_status = 1;
	    OLED_ShowHexArray(ReadData_NOICRespond, 8, 1);
	    HAL_Delay(5000);
	    OLED_Clear();

    }

}

void rfid_write_channel_data(uint8_t channel, Material_Data* data)
{

	for(uint8_t i = 1; i <= count; i++)
	{
	  memcpy(plaintext, (((uint8_t*)data) + ((i-1) * 16)), 16);
	OLED_ShowHexArray(plaintext, 8, 1);
	OLED_ShowHexArray(plaintext + 8, 8, 2);
	HAL_Delay(30000);
	OLED_Clear();
	  cipher(plaintext, key);
	  if(i % 2 == 1)
	  {
		  RFID_Rc523_Write_Block(channel, 1 + 4 * ((i-1)/2), plaintext);// 1 5 9
	  }
	  else
	  {
		  RFID_Rc523_Write_Block(channel, 2 + 4 * ((i-1)/2), plaintext);// 2 6 10
	  }

	}
//    for (uint8_t i = 1; i <= 3 ; i++)
//    {
//        RFID_Rc523_Write_Block(channel, 1 + 4 * (i-1), (uint8_t*)data + ((i - 1) * 16));
//    }
}

void rfid_read_channel_data(uint8_t channel, Material_Data* data)
{
//    for (uint8_t i = 1; i <= 3; i++)
//    {
//    	OLED_ShowNum(4, 1, i, 1);
//        RFID_Rc523_Read_Block(channel, 1 + 4 * (i - 1), (uint8_t*)data + ((i - 1) * 16));
//	    OLED_ShowHexArray((uint8_t*)data, 8, 1);
//	    OLED_ShowHexArray((uint8_t*)data + 8, 8, 2);
//	    OLED_ShowHexArray((uint8_t*)data + 16, 8, 3);
//	    OLED_ShowHexArray((uint8_t*)data + 24, 8, 4);
//	    HAL_Delay(5000);
//	    OLED_Clear();
//	    OLED_ShowHexArray((uint8_t*)data + 32, 8, 1);
//	    OLED_ShowHexArray((uint8_t*)data + 40, 8, 2);
//	    HAL_Delay(5000);
//	    OLED_Clear();
//    }

	for(uint8_t i = 1; i <= count; i++)
	{
		memset(plaintext,0, sizeof(plaintext));
		if(i % 2 == 1)
		{
			RFID_Rc523_Read_Block(channel, 1 + 4 * ((i - 1) / 2),plaintext); // 1 1 3 5 5 9
		}
		else
		{
			RFID_Rc523_Read_Block(channel, 2 + 4 * ((i - 1) / 2),plaintext); // 2 2 4 6 6 10
		}
	    OLED_ShowHexArray(plaintext, 8, 1);
	    OLED_ShowHexArray(plaintext + 8, 8, 2);

	    HAL_Delay(5000);
	    OLED_Clear();

		if(IsReadDataFlag)
		{
			invcipher(plaintext, key);
			memcpy((uint8_t*)data + ((i - 1) * 16), plaintext, 16);
			IsReadDataFlag = 0;
		}

	}

	if(read_status)
	{
		memset((uint8_t*)data, 0, sizeof(Material_Data));
		read_status = 0;
	}


}

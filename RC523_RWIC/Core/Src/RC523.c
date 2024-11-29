#include "RC523.h"
#include "spi.h"
#include "string.h"

/**
 * @brief 初始化RC523模块
 */
void RC523_Init(void) //
{
    RC523_Reset_Disable();           // 禁用RC523模块的复位功能
    RC523_NSS_Disable();             // 禁用NSS引脚，控制模块的通信状态
    PcdReset();                      // 对模块进行复位操作
    PcdConfigISOType(RC_ISO14443_A); // 设置工作方式
}

/**
 * @brief 通过SPI发送一个字节到RC523芯片
 * @param byte 要发送的字节数据
 */
static void SPI_RC523_SendByte(uint8_t byte) //
{
    // 使用SPI1发送一个字节的数据，等待时间最长为100毫秒
    HAL_SPI_Transmit(&hspi1, &byte, 1, 100);
}

/**
 * @brief 从SPI_RC523中读取一个字节的数据。
 * @return 从SPI_RC523设备接收到的一个字节数据。
 */
static uint8_t SPI_RC523_ReadByte(void) //
{
    uint8_t SPI_Data;
    HAL_SPI_Receive(&hspi1, &SPI_Data, 1, 100);
    return SPI_Data;
}

/**
 * @brief 通过SPI接口与RC523芯片通信，以读取指定地址的寄存器值
 * @param Address 寄存器地址，用于指定需要读取的寄存器位置
 * @return 返回读取到的寄存器值，作为函数的返回值
 */
static uint8_t ReadReg(uint8_t Address) //
{
    // 准备寄存器地址，通过左移操作和按位或操作，以符合SPI通信协议的要求
    uint8_t Addr, Ret;
    Addr = ((Address << 1) & 0x7E) | 0x80;

    // 启动SPI通信，通过NSS引脚使能RC523芯片
    RC523_NSS_Enable();
    // 发送准备好的寄存器地址，以启动读取操作
    SPI_RC523_SendByte(Addr);
    // 接收并存储从RC523芯片读取到的数据
    Ret = SPI_RC523_ReadByte();
    // 结束SPI通信，通过NSS引脚禁能RC523芯片
    RC523_NSS_Disable();

    // 返回读取到的寄存器值
    return Ret;
}

/**
 * @brief 向指定寄存器写入一个值。
 * @param Address 要写入的寄存器地址，内部会处理以匹配SPI通信格式。
 * @param Value 要写入寄存器的值。
 */
static void WriteReg(uint8_t Address, uint8_t Value) //
{
    uint8_t Addr;
    Addr = (Address << 1) & 0x7E;

    RC523_NSS_Enable();
    SPI_RC523_SendByte(Addr);
    SPI_RC523_SendByte(Value);
    RC523_NSS_Disable();
}

/**
 * @brief 对指定寄存器相应位置1
 * @param Reg: 寄存器地址
 * @param Mask: 要设置的位掩码，指明哪些位需要被设置为1
 */
static void SetBitMask(uint8_t Reg, uint8_t Mask)
{
    uint8_t Temp;
    // 读取当前寄存器的值
    Temp = ReadReg(Reg);
    // 将寄存器的值与掩码进行按位或操作，以设置相应的位为1
    WriteReg(Reg, Temp | Mask); // set bit mask
}

/**
 * @brief 对指定寄存器相应位置0
 * @param Reg: 寄存器地址
 * @param Mask: 需要清除的位掩码，与运算后将清除这些位
 */
static void ClearBitMask(uint8_t Reg, uint8_t Mask)
{
    uint8_t Temp;
    // 读取寄存器的当前值
    Temp = ReadReg(Reg);
    // 清除位掩码并写回寄存器
    WriteReg(Reg, Temp & (~Mask)); // clear bit mask
}

/**
 * @brief 打开天线
 */
static void PcdAntennaOn(void) //
{
    uint8_t i;
    i = ReadReg(TxControlReg);

    if (!(i & 0x03))
        SetBitMask(TxControlReg, 0x03);
}

/**
 * @brief 打开天线
 */
static void PcdAntennaOff(void) //
{
    ClearBitMask(TxControlReg, 0x03);
}

/**
 * @brief 配置PCD（读卡器）的ISO类型 A OR B。
 * @param iType ISO类型代码。RC_ISO14443_A---'A'
 */
void PcdConfigISOType(uint8_t iType) //
{
    if (iType == 'A') // ISO14443_A
    {
        ClearBitMask(Status2Reg, 0x08);

        WriteReg(ModeReg, 0x3D);
        WriteReg(RxSelReg, 0x86);
        WriteReg(RFCfgReg, 0x7F);
        WriteReg(TReloadRegL, 30);
        WriteReg(TReloadRegH, 0);
        WriteReg(TModeReg, 0x8D);
        WriteReg(TPrescalerReg, 0x3E);
        HAL_Delay(1);

        PcdAntennaOn(); // 开天线
    }
}

/**
 * @brief 重置PCD（读卡器）
 */
void PcdReset(void) //
{
    RC523_Reset_Disable();
    HAL_Delay(1);
    RC523_Reset_Enable();
    HAL_Delay(1);
    RC523_Reset_Disable();
    HAL_Delay(1);

    WriteReg(CommandReg, PCD_RESETPHASE);
    while (ReadReg(CommandReg) & 0x10)
        ;
    HAL_Delay(1);

    WriteReg(ModeReg, 0x3D);       // 定义发送和接收常用模式 和Mifare卡通讯，CRC初始值0x6363
    WriteReg(TReloadRegL, 30);     // 16位定时器低位
    WriteReg(TReloadRegH, 0);      // 16位定时器高位
    WriteReg(TModeReg, 0x8D);      // 定义内部定时器的设置
    WriteReg(TPrescalerReg, 0x3E); // 设置定时器分频系数
    WriteReg(TxAutoReg, 0x40);     // 调制发送信号为100%ASK
}

/**
 * @brief 通过RC523芯片与ISO14443卡通讯
 * @param Command 要发送的RC523命令
 * @param InData 通过RC523发送到卡片的数据
 * @param iInLenByte 发送数据的字节长度
 * @param OutData 接收到的卡片返回数据
 * @param pOutLenBit 返回数据的位长度
 * @return uint8_t  状态值 = MI_OK，成功
 */
static uint8_t PcdCmdRC523(uint8_t Command, uint8_t *InData, uint8_t iInLenByte, uint8_t *OutData, uint32_t *pOutLenBit) //
{
    uint8_t Status = MI_ERR;
    uint8_t IrqEn = 0x00;
    uint8_t WaitFor = 0x00;
    uint8_t LastBits;
    uint8_t N;
    uint32_t i;

    switch (Command)
    {
    case PCD_AUTHENT:   // Mifare认证
        IrqEn = 0x12;   // 允许错误中断请求ErrIEn,允许空闲中断IdleIEn
        WaitFor = 0x10; // 认证寻卡等待时候,查询空闲中断标志位
        break;

    case PCD_TRANSCEIVE: // 接收发送,发送接收
        IrqEn = 0x77;    // 允许TxIEn RxIEn IdleIEn LoAlertIEn ErrIEn TimerIEn
        WaitFor = 0x30;  // 寻卡等待时候 查询接收中断标志位与 空闲中断标志位
        break;

    default:
        break;
    }

    WriteReg(ComIEnReg, IrqEn | 0x80); // IRqInv置位管脚IRQ与Status1Reg的IRq位的值相反
    ClearBitMask(ComIrqReg, 0x80);     // Set1该位清零时，CommIRqReg的屏蔽位清零
    WriteReg(CommandReg, PCD_IDLE);    // 写空闲命令
    SetBitMask(FIFOLevelReg, 0x80);    // 置位FlushBuffer清除内部FIFO的读和写指针以及ErrReg的BufferOvfl标志位被清除

    for (i = 0; i < iInLenByte; i++)
    {
        WriteReg(FIFODataReg, InData[i]); // 写数据进FIFOdata
    }

    WriteReg(CommandReg, Command); // 写命令

    if (Command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80); // StartSend置位启动数据发送 该位与收发命令使用时才有效
    }

    i = 1000; // 根据时钟频率调整，操作M1卡最大等待时间25ms

    do // 认证 与寻卡等待时间
    {
        N = ReadReg(ComIrqReg); // 查询事件中断
        i--;
    } while ((i != 0) && (!(N & 0x01)) && (!(N & WaitFor))); // 退出条件i=0,定时器中断，与写空闲命令

    ClearBitMask(BitFramingReg, 0x80); // 清理允许StartSend位

    if (i != 0)
    {
        if (!((ReadReg(ErrorReg) & 0x1B))) // 读错误标志寄存器BufferOfI CollErr ParityErr ProtocolErr
        {
            Status = MI_OK;

            if (N & IrqEn & 0x01) // 是否发生定时器中断
            {
                Status = MI_NOTAGERR;
            }

            if (Command == PCD_TRANSCEIVE)
            {
                N = ReadReg(FIFOLevelReg);             // 读FIFO中保存的字节数
                LastBits = ReadReg(ControlReg) & 0x07; // 最后接收到得字节的有效位数

                if (LastBits)
                {
                    *pOutLenBit = (N - 1) * 8 + LastBits; // N个字节数减去1（最后一个字节）+最后一位的位数 读取到的数据总位数
                }
                else
                {
                    *pOutLenBit = N * 8; // 最后接收到的字节整个字节有效
                }

                if (N == 0)
                {
                    N = 1;
                }

                if (N > MAXRLEN)
                {
                    N = MAXRLEN;
                }

                for (i = 0; i < N; i++)
                {
                    OutData[i] = ReadReg(FIFODataReg);
                }
            }
        }
        else
        {
            Status = MI_ERR;
        }
    }

    SetBitMask(ControlReg, 0x80); // stop timer now
    WriteReg(CommandReg, PCD_IDLE);

    return Status;
}

/**
 * @brief 寻卡操作，若寻卡成功返回卡的类型
 * @param iReq_code 寻卡方式
 *                     = 0x52，寻感应区内所有符合14443A标准的卡
 *                     = 0x26，寻未进入休眠状态的卡
 * @param pTagType  卡片类型代码
 *                   = 0x4400，Mifare_UltraLight
 *                   = 0x0400，Mifare_One(S50)
 *                   = 0x0200，Mifare_One(S70)
 *                   = 0x0800，Mifare_Pro(X))
 *                   = 0x4403，Mifare_DESFire
 * @return uint8_t  函数执行状态，MI_OK表示成功，其他值表示错误
 */
uint8_t PcdRequest(uint8_t iReq_code, uint8_t *pTagType)
{
    uint8_t cStatus;
    uint8_t iComMF523Buf[MAXRLEN];
    uint32_t ulLen;

    ClearBitMask(Status2Reg, 0x08); // 清理指示MIFARECyptol单元接通以及所有卡的数据通信被加密的情况
    WriteReg(BitFramingReg, 0x07);  //	发送的最后一个字节的 七位
    SetBitMask(TxControlReg, 0x03); // TX1,TX2管脚的输出信号传递经发送调制的13.56的能量载波信号

    iComMF523Buf[0] = iReq_code; // 存入 卡片命令字

    cStatus = PcdCmdRC523(PCD_TRANSCEIVE, iComMF523Buf, 1, iComMF523Buf, &ulLen); // 寻卡

    if ((cStatus == MI_OK) && (ulLen == 0x10)) // 寻卡成功返回卡类型
    {
        *pTagType = iComMF523Buf[0];
        *(pTagType + 1) = iComMF523Buf[1];
    }
    else
    {
        cStatus = MI_ERR;
    }

    return cStatus;
}

/**
 * @brief 执行卡片防冲突操作，确保只选中一张卡片，并获取其UID。
 *
 * @param pSnr 卡片序列号，4字节
 * @return 返回操作状态，MI_OK表示成功，其他值表示错误
 */
uint8_t PcdAnticoll(uint8_t *pSnr)
{
    uint8_t cStatus;
    uint8_t i, iSnr_check = 0;
    uint8_t iComMF523Buf[MAXRLEN];
    uint32_t ulLen;

    ClearBitMask(Status2Reg, 0x08); // 清MFCryptol On位 只有成功执行MFAuthent命令后，该位才能置位
    WriteReg(BitFramingReg, 0x00);  // 清理寄存器 停止收发
    ClearBitMask(CollReg, 0x80);    // 清ValuesAfterColl所有接收的位在冲突后被清除

    /*
    参考ISO14443协议：https://blog.csdn.net/wowocpp/article/details/79910800
    PCD 发送 SEL = ‘93’，NVB = ‘20’两个字节
    迫使所有的在场的PICC发回完整的UID CLn作为应答。
    */
    iComMF523Buf[0] = 0x93; // 卡片防冲突命令
    iComMF523Buf[1] = 0x20;

    // 发送并接收数据 接收的数据存储于iComMF523Buf
    cStatus = PcdCmdRC523(PCD_TRANSCEIVE, iComMF523Buf, 2, iComMF523Buf, &ulLen); // 与卡片通信

    if (cStatus == MI_OK) // 通信成功
    {
        // 收到的UID 存入pSnr
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = iComMF523Buf[i]; // 读出UID
            iSnr_check ^= iComMF523Buf[i];
        }

        if (iSnr_check != iComMF523Buf[i])
        {
            cStatus = MI_ERR;
        }
    }

    SetBitMask(CollReg, 0x80);

    return cStatus;
}

/**
 * @brief 计算给定数据的CRC校验值。
 * @param InData 计算CRC16的数组。
 * @param Len 输入数据的长度。
 * @param OutData 指向存储计算结果的缓冲区。
 */
static void CalulateCRC(uint8_t *InData, uint8_t Len, uint8_t *OutData)
{
    uint8_t i, iN;

    ClearBitMask(DivIrqReg, 0x04);
    WriteReg(CommandReg, PCD_IDLE);
    SetBitMask(FIFOLevelReg, 0x80);

    for (i = 0; i < Len; i++)
    {
        WriteReg(FIFODataReg, *(InData + i));
    }

    WriteReg(CommandReg, PCD_CALCCRC);
    i = 0xFF;

    do
    {
        iN = ReadReg(DivIrqReg);
        i--;
    } while ((i != 0) && !(iN & 0x04));

    OutData[0] = ReadReg(CRCResultRegL);
    OutData[1] = ReadReg(CRCResultRegM);
}

/**
 * @brief 通过防冲撞机制来选择一个特定的PICC，并验证其UID（唯一标识符）。
 * @param *pSnr 指向存储PICC序列号的指针。
 * @return 返回选择操作的状态，MI_OK表示成功，MI_ERR表示失败。
 */
uint8_t PcdSelect(uint8_t *pSnr)
{
    uint8_t cStatus;
    uint8_t i;
    uint8_t iComMF523Buf[MAXRLEN];
    uint32_t ulLen;

    // 防冲撞 0x93
    iComMF523Buf[0] = PICC_ANTICOLL1;
    // 假设没有冲突，PCD 指定NVB为70，此值表示PCD将发送完整的UID CLn，与40位UID CLn 匹配的PICC，以SAK作为应答
    iComMF523Buf[1] = 0x70;
    iComMF523Buf[6] = 0;

    // 3 4 5 6位存放UID，第7位一直异或。。。
    for (i = 0; i < 4; i++)
    {
        iComMF523Buf[i + 2] = *(pSnr + i);
        iComMF523Buf[6] ^= *(pSnr + i);
    }

    // CRC(循环冗余校验)
    CalulateCRC(iComMF523Buf, 7, &iComMF523Buf[7]);
    ClearBitMask(Status2Reg, 0x08);

    // 发送并接收数据
    cStatus = PcdCmdRC523(PCD_TRANSCEIVE, iComMF523Buf, 9, iComMF523Buf, &ulLen);

    if ((cStatus == MI_OK) && (ulLen == 0x18))
    {
        cStatus = MI_OK;
    }
    else
    {
        cStatus = MI_ERR;
    }

    return cStatus;
}

/**
 * @brief 验证卡片密码，认证通过后才能对卡片进行读写操作
 * @param iAuth_mode A（0x60）或 B（0x61）
 * @param Addr 卡片的块地址
 * @param pKey 认证密钥的指针，密钥长度为6字节
 * @param pSnr 卡片序列号的指针，序列号长度为4字节
 * @return 返回认证状态，MI_OK表示认证成功，其他值表示认证失败
 */
uint8_t PcdAuthState(uint8_t iAuth_mode, uint8_t Addr, uint8_t *pKey, uint8_t *pSnr)
{
    uint8_t cStatus;
    uint8_t i, iComMF523Buf[MAXRLEN];
    uint32_t ulLen;

    iComMF523Buf[0] = iAuth_mode;
    iComMF523Buf[1] = Addr;

    for (i = 0; i < 6; i++)
    {
        iComMF523Buf[i + 2] = *(pKey + i);
    }

    for (i = 0; i < 6; i++)
    {
        iComMF523Buf[i + 8] = *(pSnr + i);
    }

    // 验证密钥命令
    cStatus = PcdCmdRC523(PCD_AUTHENT, iComMF523Buf, 12, iComMF523Buf, &ulLen);

    if ((cStatus != MI_OK) || (!(ReadReg(Status2Reg) & 0x08)))
    {
        cStatus = MI_ERR;
    }

    return cStatus;
}

/**
 * @brief 向MF523卡写入数据。写数据到M1卡一块
 *
 * @param Addr 要写入的块地址。
 * @param pData 指向要写入的数据， 16字节。
 * @return 返回操作状态，MI_OK表示成功，MI_ERR表示失败。
 */
uint8_t PcdWrite(uint8_t Addr, uint8_t *pData)
{
    uint8_t cStatus;
    uint8_t i, iComMF523Buf[MAXRLEN];
    uint32_t ulLen;

    iComMF523Buf[0] = PICC_WRITE;
    iComMF523Buf[1] = Addr;

    CalulateCRC(iComMF523Buf, 2, &iComMF523Buf[2]);

    cStatus = PcdCmdRC523(PCD_TRANSCEIVE, iComMF523Buf, 4, iComMF523Buf, &ulLen);

    if ((cStatus != MI_OK) || (ulLen != 4) || ((iComMF523Buf[0] & 0x0F) != 0x0A))
    {
        cStatus = MI_ERR;
    }

    if (cStatus == MI_OK)
    {
        memcpy(iComMF523Buf, pData, 16);
        for (i = 0; i < 16; i++)
        {
            iComMF523Buf[i] = *(pData + i);
        }

        CalulateCRC(iComMF523Buf, 16, &iComMF523Buf[16]);
        cStatus = PcdCmdRC523(PCD_TRANSCEIVE, iComMF523Buf, 18, iComMF523Buf, &ulLen);

        if ((cStatus != MI_OK) || (ulLen != 4) || ((iComMF523Buf[0] & 0x0F) != 0x0A))
        {
            cStatus = MI_ERR;
        }
    }

    return cStatus;
}

/**
 * @brief 从指定块地址读取数据
 * @param Addr 读取数据的块地址
 * @param pData 读出的数据，16字节
 * @return 返回操作状态，MI_OK表示成功，MI_ERR表示失败
 */
uint8_t PcdRead(uint8_t Addr, uint8_t *pData)
{
    uint8_t cStatus;
    uint8_t i, iComMF523Buf[MAXRLEN];
    uint32_t ulLen;

    iComMF523Buf[0] = PICC_READ;
    iComMF523Buf[1] = Addr;

    CalulateCRC(iComMF523Buf, 2, &iComMF523Buf[2]);
    cStatus = PcdCmdRC523(PCD_TRANSCEIVE, iComMF523Buf, 4, iComMF523Buf, &ulLen);

    if ((cStatus == MI_OK) && (ulLen == 0x90))
    {
        for (i = 0; i < 16; i++)
        {
            *(pData + i) = iComMF523Buf[i];
        }
    }
    else
    {
        cStatus = MI_ERR;
    }

    return cStatus;
}

/**
 * @brief 命令卡片进入休眠状态。
 * @return 返回状态码，表示函数执行是否成功。= MI_OK，成功
 */
uint8_t PcdHalt(void)
{
    uint8_t ucComMF523Buf[MAXRLEN];
    uint32_t ulLen;

    ucComMF523Buf[0] = PICC_HALT;
    ucComMF523Buf[1] = 0;

    CalulateCRC(ucComMF523Buf, 2, &ucComMF523Buf[2]);
    PcdCmdRC523(PCD_TRANSCEIVE, ucComMF523Buf, 4, ucComMF523Buf, &ulLen);

    return MI_OK;
}

/**
 * @brief IC卡读写
 * @param UID IC卡的唯一标识符
 * @param key_type 密钥类型，0表示KEYA，非0表示KEYB
 * @param KEY 密钥数组
 * @param RW 读写选择，1表示读，0表示写
 * @param data_addr 数据地址，指定要读写的数据块地址
 * @param data 数据数组，用于存储读取的数据或准备写入的数据
 */
void IC_RW(uint8_t *UID, uint8_t key_type, uint8_t *KEY, uint8_t RW, uint8_t data_addr, uint8_t *data)
{
    uint8_t status;
    uint8_t i = 0;
    uint8_t ucArray_ID[4] = {0}; // 先后存放IC卡的类型和UID(IC卡序列号)

    status = PcdRequest(0x52, ucArray_ID); // 寻卡
    if (status == MI_OK)
    {
        ShowID(ucArray_ID);
    }
    else
    {
        return;
    }

    status = PcdAnticoll(ucArray_ID); // 防冲撞
    if (status != MI_OK)
        return;

    status = PcdSelect(UID); // 选定卡
    if (status != MI_OK)
    {
        printf("UID don't match\r\n");
        return;
    }

    if (0 == key_type)
    {
        status = PcdAuthState(KEYA, data_addr, KEY, UID); // 校验
    }
    else
    {
        status = PcdAuthState(KEYB, data_addr, KEY, UID); // 校验
    }

    if (status != MI_OK)
    {
        printf("KEY don't match\r\n");
        return;
    }

    if (RW) // 读写选择，1是读，0是写
    {
        status = PcdRead(data_addr, data);
        if (status == MI_OK)
        {
            printf("data:");
            for (i = 0; i < 16; i++)
            {
                printf("%02x", data[i]);
            }
            printf("\r\n");
        }
        else
        {
            printf("PcdRead() failed\r\n");
            return;
        }
    }
    else
    {
        status = PcdWrite(data_addr, data);
        if (status == MI_OK)
        {
            printf("PcdWrite() finished\r\n");
        }
        else
        {
            printf("PcdWrite() failed\r\n");
            return;
        }
    }

    status = PcdHalt();
    if (status == MI_OK)
    {
        printf("PcdHalt() finished\r\n");
    }
    else
    {
        printf("PcdHalt() failed\r\n");
        return;
    }
}
/**
 * @brief 显示卡的卡号，以十六进制显示
 * @param p 指向ID信息的指针，ID信息是4字节长。
 */
void ShowID(uint8_t *p)
{
    uint8_t num[9];
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        num[i * 2] = p[i] / 16;
        num[i * 2] > 9 ? (num[i * 2] += '7') : (num[i * 2] += '0');
        num[i * 2 + 1] = p[i] % 16;
        num[i * 2 + 1] > 9 ? (num[i * 2 + 1] += '7') : (num[i * 2 + 1] += '0');
    }
    num[8] = 0;
    printf("ID>>>%s\r\n", num);
}
/**
 * @brief 等待卡离开
 * 这种方法用于确保卡片确实已经不在读卡器上，避免因读卡器灵敏度或环境因素导致的误判
 */

void WaitCardOff(void)
{
    uint8_t status;
    uint8_t TagType[2];

    while (1)
    {
        status = PcdRequest(REQ_ALL, TagType);
        if (status)
        {
            status = PcdRequest(REQ_ALL, TagType);
            if (status)
            {
                status = PcdRequest(REQ_ALL, TagType);
                if (status)
                {
                    return;
                }
            }
        }
        HAL_Delay(1000);
    }
}

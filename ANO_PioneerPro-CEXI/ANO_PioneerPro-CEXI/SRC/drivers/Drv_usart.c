/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：串口驱动
**********************************************************************************/
#include "Drv_usart.h"
#include "Ano_DT.h"
#include "Ano_OF.h"
#include "Drv_OpenMV.h"
#include "Drv_laser.h"
#include "include.h"
#include "string.h"
#include "Drv_UP_Flow.h"

//====uart2
void Usart2_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART2, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART2_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART2_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource5, GPIO_AF_USART2 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource6, GPIO_AF_USART2 );

    //配置PD5作为USART2　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );
    //配置PD6作为USART2　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );

    //配置USART2
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART2时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART2, &USART_InitStructure );
    USART_ClockInit ( USART2, &USART_ClockInitStruct );

    //使能USART2接收中断
    USART_ITConfig ( USART2, USART_IT_RXNE, ENABLE );
    //使能USART2
    USART_Cmd ( USART2, ENABLE );
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
//	}


}

u8 TxBuffer[256];
u8 TxCounter = 0;
u8 count = 0;
u8 Rx_Buf[256];	//串口接收缓存
unsigned char Re_buf[9],laser_counter=0;
static unsigned char laser_median[9];
u16 distance,intensity;
void Usart2_IRQ(void)
{
    u8 com_data;
    if (USART2->SR & USART_SR_ORE) //ORE中断
    {
        com_data = USART2->DR;
    }

//    //接收中断
//    if (USART_GetITStatus(USART2, USART_IT_RXNE))
//    {
//        USART_ClearITPendingBit(USART2, USART_IT_RXNE); //清除中断标志
//        com_data = USART2->DR;
//				drvU2GetByte(com_data);
//    }
    //发送（进入移位）中断
//    if (USART_GetITStatus(USART2, USART_IT_TXE))
//    {
//        USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
//        if (TxCounter == count)
//        {
//            USART2->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
//        }
//    }
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
			u16 distance_new;
		  USART_ClearITPendingBit(USART2,USART_IT_RXNE);//首先清除中断标志位
			laser_median[laser_counter] = USART_ReceiveData(USART2);   //接收数据
			if(laser_counter == 0 && laser_median[0] != 0x59 ) return;      //第 0 号数据不是帧头，跳过++
			laser_counter++; 
			if(laser_counter==9) //接收到 9个数据 8
			{ 				
				memcpy(Re_buf,laser_median,9);//将laser复制至Re_buf
				laser_counter=0; //重新赋值，准备下一帧数据的接收
//				checksum=BYTE0(Re_buf[0]+Re_buf[1]+Re_buf[2]+Re_buf[3]+Re_buf[4]+Re_buf[5]+Re_buf[6]+Re_buf[7]);//校验
				if(Re_buf[0]==0x59&&Re_buf[1]==0x59)       //检查帧头1 帧头2
				{ 
					distance = ((short)(Re_buf[3]<<8| Re_buf[2]));//距离 Re_buf[2]低八位 Re_buf[3]高八位左移8位等价于*256 
//					intensity = ((short)(Re_buf[5]<<8| Re_buf[4]));   //信号强度 再进行位的或运算即两数相加 最后转换为short类型两字节
//					if(distance_new<=1250)//去除非正常数据
//					{
//						distance=distance_new;//正常赋值
//						err_flag=0;
//					}
//					else
//					{
//						err_flag++;
//					}
				}
			} 			
		}
}

void Usart2_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        TxBuffer[count++] = * ( DataToSend + i );
    }

    if ( ! ( USART2->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( USART2, USART_IT_TXE, ENABLE ); //打开发送中断
    }
}
//====uart3
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "Ano_UWB.h"
void Usart3_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOB, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource10, GPIO_AF_USART3 );
    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource11, GPIO_AF_USART3 );

    //配置PD5作为USART2　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );
    //配置PD6作为USART2　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );

    //配置USART2
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART2时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART3, &USART_InitStructure );
    USART_ClockInit ( USART3, &USART_ClockInitStruct );

    //使能USART2接收中断
    USART_ITConfig ( USART3, USART_IT_RXNE, ENABLE );
    //使能USART2
    USART_Cmd ( USART3, ENABLE );
}

u8 Tx3Buffer[256];
u8 Tx3Counter = 0;
u8 count3 = 0;
unsigned char Re_buf2[8],Uwb_counter=0;
static unsigned char Uwb_median[7];
u16 Uwb_x,Uwb_y,Uwb_z;
void Usart3_IRQ ( void )
{
    u8 com_data;

    if (USART3->SR & USART_SR_ORE) //ORE中断
        com_data = USART3->DR;

    //接收中断
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
		{
//			u16 distance_new;
		  USART_ClearITPendingBit(USART3,USART_IT_RXNE);//首先清除中断标志位
			Uwb_median[Uwb_counter] = USART_ReceiveData(USART3);   //接收数据
			if(Uwb_counter == 0 && Uwb_median[0] != 0xFF ) return;      //第 0 号数据不是帧头，跳过++
			Uwb_counter++; 
			if(Uwb_counter==8) //接收到 8个数据 
			{ 				
				memcpy(Re_buf2,Uwb_median,8);//将laser复制至Re_buf
				Uwb_counter=0; //重新赋值，准备下一帧数据的接收
//				checksum=BYTE0(Re_buf[0]+Re_buf[1]+Re_buf[2]+Re_buf[3]+Re_buf[4]+Re_buf[5]+Re_buf[6]+Re_buf[7]);//校验
				if(Re_buf2[0]==0xFF&&Re_buf2[7]==0xFA)       //检查帧头1 帧头2
				{ 
					Uwb_x = ((short)(Re_buf2[2]<<8| Re_buf2[1]));//距离 Re_buf[2]低八位 Re_buf[3]高八位左移8位等价于*256 
					Uwb_y = ((short)(Re_buf2[4]<<8| Re_buf2[3]));//距离 Re_buf[4]低八位 Re_buf[3]高八位左移8位等价于*256
					Uwb_z = ((short)(Re_buf2[6]<<8| Re_buf2[5]));//距离 Re_buf[6]低八位 Re_buf[3]高八位左移8位等价于*256 					
				}
			} 			
		}
}

static void Usart3_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        Tx3Buffer[count3++] = * ( DataToSend + i );
    }
    if ( ! ( USART3->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( USART3, USART_IT_TXE, ENABLE ); //打开发送中断
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//====uart4

void Uart4_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
    //USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART4, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART4_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART4_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource0, GPIO_AF_UART4 );
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource1, GPIO_AF_UART4 );

    //配置PC12作为UART5　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    //配置PD2作为UART5　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );

    //配置UART5
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    USART_Init ( UART4, &USART_InitStructure );

    //使能UART5接收中断
    USART_ITConfig ( UART4, USART_IT_RXNE, ENABLE );
    //使能USART5
    USART_Cmd ( UART4, ENABLE );
}
u8 Tx4Buffer[256];
u8 Tx4Counter = 0;
u8 count4 = 0;

void Uart4_IRQ ( void )
{
    u8 com_data;

	if ( UART4->SR & USART_SR_ORE ) //ORE中断
        com_data = UART4->DR;
    //接收中断
    if ( USART_GetITStatus ( UART4, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART4, USART_IT_RXNE ); //清除中断标志

        com_data = UART4->DR;
				//====
				//匿名光流解析
				if(of_init_type!=2)
				{
					AnoOF_GetOneByte(com_data);
				}
				//优像光流解析
				if(of_init_type!=1)
				{
					OFGetByte(com_data);
				}
//		if(LASER_LINKOK)
//			Drv_Laser_GetOneByte( com_data);
//		else
//			AnoOF_GetOneByte ( com_data );
    }

    //发送（进入移位）中断
    if ( USART_GetITStatus ( UART4, USART_IT_TXE ) )
    {

        UART4->DR = Tx4Buffer[Tx4Counter++]; //写DR清除中断标志

        if ( Tx4Counter == count4 )
        {
            UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }

}

void Uart4_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        Tx4Buffer[count4++] = * ( DataToSend + i );
    }

    if ( ! ( UART4->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( UART4, USART_IT_TXE, ENABLE ); //打开发送中断
    }

}

//====uart5

void Uart5_Init ( u32 br_num )
{
    USART_InitTypeDef USART_InitStructure;
    //USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART5, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOC, ENABLE );
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART5_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART5_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );

    //配置PC12作为UART5　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );
    //配置PD2作为UART5　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );

    //配置UART5
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    USART_Init ( UART5, &USART_InitStructure );



    //使能UART5接收中断
    USART_ITConfig ( UART5, USART_IT_RXNE, ENABLE );
    //使能USART5
    USART_Cmd ( UART5, ENABLE );
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
//	}

}
u8 Tx5Buffer[256];
u8 Tx5Counter = 0;
u8 count5 = 0;

void Uart5_IRQ ( void )
{
	u8 com_data;
	if ( UART5->SR & USART_SR_ORE ) //ORE中断
        com_data = UART5->DR;
    //接收中断
    if ( USART_GetITStatus ( UART5, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART5, USART_IT_RXNE ); //清除中断标志

        com_data = UART5->DR;
		//====
		//Drv_Laser_GetOneByte(com_data);
		AnoDTRxOneByteUart ( com_data );
    }

    //发送（进入移位）中断
    if ( USART_GetITStatus ( UART5, USART_IT_TXE ) )
    {

        UART5->DR = Tx5Buffer[Tx5Counter++]; //写DR清除中断标志

        if ( Tx5Counter == count5 )
        {
            UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }

}

void Uart5_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        Tx5Buffer[count5++] = * ( DataToSend + i );
    }

    if ( ! ( UART5->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( UART5, USART_IT_TXE, ENABLE ); //打开发送中断
    }

}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/


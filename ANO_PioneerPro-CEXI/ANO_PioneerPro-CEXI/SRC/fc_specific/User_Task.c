#include "User_Task.h"
#include "Drv_RcIn.h"
#include "Ano_FlightCtrl.h"
#include "Drv_led.h"
#include "Ano_FlyCtrl.h"
u8 one_key_moshi_f=1,xifu_step=0,tuoluo_step=0,cexi_step=0;
u16 youmen=1500,ditance_f; 
u8 lock_cmd=1,unlock_cmd=1;
extern u16 distance;

void UserTask_OneKeyCmd(void)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0,one_key_xifu_f=1,one_key_tuoluo_f=1,xifu_flag=0,tuoluo_flag=0,err_flag;
    static u8 mission_step;
		static u16 time_dly_cnt_ms_xifu =0,time_dly_cnt_ms_tuoluo =0;
    //判断有遥控信号才执行
    if (rc_in.fail_safe == 0)
    {
        //判断第6通道拨杆位置 1300<CH_6<1700(上波1500)
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            //还没有执行
            if (one_key_takeoff_f == 0)
            {
                //标记已经执行
                one_key_takeoff_f = 1;
                //执行一键起飞
                one_key_take_off(); //参数单位：厘米； 0：默认上位机设置的高度。
								AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"ONE KEY TAKEOFF START......");//\n为回车	开启恒温功能发送信息
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_takeoff_f = 0;
        }
        //
        //判断第6通道拨杆位置 800<CH_6<1200(下拨1069)
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //还没有执行
            if (one_key_land_f == 0)
            {
                //标记已经执行
                one_key_land_f =1;
                //执行一键降落
                one_key_land();
							AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"ONE KEY LAND START......");//\n为回车	开启恒温功能发送信息
            }
        }
        else
        {
            //复位标记，以便再次执行
            one_key_land_f = 0;
        }
        //判断第7通道拨杆位置 800<CH_6<1200 脱落(下拨1067)
			if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 800 && rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 1200)
			{
				//还没有执行
				if (one_key_tuoluo_f == 0)
				{		
					if (tuoluo_step == 0&&unlock_cmd == 0)//锁定状态cc
					{
						flag.unlock_cmd = 1;//0时进行解锁
						unlock_cmd = 1; 
						tuoluo_step=1;
						AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"ONE KEY TUOLUO START......");//\n为回车	开启恒温功能发送信息
					}
					else if(tuoluo_step ==1)
					{
						youmen=1800;//1时加速
					}
					else if(distance <=10)//延时160ms -60
					{	
						tuoluo_step = 2 ;//step为2还在吸附中为达到阈值进行脱落
						youmen =1250;
					}
					else if(distance >10&&distance<=30) 
					{
						tuoluo_step = 3;//脱落离开顶板切换为3
						youmen =1400;
					}						
					else if(distance >30&&distance<=50) 
					{
						youmen =1500;
						tuoluo_step = 3 ;
						tuoluo_flag++;
						if(tuoluo_flag>=25)//定位在30~50cm之间保持500ms
						{
							one_key_tuoluo_f=1;
							tuoluo_flag=0;
							tuoluo_step = 0;
							err_flag=0;
						}
					}
					else if(distance>50&&distance<=150)
					{
						tuoluo_step = 3 ;
						youmen =1600;
						tuoluo_flag=0;
					}
					else if(tuoluo_step==3)
					{
						tuoluo_step = 3 ;
						err_flag++;
						youmen=1500;
						if(err_flag>=25)//没在指定范围时错误+1 500ms后切换回遥控控制
						{
							one_key_tuoluo_f=1;
							tuoluo_step = 0;
							tuoluo_flag=0;
							err_flag=0;
						}
					}
				}
			}
			else
			{
				//复位标记，以便再次执行
				one_key_tuoluo_f = 0;
				time_dly_cnt_ms_tuoluo = 0;
				tuoluo_step = 0;
				unlock_cmd=0;
			}
        //判断第7通道拨杆位置 1800<CH_6<2000 吸附(上拨1932)
			if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1800 && rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 2000)
			{			
				//还没有执行
				if (one_key_xifu_f == 0)
				{
					if(distance >= 30 && distance <=200)
					{								                
						xifu_step=1;
						youmen=1700;
					}
					else if (distance < 30) //
					{ 
						xifu_step=1;
						youmen=1780;
						if(distance<=8)
						{
							xifu_flag++;
						}
						else
						{
							xifu_flag=0;
						}
						if(xifu_flag>=50)//1s
						{
							youmen=1500;
							xifu_step=2;
							err_flag=0;
							one_key_xifu_f =1;//标记已经执行
							xifu_flag=0;
						}		
					}
					else
					{
						xifu_step=1;
						err_flag++;
						youmen=1500;
						if(err_flag>=50)//1s
						{
							one_key_xifu_f=1;
							xifu_step = 0;
							xifu_flag=0;
							err_flag=0;
						}		
					}
				}
			}
			else
			{
				//复位标记，以便再次执行
				one_key_xifu_f = 0;
				xifu_step =0;
				time_dly_cnt_ms_xifu =0;
				lock_cmd=0; 
			}					
    }
    ////////////////////////////////////////////////////////////////////////
}
int test=0;
int test_time=0;
extern u8 tingzhi;
void UserTask_PaXing(void)
{
	
	if (rc_in.fail_safe == 0)
	{
		if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1700)
		{
			if(test!=2)
				{
					test=2;
					test_time=0;
				IO5_L;IO6_H;
				}
			else if(test==2)
			{
				test_time++;
				if(test_time>=10)
				{
					IO5_H;IO6_H;
				}
			}
		}
		else if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 1700 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1200)
		{
			if(test!=1)
				{
					test=1;
					test_time=0;
				IO5_H;IO6_L;
				}
			else if(test==1)
			{
				test_time++;
				if(test_time>=10)
				{
					IO5_H;IO6_H;
				}
			}
		}
		else
		{
			IO5_H;IO6_H;
			test=0;
			test_time=0;
		}
		
		
		
//		//8通控制前进后退 
//		if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 1200 &&rc_in.rc_ch.st_data.ch_[ch_9_aux5]>1200&&rc_in.rc_ch.st_data.ch_[ch_9_aux5]<1700)//前进
//		{
//			
//			IO5_L;IO6_H;IO7_L;IO8_H;//前进 56右轮 78左轮
//			tingzhi=1;
//		}
//		else if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1700 && rc_in.rc_ch.st_data.ch_[ch_9_aux5] > 1200&&rc_in.rc_ch.st_data.ch_[ch_9_aux5] < 1700 )
//		{
//			IO5_H;IO6_L;IO7_H;IO8_L;//后退
//			tingzhi=0;
//		}
//		else if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1200 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 1700 && rc_in.rc_ch.st_data.ch_[ch_9_aux5] < 1200)
//		{
//			IO5_H;IO6_L;IO7_L;IO8_H;//左转 右轮前 左轮后
//			tingzhi=0;
//		}
//		else if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1200 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 1700 && rc_in.rc_ch.st_data.ch_[ch_9_aux5] > 1700)
//		{
//			IO5_L;IO6_H;IO7_H;IO8_L;//右转 右轮后 左轮前
//			tingzhi=0;
//		}
//		else
//		{
//			IO5_L;IO6_L;IO7_L;IO8_L;//停止
//			tingzhi=0;
//		}
	}
}
void UserTask_CeXi(void)
{
	static u8 one_key_cexi_f = 1,one_key_cexituoluo_f=1,cexi_flag=0,cexituoluo_flag=0,err_flag;
	if (rc_in.fail_safe == 0)
	{
		if (rc_in.rc_ch.st_data.ch_[ch_10_aux6] > 1700 )//侧吸
		{
			if(one_key_cexi_f==0)
			{
				cexi_step=1;
				AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"CEXI START......");//侧吸开始
				one_key_cexi_f=1;
			}
		}
		else
		{
				cexi_step=0;
				one_key_cexi_f=0;
		}
		if (rc_in.rc_ch.st_data.ch_[ch_10_aux6] < 1200 )
		{
			if(one_key_cexituoluo_f==0)
			{
				one_key_cexituoluo_f=1;
				AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"HUIZHENG START......");//回正开始
			}
		}
		else 
		{
				one_key_cexituoluo_f=0;
		}
	}
}


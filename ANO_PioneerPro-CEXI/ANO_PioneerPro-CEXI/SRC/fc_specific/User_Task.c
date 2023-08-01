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
    //һ�����/��������
    //////////////////////////////////////////////////////////////////////
    //�þ�̬������¼һ�����/����ָ���Ѿ�ִ�С�
    static u8 one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0,one_key_xifu_f=1,one_key_tuoluo_f=1,xifu_flag=0,tuoluo_flag=0,err_flag;
    static u8 mission_step;
		static u16 time_dly_cnt_ms_xifu =0,time_dly_cnt_ms_tuoluo =0;
    //�ж���ң���źŲ�ִ��
    if (rc_in.fail_safe == 0)
    {
        //�жϵ�6ͨ������λ�� 1300<CH_6<1700(�ϲ�1500)
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1300 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1700)
        {
            //��û��ִ��
            if (one_key_takeoff_f == 0)
            {
                //����Ѿ�ִ��
                one_key_takeoff_f = 1;
                //ִ��һ�����
                one_key_take_off(); //������λ�����ף� 0��Ĭ����λ�����õĸ߶ȡ�
								AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"ONE KEY TAKEOFF START......");//\nΪ�س�	�������¹��ܷ�����Ϣ
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_takeoff_f = 0;
        }
        //
        //�жϵ�6ͨ������λ�� 800<CH_6<1200(�²�1069)
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 800 && rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200)
        {
            //��û��ִ��
            if (one_key_land_f == 0)
            {
                //����Ѿ�ִ��
                one_key_land_f =1;
                //ִ��һ������
                one_key_land();
							AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"ONE KEY LAND START......");//\nΪ�س�	�������¹��ܷ�����Ϣ
            }
        }
        else
        {
            //��λ��ǣ��Ա��ٴ�ִ��
            one_key_land_f = 0;
        }
        //�жϵ�7ͨ������λ�� 800<CH_6<1200 ����(�²�1067)
			if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 800 && rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 1200)
			{
				//��û��ִ��
				if (one_key_tuoluo_f == 0)
				{		
					if (tuoluo_step == 0&&unlock_cmd == 0)//����״̬cc
					{
						flag.unlock_cmd = 1;//0ʱ���н���
						unlock_cmd = 1; 
						tuoluo_step=1;
						AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"ONE KEY TUOLUO START......");//\nΪ�س�	�������¹��ܷ�����Ϣ
					}
					else if(tuoluo_step ==1)
					{
						youmen=1800;//1ʱ����
					}
					else if(distance <=10)//��ʱ160ms -60
					{	
						tuoluo_step = 2 ;//stepΪ2����������Ϊ�ﵽ��ֵ��������
						youmen =1250;
					}
					else if(distance >10&&distance<=30) 
					{
						tuoluo_step = 3;//�����뿪�����л�Ϊ3
						youmen =1400;
					}						
					else if(distance >30&&distance<=50) 
					{
						youmen =1500;
						tuoluo_step = 3 ;
						tuoluo_flag++;
						if(tuoluo_flag>=25)//��λ��30~50cm֮�䱣��500ms
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
						if(err_flag>=25)//û��ָ����Χʱ����+1 500ms���л���ң�ؿ���
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
				//��λ��ǣ��Ա��ٴ�ִ��
				one_key_tuoluo_f = 0;
				time_dly_cnt_ms_tuoluo = 0;
				tuoluo_step = 0;
				unlock_cmd=0;
			}
        //�жϵ�7ͨ������λ�� 1800<CH_6<2000 ����(�ϲ�1932)
			if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] > 1800 && rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 2000)
			{			
				//��û��ִ��
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
							one_key_xifu_f =1;//����Ѿ�ִ��
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
				//��λ��ǣ��Ա��ٴ�ִ��
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
		
		
		
//		//8ͨ����ǰ������ 
//		if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 1200 &&rc_in.rc_ch.st_data.ch_[ch_9_aux5]>1200&&rc_in.rc_ch.st_data.ch_[ch_9_aux5]<1700)//ǰ��
//		{
//			
//			IO5_L;IO6_H;IO7_L;IO8_H;//ǰ�� 56���� 78����
//			tingzhi=1;
//		}
//		else if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1700 && rc_in.rc_ch.st_data.ch_[ch_9_aux5] > 1200&&rc_in.rc_ch.st_data.ch_[ch_9_aux5] < 1700 )
//		{
//			IO5_H;IO6_L;IO7_H;IO8_L;//����
//			tingzhi=0;
//		}
//		else if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1200 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 1700 && rc_in.rc_ch.st_data.ch_[ch_9_aux5] < 1200)
//		{
//			IO5_H;IO6_L;IO7_L;IO8_H;//��ת ����ǰ ���ֺ�
//			tingzhi=0;
//		}
//		else if(rc_in.rc_ch.st_data.ch_[ch_8_aux4] > 1200 &&rc_in.rc_ch.st_data.ch_[ch_8_aux4] < 1700 && rc_in.rc_ch.st_data.ch_[ch_9_aux5] > 1700)
//		{
//			IO5_L;IO6_H;IO7_H;IO8_L;//��ת ���ֺ� ����ǰ
//			tingzhi=0;
//		}
//		else
//		{
//			IO5_L;IO6_L;IO7_L;IO8_L;//ֹͣ
//			tingzhi=0;
//		}
	}
}
void UserTask_CeXi(void)
{
	static u8 one_key_cexi_f = 1,one_key_cexituoluo_f=1,cexi_flag=0,cexituoluo_flag=0,err_flag;
	if (rc_in.fail_safe == 0)
	{
		if (rc_in.rc_ch.st_data.ch_[ch_10_aux6] > 1700 )//����
		{
			if(one_key_cexi_f==0)
			{
				cexi_step=1;
				AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"CEXI START......");//������ʼ
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
				AnoDTSendStr(USE_HID|USE_U2,SWJ_ADDR,LOG_COLOR_GREEN,"HUIZHENG START......");//������ʼ
			}
		}
		else 
		{
				one_key_cexituoluo_f=0;
		}
	}
}


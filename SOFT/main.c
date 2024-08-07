#include "main.h"
#include <LPC21XX.H> 
#include "timer.h" 
#include "beep.h"
#include "control.h"
#include "25lc640.h"
#include "eeprom_map.h"
#include "ret.h"
#include "pcf8563.h"
#include "tlv2542.h"
#include "simbols.h"
#include "graphic.h"
#include "gran.h"
#include "common_func.h"
#include "lcd_AGM1232_uku100.h"
#include "memo.h"
#include "beep.h"
#include "watchdog.h"
#include "ad7705_soft.h"
#include "uart1.h"
#include "curr_version.h"

const char*  name_of_blok="���-1200-220�/7��-1�";

//-----------------------------------------------
//������
char b1000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz,b33Hz;
short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7,t0_cnt8;
char bFL5,bFL2,bFL,bFL_;

//-----------------------------------------------
//���
short adc_buff[8][16],adc_buff_[8],unet_buff[16],unet_buff_,adc_buff_U_;
char adc_cnt,adc_cnt1,adc_ch;


//-----------------------------------------------
//������������, ������������ �� EEPROM

signed short K_U;
signed short K_T;
signed short K_I_G[2];
signed short K_I_T[2];
signed short T_SIGN;
signed short T_MAX;
signed short I_MAX;
signed short PR;
signed short T_WRK;
enum_onoff ZV_AV_TEMPER;
enum_onoff ZV_AV_SRC;
signed short PU[10][5];
signed short PI[10][5];
signed short U_MAX;
signed short T_WRK_MAX;
signed short POWER;
enum_onoff 	REST;
signed short MNEMO_ON;
signed short U_MIN1;
signed short T_MIN1;
signed short U_MIN2;
signed short T_MIN2;
signed short ADR;
signed short MODE;
signed short I_VK;
enum_onoff 	auto_bl_kb;
signed short PAROL_KEYS;
signed short image_W=1;
signed short P_POV;
signed short T_POV;
signed short R_DOP;
signed short KOEF;
signed short BAUDRATE;
signed short MODBUS_TYPE;

//-----------------------------------------------
//���������
stuct_ind a,b[10];
//char image_W=1;
char dig[5];
char zero_on;
char lcd_buffer[LCD_SIZE];
char lcd_bitmap[1024];
const char ABCDEF[]={"0123456789ABCDEF"};
signed char ptr_ind=0;
char mnemo_cnt=15;
signed char parol[3];
char fl_simv_num,fl_simv_len;
char default_temp;
signed short av_j_si_max,av_j_si_max_;
char simax;
char phase;
const char sm_[]		={"                    "}; //
const char sm_exit[]	={" �����              "}; //
const char smon[]	={"���."}; 
const char smoff[]	={"���."};
const char sm_mont[13][4]={"���","���","���","���","���","���","���","���","���","���","���","���","���"}; //
char content[63];
unsigned short graph_out=0;
unsigned short capture=0;
unsigned short cont=0;
unsigned char low[488] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 
	0x00, 0x0E, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x07, 0xF8, 0x00, 0x00, 0x01, 0xFE, 0x00, 0x00, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x1F, 0xE0, 0x00, 
	0x00, 0x07, 0xF8, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x70, 0x00, 0x0F, 0x80, 0x7C, 0x00, 0x1F, 0x80, 0x7E, 0x00, 
	0x1F, 0x80, 0x7E, 0x00, 0x3F, 0x80, 0x1F, 0x00, 0x3E, 0x0C, 0x0F, 0x00, 0x3C, 0x0C, 0x07, 0x00, 
	0x38, 0x0C, 0x07, 0x00, 0x38, 0x0E, 0x0F, 0x00, 0x3C, 0x1F, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x1F, 0xFB, 0xFE, 0x00, 0x1F, 0xFB, 0xFC, 0x00, 0x0F, 0xF0, 0xF8, 0x00, 0x03, 0xE0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x00, 0x3F, 0xE0, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0xFF, 0xFC, 0x00, 0x07, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFE, 0x7F, 0x00, 0x3F, 0xF8, 0x3F, 0x00, 0x3F, 0xC0, 0x0F, 0x00, 0x3F, 0x00, 0x03, 0x00, 
	0x3C, 0x00, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x07, 0xF8, 0x00, 0x00, 0x01, 0xFE, 0x00, 0x00, 0x00, 0x7F, 0x80, 0x00, 
	0x00, 0x1F, 0xE0, 0x00, 0x00, 0x07, 0xF8, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x0C, 0x3F, 0xFF, 0xFF, 0x1C, 0x3F, 0xFF, 0xFF, 0x3C, 
	0x07, 0xF8, 0x00, 0x38, 0x01, 0xFE, 0x00, 0x30, 0x00, 0x7F, 0x80, 0x30, 0x00, 0x1F, 0xE0, 0x30, 
	0x00, 0x07, 0xF8, 0x38, 0x3F, 0xFF, 0xFF, 0x3C, 0x3F, 0xFF, 0xFF, 0x1C, 0x3F, 0xFF, 0xFF, 0x0C, 
	0x3F, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
unsigned char vacuum[488] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 
	0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC7, 0xF1, 0xF8, 0xFF, 0xC7, 0xF1, 0xF8, 0xFF, 
	0xC7, 0xF0, 0xF0, 0xFF, 0xC3, 0xE0, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x04, 0x00, 0xFF, 
	0xE0, 0x04, 0x01, 0xFF, 0xF0, 0x0F, 0x07, 0xFF, 0xF8, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xC0, 0x7F, 0xFF, 0xFF, 0xC0, 0x03, 0xFF, 0xFF, 
	0xC0, 0x00, 0x1F, 0xFF, 0xC0, 0x00, 0x01, 0xFF, 0xF0, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0xFF, 
	0xFE, 0x38, 0x00, 0xFF, 0xFE, 0x3F, 0xC0, 0xFF, 0xFE, 0x38, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0xFF, 
	0xF0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x3F, 0xFF, 0xC0, 0x03, 0xFF, 0xFF, 
	0xC0, 0x7F, 0xFF, 0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 
	0xC0, 0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x1F, 0xFF, 0xFF, 0xC0, 0x0F, 0xFF, 0xFF, 0x00, 0x03, 0xFF, 
	0xF8, 0x00, 0x00, 0xFF, 0xC0, 0x01, 0x80, 0xFF, 0xC0, 0x07, 0xC0, 0xFF, 0xC0, 0x3F, 0xF0, 0xFF, 
	0xC0, 0xFF, 0xFC, 0xFF, 0xC3, 0xFF, 0xFE, 0xFF, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 
	0xFF, 0xFF, 0xF0, 0xFF, 0xC7, 0xFF, 0xC0, 0xFF, 0xC7, 0xFE, 0x00, 0xFF, 0xC7, 0xF0, 0x00, 0xFF, 
	0xC0, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x07, 0xFF, 0xE0, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 
	0xFE, 0x00, 0x01, 0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 
	0xFF, 0xFF, 0xE0, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xF0, 0xFF, 
	0xC7, 0xFF, 0xC0, 0xFF, 0xC7, 0xFE, 0x00, 0xFF, 0xC7, 0xF0, 0x00, 0xFF, 0xC0, 0x00, 0x01, 0xFF, 
	0xC0, 0x00, 0x07, 0xFF, 0xE0, 0x00, 0x3F, 0xFF, 0xF0, 0x00, 0x0F, 0xFF, 0xFE, 0x00, 0x01, 0xFF, 
	0xFF, 0xC0, 0x00, 0xFF, 0xFF, 0xF8, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xFF, 
	0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 
	0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xFF, 0xFE, 0x00, 0xFF, 
	0xFF, 0xE0, 0x00, 0xFF, 0xFE, 0x00, 0x01, 0xFF, 0xE0, 0x00, 0x1F, 0xFF, 0xC0, 0x01, 0xFF, 0xFF, 
	0xC0, 0x1F, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xE0, 0x00, 0x7F, 0xFF, 0xFE, 0x00, 0x07, 0xFF, 
	0xFF, 0xE0, 0x00, 0xFF, 0xFF, 0xFE, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 
	0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

 



//-----------------------------------------------
//�����
unsigned short hour_cnt_5hz,hour_cnt_1hz;
char cnt_ind;

//-----------------------------------------------
//�������
unsigned short rotor_can[6];
unsigned short cnt_sec;

//-----------------------------------------------
//������
char speed,l_but,n_but;
char but;                            
unsigned long but_n,but_s;
char but0_cnt;
char but1_cnt;
char but_onL_temp;
unsigned short count_but_idle=1000;

//-----------------------------------------------
//��������� ������ �������
signed short U,t,Ig_u,Usim,P,Pmax;
signed int It;
enum_ist ist=istG;
signed long Ig,Ires;
signed long Idop;

char dummy_dumm[100];
//-----------------------------------------------
//���������� ��������
enum_wrk_state wrk_state=wrkOFF;
signed short pwmI,pwmU,pwmI_start,pwmI_max;
signed long wrk_cnt;
char wrk_phase;
signed short Isim;
signed long wrk_cnt_cnt;
signed short i_pov_cnt,i_pov_cnt1;

// char dumm @1000;

char plazma_plazma;
char plazma_ind;
char plazma;
char cnt_lcd_init;
char plazma_pal;

//-----------------------------------------------
//�������� �������   
signed short kv_cnt;
enum_av_kv_stat av_kv_stat; 
signed short kv_ind_cnt;

//-----------------------------------------------
//�������� ������
signed short av_out_cnt;
enum_av_kv_stat av_out_stat;
signed short av_out_ind_cnt;
//-----------------------------------------------
void bitmap_hndl(void)
{
short x,ii,i,y;
unsigned int ptr_bitmap;
/*static char ptr_cnt,ptr_cnt1,ptr_cnt2,ptr_cnt3;*/
for(ii=0;ii<488;ii++)
	{
	lcd_bitmap[ii]=0x00;
	}

	{
	if((!mnemo_cnt)&&(ind==iMn)&&(!((av_kv_stat==avON)&&(kv_ind_cnt<=20)))&&(!((av_out_stat==avON)&&(av_out_ind_cnt<=20)))/*&&(index_set==0)*/&&(wrk_state!=wrkOFF)&&(MNEMO_ON))
		{
		signed short temp_SS;
		
		temp_SS=U;
		//temp_SS=123;
		zero_on=1;

		if(temp_SS/1000) 
			{
			print_simb((temp_SS/1000)+0x30,0,5);
			zero_on=0;
			}
		temp_SS%=1000;

		if((!zero_on)||(temp_SS/100)) 
			{
			print_simb((temp_SS/100)+0x30,11,5);
			zero_on=0;
			}
		temp_SS%=100;

		if((!zero_on)||(temp_SS/10)) 
			{
			print_simb((temp_SS/10)+0x30,22,5);
			zero_on=0;
			}
		temp_SS%=10;

		print_simb((temp_SS)+0x30,33,5);


		print_simb('V',43,5);


		{
		signed long tempSL;

		tempSL=Ires;

		if(tempSL>=1000000)
			{
			tempSL/=10000;
			zero_on=1;
			 		
			if(tempSL/1000)
				{
				zero_on=0;
				print_simb((tempSL/1000)+0x30,55,5);
				}
			tempSL=tempSL%1000;

			if((tempSL/100)||(zero_on==0))
				{
				zero_on=0;
				print_simb((tempSL/100)+0x30,66,5);
				}
			tempSL=tempSL%100;

			if((tempSL/10)||(zero_on==0))
				{
				zero_on=0;
				print_simb((tempSL/10)+0x30,77,5);
				}
		    	tempSL=tempSL%10;

			print_simb((tempSL)+0x30,88,5);

			print_simb('m',99,5);
			print_simb('A',110,5);			
			}

		else if(tempSL>=100000)
			{
			tempSL/=1000;
			zero_on=1;
			 		
			if(tempSL/100)
				{
				zero_on=0;
				print_simb((tempSL/100)+0x30,61,5);
				}
			tempSL=tempSL%100;

			print_simb((tempSL/10)+0x30,72,5);
			tempSL=tempSL%10;
			
			print_simb('.',83,5);
			
			print_simb((tempSL)+0x30,88,5);

			print_simb('m',99,5);
			print_simb('A',110,5);			
			}

		else if(tempSL>=10000)
			{
			tempSL/=100;
			zero_on=1;
			 		
	
			print_simb((tempSL/100)+0x30,61,5);
			
			tempSL=tempSL%100;

			print_simb('.',72,5);

			print_simb((tempSL/10)+0x30,77,5);
			tempSL=tempSL%10;
			
			
			
			print_simb((tempSL)+0x30,88,5);

			print_simb('m',99,5);
			print_simb('A',110,5);			
			}
		else if(tempSL>=100)
			{
			tempSL/=10;
			zero_on=1;
			
			 		
			if(tempSL/1000)
				{
				zero_on=0;
				print_simb((tempSL/1000)+0x30,55,5);
				}
			tempSL=tempSL%1000;

			if((tempSL/100)||(zero_on==0))
				{
				zero_on=0;
				print_simb((tempSL/100)+0x30,66,5);
				}
			tempSL=tempSL%100;

			if((tempSL/10)||(zero_on==0))
				{
				zero_on=0;
				print_simb((tempSL/10)+0x30,77,5);
				}
		    	tempSL=tempSL%10;

			print_simb((tempSL)+0x30,88,5);

			print_simb('�',99,5);
			print_simb('A',110,5);			
			}	 
		else 
			{
			zero_on=1;
			
			 		
			print_simb((tempSL/10)+0x30,72,5);
			tempSL=tempSL%10;

			print_simb('.',83,5);
			print_simb((tempSL)+0x30,88,5);

			print_simb('�',99,5);
			print_simb('A',110,5);			
			}

		}

		}
	else if	((graph_out)/*&&(wrk_state==wrkON)*/)
		{
		if(!capture)
			{
			if(cont<10)
			{
			cont++;
			for(x=0;x<122;x++)
				{
				for(y=0;y<4;y++)
					{
					ptr_bitmap=y+4*(unsigned)x;
					lcd_bitmap[(3-y)*122+x]=low[ptr_bitmap];
					}		
				}
			 }
			 else
			 {
			 capture=1;
			 cont=0;
			 }
		   }
		else
			{
			if(cont<10)
			{
			cont++;
			for(x=0;x<122;x++)
				{
				for(y=0;y<4;y++)
					{
					ptr_bitmap=y+4*(unsigned)x;
					lcd_bitmap[(3-y)*122+x]=vacuum[ptr_bitmap];
					}		
				}
			 }
			 else
			 {
			 capture=0;
			 cont=0;
			 }
			}

		   }


		

	else 
		{
		for(i=0;i<4;i++)
			{
			ptr_bitmap=122*(unsigned)i;
			for(x=(20*i);x<((20*i)+20);x++)
	 			{
				lcd_bitmap[ptr_bitmap++]=caracter[(unsigned)lcd_buffer[x]*6];
				lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+1];
				lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+2];
				lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+3];
				lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+4];
				lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+5];
				} 
			} 

		}

	//draw_rectangle(0,0,20,20,0,0);
	
/*	draw_a_segm(0,5);
	draw_b_segm(0,5);
	draw_c_segm(0,5);
	draw_d_segm(0,5);
	draw_e_segm(0,5);
	draw_f_segm(0,5);
	draw_g_segm(0,5); */

/*	print_simb('3',0,5);
	print_simb('.',10,5);
	print_simb('9',14,5);
	print_simb('k',24,5);
	print_simb('V',34,5);

	print_simb('1',50,5);
	print_simb('4',61,5);
	print_simb('5',72,5);
	print_simb('7',83,5);
	print_simb('�',94,5);
	print_simb('A',105,5); */

	/*	for(i=0;i<4;i++)
		{
		ptr_bitmap=122*(unsigned)i;
		for(x=(20*i);x<((20*i)+20);x++)
	 		{
			lcd_bitmap[ptr_bitmap++]=caracter[(unsigned)lcd_buffer[x]*6];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+1];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+2];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+3];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+4];
			lcd_bitmap[ptr_bitmap++]=caracter[((unsigned)lcd_buffer[x]*6)+5];
			} 
		} */
	}	

}

//-----------------------------------------------
void ind_hndl(void)
{
//const char* ptr;
const char* ptrs[30];
const char* sub_ptrs[30];
static char sub_cnt,sub_cnt1;
char i,sub_cnt_max;
graph_out=0;

sub_cnt_max=5;
i=0;

if(ind==iDeb)
     {
     if(sub_ind==0) 
     	{
     	bgnd_par("U                   ",
     	         "Ig                  ",
     	         "It                  ",
     	         "                    ");

			    int2lcdyx(U,0,7,0);
			    int2lcdyx(Ig,1,7,0);
			    long2lcdyx(It,2,7,0);
			    if(Ires==(Ig*10))long2lcdyx(Ires,1,17,0);
			    else long2lcdyx(Ires,2,17,0);

                   int2lcdyx(i_pov_cnt1,3,14,0);
		         int2lcdyx(i_pov_cnt,3,19,0);
		}
    	else if(sub_ind==1) 
     	{
     	bgnd_par("                    ",
     	         "                    ",
     	         "                    ",
     	         "                    ");
   		

		int2lcdyx((signed short)(Ig/1000L),0,19,0);
		int2lcdyx(Ig_u,1,19,0);
		int2lcdyx(U,2,19,0);
		int2lcdyx(P,3,19,0);
		int2lcdyx(Pmax,3,13,0);
		int2lcdyx(I_MAX_PWM,0,13,0);
		int2lcdyx(pwmI_max,1,13,0);

		int2lcdyx(pwmU,0,7,0);
		int2lcdyx(pwmI,1,7,0);
		int2lcdyx(wrk_phase,0,1,0);
		int2lcdyx(pwmI_start,1,4,0);

		long2lcdyx(plazma_long,2,7,0);

    	/* 		int2lcdyx(tlv_buff[0][4],1,4,0);
		int2lcdyx(tlv_buff[0][5],1,9,0);
		int2lcdyx(tlv_buff[0][6],1,14,0);
		int2lcdyx(tlv_buff[0][7],1,19,0);

   		int2lcdyx(tlv_buff[0][8],2,4,0);
		int2lcdyx(tlv_buff[0][9],2,9,0);
		int2lcdyx(tlv_buff[0][10],2,14,0);
		int2lcdyx(tlv_buff[0][11],2,19,0);	*/ 
		}
    	else if(sub_ind==2) 
     	{
     	bgnd_par("                    ",
     	         "       0           0",
     	         "                    ",
     	         "                    ");
   		

		int2lcdyx((signed short)(Ires/1000L),0,19,0);
		int2lcdyx((signed short)(current_sigma/10L),1,18,0);
		int2lcdyx(current_sigma_stat,2,19,0);
		int2lcdyx(pwmI,3,19,0);
		int2lcdyx((signed short)(plazma_long),0,14,0);
		int2lcdyx((short)power_sigma_cnt_30min,2,15,0);
		int2lcdyx((signed short)(P),0,7,0);
		int2lcdyx((signed short)(power_sigma/10L),1,6,0);
		int2lcdyx(power_sigma_stat,2,7,0);
		int2lcdyx((signed short)(Pmax),2,3,0);
		int2lcdyx(Pmax,3,7,0);
		}
     }

else if(ind==iMn)
	{
	if(wrk_state==wrkOFF)
		{
		ptrs[0]=	" ����� ������ 0!:0@ ";
		if(image_W!=1)
		ptrs[1]=	" ��������       %�� ";
		else
		ptrs[1]=	" ��������        %% ";
		ptrs[2]=	" �������          # ";
		ptrs[3]=	" ������ ��          ";
		ptrs[4]=	" ���������          ";
	    ptrs[5]=	" ����������         ";


		if((sub_ind-index_set)>3)index_set=sub_ind-3;
		else if(sub_ind<index_set)index_set=sub_ind;

		bgnd_par(ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);

		pointer_set(0);

		if(T_WRK_MAX==0)
			{
			sub_bgnd("����.",'!',-1);
			}
		else 
			{

			int2lcd(T_WRK_MAX/60,'!',0);
			int2lcd((T_WRK_MAX)%60,'@',0);
			}

		int2lcd(POWER*image_W,'%',0);

		if(REST==stON)
			{
			sub_bgnd("���.",'#',-2);
			}
		else sub_bgnd("����.",'#',-3);
		//int2lcdyx(sub_ind,0,10,0);
		//int2lcdyx(plazma_pal,0,3,0);
		//int2lcdyx(plazma_plazma,0,8,0);
		//long2lcdhyx(but_n,2,10);
		
		}
		
	else if(wrk_state==wrkON)
		{
		ptrs[0]=	"  0|:0@      0#:0$  ";
		if(Ires>=10000)
		ptrs[1]=	" U=    x� I=   <��  ";
		else
		ptrs[1]=	" U=    x� I=   >��� ";
		if(p_pov_cnt)
			{
			if(image_W!=1)
			ptrs[2]=	" �������� ���.   (��";
			else
			ptrs[2]=	" �������� ���.   (% ";
			}
		else 
			{
			if(image_W!=1)
			ptrs[2]=	" ��������        (��";
			else
			ptrs[2]=	" ��������        (% ";
			}

		ptrs[3]=	" �������          ^ ";
		ptrs[4]=	" t���          )�C  ";
		ptrs[5]=	" ���������          ";

		
		if(temper_state==tsAV)
			{
			if(bFL2)
				{
				bgnd_par(	"                    ",
						"      ��������      ",
						"     ��������!!!    ",
						"                    ");
				}
			else bgnd_par(	"                    ",
						"                    ",
						"                    ",
						"                    ");
			}
			
		else if((av_kv_stat==avON)&&(kv_ind_cnt<=20))
			{
 			graph_out=1;

			}
		else if((av_out_stat==avON)&&(av_out_ind_cnt<=20))
			{
 			bgnd_par(	"     ���������      ",
					"       ����         ",
					"     ����������     ",
					"                    ");

			}
		else 
			{
			bgnd_par(ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2],ptrs[index_set+3]);
			if(index_set==2)lcd_buffer[60]=1;
			}
		lcd_buffer[find('z')]=1;
		if (!graph_out)
		{
		capture=0;
		cont=0;
		}

		if(T_WRK_MAX==0)
			{
			sub_bgnd("����������",'|',-2);
			}
		else 
			{
			int2lcd(T_WRK_MAX/60,'|',0);
			int2lcd((T_WRK_MAX)%60,'@',0);
			}
		int2lcd(wrk_cnt/3600L,'#',0);
		int2lcd((wrk_cnt/60L)%60L,'$',0);
		if((bFL2)&&(index_set==0))lcd_buffer[15]=' ';
		int2lcd(U,'x',0);

		if(Ires>=1000000)
			{
			int2lcd(Ires/10000,'<',0);
			}
		else if(Ires>=100000)
			{
			int2lcd(Ires/1000,'<',1);
			}
		else if(Ires>=10000)
			{
			int2lcd(Ires/100,'<',2);
			}
		/*else if((Ires>1000)&&(Ires<1000000))
			{
			int2lcd(Ires/100,'<',1);
			}*/

		else if(Ires>=1000)
			{
			int2lcd(Ires/10,'>',0);
			}
		else
			{
			int2lcd(Ires,'>',1);
			}
				
		int2lcd(t,')',0);
		if(p_pov_cnt)int2lcd(P_POV*image_W,'(',0);
		else int2lcd(POWER*image_W,'(',0);
		if(REST==stON)
			{
			sub_bgnd("���.",'^',-2);
			}
		else sub_bgnd("����.",'^',-3);		
	/*	int2lcdyx(pwmI,3,5,0);
		int2lcdyx(pwmU,3,18,0);

		int2lcdyx(P,0,4,0);			
		int2lcdyx(Pmax,0,9,0);		  */
		

		//int2lcdyx(pwmI,0,19,0);
		//int2lcdyx(plazma,0,15,0);

		}					

//int2lcdyx(plazma/*rx_wr_index1/*ad7705_res1/*ad7705_buff_[0]*/,3,5,0); 
//int2lcdyx(rx_wr_index1,3,19,0);
/*
int2lcdyx(av_out_cnt,0,2,0);  
int2lcdyx(UIB1[1],2,5,0);
int2lcdyx(UIB1[2],2,8,0);
int2lcdyx(UIB1[3],2,11,0);
int2lcdyx(UIB1[4],2,14,0); */
	//int2lcdyx(ad7705_res2/*ad7705_buff_[1]*/,3,15,0);
							
	//long2lcdyx(Ig,2,7,0);
	//long2lcdyx(plazma_long,3,7,0);

	//long2lcdyx_mmm(t_u_min1,2,6,0);
	//long2lcdyx_mmm(t_u_min2,3,6,0);

/*	if(	(av_kv_stat==avON) ||
	(temper_state!=tsNORM)||
	((Ires<100000UL)&&(U<200)))lcd_buffer[19]='A'; */


/*	int2lcdyx(modbus_rtu_plazma[0],0,3,0);
	int2lcdyx(modbus_rtu_plazma[1],0,9,0);
	int2lcdyx(modbus_rtu_plazma[2],1,3,0);
	int2lcdyx(modbus_rtu_plazma[3],1,9,0);	*/
//	int2lcdyx(lc640_read_int(EE_DEBUG),2,5,0);
	//int2lcdyx(p_pov_cnt,0,2,0); 
    	} 
     



	
	
else if(ind==iSet)
	{
	ptrs[0]=		" U����          !B  ";
	ptrs[1]=		" I����          @mA ";
	ptrs[2]=		" �����.�������.    y";
	ptrs[3]=		" t����          #�C ";
	ptrs[4]=		" t����          $�C ";
	ptrs[5]=		" �����������        ";
	ptrs[6]=		" U���1           z� ";
	ptrs[7]=		" t�����1         Zc.";
	ptrs[8]=		" U���2           [� ";
	ptrs[9]=		" t�����2         ]c.";
	if(I_VK>100) 
	ptrs[10]=		" I�����.���.    <mA ";
	else 
	ptrs[10]=		" I�����.���.    <mkA";
	ptrs[11]=		" ��������� Pmax   ^ ";
	if(image_W==1)
	ptrs[12]=		" P���.            {%";
	else 
	ptrs[12]=		" P���.          {��.";
	ptrs[13]=		" T���.         }���.";
	ptrs[14]=		" ��������.����������";
    ptrs[15]=		" R���.         q��� ";
	ptrs[16]=		" MODBUS       ASCII ";
	if(MODBUS_TYPE==1)
	ptrs[16]=		" MODBUS         RTU ";
	ptrs[17]=		" MODBUS BAUDRATE    "; 
	ptrs[18]=		"                 Y00";
	ptrs[19]=		" MODBUS ADDRESS  >h ";
	if(MODE==_7_200_)
	{
	ptrs[20]=		" �����������      Q ";
	ptrs[21]=		" �����              ";
	}
	else
	{
	ptrs[20]=		" �����              ";
	}
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("     ���������      ",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);

	pointer_set(1);
	int2lcd(U_MAX,'!',0);
	int2lcd(I_MAX,'@',0);
	if(MNEMO_ON)
	    	{
	    	sub_bgnd("����� y�.",'y',-8);
	    	int2lcd(MNEMO_ON,'y',0);
	    	}
	else sub_bgnd("����.",'y',-4);
		
	int2lcd(T_SIGN,'#',0);
	int2lcd(T_MAX,'$',0);	
	if(ZV_AV_TEMPER==stON) sub_bgnd("���.",'#',0);
	else sub_bgnd("���.",'#',0);
	if(ZV_AV_SRC==stON) sub_bgnd("���.",'$',0);
	else sub_bgnd("���.",'$',0);
	if(image_W!=1) sub_bgnd("��",'^',0);
	else sub_bgnd("%",'^',0);


	if((U_MIN1>=10)&&(U_MIN1<=100))
		{
		int2lcd(U_MIN1,'z',0);
		int2lcd(T_MIN1,'Z',0);
		}
	else 
		{
		sub_bgnd("����.",'z',-3);
		sub_bgnd("����.",'Z',-3);
		}

	if((U_MIN2>=10)&&(U_MIN2<=300))
		{
		int2lcd(U_MIN2,'[',0);
		int2lcd(T_MIN2,']',0);
		}
	else 
		{
		sub_bgnd("����.",'[',-3);
		sub_bgnd("����.",']',-3);
		}
	
	char2lcdh((char)ADR,'>');
	//int2lcd(ADR,'<',0);

	if(I_VK<10) sub_bgnd("����.",'<',-1);	
	if(I_VK>100) 	int2lcd(I_VK/10,'<',1);
	else int2lcd(I_VK*10,'<',0);

	int2lcd(P_POV*image_W,'{',0);
	if(T_POV)int2lcd(T_POV,'}',0);
	else 
		{
		sub_bgnd("����.",'}',0);
		}

     if((R_DOP<=200)&&(R_DOP>=10))int2lcd(R_DOP,'q',0);
	else 
		{
		sub_bgnd("����.",'q',0);
		}
	if(KOEF)int2lcd(25,'Q',1);
	else int2lcd(1,'Q',0);
	int2lcd(BAUDRATE,'Y',0);		 
	}       

else if(ind==iDef_set)
	{
	bgnd_par(	"    ��� ���������   ",
			"     ����������     ",
			"    �� ���������    ",
			"    �������    �    ");
	}

else if(ind==iSet_T)
	{
	static char phase_cnt;
	if(++phase_cnt>=15)
	     {
	     phase_cnt=0;
	     if(++phase>=3)phase=0;
	     }
	ptrs[0]=" 0%:0^:0& 0</>  /0{ ";
	ptrs[1]=sm_;
	if(phase==0)ptrs[2]="     <> - �����     ";
     if(phase==1)ptrs[2]="   ^v - ���������   ";
     if(phase==2)ptrs[2]="     0  - �����     ";
	
	bgnd_par(" ���������  ������� ",ptrs[0],ptrs[1],ptrs[2]);
     if(sub_ind==0)lcd_buffer[42]='^';
     else if(sub_ind==1)lcd_buffer[45]='^';
     else if(sub_ind==2)lcd_buffer[48]='^';
     else if(sub_ind==3)lcd_buffer[51]='^';
     else if(sub_ind==4)lcd_buffer[54]='^';
     else if(sub_ind==5)lcd_buffer[58]='^';
  
 	int2lcd(sec__,'&',0);
 	int2lcd(min__,'^',0);
 	int2lcd(hour__,'%',0);
 	
 	int2lcd(day__,'<',0);
 	sub_bgnd(sm_mont[month__],'>',0);
 	int2lcd(year__,'{',0);
 	if(bFL2)
 	     {
 	     lcd_buffer[find(':')]=' ';
 	     lcd_buffer[find(':')]=' ';
 	     }  
	}  

else if(ind==iABl_kb)
	{
	ptrs[0]=		" ����������     ~   ";
    ptrs[1]=		" �������� ������	 ";
    ptrs[2]=		" �����              ";

	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;
	bgnd_par("��������  ����������",
			ptrs[index_set],ptrs[index_set+1],ptrs[index_set+2]);
	pointer_set(1);

	if(auto_bl_kb==stON) sub_bgnd("���",'~',0);
	else sub_bgnd("����",'~',0);




	}


else if(ind==iK)
	{

	if(MODE==_7_1000_)
	ptrs[0]=		" ����   7�� 1�      ";
	else
	ptrs[0]=		" ����   7�� 0,2�    ";
		
	ptrs[1]=" U���	          !�  ";
	if(Ig<1000)
	ptrs[2]=" I���.��.       @mk�";
	else
     ptrs[2]=" I���.��.       @m� ";
	ptrs[3]=" I���.���.      #mk�";
     ptrs[4]=" t���.          $�C ";
     ptrs[5]=" �����              ";
	
	if((sub_ind-index_set)>2)index_set=sub_ind-2;
	else if(sub_ind<index_set)index_set=sub_ind;

	bgnd_par("     ����������     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	pointer_set(1);
	
	
	int2lcd(It,'#',1);
	if(Ig<1000)
	int2lcd((signed short)(Ig),'@',0);
	else 
	int2lcd((signed short)(Ig/100),'@',1);
	int2lcd(U,'!',0);
	int2lcd(t,'$',0);
	
	//int2lcdyx(K_I_T[0],0,5,0);
	//int2lcdyx(K_I_T[1],0,10,0);
	//int2lcdyx(tlv_buff_buff_[0],0,4,0);
	//int2lcdyx(tlv_buff_buff_[1],0,9,0);
	

	//int2lcdyx(ad7705_buff_[0],0,5,0);
	//int2lcdyx(ad7705_buff_[1],0,15,0);		
 

	//int2lcdyx(pwmU,0,5,0);
	//int2lcdyx(pwmI,0,15,0);
	}    



	
else if((ind==iSet_prl)||(ind==iK_prl)||(ind==iSpc_prl_vz)||(ind==iSet_prl_new)||(ind==iSet_prl_change)
	||(ind==iSpc_prl_ke)||(ind==iAusw_prl)||(ind==iPrltst)||(ind==iABl_kb_prl)||(ind==iRBl_kb_prl))
	{
	if(ind==iSet_prl_new) bgnd_par("������� ����� ������",sm_,sm_,sm_);
	else if(ind==iSet_prl_change) bgnd_par("������� ���� ������ ",sm_,sm_,sm_);
	else if((ind==iRBl_kb_prl)||(ind==iABl_kb_prl)) bgnd_par("������ �������������",sm_,sm_,sm_);
	else bgnd_par("  �������  ������   ",sm_,sm_,sm_);
	int2lcdyx(parol[0],1,8,0);
     int2lcdyx(parol[1],1,9,0);
     int2lcdyx(parol[2],1,10,0);
     //lcd_buffer[48+sub_ind]='�';	
     lcd_buffer[48+sub_ind]='�';
	}	
	
			
//int2lcdyx(u_necc,0,19,0);
//int2lcdyx(cntrl_stat1,0,3,0);
//int2lcdyx(cntrl_stat2,0,7,0);
//int2lcdyx(u_necc,0,19,0);
//int2lcdyx(ind,1,19,0);
//int2lcdyx(sub_ind,2,19,0);
//int2lcdyx(index_set,3,19,0);
//int2lcdyx(Kisrc[0],1,19,0);
//int2lcdyx(Kisrc[1],2,19,0);
//int2lcdyx(retcntsec,7,16,0); 
//int2lcdyx(plazma[0],1,20,0);
//int2lcdyx(plazma[1],2,20,0);

else if(ind==iDnd)
     {
     bgnd_par("      ������        "
             ,"    �������� !!!    "
             ,"                    "
             ,"                    ");	
     }
else if(ind==iPMg)
	{
     bgnd_par("       ������       "
             ,"     �������!!!     "
             ,"                    "
             ,"                    ");	
     }


else if(ind==iPr)
	{
	ptrs[0]=" U1=   @�  I1=   #��";
     ptrs[1]=" U2=   $�  I2=   %��";
	ptrs[2]=" U3=   ^�  I3=   &��";
     ptrs[3]=" U4=   *�  I4=   +��";
     ptrs[4]=" U5=   (�  I5=   )��";
     ptrs[5]=" �����              ";
	
	if(((sub_ind/2)-index_set)>2)index_set=(sub_ind/2)-2;
	else if((sub_ind/2)<index_set)index_set=(sub_ind/2);

	bgnd_par("     ������� N!     ",
			ptrs[index_set],
			ptrs[index_set+1],
			ptrs[index_set+2]);

	if((sub_ind/2)==index_set)
		{
		if((sub_ind%2)==0) lcd_buffer[20]=1;
		else lcd_buffer[30]=1;
		}
	else if((sub_ind/2)==(index_set+1))
		{
		if((sub_ind%2)==0) lcd_buffer[40]=1;
		else lcd_buffer[50]=1;
		}
	else if((sub_ind/2)==(index_set+2))
		{
		if((sub_ind%2)==0) lcd_buffer[60]=1;
		else lcd_buffer[70]=1;
		}

	int2lcd(sub_ind1+1,'!',0);
	int2lcd(PU[sub_ind1][0],'@',0);
	int2lcd(PI[sub_ind1][0],'#',0);
	int2lcd(PU[sub_ind1][1],'$',0);
	int2lcd(PI[sub_ind1][1],'%',0);
	int2lcd(PU[sub_ind1][2],'^',0);
	int2lcd(PI[sub_ind1][2],'&',0);
	int2lcd(PU[sub_ind1][3],'*',0);
	int2lcd(PI[sub_ind1][3],'+',0);
	int2lcd(PU[sub_ind1][4],'(',0);
	int2lcd(PI[sub_ind1][4],')',0);
	}

else if(ind==iSTOP_umin1)
	{
	bgnd_par(	"       �������      ",
			"     ����������     ",
			"     U��� <  !�    ",
			"                    ");
	int2lcd(U_MIN1,'!',0);

	}
else if(ind==iSTOP_umin2)
	{
	bgnd_par(	"       �������      ",
			"     ����������     ",
			"     U��� <  !�    ",
			"                    ");
	int2lcd(U_MIN2,'!',0);

	}

else if(ind==iFWabout)
	{
	bgnd_par(	" ������             ",
				" ������  0000.00.00 ",
				#ifdef WG12232A
				//" WG12232A           ",
				"                    ",
				#endif
				#ifdef WG12232L3
				" WG12232L3          ",
				#endif
				"                    ");
	int2lcdyx(BUILD_YEAR,1,12,0);
	int2lcdyx(BUILD_MONTH,1,15,0);
	int2lcdyx(BUILD_DAY,1,18,0);
	
	sprintf(&lcd_buffer[9],"%d.%d.%d",HARDVARE_VERSION,SOFT_VERSION,BUILD);
	}

/*int2lcdyx(ad7705_res1,0,5,0);
int2lcdyx(ad7705_res2,0,10,0);
int2lcdyx(ad7705_buff_[0],1,5,0);
int2lcdyx(ad7705_buff_[1],1,10,0);*/

if((bFL2)&&(fl_simv_len))
	{
	for(i=fl_simv_num;i<fl_simv_num+fl_simv_len;i++)
		{
		lcd_buffer[i]=' ';
		}
	}
/*
if(bFL5)
	{
	for(i=4;i<12;i++)
		{
		lcd_buffer[i]=' ';
		}
	}	*/

	
		   
} 
#define BUT_ON 4
#define BUT_ONL 20 

#define butU   254
#define butU_  126
#define butD   253
#define butD_  125
#define butL   251
#define butL_  123
#define butR   247
#define butR_  119
#define butE   239
#define butE_  111
#define butUD  252
#define butLD  249
#define butLR   243
//-----------------------------------------------
void but_drv(void)
{
plazma_plazma++;
but_n=IO1PIN|(0xFFFFFFFFUL&(~(1UL<<BUT0))&(~(1UL<<BUT1))&(~(1UL<<BUT2))&(~(1UL<<BUT3))&(~(1UL<<BUT4)));
if((but_n==0xffffffffUL)||(but_n!=but_s))
 	{
 	speed=0;
 
   	if (((but0_cnt>=BUT_ON)||(but1_cnt!=0))&&(!l_but))
  		{
   	     n_but=1;
          but=*(((char*)&but_s)+2);

          }
   	if (but1_cnt>=but_onL_temp)
  		{
   	     n_but=1;
 
          but=*(((char*)&but_s)+2)&0x7f;
          }
    	l_but=0;
   	but_onL_temp=BUT_ONL;
    	but0_cnt=0;
  	but1_cnt=0;          
     goto but_drv_out;
  	}
if(but_n==but_s)
 	{
  	but0_cnt++;
  	if(but0_cnt>=BUT_ON)
  		{
   		but0_cnt=0;
   		but1_cnt++;
   		if(but1_cnt>=but_onL_temp)
   			{              
    			but=but=*(((char*)&but_s)+2)&0x7f;;
    			but1_cnt=0;
    			n_but=1;
    			     
    			l_but=1;
			if(speed)
				{
    				but_onL_temp=but_onL_temp>>1;
        			if(but_onL_temp<=2) but_onL_temp=2;
				}    
   			}
  		}
 	}
but_drv_out: 
but_s=but_n;



PINSEL2&=~(1UL<<((BUT0-16)*2))&~(1UL<<(((BUT0-16)*2)+1))
	   &~(1UL<<((BUT1-16)*2))&~(1UL<<(((BUT1-16)*2)+1))
	   &~(1UL<<((BUT2-16)*2))&~(1UL<<(((BUT2-16)*2)+1))
	   &~(1UL<<((BUT3-16)*2))&~(1UL<<(((BUT3-16)*2)+1))
	   &~(1UL<<((BUT4-16)*2))&~(1UL<<(((BUT4-16)*2)+1));
IO1DIR&=~(1UL<<BUT0)&~(1UL<<BUT1)&~(1UL<<BUT2)&~(1UL<<BUT3)&~(1UL<<BUT4);
but_n=plazma_plazma;
	   
}

//-----------------------------------------------
void but_an(void)
{
signed short temp_SS;
signed int deep/*,i*/,cap,ptr;
char av_head[4];



crazy_beep=0;
if(!n_but)
     {
	count_but_idle++;
    if(count_but_idle>1000)count_but_idle=1000;
	goto but_an_end;
          }
if(count_but_idle<1000)count_but_idle=0;


if(but==butUD)	 
          {
     if(ind!=iDeb)
          {
          b[ptr_ind++]=a;
          ind=iDeb;
          sub_ind=0;
          }     
     else 
          {
          a=b[--ptr_ind];
     }
     }

if(but==butLD)	 
     {
	lcd_init();
	lcd_on();
	lcd_clear();
	}

else if(ind==iDeb)
	{
	if(but==butR)
		{
		sub_ind++;
		index_set=0;
		gran_ring_char(&sub_ind,0,2);
		}
	else if(but==butL)
		{
		sub_ind--;
		index_set=0;
		gran_ring_char(&sub_ind,0,2);
		}
     else if(but==butU)
	     {
	     index_set--;
	     gran_char(&index_set,0,4);
	     //lc640_write_int(ptr_ki_src[0],lc640_read_int(ptr_ki_src[0])+10);
	     }	
     else if(but==butD)
	     {
	     index_set++;
	     gran_char(&index_set,0,4); 
	     //lc640_write_int(ptr_ki_src[0],lc640_read_int(ptr_ki_src[0])-10);
	     }	
     else if(but==butE)
         	{
          a=b[--ptr_ind];
          }            				
	}

else if(ind==iMn)
	{
	if(wrk_state==wrkOFF)
		{
		if(but==butD)
			{
			sub_ind++;
			gran_ring_char(&sub_ind,0,5);
			//suz_temp=1;
			}
		else if(but==butU)
			{
			sub_ind--;
			gran_ring_char(&sub_ind,0,5);
			}

		else if(sub_ind==0)
			{
			if(but==butE)
				{
				//plazma_plazma=1;
				wrk_state=wrkON;
				pwmI_start=POWER*9;
				wrk_cnt_cnt=(signed long)T_WRK_MAX*300L;
				lc640_write_int(EE_WRK_STAT,wrk_state);
				ist=istT;
				t_u_min1=-10;
				t_u_min2=-10;
				}
			else if(auto_bl_kb==stON&&count_but_idle>=1000&&but)
				{
				parol_init();
				tree_up(iRBl_kb_prl,0,0,0);
				}	         
			else if(but==butR)T_WRK_MAX=((T_WRK_MAX)+1);
		     else if(but==butR_)T_WRK_MAX=((((T_WRK_MAX)/10)+1)*10);
	     	else if(but==butL)T_WRK_MAX=((T_WRK_MAX)-1);
	     	else if(but==butL_)T_WRK_MAX=((((T_WRK_MAX)/10)-1)*10);
			else if(but==butLR)T_WRK_MAX=0;
	     	gran_ring(&T_WRK_MAX,0,1440);
	     	lc640_write_int(EE_T_WRK_MAX,T_WRK_MAX);
	     	speed=1;
	     	}
		else if(sub_ind==1)
			{			
			if(but==butE)
				{
				wrk_state=wrkON;
				pwmI_start=POWER*9;
				wrk_cnt_cnt=(signed long)T_WRK_MAX*300L;
				lc640_write_int(EE_WRK_STAT,wrk_state);
				//wrk_cnt=(60L*60L*24L*5L)-10L;
				ist=istT;
				t_u_min1=-10;
				t_u_min2=-10;
				}
			else if(auto_bl_kb==stON&&count_but_idle>=1000&&but)
				{
				parol_init();
				tree_up(iRBl_kb_prl,0,0,0);
				}	         
			else if(but==butR)POWER++;
		     else if(but==butR_)POWER+=2;
	     	else if(but==butL)POWER--;
	     	else if(but==butL_)POWER-=2;
			else if(but==butLR)POWER=100;
	     	gran(&POWER,5,100);
	     	lc640_write_int(EE_POWER,POWER);
	     	speed=1;

			}
		else if(sub_ind==2)
			{
			if(but==butE)
				{
				wrk_state=wrkON;
				pwmI_start=POWER*9;
				wrk_cnt_cnt=(signed long)T_WRK_MAX*300L;
				lc640_write_int(EE_WRK_STAT,wrk_state);
				ist=istT;
				t_u_min1=-10;
				t_u_min2=-10;

				}
			else if(auto_bl_kb==stON&&count_but_idle>=1000&&but)
				{
				parol_init();
				tree_up(iRBl_kb_prl,0,0,0);
				}	         
			else
				{
				if(REST==stON)REST=stOFF;
				else REST=stON;
				}
	     	lc640_write_int(EE_REST,REST);
	     	speed=0;
			}
		else if(sub_ind==3) 
			{
			if(but==butE)
		     	{
		     	tree_up(iFWabout,0,0,0);
		     	}
			}
		else if(sub_ind==4)
			{
			if(auto_bl_kb==stON&&count_but_idle>=1000&&but)
				{
				parol_init();
				tree_up(iRBl_kb_prl,0,0,0);
				}	         
			else
			{
			parol_init();
			if(but==butE)
			tree_up(iSet_prl,0,0,0);
			}
			}
		else if(sub_ind==5)
			{
			if(auto_bl_kb==stON&&count_but_idle>=1000&&but)
				{
				parol_init();
				tree_up(iRBl_kb_prl,0,0,0);
				}	         
			else
			{
			parol_init();
			if(but==butE)tree_up(iK_prl,0,0,0);
			}
			}
		}
	else if(wrk_state==wrkON)
		{
		if(but==butD)
			{
			index_set++;
			gran_char(&index_set,0,2);
			}
		else if(but==butU)
			{
			index_set--;
			gran_char(&index_set,0,2);
			}

		else	if(but==butE)
			{
			if(index_set==2)
				{
				if(auto_bl_kb==stON&&count_but_idle>=1000&&but)
					{
					parol_init();
					tree_up(iRBl_kb_prl,0,0,0);
					}	         
				else
					{
					parol_init();
					if(but==butE)
					tree_up(iSet_prl,0,0,0);
					}
				}
			else if((!MNEMO_ON)||((mnemo_cnt)/*&&(mnemo_cnt!=MNEMO_ON)*/))
				{
				wrk_state=wrkOFF;
				lc640_write_int(EE_WRK_STAT,wrk_state);
				} 
			}
		else	if(but==butE_)
			{
			if(T_POV)
				{
				p_pov_cnt=T_POV*60;
				show_mess(	"      �������       ",
	          				"       �����        ",
	          				"     ����������     ",
	          				"      ��������      ",3000);
				 }
			}

		else if(auto_bl_kb==stON&&count_but_idle>=1000&&but)
			{
			parol_init();
			tree_up(iRBl_kb_prl,0,0,0);
			}
		else if(but==butR)POWER++;
		else if(but==butR_)POWER+=2;
	     else if(but==butL)POWER--;
	     else if(but==butL_)POWER-=2;
	     gran(&POWER,5,100);
	     lc640_write_int(EE_POWER,POWER);
	     speed=1;
	     }

	else if(wrk_state==wrkPR)
		{
		if(but==butD)
			{
			index_set=1;
			}
		else if(but==butU)
			{
			index_set=0;
			}

		else	if(but==butE)
			{
			wrk_state=wrkOFF;
			lc640_write_int(EE_WRK_STAT,wrk_state);
			}

	     }
		
	}
else if((ind==iSet_prl)||(ind==iK_prl)||(ind==iABl_kb_prl)||(ind==iSet_prl_new)||(ind==iSet_prl_change)
			||(ind==iRBl_kb_prl))
	{
	ret(50);
	if(but==butR)
		{
		sub_ind++;
		gran_ring_char(&sub_ind,0,2);
		}
	else if(but==butL)
		{
		sub_ind--;
		gran_ring_char(&sub_ind,0,2);
		}	
	else if(but==butU)
		{
		parol[sub_ind]++;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butD)
		{
		parol[sub_ind]--;
		gran_ring_char(&parol[sub_ind],0,9);
		}	
	else if(but==butE)
		{
		unsigned short tempU;
		tempU=parol[2]+(parol[1]*10U)+(parol[0]*100U);
		
		if(ind==iSet_prl)
			{
	     	if(tempU==PAROL_SET) 
				{
				tree_down(0,0);
				tree_up(iSet,0,0,0);
				ret(5000);
				}
			else 
				{
				ind=iDnd;
				ret(20);
				}
			}
		else	if(ind==iK_prl)
			{
	     	if(tempU==PAROL_KALIBR) 
				{
				tree_down(0,0);
				tree_up(iK,0,0,0);
				ret(5000);
				phase=0;
				}
			else 
				{
				ind=iDnd;
				ret(20);
				}
			}
		else	if(ind==iABl_kb_prl)
			{
	     	if(tempU==PAROL_KEYS_RESCURE||tempU==PAROL_KEYS) 
				{
				auto_bl_kb=stOFF;
				tree_down(0,0);
				ret(5000);
		 	 	lc640_write_int(EE_auto_bl_kb,auto_bl_kb);
				}
			else 
				{
				ind=iDnd;
				ret(20);
				}
			}
		else	if(ind==iSet_prl_new)
			{
	     	PAROL_KEYS=tempU;			
		 	lc640_write_int(EE_PAROL_KEYS,PAROL_KEYS);
			ind=iPMg;
			ret(20);
			}		 
		else	if(ind==iSet_prl_change)
			{
	     	if(tempU==PAROL_KEYS) 
				{
				parol_init();
				tree_down(0,0);
				tree_up(iSet_prl_new,0,0,0);
				ret(5000);
				}
			else 
				{
				ind=iDnd;
				ret(20);
				}			
			} 
		else	if(ind==iRBl_kb_prl)
			{
	     	if(tempU==PAROL_KEYS_RESCURE||tempU==PAROL_KEYS) 
				{
				tree_down(0,0);
				count_but_idle=0;
				}
			else 
				{
				ind=iDnd;
				ret(20);
				}			
			} 
		}	
	}

else if(ind==iSet)
	{
	ret(5000);
	if(but==butD)
		{
		sub_ind++;
		if(sub_ind==17)index_set=17;
		if(sub_ind==18)sub_ind++;
		if(MODE==_7_200_)gran_char(&sub_ind,0,21);
		else gran_char(&sub_ind,0,20);
		}
	else if(but==butU)
		{
		sub_ind--;
		if(sub_ind==18)sub_ind--;
		if(MODE==_7_200_)gran_char(&sub_ind,0,21);
		else gran_char(&sub_ind,0,20);
		}
	else if(but==butD_)
		{
		if(MODE==_7_200_)sub_ind=20;
		else sub_ind=19;
		}

	else if(sub_ind==0)
	     {
		if(but==butR)U_MAX++;
	     else if(but==butR_)U_MAX+=10;
	     else if(but==butL)U_MAX--;
	     else if(but==butL_)U_MAX-=10;
		else if(but==butLR)U_MAX=7000;
	     gran(&U_MAX,100,7000);
	     lc640_write_int(EE_U_MAX,U_MAX);
	     speed=1;
	     }

	else if(sub_ind==1)
	     {
		if(but==butR)I_MAX++;
	     else if(but==butR_)I_MAX+=10;
	     else if(but==butL)I_MAX--;
	     else if(but==butL_)I_MAX-=10;
		else if(but==butLR)
			{
			if(MODE==_7_1000_)I_MAX=1000;
			else I_MAX=200;
			}
	     if(MODE==_7_1000_)gran(&I_MAX,10,1000);
		else gran(&I_MAX,10,200);
	     lc640_write_int(EE_I_MAX,I_MAX);
	     speed=1;
	     }

	else if(sub_ind==2)
	     {
		if(but==butR)
			{
			MNEMO_ON++;
			if(MNEMO_ON<5)MNEMO_ON=5;
			}
		else if(but==butR_)MNEMO_ON=((((MNEMO_ON)/10)+1)*10);
	     else if(but==butL)
			{
			MNEMO_ON--;
			if((MNEMO_ON<5)&&(MNEMO_ON>0))MNEMO_ON=0;
			}
	     else if(but==butL_)MNEMO_ON=((((MNEMO_ON)/10)-1)*10);
		else if(but==butLR)MNEMO_ON=10;
	     gran_ring(&MNEMO_ON,0,60);
	     lc640_write_int(EE_MNEMO_ON,MNEMO_ON);
	     speed=1;
		}
	else if(sub_ind==3)
	     {
		if(but==butR)T_SIGN++;
	     else if(but==butR_)T_SIGN+=10;
	     else if(but==butL)T_SIGN--;
	     else if(but==butL_)T_SIGN-=10;
		else if(but==butLR)T_SIGN=50;
	     gran(&T_SIGN,30,110);
	     lc640_write_int(EE_T_SIGN,T_SIGN);
	     speed=1;
	     }
					
	else if(sub_ind==4)
	     {
		if(but==butR)T_MAX++;
	     else if(but==butR_)T_MAX+=10;
	     else if(but==butL)T_MAX--;
	     else if(but==butL_)T_MAX-=10;
		else if(but==butLR)T_MAX=70;
	     gran(&T_MAX,30,120);
	     lc640_write_int(EE_T_MAX,T_MAX);
	     speed=1;
	     }

	else if(sub_ind==5)
	     {
		if(but==butE)tree_up(iDef_set,0,0,0);
		ret(50);
	     }

	else if(sub_ind==6)
	     {
		if(but==butR)U_MIN1++;
	     else if(but==butR_)U_MIN1+=10;
	     else if(but==butL)U_MIN1--;
	     else if(but==butL_)U_MIN1-=10;

		if(U_MIN1<10)U_MIN1=9;
		if(U_MIN1>100)U_MIN1=101;
		//gran(&U_MIN1,10,100);
	     lc640_write_int(EE_U_MIN1,U_MIN1);
	     speed=1;
	     }

	else if(sub_ind==7)
	     {
		if(but==butR)T_MIN1++;
	     else if(but==butR_)T_MIN1+=10;
	     else if(but==butL)T_MIN1--;
	     else if(but==butL_)T_MIN1-=10;
		gran(&T_MIN1,1,10);
	     lc640_write_int(EE_T_MIN1,T_MIN1);
	     speed=1;
	     }

	else if(sub_ind==8)
	     {
		if(but==butR)U_MIN2++;
	     else if(but==butR_)U_MIN2+=10;
	     else if(but==butL)U_MIN2--;
	     else if(but==butL_)U_MIN2-=10;

		if(U_MIN2<10)U_MIN2=9;
		if(U_MIN2>300)U_MIN2=301;

		//gran(&U_MIN2,10,300);
	     lc640_write_int(EE_U_MIN2,U_MIN2);
	     speed=1;
	     }

	else if(sub_ind==9)
	     {
		if(but==butR)T_MIN2++;
	     else if(but==butR_)T_MIN2+=10;
	     else if(but==butL)T_MIN2--;
	     else if(but==butL_)T_MIN2-=10;
		gran(&T_MIN2,5,60);
	     lc640_write_int(EE_T_MIN2,T_MIN2);
	     speed=1;
	     }

	else if(sub_ind==10)
	     {
		if(but==butR)
			{
			if(I_VK<100)I_VK++;
			else I_VK=((I_VK/10)+1)*10;
			}
	     else if(but==butR_)
			{
			if(I_VK<100)I_VK+=2;
			else I_VK=((I_VK/10)+2)*10;
			}

	     else if(but==butL)
			{
			if(I_VK<100)I_VK--;
			else I_VK=((I_VK/10)-1)*10;
			}

	     else if(but==butL_)
			{
			if(I_VK<100)I_VK-=2;
			else I_VK=((I_VK/10)-2)*10;
			}

		gran_ring(&I_VK,9,10000);
	     lc640_write_int(EE_I_VK,I_VK);
	     speed=1;
	     }


	else if(sub_ind==11)
	     {
		if(but==butL)image_W=1;			 //����������� � %
	     else if(but==butR)				 //����������� � ��
		 {
			if(MODE==_7_1000_)
				{
				image_W=10;
				}
			else image_W=2; 
	     }
		 lc640_write_int(EE_image_W,image_W);
		 }
	else if(sub_ind==12)
	     {
		if(but==butR)P_POV++;
	     else if(but==butR_)P_POV+=10;
	     else if(but==butL)P_POV--;
	     else if(but==butL_)P_POV-=10;
	     gran(&P_POV,10,100);
	     lc640_write_int(EE_P_POV,P_POV);
	     speed=1;
	     }
	else if(sub_ind==13)
	     {
		if(but==butR)T_POV=((T_POV/5)+1)*5;
	     else if(but==butR_)T_POV=((T_POV/5)+2)*5;
	     else if(but==butL)T_POV=((T_POV/5)-1)*5;
	     else if(but==butL_)T_POV=((T_POV/5)-2)*5;
	     gran_ring(&T_POV,0,120);
	     lc640_write_int(EE_T_POV,T_POV);
	     speed=1;
	     }
		
	else if(sub_ind==14)
	     {
			if(but==butE)
			{
				tree_up(iABl_kb,0,0,0);
				ret(5000);
			}
		 }

	else if(sub_ind==15)
	     {
		if(but==butR)R_DOP++;
	     else if(but==butR_)R_DOP++;
	     else if(but==butL)R_DOP--;
	     else if(but==butL_)R_DOP--;
	     gran_ring(&R_DOP,9,200);
	     lc640_write_int(EE_R_DOP,R_DOP);
	     speed=1;
	     }

 	else if(sub_ind==16)
		{
		if((but==butR)||(but==butR_))
			{
			lc640_write_int(EE_MODBUS_TYPE,1);
			}
		else if((but==butL)||(but==butL_))
			{
			lc640_write_int(EE_MODBUS_TYPE,0);
			}
		}

	else if(sub_ind==17)
		{
		if((but==butR)||(but==butR_))
               {
               speed=1;
               if(BAUDRATE==96)BAUDRATE=192;
               else if(BAUDRATE==192)BAUDRATE=384;
               else if(BAUDRATE==384)BAUDRATE=576;
               else if(BAUDRATE==576)BAUDRATE=1152;
               else BAUDRATE=96;
               lc640_write_int(EE_BAUDRATE,BAUDRATE);
               uart1_init();
               }
          else if((but==butL)||(but==butL_))
               {
               speed=1;
               if(BAUDRATE==96)BAUDRATE=1152;
               else if(BAUDRATE==192)BAUDRATE=96;
               else if(BAUDRATE==384)BAUDRATE=192;
               else if(BAUDRATE==576)BAUDRATE=384;
               else BAUDRATE=576;
               lc640_write_int(EE_BAUDRATE,BAUDRATE);
               uart1_init();
               }
	     }

  	else if(sub_ind==19)
	     {
		if(but==butR)ADR++;
	     else if(but==butR_)ADR+=10;
	     else if(but==butL)ADR--;
	     else if(but==butL_)ADR-=10;
	     gran(&ADR,1,99);
	     lc640_write_int(EE_ADR,ADR);
	     speed=1;
	     }
	
	else if((MODE==_7_200_)&&(sub_ind==20))
		{
		if(KOEF)KOEF=0;
	     else KOEF=1;
	     
	     lc640_write_int(EE_KOEF,KOEF);
	     }

	else if(((MODE==_7_200_)&&(sub_ind==21))||(sub_ind==20))
	     {
		if(but==butE)tree_down(0,0);
	     }	
     }

else if(ind==iABl_kb)
	{
	ret(5000);
	if(but==butD)
		{
		sub_ind++;
		gran_char(&sub_ind,0,2);
		}
	else if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,2);
		}

	else if(sub_ind==0)
	     {
		if(but==butL&&auto_bl_kb==stON)
			{
			parol_init();
			tree_up(iABl_kb_prl,0,0,0);
			}	         
		else if(but==butR)
			{
			auto_bl_kb=stON;
		  	lc640_write_int(EE_auto_bl_kb,auto_bl_kb);
			}
		else if(but==butE&&auto_bl_kb==stON)
			{
			parol_init();
			tree_up(iABl_kb_prl,0,0,0);
			}
		else if(but==butE&&auto_bl_kb==stOFF)
			{
			auto_bl_kb=stON;
		  	lc640_write_int(EE_auto_bl_kb,auto_bl_kb);
			}
		 }

	else if(sub_ind==1)
	     {
			if(but==butE)
			{
				if(!PAROL_KEYS)
				{
				parol_init();
//				tree_down(0,0);
				tree_up(iSet_prl_new,0,0,0);
				ret(5000);
				}
				else
				{
				parol_init();
//				tree_down(0,0);
				tree_up(iSet_prl_change,0,0,0);
				ret(5000);
				}
			}
		 }

	else if(sub_ind==2)
	     {
		if(but==butE)tree_down(0,0);
		 }

	}

else if(ind==iDef_set)
	{
	if(but==butE)
		{
		if(MODE==_7_1000_) lc640_write_int(EE_I_MAX,1000);
		else lc640_write_int(EE_I_MAX,200);
		
		lc640_write_int(EE_U_MAX,7000);
	     lc640_write_int(EE_MNEMO_ON,10);
	     lc640_write_int(EE_T_SIGN,50);
	     lc640_write_int(EE_T_MAX,70);
	     lc640_write_int(EE_U_MIN1,50);
	     lc640_write_int(EE_T_MIN1,2);
	     lc640_write_int(EE_U_MIN2,200);
	     lc640_write_int(EE_T_MIN2,10);
	     lc640_write_int(EE_PAROL_KEYS,000);
         lc640_write_int(EE_auto_bl_kb,0);
		 lc640_write_int(EE_image_W,1);


		tree_down(0,0);
		show_mess("     ���������      ",
	          	"    ����������      ",
	          	"    �� ���������    ",
	          	"    �����������     ",3000);

	     }	
	}

else if(ind==iK)
	{
	signed short temp_SS;
	ret(5000);
	if(but==butU)
		{
		sub_ind--;
		gran_char(&sub_ind,0,5);
		phase=0;
		}
	else if(but==butD)
		{ 
		sub_ind++;
		gran_char(&sub_ind,0,5);
		phase=0;
		}
	else if(but==butD_)
		{
		sub_ind=5;
		}

	else if(sub_ind==0)
		{
		if(but==butE)
			{
			if(MODE==_7_1000_)
				{
				MODE=_7_200_;
				lc640_write_int(EE_MODE,_7_200_);
				}
			else
				{
				MODE=_7_1000_;
				lc640_write_int(EE_MODE,_7_1000_);
				}
		if(image_W!=1)				 //����������� � ��
		 {
			if(MODE==_7_1000_)
				{
				image_W=10;
				}
			else image_W=2; 
	     }
		 lc640_write_int(EE_image_W,image_W);
		 
			}
	     speed=0;
		}

	else if(sub_ind==1)
		{
		if(phase==0)
		     {
		     if(but==butE)
		          {
		     	phase=1;
		          }
		     else phase=1;     
		     }
		else if(phase==1)
		     {
			temp_SS=lc640_read_int(EE_K_U);
	     	if(but==butR) temp_SS++;
			else if(but==butR_)	temp_SS+=2;
			else if(but==butL) temp_SS--;
			else if(but==butL_)	temp_SS-=2;
			gran(&temp_SS,1700,2100);
			lc640_write_int(EE_K_U,temp_SS);					
			speed=1;
			}	
		}
	else if(sub_ind==2)
		{

		if(phase==0)
		     {
			phase=1;     
		     }
		else if(phase==1)
		     {
			temp_SS=lc640_read_int(EE_K_I_G1);
		     if(but==butR) temp_SS++;
	     	else if(but==butR_)	temp_SS+=2;
	     	else if(but==butL) temp_SS--;
	     	else if(but==butL_)	temp_SS-=2;
			gran(&temp_SS,100,1250);
	     	lc640_write_int(EE_K_I_G1,temp_SS);
			speed=1;			
	     	}

		}
	else if(sub_ind==3)
		{
		if(phase==0)
		     {
		     if(but==butE)
		          {
				temp_SS=tlv_buff_buff_[0];
		          lc640_write_int(EE_K_I_T0,temp_SS);
		     	phase=1;
		          }
		     else phase=1;     
		     }
		else if(phase==1)
		     {
			temp_SS=lc640_read_int(EE_K_I_T1);
		     if(but==butR) temp_SS++;
	     	else if(but==butR_)	temp_SS+=2;
	     	else if(but==butL) temp_SS--;
	     	else if(but==butL_)	temp_SS-=2;
			gran(&temp_SS,150,1000);
	     	lc640_write_int(EE_K_I_T1,temp_SS);
			speed=1;			
	     	}		}
	else if(sub_ind==4)
		{

		temp_SS=lc640_read_int(EE_K_T);
		if(but==butR) temp_SS++;
		else if(but==butR_)	temp_SS+=3;
		else if(but==butL) temp_SS--;
		else if(but==butL_)	temp_SS-=3;
		gran(&temp_SS,1900,2100);
		lc640_write_int(EE_K_T,temp_SS);				
		speed=1;
		}
	else if(sub_ind==5)
		{
		tree_down(0,0);
		}
	}

else if(ind==iSTOP_umin1)
	{
	tree_down(0,0);
	}
else if(ind==iSTOP_umin2)
	{
	tree_down(0,0);
	}
else if(ind==iFWabout)
	{
	ret(1000);
	if(but==butE)
	     {
	     tree_down(0,0);
	     ret(0);
	     }
	}

if(MNEMO_ON)mnemo_cnt=MNEMO_ON;

but_an_end:

n_but=0;

}	   

//***********************************************
//***********************************************
//***********************************************
//***********************************************
//***********************************************
__irq void timer1_interrupt() 
{
T1IR = 0xff;

if(tx_wd_cnt)
	{
	tx_wd_cnt--;

	}

if(++t0_cnt8>=33)
     {
     t0_cnt8=0;
     b33Hz=1;
	}

if(++t0cnt>=10)
     {
     t0cnt=0;
     b100Hz=1;

     if(++t0cnt0>=10)
	     {
	     t0cnt0=0;
	     b10Hz=1;
	     //beep_drv();
	     }

     if(t0cnt0==5)
	     {
	     ///beep_drv();
	     }

     if(++t0cnt1>=20)
	     {
	     t0cnt1=0;
	     b5Hz=1;
	     
 		if(bFL5)bFL5=0;
  		else bFL5=1;	     
	     }

     if(++t0cnt2>=50)
	     {
	     t0cnt2=0;
	     b2Hz=1;
	     
	     if(bFL2)bFL2=0;
	     else bFL2=1;
	     }         

     if(++t0cnt3>=100)
	     {
	     t0cnt3=0;
	     b1Hz=1;
	     }
     }

if (_485_last_cnt)
	{
	_485_last_cnt--;
	if(_485_last_cnt==0)
		{
		//char i;
		//for(i=0;i<rx_counter1;i++)
			//{
			//UIB1[i]=rx_buffer1[i] ;

			//UIB1[0]=0x34;
			//}
		bRXIN1=1;
			//plazma++;
		//rx_counter1=0;
		//rx_wr_index1=0;

		}
	}

if(modbus_timeout_cnt<6)
	{
	modbus_timeout_cnt++;
	if(modbus_timeout_cnt>=6)
		{
		bMODBUS_TIMEOUT=1;
		}
	}
else if (modbus_timeout_cnt>6)
	{
	modbus_timeout_cnt=0;
	bMODBUS_TIMEOUT=0;
	}


VICVectAddr = 0;
}

//***********************************************
__irq void timer0_interrupt() 
{	
T0IR = 0xff;

SET_REG(T0EMR,1,1,1);
SET_REG(T0EMR,1,3,1);

VICVectAddr = 0;
}


//===============================================
//===============================================
//===============================================
//===============================================
int main (void)
{

SET_REG(PINSEL1,0,(23-16)*2,1); //���� PV � 485
IO0DIR|=(1<<23);
IO0CLR|=(1<<23);

IO1DIR|=(1<<26);
IO1SET|=(1<<26);   //������������� ��������

t0_init();
t1_init();
 
IO1DIR|=(1UL<<31);	
IO1SET=(1UL<<31);	//������������� TLV


IO1DIR|=(1UL<<21);
IO1SET=(1UL<<21);  	//������������� 25LC640

IO1DIR|=(1UL<<21);
IO1SET=(1UL<<21);  	//������� �� ���������

lcd_init();
lcd_init();
lcd_init();
lcd_init();
lcd_init();
lcd_on();
lcd_clear();


ad7705_reset();
delay_ms(20);

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44);

ad7705_reset();
delay_ms(20);

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44);

ad7705_reset();
delay_ms(20);  

ad7705_write(0x20);
ad7705_write(BIN8(1101)); 
ad7705_write(0x10);
ad7705_write(0x44); 

SET_REG(PINSEL1,0,(23-16)*2,1);	//���� PV � 485
IO0DIR|=(1<<23);
IO0SET|=(1<<23);

memo_read();
if (image_W!=1&&image_W!=2&&image_W!=10) 
{
image_W=1;
lc640_write_int(EE_image_W,image_W);
image_W=lc640_read_int(EE_image_W);
}

__enable_irq();

BAUDRATE=lc640_read_int(EE_BAUDRATE);
uart1_init();

mnemo_cnt=100;

watchdog_init(60000000,2000);


lc640_write_int(EE_REST_CNT,lc640_read_int(EE_REST_CNT)+1);
for(;;)
	{


	watchdog_reset();
   	if(!tx_wd_cnt) 
		{
		SET_REG(PINSEL1,0,(22-16)*2,1); //���� PV � 485
		IO0DIR|=(1<<22);
		IO0CLR|=(1<<22);
		}	
	if(bRXIN1)
		{
		bRXIN1=0;
		//uart_in_an1();
		if(MODBUS_TYPE==0)uart_in_an1();
		//if(MODBUS_TYPE==1)modbus_in();
		}	
	if(bMODBUS_TIMEOUT)
		{
		bMODBUS_TIMEOUT=0;
		//uart_in_an1();
		//if(MODBUS_TYPE==0)uart_in_an1();
		if(MODBUS_TYPE==1)modbus_in();
		}			
 	if(b100Hz)
		{
		b100Hz=0; 
		adc_drv();
		but_drv();
		but_an();
		}
		
	if(b33Hz)
		{
		b33Hz=0;
		}
			 
	if(b10Hz)
		{
		b10Hz=0; 
		
		matemat();
	     tlv_drv();
	 	ind_hndl();
		bitmap_hndl();
		lcd_out(lcd_bitmap);
         	ret_hndl();
		pwm_drv();

		
///		IO0DIR|=(1<<8);
///		if((++plazma_ind)&0x01) IO0SET|=(1<<8);
///		else IO0CLR|=(1<<8)|(1<<9);
		//IO1DIR|=(1<<24);
		//IO1SET|=(1<<24);

		u_min_drv();
/*		plazma++;
		IO0DIR|=(1<<19);
		if((plazma)&0x01) IO0SET|=(1<<19);
		else IO0CLR|=(1<<19);		
		IO0DIR|=(1<<20);
		if((plazma)&0x01) IO0SET|=(1<<20);
		else IO0CLR|=(1<<20);*/
		//
		//IO1SET|=(1<<24);
		kv_hndl();
		av_out_hndl();
		out_drv();

 		}	
	if(b5Hz )
		{
		b5Hz=0;
		
		ad7705_drv();

		ret_ind_hndl();
		
	    time_drv();
		memo_read();
		
		wrk_hndl();
		vent_drv();
		
  		}
	if(b2Hz)
		{
		b2Hz=0;
          
				}
										
	if(b1Hz)
		{
		b1Hz=0;
		
		plazma_pal++;

		lc640_write_int(EE_DEBUG,lc640_read_int(EE_DEBUG)+1);
		
		ret_ind_sec_hndl();  
		if(MNEMO_ON)
			{
			if(mnemo_cnt) mnemo_cnt--;
			}
		
		temper_drv();

		p_pov_drv();
		i_pov_drv();

		
		if(++cnt_lcd_init>180)
			{
			SET_REG(PINSEL1,0,(28-16)*2,2);
			IO0DIR|=(1<<28);
			IO0CLR|=(1<<28);
			cnt_7705_invalid=4;
			lcd_init();
			lcd_on();
			lcd_clear();
			cnt_lcd_init=0;
			IO0SET|=(1<<28);


			} 

		if(lc640_read_int(EE_REST)==stON)
			{
			if((lc640_read_int(EE_WRK_STAT)==wrkON)&&(wrk_state!=wrkON))
				{
				wrk_state=wrkON;
				pwmI_start=POWER*9;
				wrk_cnt_cnt=(signed long)T_WRK_MAX*300L;
				ist=istT;
				t_u_min1=-10;
				t_u_min2=-10;

				lcd_init();
				lcd_on();
				lcd_clear();

				}
			}
		else 
			{
			lc640_write_int(EE_WRK_STAT,wrkOFF);
			}
		 

		if(MODE==_7_1000_) current_integral_7_1000();
		else if(MODE==_7_200_) 
			{
			current_integral_7_200();
			power_integral_7_200();
			}	

		}
  	}
}


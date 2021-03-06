#include "control.h"
#include <LPC21XX.H> 
#include "main.h"
#include "gran.h"
#include "25lc640.h"
#include "eeprom_map.h"
#include "pcf8563.h"
#include "tlv2542.h"
#include "beep.h"
#include "ad7705.h"
#include "common_func.h"

//-----------------------------------------------
//���
unsigned main_apv_cnt,hour_apv_cnt[2],reset_apv_cnt[2];
unsigned short apv_cnt_sec[2],apv_cnt[2];
char apv_cnt_1;
short adc_buff_out_[3];

//-----------------------------------------------
//���������� �����������
char cnt_src[2];
signed short cntrl_stat=600,old_cntrl_stat,cntrl_stat1,cntrl_stat2;
signed short u_necc,u_necc_;
char cnt_blok;
enum_ibs Ibs=iiM,Ibso=iiM;
char bmm_cnt,bmp_cnt;
char bS=1;
signed char num_necc;
signed char cnt_num_necc;
char tzas_cnt;
char bBLOK;




//-----------------------------------------------
//����������
char bit_minus;

//-----------------------------------------------
//����
char cnt_beep;

//const char dumm[1000];

//-----------------------------------------------
//�����������

signed short t_sign_cnt,t_max_cnt;
enum_temper_state temper_state,temper_state_old;
char crazy_beep;			


//-----------------------------------------------
//������ �� �������� ����������
signed short t_u_min1,t_u_min2;
char bSTOP_umin1,bSTOP_umin2;


//-----------------------------------------------
//������ �� ��������� ���������(���������� ���� � ��������)
signed long current_sigma=0;
char current_sigma_stat=0;
signed long power_sigma=0;
char power_sigma_stat=0;
signed long power_sigma_cnt_30min=0;

signed long plazma_long;


//-----------------------------------------------
//���������� ��������
signed short p_pov_cnt;

//-----------------------------------------------
void cntrl_drv(void)
{
signed short temp_SS;
temp_SS=cntrl_stat1;
gran(&temp_SS,30,1010);
T0MR1=temp_SS;


temp_SS=cntrl_stat2;
gran(&temp_SS,30,1010);
T0MR3=temp_SS;


}


 //-----------------------------------------------
void av_out_hndl(void)
{
if(wrk_state==wrkOFF)
	{
	av_out_cnt=0;
	av_out_stat=avOFF;
	}
else 
	{
	if((Ires<100000UL)&&(U<200))
		{
		av_out_cnt++;
		if(av_out_cnt>95)av_out_stat=avON;
		}
	else 
		{
		av_out_cnt--;
		if(av_out_cnt<5)av_out_stat=avOFF;
		}

	}
gran(&av_out_cnt,0,100);
av_out_ind_cnt++;
gran_ring(&av_out_ind_cnt,0,100);
}

 //-----------------------------------------------
void kv_hndl(void)
{
if((wrk_state==wrkOFF)||(I_VK<10))
	{
	kv_cnt=0;
	av_kv_stat=avOFF;
	}
else 
	{
	if(Ires>(I_VK*100))
		{
		kv_cnt++;
		if(kv_cnt>18)av_kv_stat=avON;
		}
	else 
		{
		kv_cnt--;
		if(kv_cnt<2)av_kv_stat=avOFF;
		}

	}
//av_kv_stat=avON;
gran(&kv_cnt,0,20);
kv_ind_cnt++;
gran_ring(&kv_ind_cnt,0,100);
}

//-----------------------------------------------
void out_drv(void)
{
SET_REG(IO1DIR,1,25,1);
if(	(av_kv_stat==avON) ||
	(temper_state!=tsNORM)||
	((Ires<100000UL)&&(U<200)))SET_REG(IO1CLR,1,25,1);
else SET_REG(IO1SET,1,25,1);
}

//-----------------------------------------------
void pwm_drv(void)
{
/*if((MODE==_7_1000_)&&current_sigma_stat)gran(&pwmI,10,450);
else */gran(&pwmI,10,970);
gran(&pwmU,10,970);
//pwmU=970;
//pwmI=970;
T0MR3 = pwmI;
T0MR1 = pwmU; 

IO1DIR|=(1<<26);
if(bBLOK) IO1SET|=(1<<26);
else IO1CLR|=(1<<26);
//IO1CLR|=(1<<26);
}


//-----------------------------------------------
void wrk_hndl(void)
{
static char wrk_hndl_cnt,wh_1Hz;
if(wrk_hndl_cnt<4)wrk_hndl_cnt++;
else
	{
	wrk_hndl_cnt=0;
	wh_1Hz=1;
	wrk_cnt++;
	}


if(ind==iK)
	{
	if(phase==0)
		{
		pwmU=30;
		pwmI=30;
		bBLOK=1;
		}
	else if(phase==1)
		{
		if(pwmU<970)pwmU+=30;
		else pwmU=970;
		if(pwmI<970)pwmI+=30;
		else pwmI=970;
		bBLOK=0;
		}
	}
else if((wrk_state==wrkOFF) &&(ind!=iDeb))
	{
	wrk_cnt=0;
	pwmU=30;
	pwmI=30;
	bBLOK=1;
	wrk_phase=0;
	Isim=0;
	}

else if((temper_state==tsAV) &&(ind!=iDeb))
	{
	wrk_cnt=0;
	pwmU=30;
	pwmI=30;
	bBLOK=1;
	wrk_phase=0;
	Isim=0;
	}
else if((wrk_state==wrkON) /*&&(ind!=iDeb)*/)
	{
	pwmU=(signed short)((1000L*(long)U_MAX)/U_MAX_PWM);

	bBLOK=0;

	if(wrk_phase==0)
		{
		if((pwmI<pwmI_start)&&(U<1000))
			{
			pwmI+=pwmI_start/50;
			if((pwmI_start/50)<1)pwmI++;
			}
		else wrk_phase=1;
		}
	else 
		{
		if(U<1000)
			{
			if(wh_1Hz==1)
				{
				signed long temp_SL;
				temp_SL=(signed long)I_MAX;
				if(p_pov_cnt) temp_SL*=(signed long)P_POV;
				else temp_SL*=(signed long)POWER;
				if((KOEF==1)&&(power_sigma_stat==0)&&(current_sigma_stat==0)&&(MODE==_7_200_))temp_SL/=40L;
				else if((MODE==_7_1000_)&&(current_sigma_stat==1))temp_SL/=166L;
				else temp_SL/=100L;
				//temp_SL=100;
				plazma_long=temp_SL;


				if((Ig/1000L)<(plazma_long))
					{
					if(((plazma_long)-(Ig/1000L))>100)pwmI+=10;
					else pwmI++;
					}
				else if((Ig/1000L)>(plazma_long))
					{
					if(((Ig/1000L)-(plazma_long))>100)pwmI-=10;
					else pwmI--;
					}
				}
			}
 		else
			{
			if(wh_1Hz==1)
				{
				if(P>Pmax)
					{
					if((P-Pmax)>100)pwmI-=10;
					else pwmI--;
					}
				else if(Pmax>P)
					{
					if((Pmax-P)>100)pwmI+=10;
					else pwmI++;
					}
			  	gran(&pwmI,0,pwmI_max);
				}
			}
		}
	if(wrk_cnt_cnt)
		{
		wrk_cnt_cnt--;
		if(!wrk_cnt_cnt)
			{
			wrk_state=wrkOFF;
			lc640_write_int(EE_WRK_STAT,wrk_state);
			}
		}
 
		{
	/*	signed long temp_SL;
		temp_SL=(signed long)I_MAX;
		temp_SL*=1000L;
		temp_SL/=I_MAX_PWM;
		gran(&pwmI,30,(signed short)temp_SL);*/
		}
	}
wh_1Hz=0;


if((t>T_SIGN)/*||(i_pov_cnt)*/)gran(&pwmI,30,500);
}



//-----------------------------------------------
void matemat(void)
{
//signed short temp_SS;
signed long temp_SL,temp_SL1;
char temp;

temp_SL=(signed long)adc_buff_[5];
temp_SL*=K_T;
temp_SL/=5000L;
temp_SL-=273L;
t=(signed short)temp_SL;

temp_SL=(signed long)tlv_buff_[1];
temp_SL*=K_U;
temp_SL/=966L;
U=(signed short)temp_SL;


temp_SL=(signed long)adc_buff_U_;
temp_SL*=K_U;
temp_SL/=250L;
Usim=(signed short)temp_SL;

Idop=0;
if((R_DOP<=200)&&(R_DOP>=10))
     {
     temp_SL=(signed long)U;
     temp_SL/=R_DOP/10;
     Idop=(signed short)temp_SL;
     }


temp_SL=(signed long)ad7705_buff_[1];
//temp_SL-=K_I_G[0];
//if(temp_SL<0)temp_SL=0;
temp_SL*=(signed long)K_I_G[1];
if(MODE==_7_1000_)temp_SL/=16L;
else temp_SL/=80L;			
Ig=temp_SL;
//Ig=(signed short)ad7705_buff_[0];
//Ig=100000L;
//Ig=3000000L;

temp_SL=(signed long)tlv_buff_[1];
temp_SL-=K_I_G[0];
if(temp_SL<0)temp_SL=0;
temp_SL*=K_I_G[1];
temp_SL/=1000L;
Ig_u=(signed short)temp_SL;

temp_SL=(signed long)ad7705_buff_[0];
//temp_SL-=K_I_T[0];
//if(temp_SL<0)temp_SL=0;
temp_SL*=K_I_T[1];
temp_SL/=1550L;
It=(signed short)temp_SL;

It=It-Idop;
if(It<0) It=0;

temp_SL=(signed long)U;
temp_SL*=(signed long)Ig;
temp_SL/=1000000L;
P=(signed short)temp_SL;

if(MODE==_7_1000_)temp_SL=P_MAX;
else temp_SL=P_MAX/5;
if(p_pov_cnt)temp_SL*=(signed long)P_POV;
else temp_SL*=(signed long)POWER;
if((KOEF==1)&&(power_sigma_stat==0)&&(current_sigma_stat==0)&&(MODE==_7_200_))temp_SL/=40L;
else if((MODE==_7_1000_)&&(current_sigma_stat==1))temp_SL/=166L;
else temp_SL/=100L;
Pmax=(signed short)temp_SL;

temp_SL=(signed long)Pmax;
temp_SL*=1000L;
temp_SL/=(signed long)U;
temp_SL*=1000L;
if(MODE!=_7_1000_)temp_SL*=5L;
temp_SL/=(signed long)I_MAX_PWM;
temp_SL+=50L;

pwmI_max=(signed short)temp_SL;

//if((tlv_buff_[0]<3700)&&(ist!=istT)) ist=istT;
//if((tlv_buff_[0]>3900)&&(ist!=istG)) ist=istG;

/*if((Ig<1)&&(ist!=istT)) ist=istT;
if((It>1500)&&(ist!=istG)) ist=istG;*/

temp_SL=(signed long)It;

temp_SL1=((signed long)Ig)*1000L;

//if(temp_SL>temp_SL1) 
//if(temp_SL<temp_SL1) 

if(temp_SL<1000) ist=istT;
else if(temp_SL<temp_SL1) ist=istG;
else ist=istT;

/*for(temp_SL=0;temp_SL<1000;temp_SL++)
	{
	temp=dumm[temp_SL];
	}
	*/
	
Ires=Ig*10L;
if(MODE==_7_1000_)
	{
	if((It>Ires)||(Ig<300))Ires=It;
	} 
else	 
	{
	if((It>Ires)||(Ig<100))Ires=It;
	} 
//Ires=9900L;
//Ires=10000;	
}


//-----------------------------------------------
void u_min_drv(void)
{

if(wrk_state==wrkON)
	{

	if((U<U_MIN1)&&(U_MIN1>=10)&&(U_MIN1<=100))
		{
		if(t_u_min1<(T_MIN1*10))
			{
			t_u_min1++;
			if(t_u_min1>=(T_MIN1*10))
				{
				bSTOP_umin1=1;
				}
			}
		}
	else 
		{
		if(t_u_min1<0)	t_u_min1++;
		else t_u_min1=0;
		}

	if((U<U_MIN2)&&(U_MIN2>=10)&&(U_MIN2<=300))
		{
		if(t_u_min2<(T_MIN2*10))
			{
			t_u_min2++;
			if(t_u_min2>=(T_MIN2*10))
				{
				bSTOP_umin2=1;
				}
			}
		}
	else 
		{
		if(t_u_min2<0)	t_u_min2++;
		else t_u_min2=0;
		}


	if(bSTOP_umin1)
		{
		wrk_state=wrkOFF;
		lc640_write_int(EE_WRK_STAT,wrk_state);
		tree_up(iSTOP_umin1,0,0,0);
		bSTOP_umin1=0;
		}

	if(bSTOP_umin2)
		{
		wrk_state=wrkOFF;
		lc640_write_int(EE_WRK_STAT,wrk_state);
		tree_up(iSTOP_umin2,0,0,0);
		bSTOP_umin2=0;
		}


	}


}


//-----------------------------------------------
void temper_drv(void)
{
if((t>T_SIGN)&&(wrk_state==wrkON))
	{
	t_sign_cnt++;
	}
else if(t<(T_SIGN-5))
	{
	t_sign_cnt--;
	}

gran(&t_sign_cnt,0,10); 

if((t>T_MAX)&&(wrk_state==wrkON))
	{
	t_max_cnt++;
	}
else if(t<(T_MAX-5))
	{
	t_max_cnt--;
	}

gran(&t_max_cnt,0,10);


if(t_max_cnt>=9)temper_state=tsAV;
else if(t_max_cnt<=2)
	{
	if(t_sign_cnt>=9)temper_state=tsSIGN;
	else if(t_sign_cnt<=2)temper_state=tsNORM;
	}

if((temper_state!=temper_state_old)&&(temper_state==tsAV))
	{
	crazy_beep=1;
	}

if(crazy_beep)beep_init(0xffffffff,'R');
else if(temper_state==tsSIGN)beep_init(0x00000001,'R');
else beep_init(0x00000000,'R');

temper_state_old=temper_state;
}

//-----------------------------------------------
void time_drv(void)
{   
static char _5hz_cnt_;

hour_cnt_5hz++;
if(hour_cnt_5hz>=18000) hour_cnt_5hz=0;

_5hz_cnt_++;
if(_5hz_cnt_>=5)
	{
	_5hz_cnt_=0;
	hour_cnt_1hz++;
	if(hour_cnt_1hz>=3600)hour_cnt_1hz=0;
//	if(main_cnt<200) main_cnt++;
	
	cnt_ind++;
	if(cnt_ind>9)cnt_ind=0;
	}  


}



 

//-----------------------------------------------
void vent_drv(void)
{
static char vent_stat;


if((t>(T_SIGN-5)) ||  ((P>50)) ||  ((Ig>10000))) vent_stat=0;
else if((t<(T_SIGN-10)) && (P<45) && (Ig<9000)) vent_stat=1;

SET_REG(IO1DIR,1,29,1);
if(vent_stat==0)	SET_REG(IO1CLR,1,29,1);
else 			SET_REG(IO1SET,1,29,1);

}


//-----------------------------------------------
void adc_drv(void)
{
short temp_S;
char i;
if(GET_REG(ADDR,31,1))//ADDR_bit.DONE)
	{
	adc_buff[adc_ch][adc_cnt]=GET_REG(ADDR,6,10);//ADDR_bit.VVDDA;
	if((adc_cnt&0x03)==0)
		{
		temp_S=0;
		for(i=0;i<16;i++)
			{
			temp_S+=adc_buff[adc_ch][i];
			adc_buff_[adc_ch]=temp_S>>4;
			}
		}
	
	
	if(++adc_ch>=8) 
		{
		adc_ch=0;
		if(++adc_cnt>=16) 
			{
			adc_cnt=0;
			}
		}
	
	
	}

SET_REG(PINSEL1,1,(30-16)*2,2);

SET_REG(PINSEL1,0,(21-16)*2,2);
SET_REG(PINSEL1,0,(24-16)*2,2);
SET_REG(PINSEL1,0,(25-16)*2,2);


SET_REG(IO0DIR,1,21,1);
SET_REG(IO0DIR,1,24,1);
SET_REG(IO0DIR,1,25,1);



if(adc_ch&0x01)IO0SET|=((long)1UL<<21);
else IO0CLR|=((long)1UL<<21);
if(adc_ch&0x02)IO0SET|=((long)1UL<<24);
else IO0CLR|=((long)1UL<<24);
if(adc_ch&0x04)IO0SET|=((long)1UL<<25);
else IO0CLR|=((long)1UL<<25);

//ADCR_bit.PDN=1;
SET_REG(ADCR,1,21,1);
//ADCR_bit.CLKDIV=14;
SET_REG(ADCR,14,8,8);
//ADCR_bit.BURST=0;
SET_REG(ADCR,0,16,1);
//ADCR_bit.CLKS=0;
SET_REG(ADCR,0,17,3);
//ADCR_bit.TEST=0;
SET_REG(ADCR,0,22,2);
SET_REG(ADCR,8,0,8);//ADCR_bit.SEL=8;
//ADCR_bit.START=1;
SET_REG(ADCR,1,24,3);

}

//-----------------------------------------------
void current_integral_7_1000(void)
{
/*if(wrk_state==wrkON)
	{
	}
else 
	{
	current_sigma=0;
	}*/

if(Ires>6000000)
	{
	current_sigma+=((Ires/10000)-600);
	}
else if(Ires<6000000)
	{
	if(current_sigma>0)current_sigma-=(600-(Ires/10000));
	}

if(current_sigma<0) current_sigma=0;

if(current_sigma>=72000L)current_sigma_stat=1;
else if(current_sigma<=0L) current_sigma_stat=0; 
}


//-----------------------------------------------
void current_integral_7_200(void)
{


if(Ires>2000000)
	{
	current_sigma+=((Ires/10000)-200);
	}
else if(Ires<2000000)
	{
	if(current_sigma>0)current_sigma-=(200-(Ires/10000));
	}

if(current_sigma<0) current_sigma=0;

if(current_sigma>=54000L)current_sigma_stat=1;
else if(current_sigma<=0L) current_sigma_stat=0; 
}

//-----------------------------------------------
void power_integral_7_200(void)
{


if(P>200)
	{
	power_sigma+=((P)-200);
	}
else if(P<200)
	{
	if(power_sigma>0)power_sigma-=(200-(P));
	}

if(power_sigma<0) power_sigma=0;

if(power_sigma>=54000L)power_sigma_stat=1;
else if(power_sigma<=0L) power_sigma_stat=0; 


if((power_sigma>=54000L)&&(P<250))
	{
	power_sigma_cnt_30min++;
	if(power_sigma_cnt_30min>=1800)
		{
		power_sigma=0;
		power_sigma_cnt_30min=0;
		power_sigma_stat=0;
		}
	}
else power_sigma_cnt_30min=0;

}

//-----------------------------------------------
void p_pov_drv(void)
{

if(p_pov_cnt)p_pov_cnt--;
if(wrk_state==wrkOFF)p_pov_cnt=0;

}

//-----------------------------------------------
void i_pov_drv(void)
{

if(i_pov_cnt)
     {
     i_pov_cnt--;
     i_pov_cnt1=0;
     }

if(wrk_state==wrkOFF)
     {
     i_pov_cnt=0;
     i_pov_cnt1=0;
     }

if(wrk_state!=wrkOFF)
     {
     if(Ires>7000000L)
          {
          i_pov_cnt1++;
          if(i_pov_cnt1>180)
               {
               i_pov_cnt=600;
               }
          }
     else i_pov_cnt1=0;
     }

}


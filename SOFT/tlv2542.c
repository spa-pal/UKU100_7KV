#include "tlv2542.h"
#include <LPC21XX.H> 
#include "main.h"


signed short tlv[2];
signed long tlv_buff[2][64],tlv_buff_[2],tlv_buff_buff[2][16],tlv_buff_buff_[2];
char tlv_cnt,tlv_cnt_cnt;

//+-----------------------------------------------
void tlv2542_read(signed short *adr1, signed short *adr2)
{
#define CS     31
#define SCLK   17
#define DAT    18

#define IMP	     IO0SET|=(1UL<<SCLK);/*IO0SET|=(1UL<<SCLK);*/IO0CLR|=(1UL<<SCLK);/*IO0CLR|=(1UL<<SCLK);*/
#define IMP_on	     IO0SET|=(1UL<<SCLK);IO0SET|=(1UL<<SCLK);
#define IMP_off     IO0CLR|=(1UL<<SCLK);IO0CLR|=(1UL<<SCLK);
short temp0,temp1;
char i; 

IO1DIR|=(1UL<<CS);
IO0DIR|=(1UL<<SCLK);
IO0DIR&=(~(1UL<<DAT));

IO1SET|=(1UL<<CS);
IO0CLR|=(1UL<<DAT)|(1UL<<SCLK);

IO0CLR|=(1UL<<DAT)|(1UL<<SCLK);
IO0CLR|=(1UL<<DAT)|(1UL<<SCLK);

IMP
IO1CLR|=(1UL<<CS);
for(i=0;i<6;i++)
	{
	IMP
	}  
IO1SET|=(1UL<<CS);
IMP	
IMP
IO1CLR|=(1UL<<CS); 


for(i=0;i<26;i++)
	{
	IMP
	}  
IO1SET|=(1UL<<CS);
IMP	
IMP
IO1CLR|=(1UL<<CS);

temp0=0;

for(i=0;i<12;i++)
	{
	IMP_on
	temp0<<=1;
	if(IO0PIN&(1UL<<DAT))
		{
		temp0|=0x0001;
		}             
	else temp0&=0xfffe;
	
	IMP_off	
	}  
for(i=0;i<14;i++)
	{
	IMP
	}  
IO1SET|=(1UL<<CS);
IMP	
IMP
IO1CLR|=(1UL<<CS);

temp1=0;
for(i=0;i<12;i++)
	{
	IMP_on
	temp1<<=1;
	if(IO0PIN&(1UL<<DAT))
		{
		temp1|=0x0001;
		}             
	else temp1&=0xfffe;
	
	IMP_off;	
	}  
for(i=0;i<14;i++)
	{
	IMP
	}  
IO1SET|=(1UL<<CS);
IMP	
IMP

*adr1=temp0;
*adr2=temp1;

}

//-----------------------------------------------
void tlv_drv(void)
{
char i;
signed long temp_SL;
tlv2542_read(&tlv[0],&tlv[1]);

tlv_buff[0][tlv_cnt]=(signed long)tlv[0];
tlv_buff[1][tlv_cnt]=(signed long)tlv[1];

if((tlv_cnt&0x03)==0)
   	{
	temp_SL=0;
	for(i=0;i<64;i++)
     	{
     	temp_SL+=tlv_buff[0][i];
     	}
	tlv_buff_[0]=(short)(temp_SL>>6);    

	temp_SL=0;
	for(i=0;i<64;i++)
     	{
     	temp_SL+=tlv_buff[1][i];
     	}
	tlv_buff_[1]=(short)(temp_SL>>6);
	} 

if(++tlv_cnt>=64)
	{
	tlv_cnt=0;

	tlv_buff_buff[0][tlv_cnt_cnt]=tlv_buff_[0];
	tlv_buff_buff[1][tlv_cnt_cnt]=tlv_buff_[1];

	temp_SL=0;
	for(i=0;i<4;i++)
     	{
     	temp_SL+=tlv_buff_buff[0][i];
     	}
	tlv_buff_buff_[0]=(short)(temp_SL>>2);    

	temp_SL=0;
	for(i=0;i<4;i++)
     	{
     	temp_SL+=tlv_buff_buff[1][i];
     	}
	tlv_buff_buff_[1]=(short)(temp_SL>>2); 

	if(++tlv_cnt_cnt>=4)
		{
		tlv_cnt_cnt=0;
		
		}

   	}
}

#include "beep.h"
#include "main.h"
#include <LPC21XX.H>

unsigned long beep_stat_temp,beep_stat;
char beep_stat_cnt;
char beep_cnt;

//-----------------------------------------------
void beep_drv(void)
{
IO1DIR|=(1UL<<27);
if((beep_stat_temp&0x00000001UL))
    	{
    	IO1SET|=(1UL<<27);
    	beep_cnt=6;
    	}
else IO1CLR|=(1UL<<27);

beep_stat_temp>>=1;
if(--beep_stat_cnt==0)
	{
	beep_stat_cnt=32;
	beep_stat_temp=beep_stat;
	}
}

//-----------------------------------------------
void beep_init(long zvuk,char fl) 
{
if(fl=='O')
	{
	beep_stat_temp=zvuk;
	beep_stat=0x0L;
	beep_stat_cnt=32;
	} 
else if(fl=='A')
	{
	beep_stat_temp=zvuk;
	beep_stat=zvuk;
	beep_stat_cnt=32;
	}	 

else if(fl=='R')
	{
	beep_stat=zvuk;
	}	
		          
else if(fl=='S')
	{
	beep_stat_temp=0x0L;
	beep_stat=0x0L;
	beep_stat_cnt=32;
	}	
}

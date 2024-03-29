#include "memo.h"
#include "25lc640.h"
#include <LPC21xx.H>
#include "main.h"
#include "eeprom_map.h"

//-----------------------------------------------
void memo_read (void)
{
short i,ii,iii;
K_T=lc640_read_int(EE_K_T);
K_U=lc640_read_int(EE_K_U);
K_I_G[0]=lc640_read_int(EE_K_I_G0);
K_I_G[1]=lc640_read_int(EE_K_I_G1);
K_I_T[0]=lc640_read_int(EE_K_I_T0);
K_I_T[1]=lc640_read_int(EE_K_I_T1);
T_SIGN=lc640_read_int(EE_T_SIGN);
T_MAX=lc640_read_int(EE_T_MAX);
I_MAX=lc640_read_int(EE_I_MAX);
PR=lc640_read_int(EE_PR);
T_WRK=lc640_read_int(EE_T_WRK);
ZV_AV_TEMPER=(enum_onoff)lc640_read_int(EE_ZV_AV_TEMPER);
ZV_AV_SRC=(enum_onoff)lc640_read_int(EE_ZV_AV_SRC);
U_MAX=lc640_read_int(EE_U_MAX);
T_WRK_MAX=lc640_read_int(EE_T_WRK_MAX);
POWER=lc640_read_int(EE_POWER);
REST=(enum_onoff)lc640_read_int(EE_REST);
MNEMO_ON=lc640_read_int(EE_MNEMO_ON);
U_MIN1=lc640_read_int(EE_U_MIN1);
T_MIN1=lc640_read_int(EE_T_MIN1);
U_MIN2=lc640_read_int(EE_U_MIN2);
T_MIN2=lc640_read_int(EE_T_MIN2);
ADR=lc640_read_int(EE_ADR);
MODE=lc640_read_int(EE_MODE);
I_VK=lc640_read_int(EE_I_VK);
auto_bl_kb=(enum_onoff)lc640_read_int(EE_auto_bl_kb);
PAROL_KEYS=lc640_read_int(EE_PAROL_KEYS);
image_W=lc640_read_int(EE_image_W);
P_POV=lc640_read_int(EE_P_POV);
T_POV=lc640_read_int(EE_T_POV);
R_DOP=lc640_read_int(EE_R_DOP);
KOEF=lc640_read_int(EE_KOEF);
BAUDRATE=lc640_read_int(EE_BAUDRATE);
MODBUS_TYPE=lc640_read_int(EE_MODBUS_TYPE); 
iii=SECTOR_PROF;
for(i=0;i<10;i++)
	{
	for(ii=0;ii<5;ii++)
		{
		PU[i][ii]=lc640_read_int(iii);
		iii+=2;
		PI[i][ii]=lc640_read_int(iii);
		iii+=2;
		}
	}


}



extern const char*  name_of_blok; 


//#define UMAX_SET	6000
//#define UMIN_SET	100
//#define IMAX_SET	1000
//#define IMIN_SET	10

#define U_MAX_PWM	7000L
#define I_MAX_PWM	1000L
#define P_MAX		1000L

#define MASK(lengt) 		(0xffffffff>>(32-lengt))
#define MASK_OFFSET(shift,lengt)	(MASK(lengt)<<shift)

#define GET_REG( reg, shift, lengt) 		( (reg & MASK_OFFSET(shift,lengt)) >> shift)
#define SET_REG( reg, val, shift, lengt)  	reg = ( (reg & ~MASK_OFFSET(shift,lengt)) | (val << shift) )
//#define CHK_REG( reg, mask ) ( (reg) & (mask) == (mask) )


#define delay_ms(x) {long xx; xx=(unsigned long)x * 12000UL; while(xx)xx--;}
#define delay_us(x) {long xx; xx=(unsigned long)x * 12UL; while(xx)xx--;}


#define _7_1000_	7100
#define _7_200_	7200


#define BAUD_RATE0 9600UL
#define BAUD_RATE1 9600UL


#define AVAR_UNET_ON 	5
#define AV_UMAX_ON 		20
#define AV_UMIN_ON 		30 
#define AV_T_ON 		20
#define AV_UBAT_ON 		25
#define AV_IBAT_ON 		15 

#define LCD_SIZE 200

#define _year 		year__
#define _month		month__
#define _date 		day__
#define _hour 		hour__
#define _min 		min__
#define _sec 		sec__

#define SDA__  3
#define SCL__  2

#define BIN__N(x) (x) | x>>3 | x>>6 | x>>9
#define BIN__B(x) (x) & 0xf | (x)>>12 & 0xf0
#define BIN8(v) (BIN__B(BIN__N(0x##v)))

#define NUM_OF_SET_TABL	1
#define DEEP_OF_SET_TABL 18

#define BUT0	16
#define BUT1	17
#define BUT2	18
#define BUT3	19
#define BUT4	20   
#define BUT_MASK (1UL<<BUT0)|(1UL<<BUT1)|(1UL<<BUT2)|(1UL<<BUT3)|(1UL<<BUT4)

//#define PAROL_ALL_ZERO


#ifndef PAROL_ALL_ZERO_
#define PAROL_KALIBR 873
#define PAROL_SET 184
#define PAROL_MODE 0 
#define PAROL_ALLER 815 
#define PAROL_BAT_IN 722 
#define PAROL_BAT_OUT 722
#define PAROL_KE 125
#define PAROL_VZ 126
#define PAROL_TST 999 
#define PAROL_DEFAULT 295
#define PAROL_AUSW 949
#define PAROL_KEYS_RESCURE 911
#endif                

#ifdef PAROL_ALL_ZERO_
#define PAROL_KALIBR 0
#define PAROL_SET 0
#define PAROL_MODE 0 
#define PAROL_ALLER 0 
#define PAROL_BAT_IN 0 
#define PAROL_BAT_OUT 0
#define PAROL_KE 0
#define PAROL_VZ 0
#define PAROL_TST 0
#define PAROL_DEFAULT 0
#define PAROL_AUSW 0
#endif

//-----------------------------------------------
//������
extern char b1000Hz,b100Hz,b50Hz,b10Hz,b5Hz,b2Hz,b1Hz;
extern short t0cnt,t0cnt0,t0cnt1,t0cnt2,t0cnt3,t0cnt4,t0cnt5,t0cnt6,t0_cnt7;
extern char bFL5,bFL2,bFL,bFL_;

//-----------------------------------------------
//���
extern short adc_buff[8][16],adc_buff_[8],unet_buff[16],unet_buff_,adc_buff_U_;
extern char adc_cnt,adc_cnt1,adc_ch;

//-----------------------------------------------
//������������, ������������ �� EEPROM
extern signed short K_U;
extern signed short K_T;
extern signed short K_I_G[2];
extern signed short K_I_T[2];
extern signed short T_SIGN;
extern signed short T_MAX;
extern signed short I_MAX;
extern signed short PR;
extern signed short T_WRK;
typedef enum  {stOFF=0,stON=1}enum_onoff;
extern enum_onoff ZV_AV_TEMPER;
extern enum_onoff ZV_AV_SRC;
extern signed short PU[10][5];
extern signed short PI[10][5];
extern signed short U_MAX;
extern signed short T_WRK_MAX;
extern signed short POWER;
extern enum_onoff 	REST;
extern signed short MNEMO_ON;
extern signed short U_MIN1;
extern signed short T_MIN1;
extern signed short U_MIN2;
extern signed short T_MIN2;
extern signed short ADR;
extern signed short MODE;
extern signed short I_VK;
extern enum_onoff 	auto_bl_kb;
extern signed short PAROL_KEYS;
extern signed short image_W;
extern signed short P_POV;
extern signed short T_POV;
extern signed short R_DOP;
extern signed short KOEF;
extern signed short BAUDRATE;
extern signed short MODBUS_TYPE;

//-----------------------------------------------
//���������
typedef struct  
{
enum {iMn,iSrv_sl,iNet,iSet,iPr,iSrc,iS2,iSet_prl,iK_prl,iDnd,iABl_kb,iABl_kb_prl,iK,
	iSpcprl,iSpc,k,Crash_0,Crash_1,iKednd,iSet_prl_new,iSet_prl_change,iPMg,iRBl_kb_prl,
	iLoad,iSpc_prl_vz,iSpc_prl_ke,iKe,iVz,iAVAR,iStr,iVrs,iPrltst,iApv,
	iK_src,iK_bat,iK_load,iK_net,iTst,iTst_klbr,iTst_BPS1,iTst_BPS2,
	iTst_BPS12,iDebug,iDef,iSet_st_prl,iK_pdp,iSet_T,iDeb,iJ_bat,
	iPrl_bat_in_out,iPdp1,iDef_set,iSM,iJAv_net,iJAv_src1,
	iJAv_src2,iJAv_bat,iJAv_bat_sel,iAusw,iAusw_prl,iAusw_set,iK_t_out,
	iJ_bat_ke_sel,iJ_bat_ke,iJ_bat_vz_sel,iJ_bat_vz,iJ_bat_wrk_sel,iJ_bat_wrk,
	iExt_set,iExt_dt,iExt_sk,iAvz,iJAv_src1_sel,iJAv_src2_sel,iSTOP_umin2,iSTOP_umin1,
	iFWabout}i;

signed char s_i;
signed char s_i1;
signed char s_i2;
signed char i_s;
} stuct_ind;

extern stuct_ind a,b[10];

#define ind     a.i
#define sub_ind     a.s_i
#define sub_ind1     a.s_i1
#define sub_ind2     a.s_i2
#define index_set     a.i_s

//extern char image_W;
extern char dig[5];
extern char zero_on;
extern char lcd_buffer[LCD_SIZE];
extern char lcd_bitmap[1024];
extern const char ABCDEF[];
extern signed char ptr_ind;
extern char mnemo_cnt;
extern signed char parol[3];
extern char fl_simv_num,fl_simv_len;
extern char default_temp;
extern signed short av_j_si_max,av_j_si_max_;
extern char simax;
extern char phase;
extern const char sm_[]; //
extern const char sm_exit[]; //
extern const char smon[]; 
extern const char smoff[];
extern const char sm_mont[13][4]; //
extern char content[63]; 


//-----------------------------------------------
//�����
extern unsigned short hour_cnt_5hz,hour_cnt_1hz;
extern char cnt_ind;

//-----------------------------------------------
//�������
extern unsigned short rotor_can[6];
extern unsigned short cnt_sec;

//-----------------------------------------------
//������
extern char speed,l_but,n_but;
extern char but;                            
extern unsigned long but_n,but_s;
extern char but0_cnt;
extern char but1_cnt;
extern char but_onL_temp;

//-----------------------------------------------
//��������� ������ ������� 
extern signed short U,t,Ig_u,Usim,P,Pmax;
extern signed int It;
typedef enum {istG,istT}enum_ist;
extern enum_ist ist;
extern signed long Ig,Ires;
extern signed long Idop;

void bitmap_hndl(void);
void ind_hndl(void);
void but_drv(void);
__irq void timer0_interrupt(void); 
__irq void timer1_interrupt(void);
//__irq void can1_rx_interrupt(void);
 
//-----------------------------------------------
//���������� ��������
typedef enum {wrkOFF=3,wrkTIME,wrkI,wrkPR,wrkON=4} enum_wrk_state;
extern enum_wrk_state wrk_state;
extern signed short pwmI,pwmU,pwmI_start,pwmI_max;
extern signed long wrk_cnt;
extern char wrk_phase;
extern signed short Isim;
extern signed long wrk_cnt_cnt;
extern signed short i_pov_cnt,i_pov_cnt1;

//-----------------------------------------------
//�������� ������� 
typedef enum {avOFF,avON} enum_av_kv_stat;  
extern signed short kv_cnt;
extern enum_av_kv_stat av_kv_stat;
extern signed short kv_ind_cnt;

//-----------------------------------------------
//�������� ������
extern signed short av_out_cnt;
extern enum_av_kv_stat av_out_stat;
extern signed short av_out_ind_cnt;

//extern const char dumm[1000];
extern char plazma;



    
///
 


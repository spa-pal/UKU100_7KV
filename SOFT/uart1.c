#include "uart1.h"
#include <LPC21XX.H> 
#include "main.h"
#include "25lc640.h"
#include "eeprom_map.h"
#include "control.h"
#include "gran.h"
#include "common_func.h"
#include <string.h>
//#include <stdio.h>
#include <stdlib.h>

char bUART1TX;
char bRXIN1;
char UIB1[20];
char flag1;
char rx_buffer1[RX_BUFFER_SIZE1];
char tx_buffer1[TX_BUFFER_SIZE1];
unsigned char rx_wr_index1,rx_rd_index1,rx_counter1;
unsigned char tx_wr_index1,tx_rd_index1,tx_counter1;
char rx_buffer_overflow1;
char plazma_uart1;
char uart1_mess[10];
char data_rs1[40];
enum_usart1_router_stat usart1_router_stat;
char usart1_router_wrk;
char memo_out0[100];
char suzz[4];
char UIB10[30];
char usart1_router_cnt;
char _485_last_cnt;
char UIB[30];
extern char plazma;
short plazma_ppp;

unsigned char modbus_an_buffer[30];    	//Буфер, куда они потом копируются для анализа
unsigned char modbus_rx_buffer_ptr;	//Указатель на текущую позицию принимающего буфера
unsigned char modbus_rx_counter;		//Количество принятых байт, используется при анализе целостности посылки и при расшифровке
char modbus_timeout_cnt;
char bMODBUS_TIMEOUT;

short modbus_plazma;				//Отладка
short modbus_plazma1;				//Отладка
short modbus_plazma2;				//Отладка
short modbus_plazma3;				//Отладка

short modbus_rtu_plazma[5];
short pvlk;

char tx_wd_cnt=100;
unsigned long BAUD_RATE_1;
char modbus_tx_buff[100];

const char Table87[]={
0x00, 0x0E, 0x1C, 0x12, 0x38, 0x36, 0x24, 0x2A, 0x70, 0x7E, 0x6C, 0x62, 0x48, 0x46, 0x54, 0x5A,
0xE0, 0xEE, 0xFC, 0xF2, 0xD8, 0xD6, 0xC4, 0xCA, 0x90, 0x9E, 0x8C, 0x82, 0xA8, 0xA6, 0xB4, 0xBA,
0xCE, 0xC0, 0xD2, 0xDC, 0xF6, 0xF8, 0xEA, 0xE4, 0xBE, 0xB0, 0xA2, 0xAC, 0x86, 0x88, 0x9A, 0x94,
0x2E, 0x20, 0x32, 0x3C, 0x16, 0x18, 0x0A, 0x04, 0x5E, 0x50, 0x42, 0x4C, 0x66, 0x68, 0x7A, 0x74,
0x92, 0x9C, 0x8E, 0x80, 0xAA, 0xA4, 0xB6, 0xB8, 0xE2, 0xEC, 0xFE, 0xF0, 0xDA, 0xD4, 0xC6, 0xC8,
0x72, 0x7C, 0x6E, 0x60, 0x4A, 0x44, 0x56, 0x58, 0x02, 0x0C, 0x1E, 0x10, 0x3A, 0x34, 0x26, 0x28,
0x5C, 0x52, 0x40, 0x4E, 0x64, 0x6A, 0x78, 0x76, 0x2C, 0x22, 0x30, 0x3E, 0x14, 0x1A, 0x08, 0x06,
0xBC, 0xB2, 0xA0, 0xAE, 0x84, 0x8A, 0x98, 0x96, 0xCC, 0xC2, 0xD0, 0xDE, 0xF4, 0xFA, 0xE8, 0xE6,
0x2A, 0x24, 0x36, 0x38, 0x12, 0x1C, 0x0E, 0x00, 0x5A, 0x54, 0x46, 0x48, 0x62, 0x6C, 0x7E, 0x70,
0xCA, 0xC4, 0xD6, 0xD8, 0xF2, 0xFC, 0xEE, 0xE0, 0xBA, 0xB4, 0xA6, 0xA8, 0x82, 0x8C, 0x9E, 0x90,
0xE4, 0xEA, 0xF8, 0xF6, 0xDC, 0xD2, 0xC0, 0xCE, 0x94, 0x9A, 0x88, 0x86, 0xAC, 0xA2, 0xB0, 0xBE,
0x04, 0x0A, 0x18, 0x16, 0x3C, 0x32, 0x20, 0x2E, 0x74, 0x7A, 0x68, 0x66, 0x4C, 0x42, 0x50, 0x5E,
0xB8, 0xB6, 0xA4, 0xAA, 0x80, 0x8E, 0x9C, 0x92, 0xC8, 0xC6, 0xD4, 0xDA, 0xF0, 0xFE, 0xEC, 0xE2,
0x58, 0x56, 0x44, 0x4A, 0x60, 0x6E, 0x7C, 0x72, 0x28, 0x26, 0x34, 0x3A, 0x10, 0x1E, 0x0C, 0x02,
0x76, 0x78, 0x6A, 0x64, 0x4E, 0x40, 0x52, 0x5C, 0x06, 0x08, 0x1A, 0x14, 0x3E, 0x30, 0x22, 0x2C,
0x96, 0x98, 0x8A, 0x84, 0xAE, 0xA0, 0xB2, 0xBC, 0xE6, 0xE8, 0xFA, 0xF4, 0xDE, 0xD0, 0xC2, 0xCC};



const char Table95[]={
0x00, 0x2A, 0x54, 0x7E, 0xA8, 0x82, 0xFC, 0xD6, 0x7A, 0x50, 0x2E, 0x04, 0xD2, 0xF8, 0x86, 0xAC,
0xF4, 0xDE, 0xA0, 0x8A, 0x5C, 0x76, 0x08, 0x22, 0x8E, 0xA4, 0xDA, 0xF0, 0x26, 0x0C, 0x72, 0x58,
0xC2, 0xE8, 0x96, 0xBC, 0x6A, 0x40, 0x3E, 0x14, 0xB8, 0x92, 0xEC, 0xC6, 0x10, 0x3A, 0x44, 0x6E,
0x36, 0x1C, 0x62, 0x48, 0x9E, 0xB4, 0xCA, 0xE0, 0x4C, 0x66, 0x18, 0x32, 0xE4, 0xCE, 0xB0, 0x9A,
0xAE, 0x84, 0xFA, 0xD0, 0x06, 0x2C, 0x52, 0x78, 0xD4, 0xFE, 0x80, 0xAA, 0x7C, 0x56, 0x28, 0x02,
0x5A, 0x70, 0x0E, 0x24, 0xF2, 0xD8, 0xA6, 0x8C, 0x20, 0x0A, 0x74, 0x5E, 0x88, 0xA2, 0xDC, 0xF6,
0x6C, 0x46, 0x38, 0x12, 0xC4, 0xEE, 0x90, 0xBA, 0x16, 0x3C, 0x42, 0x68, 0xBE, 0x94, 0xEA, 0xC0,
0x98, 0xB2, 0xCC, 0xE6, 0x30, 0x1A, 0x64, 0x4E, 0xE2, 0xC8, 0xB6, 0x9C, 0x4A, 0x60, 0x1E, 0x34,
0x76, 0x5C, 0x22, 0x08, 0xDE, 0xF4, 0x8A, 0xA0, 0x0C, 0x26, 0x58, 0x72, 0xA4, 0x8E, 0xF0, 0xDA,
0x82, 0xA8, 0xD6, 0xFC, 0x2A, 0x00, 0x7E, 0x54, 0xF8, 0xD2, 0xAC, 0x86, 0x50, 0x7A, 0x04, 0x2E,
0xB4, 0x9E, 0xE0, 0xCA, 0x1C, 0x36, 0x48, 0x62, 0xCE, 0xE4, 0x9A, 0xB0, 0x66, 0x4C, 0x32, 0x18,
0x40, 0x6A, 0x14, 0x3E, 0xE8, 0xC2, 0xBC, 0x96, 0x3A, 0x10, 0x6E, 0x44, 0x92, 0xB8, 0xC6, 0xEC, 
0xD8, 0xF2, 0x8C, 0xA6, 0x70, 0x5A, 0x24, 0x0E, 0xA2, 0x88, 0xF6, 0xDC, 0x0A, 0x20, 0x5E, 0x74, 
0x2C, 0x06, 0x78, 0x52, 0x84, 0xAE, 0xD0, 0xFA, 0x56, 0x7C, 0x02, 0x28, 0xFE, 0xD4, 0xAA, 0x80, 
0x1A, 0x30, 0x4E, 0x64, 0xB2, 0x98, 0xE6, 0xCC, 0x60, 0x4A, 0x34, 0x1E, 0xC8, 0xE2, 0x9C, 0xB6, 
0xEE, 0xC4, 0xBA, 0x90, 0x46, 0x6C, 0x12, 0x38, 0x94, 0xBE, 0xC0, 0xEA, 0x3C, 0x16, 0x68, 0x42}; 

//-----------------------------------------------
void putchar1(char c)
{
while (tx_counter1 == TX_BUFFER_SIZE1);
if (tx_counter1 || ((U1LSR & 0x60)==0))
   {
   tx_buffer1[tx_wr_index1]=c;
   if (++tx_wr_index1 == TX_BUFFER_SIZE1) tx_wr_index1=0;
   ++tx_counter1;
   }
else 
	{


	U1THR=c;
	tx_wd_cnt=100;
	}
}

//-----------------------------------------------
void uart_out1 (char num,char data0,char data1,char data2,char data3,char data4,char data5)
{
char i,t=0;
//char *ptr=&data1;
char UOB1[16]; 
UOB1[0]=data0;
UOB1[1]=data1;
UOB1[2]=data2;
UOB1[3]=data3;
UOB1[4]=data4;
UOB1[5]=data5;

for (i=0;i<num;i++)
	{
	t^=UOB1[i];
	}    
UOB1[num]=num;
t^=UOB1[num];
UOB1[num+1]=t;
//UOB1[num+2]=END;

for (i=0;i<num+3;i++)
	{
	putchar1(UOB1[i]);
	}   	
}

//-----------------------------------------------
void uart_out_adr1 (char *ptr, char len)
{
char UOB[110]/*={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}*/;
char i/*,t=0*/;

for(i=0;i<len;i++)
	{
	UOB[i]=ptr[i];
	}
	
//puts(UOB);

SET_REG(PINSEL1,0,(22-16)*2,2); //Вход EN у 485
IO0DIR|=(1<<22);
IO0SET|=(1<<22); 
for (i=0;i<len;i++)
	{
	putchar1(UOB[i]);
	}   
}
//-----------------------------------------------
void uart1_init(void)
{

BAUD_RATE_1=(unsigned long)(BAUDRATE)*100UL;


SET_REG(PINSEL0,1,8*2,2);
SET_REG(PINSEL0,1,9*2,2);

SET_REG(U1LCR,1,7/*DLAB*/,1);//U0LCR_bit.DLAB=1;
U1DLL=60000000UL/(BAUD_RATE_1*16UL);
U1DLM=60000000UL/(BAUD_RATE_1*16UL*256UL);
SET_REG(U1LCR,0,7/*DLAB*/,1);//U0LCR_bit.DLAB=0;
U1LCR=0x03;
U1FCR=0;

VICProtection = 0;
VICIntEnClr |= (1 << VIC_UART1); 
VICIntSelect &= ~(1 << VIC_UART1);
VICVectAddr3=(unsigned)uart1_interrupt;
VICVectCntl3 = 0x20 | VIC_UART1;
VICIntEnable |= (1  << VIC_UART1);

U1IER=0x03;

}


//-----------------------------------------------
char getchar1(void)
{
char data;
while (rx_counter1==0);
data=rx_buffer1[rx_rd_index1];
if (++rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1=0;
--rx_counter1;
return data;
}

//***********************************************
__irq void uart1_interrupt(void)
{
char /*status,*/u1iir,data;

//plazma++;

//status=U1LSR;
u1iir=U1IIR;
   	
if(((u1iir&0x0f)==4)	)
	{
	//modbus_rtu_plazma[2]++;
	data=U1RBR;
	if(bUART1TX==0)
	{
	
	rx_buffer1[rx_wr_index1]=data;
   	//bRXIN1=1;
  	if (++rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0;
   	if (++rx_counter1 == RX_BUFFER_SIZE1)
      	{
      	rx_counter1=0;
      	rx_buffer_overflow1=1;
      	}

	if((data==0x0d)||(data==0x0a))_485_last_cnt=5;
	else _485_last_cnt=0;
    
	modbus_timeout_cnt=0;
	}
	//else  modbus_rtu_plazma[3]++;
	//plazma_uart11++;
   	}
else if((u1iir&0x0f)==2)
	{
	if (tx_counter1)
   		{
   		//usart1_router_wrk=1;
   		--tx_counter1;
   		U1THR=tx_buffer1[tx_rd_index1];
   		if (++tx_rd_index1 == TX_BUFFER_SIZE1) tx_rd_index1=0;
		bUART1TX=1;

		tx_wd_cnt=100;
   		}
   	else 
		{
		SET_REG(PINSEL1,0,(22-16)*2,1); //Вход PV у 485
		IO0DIR|=(1<<22);
		IO0CLR|=(1<<22);
		bUART1TX=0;
		}
			
	} 
VICVectAddr = 0;  	
}
 

//-----------------------------------------------
void uart_in_an1(void)
{
char i;

char adrh,adrl;

		
		for(i=0;i<rx_counter1;i++)
			{
			UIB1[i]=rx_buffer1[i] ;

			//UIB1[0]=0x34;
			}
		//bRXIN1=1;
			//plazma++;
		rx_counter1=0;
		rx_wr_index1=0;


adrl=ABCDEF[ADR%16];
adrh=ABCDEF[ADR/16];

//plazma=UIB1[1];
//

if((UIB1[0]=='Z'))
	{
	//uart_out1(3,1,2,3,4,5,6);/
	if((UIB1[1]==adrh)&&(UIB1[2]==adrl))
		{
	 	if((UIB1[3]=='N')&&(UIB1[5]==0x0d))
			{
			if(UIB1[4]=='N')
				{
//				char i;
				plazma++;
				memo_out0[0]='!';
				memo_out0[1]=adrh;
				memo_out0[2]=adrl;
				memo_out0[3]='N';
				memo_out0[4]='>';
				//memo_out0[5]='>';
				i=0;
				while(name_of_blok[i])
					{
					memo_out0[5+i]=name_of_blok[i];
					i++;
					}
				memo_out0[5+i]=0x0d;
				uart_out_adr1(memo_out0,6+i);
				//uart_out1(3,1,2,3,4,5,6);
				}
			}

	 	else if((UIB1[3]=='4')/*&&(UIB1[5]==0x0d)*/)
			{
			signed short tempSS;
			signed long tempSL;
			//char temp;
			//memo_out0[5]=0x0d;
			//uart_out_adr1(memo_out0,6);
			if(UIB1[4]=='1')    //Текущее напряжение и ток
				{
//				char i;
				plazma++;
				memo_out0[0]='!';
				memo_out0[1]=adrh;
				memo_out0[2]=adrl;
				memo_out0[3]='4';
				memo_out0[4]='>';
				memo_out0[5]='1';
				//memo_out0[5]='>';
			/*	i=0;
				while(name_of_blok[i])
					{
					memo_out0[5+i]=name_of_blok[i];
					i++;
					}
				memo_out0[5+i]=0x0d;*/
				memo_out0[6]='+';
				memo_out0[7]='0'+ (U/10000);
				tempSS=U%10000;

				memo_out0[8]='0'+ (tempSS/1000);
				tempSS=tempSS%1000;

				memo_out0[9]='0'+ (tempSS/100);
				tempSS=tempSS%100;

				memo_out0[10]='0'+ (tempSS/10);
				tempSS=tempSS%10;

				memo_out0[11]='0'+ tempSS;
				
				tempSL=(signed long)Ires/10L;


				memo_out0[12]='+';

				memo_out0[13]='0'+ (char)(tempSL/100000L);
				tempSL=tempSL%100000L;
				
				memo_out0[14]='0'+ (char)(tempSL/10000L);
				tempSL=tempSL%10000L;

				memo_out0[15]='0'+ (char)(tempSL/1000L);
				tempSL=tempSL%1000L;
				
				memo_out0[16]=',';
				
				memo_out0[17]='0'+ (char)(tempSL/100L);
				tempSL=tempSL%100L;
								
				memo_out0[18]='0'+ (char)(tempSL/10L);
				tempSL=tempSL%10L;
				
				memo_out0[19]='0'+ (char)tempSL;
				
				memo_out0[20]=0x0d;

				uart_out_adr1(memo_out0,21);
				//uart_out1(3,1,2,3,4,5,6);
				}



			if(UIB1[4]=='4')    //Текущее напряжение и ток
				{
//				char i;
				plazma++;
				memo_out0[0]='!';
				memo_out0[1]=adrh;
				memo_out0[2]=adrl;
				memo_out0[3]='4';
				memo_out0[4]='>';
				memo_out0[5]='5';
				//memo_out0[5]='>';
			/*	i=0;
				while(name_of_blok[i])
					{
					memo_out0[5+i]=name_of_blok[i];
					i++;
					}
				memo_out0[5+i]=0x0d;*/
				memo_out0[6]='+';
				memo_out0[7]='0'+ (U/10000);
				tempSS=U%10000;

				memo_out0[8]='0'+ (tempSS/1000);
				tempSS=tempSS%1000;

				memo_out0[9]='0'+ (tempSS/100);
				tempSS=tempSS%100;

				memo_out0[10]='0'+ (tempSS/10);
				tempSS=tempSS%10;

				memo_out0[11]='0'+ tempSS;
				
				tempSL=(signed long)Ires;


				memo_out0[12]='+';

				memo_out0[13]='0'+ (char)(tempSL/1000000L);
				tempSL=tempSL%1000000L;
				
				memo_out0[14]='0'+ (char)(tempSL/100000L);
				tempSL=tempSL%100000L;

				memo_out0[15]='0'+ (char)(tempSL/10000L);
				tempSL=tempSL%10000L;
				
				memo_out0[16]=',';
				
				memo_out0[17]='0'+ (char)(tempSL/1000L);
				tempSL=tempSL%1000L;
								
				memo_out0[18]='0'+ (char)(tempSL/100L);
				tempSL=tempSL%100L;

				memo_out0[19]='0'+ (char)(tempSL/10L);
				tempSL=tempSL%10L;
				
				memo_out0[20]='0'+ (char)tempSL;
				
				memo_out0[21]=0x0d;

				uart_out_adr1(memo_out0,22);
				//uart_out1(3,1,2,3,4,5,6);
				}
			}

	 	else if(UIB1[3]=='3')         //установка параметра
			{
			signed short tempSS;
//			char temp;
//			unsigned char *s;

			if(UIB1[4]=='1')         //первый канал
				{
				if(UIB1[5]=='U')    //Установка начального напряжения 
                         {
                         char i,ii;
                         char strIng[20]; 
						 i=5;
						 //ii=strpos(UIB,'a');

  
  //unsigned char buf [] = "This is a test";

  						//	ii = strpos (UIB, 't');
						 
						 //s = strchr(&UIB1[6],0x0d);

						 
						                     
                        for(i=6;i<25;i++)
                              {
                              if(UIB1[i]==0x0d)
                                   {
                                   ii=i-1;
                                   break;
                                   }
							  
                              }	
						plazma_ppp=ii;
                        memcpy(strIng, &UIB1[6], ii-5);
						strIng[ii-5]=0; 
                        
						/* tempSS=0;
                         for(i=ii;i>5;i--)
                              {
                              tempSS+=((UIB1[i]-0x30)*(pow(10,(-(i-ii)))));
                              }

                         U1=tempSS;
                         gran(&U1,10,16000);   */

						tempSS=(signed short)atoi(strIng);
     			     	lc640_write_int(EE_U_MAX,tempSS);
					//	UIB1[3]=0; 
                        
						
						
						memo_out0[0]='!';
				     	memo_out0[1]=adrh;
				     	memo_out0[2]=adrl;
				     	memo_out0[3]='3';
				     	memo_out0[4]=0x0d;
				     	uart_out_adr1(memo_out0,5);

                         }
				if(UIB1[5]=='I')    //Установка начального напряжения 
                	{
                    char i,ii;
                    char strIng[20]; 
					i=5;
                    
					for(i=6;i<25;i++)
                    	{
                        if(UIB1[i]==0x0d)
                        	{
                            ii=i-1;
                            break;
                            }
							  
                        }	
					plazma_ppp=ii;
                   	memcpy(strIng, &UIB1[6], ii-5);
					strIng[ii-5]=0; 

					tempSS=(signed short)atoi(strIng);
     			    lc640_write_int(EE_I_MAX,tempSS);
					//	UIB1[3]=0; 
                        
						
						
					memo_out0[0]='!';
				    memo_out0[1]=adrh;
				    memo_out0[2]=adrl;
				   	memo_out0[3]='3';
				    memo_out0[4]=0x0d;
				    uart_out_adr1(memo_out0,5);

                    }	
				if(UIB1[5]=='R')    //Установка начального напряжения 
                	{
                    char i,ii;
                    char strIng[20]; 
					i=5;
                    
					for(i=6;i<25;i++)
                    	{
                        if(UIB1[i]==0x0d)
                        	{
                            ii=i-1;
                            break;
                            }
							  
                        }	
					plazma_ppp=ii;
                   	memcpy(strIng, &UIB1[6], ii-5);
					strIng[ii-5]=0; 

					tempSS=(signed short)(atoi(strIng)/10);
     			    lc640_write_int(EE_I_VK,tempSS);
					//	UIB1[3]=0; 
                        
						
						
					memo_out0[0]='!';
				    memo_out0[1]=adrh;
				    memo_out0[2]=adrl;
				   	memo_out0[3]='3';
				    memo_out0[4]=0x0d;
				    uart_out_adr1(memo_out0,5);

                    }			
				}

			}

	 	else if((UIB1[3]=='1')&&(UIB1[5]==0x0d))    //включение-выключение
			{
			if(UIB1[4]=='E')
				{
//				char i;

				wrk_state=wrkON;
				lc640_write_int(EE_WRK_STAT,wrk_state);
				wrk_cnt=0;

				memo_out0[0]='!';
				memo_out0[1]=adrh;
				memo_out0[2]=adrl;
				memo_out0[3]='1';
				memo_out0[4]=0x0d;
				uart_out_adr1(memo_out0,5);
				
				}

			else if(UIB1[4]=='D')
				{
//				char i;

				wrk_state=wrkOFF;
				lc640_write_int(EE_WRK_STAT,wrk_state);

				memo_out0[0]='!';
				memo_out0[1]=adrh;
				memo_out0[2]=adrl;
				memo_out0[3]='1';
				memo_out0[4]=0x0d;
				uart_out_adr1(memo_out0,5);
				}

			}

	 	}
	 
	}
/*if((UIB1[0]==0x55)&&(UIB1[1]==0x66))
	{
	
	uart_out1(2,0x57,0x66,0,0,0,0);
	}*/

/*else if((UIB1[0]==CMND)&&(UIB1[1]==1))
	{
	adc_buff_out_[0]=UIB1[2]+(UIB1[3]*256);
	adc_buff_out_[1]=UIB1[4]+(UIB1[5]*256);
	}

else if((UIB1[0]==CMND)&&(UIB1[1]==2))
	{
	adc_buff_out_[2]=UIB1[2]+(UIB1[3]*256);
	in_stat_out[0]=UIB1[4];
	in_stat_out[1]=UIB1[5];
	}
	
else if((UIB1[0]==CMND)&&(UIB1[1]==QWEST)&&(UIB1[2]==PUTTM))
	{
	adc_buff_out_[0]=UIB1[3]+(UIB1[4]*256);
	adc_buff_out_[1]=UIB1[5]+(UIB1[6]*256);
	adc_buff_out_[2]=UIB1[7]+(UIB1[8]*256);
	in_stat_out[0]=UIB1[9];
	in_stat_out[1]=UIB1[10];
	in_stat_out[2]=UIB1[11];
	in_stat_out[3]=UIB1[12];	
	}*/	
		
}

//-----------------------------------------------
char crc_87(char* ptr,char num)
{
char r,j;
r=*ptr;

for(j=1;j<num;j++)
	{
     ptr++;
	r=((*ptr)^Table87[r]);
	}

return r;	
} 

//-----------------------------------------------
char crc_95(char* ptr,char num)
{
char r,j;
r=*ptr;

for(j=1;j<num;j++)
	{
     ptr++;
	r=((*ptr)^Table95[r]);
	}

return r;	
}



//-----------------------------------------------
unsigned short CRC16_2(char* buf, short len)
{
unsigned short crc = 0xFFFF;
short pos;
short i;

for (pos = 0; pos < len; pos++)
  	{
    	crc ^= (unsigned short)buf[pos];          // XOR byte into least sig. byte of crc

    	for ( i = 8; i != 0; i--) 
		{    // Loop over each bit
      	if ((crc & 0x0001) != 0) 
			{      // If the LSB is set
        		crc >>= 1;                    // Shift right and XOR 0xA001
        		crc ^= 0xA001;
      		}
      	else  crc >>= 1;                    // Just shift right
    		}
  	}
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
return crc;
}

//-----------------------------------------------
void modbus_hold_coils_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[150];
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;

Ires=123456;
U=654;
//modbus_rtu_plazma[0]++;//=modbus_rx_counter;

modbus_registers[20]=(signed char)(*(((char*)(&Ires))+1));					//Рег11	Старший байт младшего слова Iвых
modbus_registers[21]=(signed char)(*(((char*)(&Ires))));					//    	Младший байт младшего слова Iвых
modbus_registers[22]=(signed char)(*(((char*)(&Ires))+3));					//Рег12	Старший байт старшего слова Iвых
modbus_registers[23]=(signed char)(*(((char*)(&Ires))+2));					//   	Младший байт старшего слова Iвых
modbus_registers[24]=(signed char)(*(((char*)(&U))+1));						//Рег13	Старший байт младшего слова Uвых
modbus_registers[25]=(signed char)(*(((char*)(&U))));						//    	Младший байт младшего слова Uвых
modbus_registers[26]=0;														//Рег14	Старший байт старшего слова Uвых
modbus_registers[27]=0;														//   	Младший байт старшего слова Uвых
modbus_registers[28]=(signed char)(*(((char*)(&I_MAX))+1));					//Рег15	Старший байт младшего слова уставки тока
modbus_registers[29]=(signed char)(*(((char*)(&I_MAX))));					//    	Младший байт младшего слова уставки тока
modbus_registers[30]=0;														//Рег16	Старший байт старшего слова уставки тока
modbus_registers[31]=0;														//   	Младший байт старшего слова уставки тока
modbus_registers[32]=(signed char)(*(((char*)(&U_MAX))+1));					//Рег17	Старший байт младшего слова уставки напряжения
modbus_registers[33]=(signed char)(*(((char*)(&U_MAX))));					//    	Младший байт младшего слова уставки напряжения
modbus_registers[34]=0;														//Рег18	Старший байт старшего слова уставки напряжения
modbus_registers[35]=0;														//   	Младший байт старшего слова уставки напряжения


/*modbus_registers[28]=(char)((LPC_RTC->MIN)>>8);			//Рег15  Время, минуты
modbus_registers[29]=(char)((LPC_RTC->MIN));
modbus_registers[30]=(char)((LPC_RTC->SEC)>>8);			//Рег16  Время, секунды
modbus_registers[31]=(char)((LPC_RTC->SEC));
modbus_registers[38]=(char)(NUMIST>>8);				//Рег20  Количество выпрямителей в структуре
modbus_registers[39]=(char)(NUMIST);
//modbus_registers[40]=(char)(PAR>>8);					//Рег21  Параллельная работа выпрямителей вкл./выкл.
//modbus_registers[41]=(char)(PAR);
modbus_registers[42]=(char)(ZV_ON>>8);					//Рег22  Звуковая аварийная сигнализация вкл./выкл.
modbus_registers[43]=(char)(ZV_ON);
modbus_registers[46]=(char)(UBM_AV>>8);				//Рег24  Аварийный уровень отклонения напряжения средней точки батареи, %
modbus_registers[47]=(char)(UBM_AV);
modbus_registers[58]=(char)(TBAT>>8);					//Рег30  Период проверки цепи батареи, минут.
modbus_registers[59]=(char)(TBAT);
modbus_registers[60]=(char)(UMAX>>8);					//Рег31  Максимальное (аварийное) напряжение выпрямителей, 0.1В
modbus_registers[61]=(char)(UMAX);
//modbus_registers[62]=(char)((UB20-DU)>>8);				//Рег32  Минимальное (аварийное) напряжение выпрямителей, 0.1В
//modbus_registers[63]=(char)((UB20-DU));
//modbus_registers[64]=(char)(UB0>>8);					//Рег33  Напряжение содержания батареи при 0гЦ, 0.1В
//modbus_registers[65]=(char)(UB0);
//modbus_registers[66]=(char)(UB20>>8);					//Рег34  Напряжение содержания батареи при 20гЦ, 0.1В
//modbus_registers[67]=(char)(UB20);
modbus_registers[68]=(char)(USIGN>>8);					//Рег35  Минимальное (сигнальное) напряжение батареи, 1В
modbus_registers[69]=(char)(USIGN);
modbus_registers[70]=(char)(UMN>>8);					//Рег36  Минимальное (аварийное) напряжение питающей сети, 1В
modbus_registers[71]=(char)(UMN);
modbus_registers[72]=(char)(U0B>>8);					//Рег37  Рабочее напряжение при невведенных батареях, 0.1В
modbus_registers[73]=(char)(U0B);
modbus_registers[74]=(char)(IKB>>8);					//Рег38  Ток контроля наличия батареи, 0.1а
modbus_registers[75]=(char)(IKB);
modbus_registers[76]=(char)(IZMAX>>8);					//Рег39  Ток заряда батареи максимальный, 0.1А
modbus_registers[77]=(char)(IZMAX);
modbus_registers[78]=(char)(IMAX>>8);					//Рег40  Ток переключения на большее кол-во выпрямителей, 0.1А
modbus_registers[79]=(char)(IMAX);
modbus_registers[80]=(char)(IMIN>>8);					//Рег41  Ток переключения на меньшее кол-во выпрямителей, 0.1А
modbus_registers[81]=(char)(IMIN);
//modbus_registers[82]=(char)(UVZ>>8);					//Рег42  Напряжение выравнивающего заряда, 0.1В
//modbus_registers[83]=(char)(UVZ);
modbus_registers[84]=(char)(TZAS>>8);					//Рег43  Время задержки включения выпрямителей, сек
modbus_registers[85]=(char)(TZAS);
modbus_registers[86]=(char)(TMAX>>8);					//Рег44  Температура выпрямителей аварийная, 1гЦ
modbus_registers[87]=(char)(TMAX);
modbus_registers[88]=(char)(TSIGN>>8);					//Рег45  Температура выпрямителей сигнальная, 1гЦ
modbus_registers[89]=(char)(TSIGN);
modbus_registers[90]=(char)(TBATMAX>>8);				//Рег46  Температура батареи аварийная, 1гЦ
modbus_registers[91]=(char)(TBATMAX);
modbus_registers[92]=(char)(TBATSIGN>>8);				//Рег47  Температура батареи сигнальная, 1гЦ
modbus_registers[93]=(char)(TBATSIGN);
modbus_registers[94]=(char)(speedChrgCurr>>8);					//Рег48  Ток ускоренного заряда, 0.1А
modbus_registers[95]=(char)(speedChrgCurr);
modbus_registers[96]=(char)(speedChrgVolt>>8);				//Рег49	 Напряжение ускоренного заряда, 0.1В 
modbus_registers[97]=(char)(speedChrgVolt);
modbus_registers[98]=(char)(speedChrgTimeInHour>>8);				//Рег50	 Время ускоренного заряда, 1ч
modbus_registers[99]=(char)(speedChrgTimeInHour);
modbus_registers[100]=(char)(U_OUT_KONTR_MAX>>8);					//Рег51	 Контроль выходного напряжения, Umax, 0.1В
modbus_registers[101]=(char)(U_OUT_KONTR_MAX);
modbus_registers[102]=(char)(U_OUT_KONTR_MIN>>8);					//Рег52	 Контроль выходного напряжения, Umin, 0.1В
*/
modbus_registers[103]=(char)(3);
modbus_registers[104]=(char)(4);				//Рег53	 Контроль выходного напряжения, Tзадержки, 1сек.
modbus_registers[105]=(char)(5);




if(prot==0)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;
	modbus_tx_buff[2]=(char)((reg_quantity/8)+1);
	memcpy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	
	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

	SET_REG(PINSEL1,0,(22-16)*2,2); //Вход EN у 485
	IO0DIR|=(1<<22);
	IO0SET|=(1<<22);

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar1(modbus_tx_buff[i]);
		}

	}

}

//-----------------------------------------------
void modbus_hold_registers_transmit(unsigned char adr,unsigned char func,unsigned short reg_adr,unsigned short reg_quantity, char prot)
{
signed char modbus_registers[150];
//char modbus_tx_buff[150];
unsigned short crc_temp;
char i;

//Ires=123456;
//U=654;
//modbus_rtu_plazma[0]++;//=modbus_rx_counter;

modbus_registers[20]=(signed char)(*(((char*)(&Ires))+1));					//Рег11	Старший байт младшего слова Iвых
modbus_registers[21]=(signed char)(*(((char*)(&Ires))));					//    	Младший байт младшего слова Iвых
modbus_registers[22]=(signed char)(*(((char*)(&Ires))+3));					//Рег12	Старший байт старшего слова Iвых
modbus_registers[23]=(signed char)(*(((char*)(&Ires))+2));					//   	Младший байт старшего слова Iвых
modbus_registers[24]=(signed char)(*(((char*)(&U))+1));						//Рег13	Старший байт младшего слова Uвых
modbus_registers[25]=(signed char)(*(((char*)(&U))));						//    	Младший байт младшего слова Uвых
modbus_registers[26]=0;														//Рег14	Старший байт старшего слова Uвых
modbus_registers[27]=0;														//   	Младший байт старшего слова Uвых
modbus_registers[28]=(signed char)(*(((char*)(&I_MAX))+1));					//Рег15	Старший байт младшего слова уставки тока
modbus_registers[29]=(signed char)(*(((char*)(&I_MAX))));					//    	Младший байт младшего слова уставки тока
modbus_registers[30]=0;														//Рег16	Старший байт старшего слова уставки тока
modbus_registers[31]=0;														//   	Младший байт старшего слова уставки тока
modbus_registers[32]=(signed char)(*(((char*)(&U_MAX))+1));					//Рег17	Старший байт младшего слова уставки напряжения
modbus_registers[33]=(signed char)(*(((char*)(&U_MAX))));					//    	Младший байт младшего слова уставки напряжения
modbus_registers[34]=0;														//Рег18	Старший байт старшего слова уставки напряжения
modbus_registers[35]=0;														//   	Младший байт старшего слова уставки напряжения


/*modbus_registers[28]=(char)((LPC_RTC->MIN)>>8);			//Рег15  Время, минуты
modbus_registers[29]=(char)((LPC_RTC->MIN));
modbus_registers[30]=(char)((LPC_RTC->SEC)>>8);			//Рег16  Время, секунды
modbus_registers[31]=(char)((LPC_RTC->SEC));
modbus_registers[38]=(char)(NUMIST>>8);				//Рег20  Количество выпрямителей в структуре
modbus_registers[39]=(char)(NUMIST);
//modbus_registers[40]=(char)(PAR>>8);					//Рег21  Параллельная работа выпрямителей вкл./выкл.
//modbus_registers[41]=(char)(PAR);
modbus_registers[42]=(char)(ZV_ON>>8);					//Рег22  Звуковая аварийная сигнализация вкл./выкл.
modbus_registers[43]=(char)(ZV_ON);
modbus_registers[46]=(char)(UBM_AV>>8);				//Рег24  Аварийный уровень отклонения напряжения средней точки батареи, %
modbus_registers[47]=(char)(UBM_AV);
modbus_registers[58]=(char)(TBAT>>8);					//Рег30  Период проверки цепи батареи, минут.
modbus_registers[59]=(char)(TBAT);
modbus_registers[60]=(char)(UMAX>>8);					//Рег31  Максимальное (аварийное) напряжение выпрямителей, 0.1В
modbus_registers[61]=(char)(UMAX);
//modbus_registers[62]=(char)((UB20-DU)>>8);				//Рег32  Минимальное (аварийное) напряжение выпрямителей, 0.1В
//modbus_registers[63]=(char)((UB20-DU));
//modbus_registers[64]=(char)(UB0>>8);					//Рег33  Напряжение содержания батареи при 0гЦ, 0.1В
//modbus_registers[65]=(char)(UB0);
//modbus_registers[66]=(char)(UB20>>8);					//Рег34  Напряжение содержания батареи при 20гЦ, 0.1В
//modbus_registers[67]=(char)(UB20);
modbus_registers[68]=(char)(USIGN>>8);					//Рег35  Минимальное (сигнальное) напряжение батареи, 1В
modbus_registers[69]=(char)(USIGN);
modbus_registers[70]=(char)(UMN>>8);					//Рег36  Минимальное (аварийное) напряжение питающей сети, 1В
modbus_registers[71]=(char)(UMN);
modbus_registers[72]=(char)(U0B>>8);					//Рег37  Рабочее напряжение при невведенных батареях, 0.1В
modbus_registers[73]=(char)(U0B);
modbus_registers[74]=(char)(IKB>>8);					//Рег38  Ток контроля наличия батареи, 0.1а
modbus_registers[75]=(char)(IKB);
modbus_registers[76]=(char)(IZMAX>>8);					//Рег39  Ток заряда батареи максимальный, 0.1А
modbus_registers[77]=(char)(IZMAX);
modbus_registers[78]=(char)(IMAX>>8);					//Рег40  Ток переключения на большее кол-во выпрямителей, 0.1А
modbus_registers[79]=(char)(IMAX);
modbus_registers[80]=(char)(IMIN>>8);					//Рег41  Ток переключения на меньшее кол-во выпрямителей, 0.1А
modbus_registers[81]=(char)(IMIN);
//modbus_registers[82]=(char)(UVZ>>8);					//Рег42  Напряжение выравнивающего заряда, 0.1В
//modbus_registers[83]=(char)(UVZ);
modbus_registers[84]=(char)(TZAS>>8);					//Рег43  Время задержки включения выпрямителей, сек
modbus_registers[85]=(char)(TZAS);
modbus_registers[86]=(char)(TMAX>>8);					//Рег44  Температура выпрямителей аварийная, 1гЦ
modbus_registers[87]=(char)(TMAX);
modbus_registers[88]=(char)(TSIGN>>8);					//Рег45  Температура выпрямителей сигнальная, 1гЦ
modbus_registers[89]=(char)(TSIGN);
modbus_registers[90]=(char)(TBATMAX>>8);				//Рег46  Температура батареи аварийная, 1гЦ
modbus_registers[91]=(char)(TBATMAX);
modbus_registers[92]=(char)(TBATSIGN>>8);				//Рег47  Температура батареи сигнальная, 1гЦ
modbus_registers[93]=(char)(TBATSIGN);
modbus_registers[94]=(char)(speedChrgCurr>>8);					//Рег48  Ток ускоренного заряда, 0.1А
modbus_registers[95]=(char)(speedChrgCurr);
modbus_registers[96]=(char)(speedChrgVolt>>8);				//Рег49	 Напряжение ускоренного заряда, 0.1В 
modbus_registers[97]=(char)(speedChrgVolt);
modbus_registers[98]=(char)(speedChrgTimeInHour>>8);				//Рег50	 Время ускоренного заряда, 1ч
modbus_registers[99]=(char)(speedChrgTimeInHour);
modbus_registers[100]=(char)(U_OUT_KONTR_MAX>>8);					//Рег51	 Контроль выходного напряжения, Umax, 0.1В
modbus_registers[101]=(char)(U_OUT_KONTR_MAX);
modbus_registers[102]=(char)(U_OUT_KONTR_MIN>>8);					//Рег52	 Контроль выходного напряжения, Umin, 0.1В
*/
modbus_registers[103]=(char)(3);
modbus_registers[104]=(char)(4);				//Рег53	 Контроль выходного напряжения, Tзадержки, 1сек.
modbus_registers[105]=(char)(5);




if(prot==0)
	{
	modbus_tx_buff[0]=adr;
	modbus_tx_buff[1]=func;
	modbus_tx_buff[2]=(char)(reg_quantity*2);
	memcpy((char*)&modbus_tx_buff[3],(char*)&modbus_registers[(reg_adr-1)*2],reg_quantity*2);
	
	crc_temp=CRC16_2(modbus_tx_buff,(reg_quantity*2)+3);

	modbus_tx_buff[3+(reg_quantity*2)]=(char)crc_temp;
	modbus_tx_buff[4+(reg_quantity*2)]=crc_temp>>8;

	SET_REG(PINSEL1,0,(22-16)*2,2); //Вход EN у 485
	IO0DIR|=(1<<22);
	IO0SET|=(1<<22);

	for (i=0;i<(5+(reg_quantity*2));i++)
		{
		putchar1(modbus_tx_buff[i]);
		}

	}

}

//-----------------------------------------------
void modbus_in(void)
{
short crc16_calculated;		//вычисляемая из принятых данных CRC
short crc16_incapsulated;	//встроеннная в посылку CRC
unsigned short modbus_rx_arg0;		//встроенный в посылку первый аргумент
unsigned short modbus_rx_arg1;		//встроенный в посылку второй аргумент
//unsigned short modbus_rx_arg2;		//встроенный в посылку третий аргумент
//unsigned short modbus_rx_arg3;		//встроенный в посылку четвертый аргумент
unsigned char modbus_func;			//встроенный в посылку код функции

memcpy(modbus_an_buffer,rx_buffer1,rx_wr_index1);
modbus_rx_counter=rx_wr_index1;
//modbus_rx_buffer_ptr=0;
//bMODBUS_TIMEOUT=0;
		rx_counter1=0;
		rx_wr_index1=0;


modbus_rtu_plazma[1]=modbus_rx_counter;

//memcpy(modbus_an_buffer,UIB1,modbus_rx_counter);
//modbus_rx_counter=modbus_rx_buffer_ptr;
//modbus_rx_buffer_ptr=0;
//bMODBUS_TIMEOUT=0;
	
crc16_calculated  = CRC16_2((char*)modbus_an_buffer, modbus_rx_counter-2);
crc16_incapsulated = *((short*)&modbus_an_buffer[modbus_rx_counter-2]);

modbus_plazma1=modbus_rx_counter;
modbus_plazma2=crc16_calculated;
modbus_plazma3=crc16_incapsulated;

modbus_func=modbus_an_buffer[1];
modbus_rx_arg0=(((unsigned short)modbus_an_buffer[2])*((unsigned short)256))+((unsigned short)modbus_an_buffer[3]);
modbus_rx_arg1=(((unsigned short)modbus_an_buffer[4])*((unsigned short)256))+((unsigned short)modbus_an_buffer[5]);
//modbus_rx_arg2=(((unsigned short)modbus_an_buffer[6])*((unsigned short)256))+((unsigned short)modbus_an_buffer[7]);
//modbus_rx_arg3=(((unsigned short)modbus_an_buffer[8])*((unsigned short)256))+((unsigned short)modbus_an_buffer[9]);

//#define IPS_CURR_AVG_MODBUS_ADRESS	1

modbus_rtu_plazma[2]=crc16_calculated;
modbus_rtu_plazma[3]=crc16_incapsulated;

if((crc16_calculated==crc16_incapsulated) ||(modbus_func==0x10))
	{
	
	if(modbus_an_buffer[0]==ADR)
		{
		 

		if(modbus_func==1)		//чтение произвольного кол-ва битов хранения
			{
			unsigned short crc_temp;
			char i;
			//modbus_hold_coils_transmit(ADR,modbus_func,modbus_rx_arg0,modbus_rx_arg1,0);
			if((modbus_rx_arg0==11)&&(modbus_rx_arg1==1))
				{
				modbus_tx_buff[0]=ADR;
				modbus_tx_buff[1]=modbus_func;
				modbus_tx_buff[2]=1;
				modbus_tx_buff[3]=0;
				if(wrk_state==wrkON)
				modbus_tx_buff[3]=1;
				
				crc_temp=CRC16_2(modbus_tx_buff,4);
			
				modbus_tx_buff[4]=(char)crc_temp;
				modbus_tx_buff[5]=crc_temp>>8;
			
				SET_REG(PINSEL1,0,(22-16)*2,2); //Вход EN у 485
				IO0DIR|=(1<<22);
				IO0SET|=(1<<22);
			
				for (i=0;i<6;i++)
					{
					putchar1(modbus_tx_buff[i]);
					}
			
				}
			}

		else if(modbus_func==5)		//чтение произвольного кол-ва битов хранения
			{
			unsigned short crc_temp;
			char i;
			//modbus_hold_coils_transmit(ADR,modbus_func,modbus_rx_arg0,modbus_rx_arg1,0);
			
			if((modbus_rx_arg0==11))
				{
				if((modbus_rx_arg1==0xff00)&&(wrk_state!=wrkON))
					{
					
					wrk_state=wrkON;
					pwmI_start=POWER*9;
					wrk_cnt_cnt=(signed long)T_WRK_MAX*300L;
					lc640_write_int(EE_WRK_STAT,wrk_state);
					ist=istT;
					t_u_min1=-10;
					t_u_min2=-10;
					}
				else if((modbus_rx_arg1==0)&&(wrk_state!=wrkOFF))
					{
					wrk_state=wrkOFF;
					lc640_write_int(EE_WRK_STAT,wrk_state);
					}


				modbus_tx_buff[0]=ADR;
				modbus_tx_buff[1]=modbus_func;
				modbus_tx_buff[2]=0;
				modbus_tx_buff[3]=11;
				modbus_tx_buff[4]=0;
				modbus_tx_buff[5]=modbus_rx_arg1;
				
				crc_temp=CRC16_2(modbus_tx_buff,6);
				
				modbus_tx_buff[6]=(char)crc_temp;
				modbus_tx_buff[7]=crc_temp>>8;
			
				SET_REG(PINSEL1,0,(22-16)*2,2); //Вход EN у 485
				IO0DIR|=(1<<22);
				IO0SET|=(1<<22);
			
				for (i=0;i<8;i++)
					{
					putchar1(modbus_tx_buff[i]);
					}
			
				}
			}

		else if(modbus_func==3)		//чтение произвольного кол-ва регистров хранения
			{
			modbus_plazma++;
			modbus_hold_registers_transmit(ADR,modbus_func,modbus_rx_arg0,modbus_rx_arg1,0);
			}

		else if(modbus_func==4)		//чтение произвольного кол-ва регистров	входов
			{
//			modbus_input_registers_transmit(ADR,modbus_func,modbus_rx_arg0,modbus_rx_arg1,0);
			}

		else if(modbus_func==6) 	//запись регистра хранения
			{
			unsigned short crc_temp;
			char i;
			if(modbus_rx_arg0==15)		//ток максимальный
				{
				if((modbus_rx_arg1>=10)&&(modbus_rx_arg1<=1000))
  				lc640_write_int(EE_I_MAX,modbus_rx_arg1);
				}
			if(modbus_rx_arg0==17)		//напряжение максимальное
				{
				if((modbus_rx_arg1>=10)&&(modbus_rx_arg1<=7000))
				//gran(&U_MAX,100,7000);
	     		lc640_write_int(EE_U_MAX,modbus_rx_arg1); 
				}
				
			modbus_tx_buff[0]=ADR;
			modbus_tx_buff[1]=modbus_func;
			modbus_tx_buff[2]=*((char*)&modbus_rx_arg0);
			modbus_tx_buff[3]=*(((char*)&modbus_rx_arg0)+1);
			modbus_tx_buff[4]=*((char*)&modbus_rx_arg1);
			modbus_tx_buff[5]=*(((char*)&modbus_rx_arg1)+1);
			
			crc_temp=CRC16_2(modbus_tx_buff,6);
			
			modbus_tx_buff[6]=(char)crc_temp;
			modbus_tx_buff[7]=crc_temp>>8;
		
			SET_REG(PINSEL1,0,(22-16)*2,2); //Вход EN у 485
			IO0DIR|=(1<<22);
			IO0SET|=(1<<22);
		
			for (i=0;i<8;i++)
				{
				putchar1(modbus_tx_buff[i]);
				}
			}

		else if(modbus_func==15)		//запись произвольного кол-ва битов хранения
			{
			unsigned short crc_temp;
			char i;
			unsigned short modbus_rx_arg[4];
			//modbus_hold_coils_transmit(ADR,modbus_func,modbus_rx_arg0,modbus_rx_arg1,0);
			
			modbus_rx_arg[0]=((unsigned short)modbus_an_buffer[7]);//(((unsigned short)modbus_an_buffer[7])*((unsigned short)256))+((unsigned short)modbus_an_buffer[8]);

			//modbus_rtu_plazma[0]=modbus_rx_arg[0];

			if((modbus_rx_arg0==11)&&(modbus_rx_arg1==1))
				{
				if((modbus_rx_arg[0]==0x0001)&&(wrk_state!=wrkON))
					{
					
					wrk_state=wrkON;
					pwmI_start=POWER*9;
					wrk_cnt_cnt=(signed long)T_WRK_MAX*300L;
					lc640_write_int(EE_WRK_STAT,wrk_state);
					ist=istT;
					t_u_min1=-10;
					t_u_min2=-10;
					}
				else if((modbus_rx_arg[0]==0)&&(wrk_state!=wrkOFF))
					{
					wrk_state=wrkOFF;
					lc640_write_int(EE_WRK_STAT,wrk_state);
					}


				modbus_tx_buff[0]=ADR;
				modbus_tx_buff[1]=modbus_func;
				modbus_tx_buff[2]=0;
				modbus_tx_buff[3]=11;
				modbus_tx_buff[4]=0;
				modbus_tx_buff[5]=modbus_rx_arg1;
				
				crc_temp=CRC16_2(modbus_tx_buff,6);
				
				modbus_tx_buff[6]=(char)crc_temp;
				modbus_tx_buff[7]=crc_temp>>8;
			
				SET_REG(PINSEL1,0,(22-16)*2,2); //Вход EN у 485
				IO0DIR|=(1<<22);
				IO0SET|=(1<<22);
			
				for (i=0;i<8;i++)
					{
					putchar1(modbus_tx_buff[i]);
					}
			
				}
			}

		else if(modbus_func==16) 	//запись регистра хранения
			{
			unsigned short crc_temp;
			char i;
			unsigned short modbus_rx_arg[4];
//modbus_rtu_plazma[2]++;

			modbus_rx_arg[0]=(((unsigned short)modbus_an_buffer[7])*((unsigned short)256))+((unsigned short)modbus_an_buffer[8]);
			modbus_rx_arg[1]=(((unsigned short)modbus_an_buffer[11])*((unsigned short)256))+((unsigned short)modbus_an_buffer[12]);

			if(modbus_rx_arg0==15) 		
				{
				if(modbus_rx_arg1>=1)
					{
					if((modbus_rx_arg[0]>=10)&&(modbus_rx_arg[0]<=1000))
  					lc640_write_int(EE_I_MAX,modbus_rx_arg[0]);
					}
				if(modbus_rx_arg1>=2)
					{
					if((modbus_rx_arg[1]>=10)&&(modbus_rx_arg[1]<=7000))
	     			lc640_write_int(EE_U_MAX,modbus_rx_arg[1]); 
					}
				}
			if(modbus_rx_arg0==17)		//напряжение максимальное
				{
				if(modbus_rx_arg1>=1)
					{
					if((modbus_rx_arg[0]>=10)&&(modbus_rx_arg[0]<=7000))
	     			lc640_write_int(EE_U_MAX,modbus_rx_arg[0]); 
					}
				}
				
			modbus_tx_buff[0]=ADR;
			modbus_tx_buff[1]=modbus_func;
			modbus_tx_buff[2]=*(((char*)&modbus_rx_arg0)+1);
			modbus_tx_buff[3]=*((char*)&modbus_rx_arg0);
			modbus_tx_buff[4]=*(((char*)&modbus_rx_arg1)+1);
			modbus_tx_buff[5]=*((char*)&modbus_rx_arg1);
			
			crc_temp=CRC16_2(modbus_tx_buff,6);
			
			modbus_tx_buff[6]=(char)crc_temp;
			modbus_tx_buff[7]=crc_temp>>8;
		
			SET_REG(PINSEL1,0,(22-16)*2,2); //Вход EN у 485
			IO0DIR|=(1<<22);
			IO0SET|=(1<<22);
		
			for (i=0;i<8;i++)
				{
				putchar1(modbus_tx_buff[i]);
				}
			}
		} 
	}
}
 
//-----------------------------------------------
char index_offset1 (signed char index,signed char offset)
{
index=index+offset;
if(index>=RX_BUFFER_SIZE1) index-=RX_BUFFER_SIZE1; 
if(index<0) index+=RX_BUFFER_SIZE1;
return index;
}

//-----------------------------------------------
char control_check1(char index)
{
char i=0,ii=0,iii;

//if(rx_buffer1[index]!=END) goto error_cc;

ii=rx_buffer1[index_offset1(index,-2)];
iii=0;
for(i=0;i<=ii;i++)
	{
	iii^=rx_buffer1[index_offset1(index,-2-ii+i)];
	}
if (iii!=rx_buffer1[index_offset1(index,-1)]) goto error_cc;	


//success_cc:
return 1;
//goto end_cc;
error_cc:
return 0;
//goto end_cc;

//end_cc:
//__nop();
}




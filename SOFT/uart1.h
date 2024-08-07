#define RX_BUFFER_SIZE1 64
#define TX_BUFFER_SIZE1 100

#define PT1	(UIB1[1]&0x80)
#define C_D1	(UIB1[1]&0x40)
#define FR1	(UIB1[1]&0x20)

#define VIC_UART1 	7

extern char bUART1TX;
extern char bRXIN1;
extern char UIB1[20];
extern char flag1;
extern char rx_buffer1[RX_BUFFER_SIZE1];
extern char tx_buffer1[TX_BUFFER_SIZE1];
extern unsigned char rx_wr_index1,rx_rd_index1,rx_counter1;
extern unsigned char tx_wr_index1,tx_rd_index1,tx_counter1;
extern char rx_buffer_overflow1;
extern char plazma_uart1;
extern char uart1_mess[10];
extern char data_rs1[40];
typedef enum {ursMEGA=0x55,ursXPORT=0xaa}enum_usart1_router_stat;
extern enum_usart1_router_stat usart1_router_stat;
extern char usart1_router_wrk;
extern char memo_out0[100];
extern char suzz[4];
extern char UIB10[30];
extern char usart1_router_cnt;
extern char _485_last_cnt;
extern char UIB[30];
extern char tx_wd_cnt;
extern short plazma_ppp;

extern short modbus_rtu_plazma[5];

extern char modbus_timeout_cnt;
extern char bMODBUS_TIMEOUT;

void putchar1(char c);
void uart_out1 (char num,char data0,char data1,char data2,char data3,char data4,char data5);
void uart_out_adr1 (char *ptr, char len);
void uart1_init(void);
char getchar1(void);
__irq void uart1_interrupt(void);
void uart_in_an1(void);
char index_offset1 (signed char index,signed char offset);
char control_check1(char index);
void uart_in1(void);


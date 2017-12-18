#include <stdio.h>
#include <msp430.h> 
#include "math.h"
#include <intrinsics.h>
#include <string.h>
#include <stdlib.h>
#include "driverlib.h"
//====LCD====//
#include "grlib.h"
#include "HAL_MSP430F5529_Sharp96x96.h"
#include "Sharp96x96.h"
//====MSP430电源管理====//
#include "HAL_UCS.h"
#include "HAL_PMM.h"
//====类型定义====//
#define u8 unsigned char
#define s8 char
#define u16 unsigned int
#define s16 int
#define u32 unsigned long
#define s32 long
#define CR 0x0D
/**********全局变量***********/
//====计算队列长度====//
#define MAX_LENGTH 	128
//====窗口队列长度====//
#define WIN_LENGTH	32
//====IIR队列长度====//
#define IIR_NSEC			3
//====IIR滤波器结构体====//
typedef struct {
	int x1;	//x(n+1)
	int x0;	//x(n)
	int y1;	//y(n+1)
	int y2;	//y(n+2)
}IROLD;
typedef struct {
	int x1;	//x(n+1)
	int x0;	//x(n)
	int y1;	//y(n+1)
	int y2;	//y(n+2)
}RDOLD;
//====IIR FILTER 参数====//
const double IR_B[IIR_NSEC] = { 0.003621 ,  0.007243,  0.003621  };
const double IR_A[IIR_NSEC] = { 1        		  , -1.822694, 	0.837181  };
const double RD_B[IIR_NSEC] = { 0.003621 ,  0.007243,  0.003621 };
const double RD_A[IIR_NSEC] = { 1        		  , -1.822694, 	0.837181  };
//================//
IROLD IR_Old;
RDOLD RD_Old;
//====脉搏血氧信号====//
s16 IR_Signal=0;							// 红外光，滤波后进入计算窗口
s16 RD_Signal=0;						// 红光   ，滤波后进入计算窗口
//====血氧计算变量====//
double x=0;
double y=0;
double SUM_RD_Diff=0;	// SUM_X
double SUM_IR_Diff=0;	// SUM_Y
double SUM_X2=0;
double SUM_XY=0;
double NUM1=0;
double NUM2=0;
double DEN1=0;
double DEN2=0;
int R;

//====计算队列====//
long RD_group[MAX_LENGTH] ;					//用于显示，循环队列，存储几个周期内的信号
long IR_group[MAX_LENGTH];
long Diff_RD_group[MAX_LENGTH];
long Diff_IR_group[MAX_LENGTH];
long moving_window[WIN_LENGTH];			//循环队列，以滑动窗口的形式判断当前是否为脉搏波波谷
u8 RD_offset_wave = 0;
u8 IR_offset_wave = 0;
u8 window_offset_wave = 0;						//队列头位置
long min;													//队列中最小值
u8 location_min;				 							//最小值位置
u8 location_min_adjust;								//最小值相对与队列头的位置，如果是32则确认一个波谷

u8 TIR=0;
u8 TRD=0;
u8 ircurrentlocal;
u8 irlastlocal;
u8 rdcurrentlocal;
u8 rdlastlocal;

//====各种标记位====//
u8 read_tab = 0;					//可读标志
u8 flag_initial = 1;				//程序初始化标志位
u8 num_beat = 1;				//初始值为1，下一个脉搏后为2，计算脉搏周期，重新置1
u8 flag_jump = 0;				//脉搏波周期判断，是否处于离开波谷的状态
u8 sample_jump = 0;			//离开波谷时的采样计数，到20则已离开波谷，置0 ，flag_jump 置1
int sample_count = 0;			//采样计数，每个周期清空，重新计数
int beats=0;						//脉率计数标记
int in=0;							//中断进入标记
int s1=0;							//脉率跳变修正标记
u8 figner=1;						//检测手指是否放入标
u8 key2=2;
u32 key1=0;
u32 key3=0;
u8 key4=2;
int cnt=0;
int line=0;
//===============//
//====脉率结果====//
//int heartdiff=0;												//两个前后脉率差分值
unsigned int heart_rate = 70;//心率
unsigned int group_heart_rate[8]={7000,7000,7000,7000,7000,7000,7000,7000};
unsigned int sample_heart_rate=7000;
int32_t sum_heart_rate=56000;
int 	  offset_heart_rate=0;
//==============//
//====血氧结果====//
int SpO2 = 97;												//血氧饱和度最终测量结果，初始值为97.00
unsigned int group_SpO2[8]={9700,9700,9700,9700,9700,9700,9700,9700};			//最近8秒内的血氧饱和度，循环队列，初始化为9500
int32_t sum_SpO2=77600;
int offset_SpO2 = 0;										//队列头
//==============//
//=====SYSTEM=====//
void CLK_Init(void);
void Set_SMCLK8MHZ(void);
void UART_Init(void);
//================//
//====CALCULATE====//
int IR_Filter(int input);
void IR_reset(void);
int RD_Filter(int input);
void RD_reset(void);
long  mean(long* group, u8 c, u8 l);
long diff(long *group, int j);
//===============//
//====== KEY======//
void KEY_Init(void);
void Timer2_Init(void);
//===============//
//====== LED ======//
void LedDriver_Init(void);
void RD_OFF(void);
void IR_OFF(void);
void RD_ON(void);
void IR_ON(void);
//================//
//====TA CAPTURE====//
void Timer0_Init(void);
//====TA COMPARE===//
void Timer1_Init(void);
void Timer2_Init(void);
//================//
//====BLUETOOTH====//
void BLE_Init(void);
u8    BLE_CHECK(void);
void BLE_ON(void);
void BLE_OFF(void);
void SEND_Init(void);
unsigned char txString[13]={0};
unsigned char tx[27]={0};
//================//
//=====BUZZER======//
void BEEP(u8 cmd);
//================//
//====DELAY LOOP====//
//#define CPUCLK  32768					// 32.768KHZ---一个周期0.030ms
#define CPUCLK  			25000000		// 8MHZ       ---一个周期0.125us
#define	 delay_us(us)		__delay_cycles((long)(CPUCLK*(double)us/1000000.0))
#define delay_ms(ms)	__delay_cycles((long)(CPUCLK*(double)ms/1000.0))
//================================================//
//===== DISPLAY =====//
#define _LED			BIT0
#define _SCLK		BIT2					// SPI clock
#define _MOSI		BIT0					// SPI data (sent to display)
#define _CS			BIT6					// SPI chip select
#define _DISP		BIT6					// Turn display on/off
Graphics_Context g_sContext;
Graphics_Rectangle myRectangle1 = { 42, 12, 95, 47};
Graphics_Rectangle myRectangle2 = { 35, 47, 95, 95};
Graphics_Rectangle myRectangle3 = { 0, 95, 12, 95};
Graphics_Rectangle myRectangle4 = { 13, 95, 95, 95};
Graphics_Rectangle myRectangle5 = { 39, 39, 85, 85};
Graphics_Rectangle myRectangle6 = { 0, 60, 32, 95};
Graphics_Rectangle arrow0 = { 2, 25, 12, 86 };
Graphics_Rectangle arrow1 = { 2, 49, 12, 86 };
Graphics_Rectangle arrow2 = { 2, 0, 12, 25 };
Graphics_Rectangle arrow3 = { 2, 70, 12, 86 };
Graphics_Rectangle arrow4 = { 2, 0, 12, 49 };
Graphics_Rectangle arrow5 = { 2, 0, 12, 70 };
void Timer2_Init(void);
void choose(void);
void choosePPG0(void);
void chooseBLE0(void);
void chooseALARM0(void);
void chooseCANCEL0(void);
void ARROW(u8 cmd);


//====NUM2STR====//
void SpO2str(int  num);
void BPM2str(int num);
void ALARM2str(int num);
void PI2str(int num);
int8_t  spo2[3];
int8_t  bpm[4];
int BPM=80;
int8_t alarm[3];
//==============//
unsigned int temp=0;
long sss=0;
long ssr=0;
int rd;
int ir;

int value=90;
int setvalue=90;
int A=0;
int i=0;
int j=0;
int pulse1[96]={0};
int pulse2[96]={0};

u16  IR_avg=0;
u16  RD_avg=0;
int KRD=0;
int KIR=0;
int KBPM=0;
int XBPM=0;
int YBPM=0;
unsigned char pi[27];
//====可管理标记====//
u8 update=0;													//读取数据后，更新标志位
u8 display=7;
u8 first=0;
u8 SEND_FLAG=0;
u8 PPG_FLAG=0;
u8 sure=0;
void FLAGRST(void);

long RD_send[256] ;					//用于显示，循环队列，存储几个周期内的信号
long IR_send[256];
int send_wave1=0;
int send_wave2=0;
u8 Send=1;

int acIR;
int acRD;
float sum_PI=44.0f;
float PITEMP=5.5f;
float PI[8]={5.5f,5.5f,5.5f,5.5f,5.5f,5.5f,5.5f,5.5f};
u8 PI_wave=0;
int8_t P[3];
int P1=0;
int P2=0;
u8 p=0;

void main(void) {
	WDTCTL = WDTPW + WDTHOLD;       //Stop watchdog timer
	CLK_Init();											// 25MHz
    UART_Init();										// USART-0 , 115200
	IR_reset();											//重置滤波器
	RD_reset();
	SEND_Init();
	LedDriver_Init();
    KEY_Init();
    HAL_LCD_initDisplay();
    GrContextInit(&g_sContext, &g_sharp96x96LCD);
    GrContextForegroundSet(&g_sContext,  ClrBlack);
    GrContextBackgroundSet(&g_sContext, ClrWhite);
    GrContextFontSet(&g_sContext, &g_sFontCm12);
    GrClearDisplay(&g_sContext);
	GrStringDraw(&g_sContext, "SPO2", AUTO_STRING_LENGTH, 2, 0, TRANSPARENT_TEXT	);
	GrStringDraw(&g_sContext, "BPM", AUTO_STRING_LENGTH, 2, 47, TRANSPARENT_TEXT);
	GrFlush(&g_sContext);
    delay_ms(100);
    HAL_LCD_enableDisplay();
    BLE_Init();
    BLE_OFF();
    Timer0_Init();
  	Timer1_Init();
	_EINT();
	while(1){
 		if(update){
 			update=0;
			if((display==0)){
				Graphics_setForegroundColor(&g_sContext, ClrWhite );
				Graphics_fillRectangle(&g_sContext, &myRectangle1);
				Graphics_fillRectangle(&g_sContext, &myRectangle2);
				Graphics_setForegroundColor(&g_sContext, ClrBlack );
				GrContextFontSet(&g_sContext, &g_sFontCm42);
				if(SpO2<=value){
					A++;
					if(A>2)	BEEP(1);
				}
				SpO2str(SpO2);
				GrStringDraw(&g_sContext, spo2, AUTO_STRING_LENGTH, 52, 12, TRANSPARENT_TEXT);
				if(p){
					PITEMP+=9.0f;
					sum_PI -= PI[PI_wave];
					PI[PI_wave] =PITEMP ;
					sum_PI +=  PI[PI_wave];
					PI_wave=(PI_wave+1) & 0x07;
					PITEMP=sum_PI/8;
					if(PITEMP<=0.0f){
					}
					else if(PITEMP>=10.0f){
						P1=((int)(PITEMP))/10;
						P2=(int)(PITEMP-P1*10);
						PI2str( P1 );
						GrStringDraw(&g_sContext, P, AUTO_STRING_LENGTH, 54 , 56, TRANSPARENT_TEXT);
						PI2str( P2 );
						GrStringDraw(&g_sContext, P, AUTO_STRING_LENGTH, 70 , 56, TRANSPARENT_TEXT);
					}
					else{
						P1=(int)PITEMP;
						P2=(int)(PITEMP*10-P1*10);
						PI2str( P1 );
						GrStringDraw(&g_sContext, P, AUTO_STRING_LENGTH, 44 , 56, TRANSPARENT_TEXT);
						GrStringDraw(&g_sContext, ".", AUTO_STRING_LENGTH, 62 , 56, TRANSPARENT_TEXT);
						PI2str( P2 );
						GrStringDraw(&g_sContext, P, AUTO_STRING_LENGTH, 68 , 56, TRANSPARENT_TEXT);
					}
				}
				else{
					BPM2str( heart_rate );
					GrStringDraw(&g_sContext, bpm, AUTO_STRING_LENGTH, 36 , 56, TRANSPARENT_TEXT);
				}
				GrFlush(&g_sContext);
			}
			if(PPG_FLAG){
				if(cnt>=3){
					cnt=0;
					j++;
					if(j>95)	j=1;
					Graphics_setForegroundColor(&g_sContext, ClrBlack );
					Graphics_setBackgroundColor(&g_sContext,ClrWhite);
					acIR=IR_group[IR_offset_wave] -IR_avg;
					acRD=RD_group[RD_offset_wave] -RD_avg;
					pulse2[j-1] =acIR/20+29;// KIR;
					pulse1[j-1] = acRD/20+78;// KRD;

					if((pulse2[j-1]<47)&&(pulse2[j-1]>11))
						Graphics_drawLineV(&g_sContext, j-1, pulse2[j-1], 46);
					else if(pulse2[j-1]<=11)
						Graphics_drawLineV(&g_sContext, j-1,  12, 46);
					else
						Graphics_drawPixel(&g_sContext, j-1, 46);

					if((pulse1[j-1]>59)&&(pulse1[j-1]<95))
						Graphics_drawLineV(&g_sContext, j-1, pulse1[j-1], 95);
					else if(pulse1[j-1]<=59)
						Graphics_drawLineV(&g_sContext, j-1,  59, 95);
					else
						Graphics_drawPixel(&g_sContext, j-1, 95);

					GrContextForegroundSet(&g_sContext,  ClrWhite);
					Graphics_drawLineV(&g_sContext, j, 12, 46);
					Graphics_drawLineV(&g_sContext, j, 59, 95);
					GrFlush(&g_sContext);
				}
			}
		}
	}
}


#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void) {
	// 读红光RD
	if(read_tab==0){
		read_tab=1;
		IR_OFF();
		RD_ON();
		sss=12500000/temp;
		IR_group[IR_offset_wave]= IR_Filter(sss);
		IR_send[send_wave1]=sss;
		Diff_IR_group[IR_offset_wave] = diff( IR_group , IR_offset_wave);
		if(SEND_FLAG==1){
			if(Send==1){
				txString[1] = (unsigned char)(send_wave1 & 0x000000FF);
				txString[2] = (unsigned char)((send_wave1 & 0x0000FF00)>>8);
			//	txString[1] = (unsigned char)(IR_offset_wave & 0x000000FF);
			//	txString[2] = (unsigned char)((IR_offset_wave & 0x0000FF00)>>8);
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[0];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[1];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[2];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[8];
				Send=2;
			}
			else if(Send==2){
				txString[3] = (unsigned char)(RD_send[send_wave2] & 0x000000FF);
				txString[4] = (unsigned char)((RD_send[send_wave2] & 0x0000FF00) >> 8);
				send_wave2 = (send_wave2 + 1) & 0xff;
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[3];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[4];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[5];
				Send=3;
			}

		}
		if(TIR){
			TIR=0;
			IR_avg = mean(IR_group,ircurrentlocal,irlastlocal);
			ircurrentlocal = irlastlocal;
		}
		cnt ++;
		if(PPG_FLAG){
			if(IR_offset_wave==0){
				IR_avg = mean(IR_group,ircurrentlocal,irlastlocal);
			}
		}
		// 更新队列

		IR_offset_wave = (IR_offset_wave + 1) & 0x7f;

		/*
		 * 初始值为1，下一个脉搏后为2，计算平均功率和血氧饱和度，重新置1
		 * 是否处于找到波谷的状态（num_beat由1变为2，找到，否则未找到）
		 * 未找到则计算两路平方和，采样计数累加，
		 * 找到则计算脉搏血氧饱和度和脉率，平方和、采样计数置0
		 */
		if((num_beat >= 1)) {
			sample_count++;					//采样计数累加
		 	x  = ((RD_group[RD_offset_wave]>>8) * (Diff_IR_group[IR_offset_wave]));
		 	y =((IR_group[IR_offset_wave]>>8) * (Diff_RD_group[RD_offset_wave]));
		 	SUM_RD_Diff += (x);	// SUM_X
		 	SUM_IR_Diff += (y);	// SUM_Y
		 	SUM_X2 += ((x * x));
		 	SUM_XY += ((x * y));
		 }
		 if ((num_beat >= 2)) {
			NUM1 = ((sample_count * SUM_XY)) ;
		 	NUM2 =((SUM_RD_Diff * SUM_IR_Diff));
		 	DEN1  =(( sample_count * SUM_X2))  ;
		 	DEN2  = ((SUM_RD_Diff * SUM_RD_Diff));
		 	R = (100*(NUM1-NUM2))  /  ( DEN1-DEN2);

		 	if(!(flag_initial)){
		 		sum_SpO2 -= group_SpO2[offset_SpO2];		//8秒内血氧饱和度之和减去8秒前的值
		 		group_SpO2[offset_SpO2] = 11200 - 25 * R ;	//计算当先新的脉搏血氧饱和度，拟合公式110-25×R，R为平均功率之比
		 	//	group_SpO2[offset_SpO2] =group_SpO2[offset_SpO2] ;
		 		//限幅正负300，(offset_SpO2-1)取低3位，使小于8，再加300
		 		//调整新的脉搏血氧饱和度，变化不能超过3个百分点，范围在85到100之间，文献说明
		 		if (group_SpO2[offset_SpO2]>(group_SpO2[(offset_SpO2 - 1) & 0x07] + 300))
		 			group_SpO2[offset_SpO2] = group_SpO2[(offset_SpO2 - 1) & 0x07] + 300;
		 		else if (group_SpO2[offset_SpO2]<(group_SpO2[(offset_SpO2 - 1) & 0x07] - 300))
		 			group_SpO2[offset_SpO2] = group_SpO2[(offset_SpO2 - 1) & 0x07] - 300;
		 		else;
		 		if(group_SpO2[offset_SpO2]>=10000)		group_SpO2[offset_SpO2]=9900;
		 	//	group_SpO2[offset_SpO2] =group_SpO2[offset_SpO2] / 100;
		 		//8秒内血氧饱和度之和加上当前的值
		 		sum_SpO2 += group_SpO2[offset_SpO2];
		 		offset_SpO2 = (offset_SpO2 + 1) & 0x07;
		 		//计算平均值，得到最终结果
		 		SpO2 = sum_SpO2 / 800;
		 	}
		 	//回复初始状态
		 	SUM_RD_Diff = 0;	// SUM_X
		 	SUM_IR_Diff = 0;	// SUM_Y
		 	SUM_X2 = 0;
		 	SUM_XY = 0;

		 	sample_heart_rate = 600000 / sample_count;
		 	num_beat = 1;
		 	sample_count = 0;
		 	if(sample_heart_rate<1000||sample_heart_rate>20000);
		 	else{
		 		sum_heart_rate -= group_heart_rate[offset_heart_rate];
		 	    group_heart_rate[offset_heart_rate] = sample_heart_rate;
		 	    sum_heart_rate += group_heart_rate[offset_heart_rate];
		 	    offset_heart_rate = (offset_heart_rate+1) & 0x07;
		 	    heart_rate=sum_heart_rate/800;
		 	}
	 	 	if(display==0){
	 	 		update=1;
	 	 	}
		 }
	}
	else{
		read_tab=0;
		RD_OFF();
		IR_ON();
		ssr=12500000/temp;
		RD_group[RD_offset_wave] = RD_Filter(ssr);
		RD_send[send_wave2]= ssr;
		moving_window[window_offset_wave] = RD_group[RD_offset_wave];
		window_offset_wave = (window_offset_wave + 1) & 0x1f ;
		Diff_RD_group[RD_offset_wave] = diff( RD_group, RD_offset_wave);

		if(SEND_FLAG==1){
			if(Send==3){
				txString[6] = (unsigned char)(IR_send[send_wave1]  & 0x000000FF);
				txString[7] = (unsigned char)((IR_send[send_wave1] & 0x0000FF00) >> 8);
				send_wave1 = (send_wave1 + 1) & 0xff;
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[6];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[7];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[9];
				Send=4;
			}
			else if(Send==4){

				txString[11] = (unsigned char)(SpO2);
				txString[10]=(unsigned char)(heart_rate);
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[10];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[11];
				while (!(UCA0IFG&UCTXIFG));             	// USCI_A0 TX buffer ready?);
				UCA0TXBUF = txString[12];

				Send=1;
			}

		}
		if(TRD){
			TRD=0;
			RD_avg = mean(RD_group,rdcurrentlocal,rdlastlocal);
			rdcurrentlocal = rdlastlocal;
		}
		if(display==0){
			acRD = RD_group[RD_offset_wave]- RD_avg;
			PITEMP = (float)(acRD*100.0f / RD_avg);
		}
		if(PPG_FLAG){
			if(RD_offset_wave==0){
				RD_avg = mean(RD_group,rdcurrentlocal,rdlastlocal);
			}
			update=1;
		}
		// 队列更新

		RD_offset_wave = (RD_offset_wave + 1) & 0x7f;
		if (flag_initial == 1) {
			if (RD_offset_wave >= MAX_LENGTH-1) {
				flag_initial = 0;
				//SEND_FLAG=1;
				display=0;
			}
		}
		else{
			// 脉搏周期判断
			if (flag_jump == 0) {		 // flag_jump==0，表示处在寻找波谷状态
				sample_jump = 0;		 // 离开波谷时的采样计数置0
				min = moving_window[0]; //寻找group_caculate [64]循环队列中的最小值及其位置
				location_min = 0;
				for (i = 1; i<WIN_LENGTH; i++) {
					if (min>moving_window[i]) {
						min = moving_window[i];
						location_min = i;
					}
				}
				//计算最小值位置距离队列头距离
				if (location_min <= window_offset_wave) {
					location_min_adjust = window_offset_wave - location_min;
				}
				else {
					location_min_adjust = window_offset_wave + WIN_LENGTH - location_min;
				}
				//最小值是否在队列正中
				if (location_min_adjust == (WIN_LENGTH/2) || location_min_adjust == ((WIN_LENGTH/2)+1) ) {
					flag_jump = 1;	//如果是，找到波谷，进入离开波谷状态
					num_beat++;		//脉搏计数增加，如果是程序第一次找到，则由0到1，以后则总是由1到2
					P1OUT ^= BIT0;
					irlastlocal = IR_offset_wave ;
					rdlastlocal = RD_offset_wave ;
					TRD=1;
					TIR=1;
				}
			}
			else {
				sample_jump++;// 离开波谷时的采样计数
				if (sample_jump >=35) {	// 离开波谷时的采样计数到达20
					flag_jump = 0;//认为已离开波谷，则重新寻找下一个波谷
				}
			}
		}

	}
}



#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void) {
	int t=0;
	switch (P1IV) {
	case P1IV_P1IFG1:
		delay_ms(10);
		while(!(P1IN&BIT1)) {
			key3++;
			if(key3==1638400){
				key4=1;
				break;
			}
			else
				key4=0;
		}
		key3=0;
		if(key4==0){
			//BEEP(1);
			if(sure&&(display!=5))	{
				GrContextForegroundSet(&g_sContext,  ClrBlack);
				GrContextBackgroundSet(&g_sContext, ClrWhite);
				GrContextFontSet(&g_sContext, &g_sFontCm12);
				GrClearDisplay(&g_sContext);
				FLAGRST();
				display=1;
				choose();
				ARROW(1);
				GrFlush(&g_sContext);
				sure=0;
			}
			GrContextForegroundSet(&g_sContext,  ClrBlack);
			GrContextBackgroundSet(&g_sContext, ClrWhite);
			GrContextFontSet(&g_sContext, &g_sFontCm12);
			key4=2;
			if(display==0){
				display=4;
				GrClearDisplay(&g_sContext);
			}
			else if(display==4){
				display=1;
				choose();
				ARROW(1);
				GrFlush(&g_sContext);
			}
			else if(display==1){
				display=2;
				choose();
				ARROW(2);
				GrFlush(&g_sContext);
			}
			else if(display==2){
				display=3;
				choose();
				ARROW(3);
				GrFlush(&g_sContext);
			}
			else if(display==3){
				display=4;
				choose();
				ARROW(4);
				GrFlush(&g_sContext);
			}
			else if(display==5){
				setvalue++;
				if(setvalue>99)	setvalue=99;
				ALARM2str(setvalue);
				GrContextBackgroundSet(&g_sContext, ClrWhite);
				GrContextForegroundSet(&g_sContext,  ClrWhite);
			    Graphics_fillRectangle(&g_sContext, &myRectangle5);
			    GrContextForegroundSet(&g_sContext,  ClrBlack);
				GrContextFontSet(&g_sContext, &g_sFontCm42);
				GrStringDraw(&g_sContext , alarm,  AUTO_STRING_LENGTH, 40 , 40, TRANSPARENT_TEXT);
				GrFlush(&g_sContext);
			}
		}
			else	if(key4==1){
				key4=2;
				sure=1;
				switch(display){
				case 0:
					p=1;
					FLAGRST();
					GrClearDisplay(&g_sContext);
					GrContextForegroundSet(&g_sContext,  ClrBlack);
					GrContextBackgroundSet(&g_sContext, ClrWhite);
					GrContextFontSet(&g_sContext, &g_sFontCm12);
					GrClearDisplay(&g_sContext);
					GrStringDraw(&g_sContext, "SPO2", AUTO_STRING_LENGTH, 2, 0, TRANSPARENT_TEXT);
					GrStringDraw(&g_sContext, "PI %", AUTO_STRING_LENGTH, 2, 47, TRANSPARENT_TEXT);
					GrFlush(&g_sContext);
					break;
				case 1://PPGs
					GrContextForegroundSet(&g_sContext,  ClrBlack);
					GrContextBackgroundSet(&g_sContext, ClrWhite);
					GrContextFontSet(&g_sContext, &g_sFontCm12);
					GrClearDisplay(&g_sContext);
					PPG_FLAG=1;
					GrStringDraw(&g_sContext, " IR Signal", AUTO_STRING_LENGTH, 2, 0, TRANSPARENT_TEXT	);
					GrStringDraw(&g_sContext, "RD Signal", AUTO_STRING_LENGTH, 2, 47, TRANSPARENT_TEXT);
					GrFlush(&g_sContext);
					break;
				case 2:
    				Graphics_setForegroundColor(&g_sContext, ClrBlack );
    				Graphics_setBackgroundColor(&g_sContext,ClrWhite);
    				GrClearDisplay(&g_sContext);
    				GrStringDraw(&g_sContext, "CONNECTING", AUTO_STRING_LENGTH, 6, 37, TRANSPARENT_TEXT);
    				GrFlush(&g_sContext);
    				while(BLE_CHECK()){
    					for(t=0; t<80; t++){
    						Graphics_drawLineV(&g_sContext,6+t, 50, 60);
							GrFlush(&g_sContext);
							delay_ms(1);
    					}
    					GrClearDisplay(&g_sContext);
						GrStringDraw(&g_sContext, "CONNECTING", AUTO_STRING_LENGTH, 6, 37, TRANSPARENT_TEXT);
						GrFlush(&g_sContext);
    				}
    				GrClearDisplay(&g_sContext);
    				GrStringDraw(&g_sContext, "CONNECTED", AUTO_STRING_LENGTH, 6, 49, TRANSPARENT_TEXT	);
    				GrFlush(&g_sContext);
    				BLE_ON();
    				SEND_FLAG=1;
					break;
				case 3:
				    GrContextForegroundSet(&g_sContext,  ClrBlack);
				    GrContextBackgroundSet(&g_sContext, ClrWhite);
					GrClearDisplay(&g_sContext);
					GrStringDraw(&g_sContext, "Alarm", AUTO_STRING_LENGTH, 2, 2, TRANSPARENT_TEXT);
					GrStringDraw(&g_sContext, "Value", AUTO_STRING_LENGTH, 2, 14, TRANSPARENT_TEXT);
					GrStringDraw(&g_sContext, "SET:", AUTO_STRING_LENGTH, 2, 47, TRANSPARENT_TEXT);
					display=5;
					ALARM2str(value);
					GrContextBackgroundSet(&g_sContext, ClrWhite);
					GrContextForegroundSet(&g_sContext,  ClrWhite);
				    Graphics_fillRectangle(&g_sContext, &myRectangle5);
				    GrContextForegroundSet(&g_sContext,  ClrBlack);
					GrContextFontSet(&g_sContext, &g_sFontCm42);
					GrStringDraw(&g_sContext , alarm,  AUTO_STRING_LENGTH, 40 , 40, TRANSPARENT_TEXT);
					GrFlush(&g_sContext);
					break;
				case 4:
					FLAGRST();
					GrClearDisplay(&g_sContext);
					GrContextForegroundSet(&g_sContext,  ClrBlack);
					GrContextBackgroundSet(&g_sContext, ClrWhite);
					GrContextFontSet(&g_sContext, &g_sFontCm12);
					GrClearDisplay(&g_sContext);
					GrStringDraw(&g_sContext, "SPO2", AUTO_STRING_LENGTH, 2, 0, TRANSPARENT_TEXT);
					GrStringDraw(&g_sContext, "BPM", AUTO_STRING_LENGTH, 2, 47, TRANSPARENT_TEXT);
					GrFlush(&g_sContext);
					break;
				case 5:
					value = setvalue;
					GrClearDisplay(&g_sContext);
					display=0;
					GrContextForegroundSet(&g_sContext,  ClrBlack);
					GrContextBackgroundSet(&g_sContext, ClrWhite);
					GrContextFontSet(&g_sContext, &g_sFontCm12);
					GrClearDisplay(&g_sContext);
					GrStringDraw(&g_sContext, "SPO2", AUTO_STRING_LENGTH, 2, 0, TRANSPARENT_TEXT);
					GrStringDraw(&g_sContext, "BPM", AUTO_STRING_LENGTH, 2, 47, TRANSPARENT_TEXT);
					GrFlush(&g_sContext);
				default:break;
				}
			}
		P1IFG &= ~BIT1;
		break;
	case P1IV_NONE:	 break;
	}
}

// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR  				// DRDY interrupt
__interrupt void Port_2(void) {
	switch (P2IV) {
	case P2IV_P2IFG1:
		delay_ms(10);
		while(!(P2IN&BIT1)){
			key1++;
			if(key1==1638400){
				key2=1;
				break;
			}
			else
				key2=0;
		}
		key1=0;
		if(key2==0){
			if(A>1){
				BEEP(0);
				A=0;
			}
			GrContextForegroundSet(&g_sContext,  ClrBlack);
			GrContextBackgroundSet(&g_sContext, ClrWhite);
			GrContextFontSet(&g_sContext, &g_sFontCm12);
			key2=2;
			if(display==0){
				display=4;
				GrClearDisplay(&g_sContext);
				choose();
				ARROW(4);
				GrFlush(&g_sContext);
			}
			else if(display==4){
				display=3;
				choose();
				ARROW(3);
				GrFlush(&g_sContext);
			}
			else if(display==3){
				display=2;
				choose();
				ARROW(2);
				GrFlush(&g_sContext);
			}
			else if(display==2){
				display=1;
				choose();
				ARROW(1);
				GrFlush(&g_sContext);
			}
			else if(display==1){
				display=4;
				choose();
				ARROW(4);
				GrFlush(&g_sContext);
			}
			else if(display==5){
				setvalue--;
				if(setvalue<0)	setvalue=0;
				ALARM2str(setvalue);
				GrContextBackgroundSet(&g_sContext, ClrWhite);
				GrContextForegroundSet(&g_sContext,  ClrWhite);
		        Graphics_fillRectangle(&g_sContext, &myRectangle5);
				GrContextForegroundSet(&g_sContext,  ClrBlack);
				GrContextFontSet(&g_sContext, &g_sFontCm42);
				GrStringDraw(&g_sContext , alarm,  AUTO_STRING_LENGTH, 40 , 40, TRANSPARENT_TEXT);
				GrFlush(&g_sContext);
			}
		}
		else if(key2==1){
			key2=2;
			FLAGRST();
			BLE_OFF();
			SEND_FLAG=0;
			p=0;
			GrContextForegroundSet(&g_sContext,  ClrBlack);
			GrContextBackgroundSet(&g_sContext, ClrWhite);
			GrContextFontSet(&g_sContext, &g_sFontCm12);
			 GrClearDisplay(&g_sContext);
			GrStringDraw(&g_sContext, "SPO2", AUTO_STRING_LENGTH, 2, 0, TRANSPARENT_TEXT	);
			GrStringDraw(&g_sContext, "BPM", AUTO_STRING_LENGTH, 2, 47, TRANSPARENT_TEXT);
			/*
			GrClearDisplay(&g_sContext);
		    GrContextForegroundSet(&g_sContext,  ClrBlack);
		    GrContextBackgroundSet(&g_sContext, ClrWhite);
		    display=1;
			choose();
			ARROW(1);
			*/
			GrFlush(&g_sContext);
		}
	P2IFG &= ~BIT1;	 break;
	case P2IV_NONE:	 break;
	}
}
/*
#pragma vector=TIMER2_A0_VECTOR
__interrupt void Timer2_A0_ISR(void) {
	if(!(P6OUT&BIT6)){
		Sharp96x96_SendToggleVCOMCommand();
	}
	else;
	TA2CCR0 +=  7276;
	TA2CTL &= ~TAIFG;
}
*/
void FLAGRST(void){
	update=0;													//读取数据后，更新标志位
	display=0;
	first=0;
	//SEND_FLAG=0;
	PPG_FLAG=0;
}

void SEND_Init(void){
	//初始化发送队列格式
	txString[0] = (unsigned char) (0xff);
	txString[1] = (unsigned char)(0x00);
	txString[2] = (unsigned char)(0x00);
	txString[8] = (unsigned char)(0x0D);
	txString[5] = (unsigned char) CR;
	txString[9] = (unsigned char) CR;
	txString[12] = (unsigned char) CR;
}

void Timer0_Init(void){
    P1DIR&=~BIT2;
    P1REN|=BIT2;
    P1SEL |= BIT2;
	TA0CCTL1 |= CAP + CM_2 + CCIS_0 + SCS + CCIE;
	TA0CTL |= TASSEL__SMCLK + MC_2 + TACLR + TAIE;
}

void Timer1_Init(void){
	TA1CTL = TACLR + TAIE;			 //开启中断并清零
	TA1CTL = TASSEL_1 + MC_1 +  TACLR;//选择SCLK32.768KHZ作为时钟，选用连续模式，并开启中断
	//中断频率200HZ
	TA1CCTL0 = CCIE;                          // CCR0 interrupt enabled
	TA1CCR0 = 163;
}

void Timer2_Init(void){
	TA2CTL = TACLR + TAIE;			 //开启中断并清零
	TA2CTL = TASSEL_1 + MC_1 +  TACLR;//选择SCLK32.768KHZ作为时钟，选用连续模式，并开启中断
	//中断频率50HZ
	TA2CCTL0 = CCIE;                          // CCR0 interrupt enabled
	TA2CCR0 = 7276;
}

/*TIMER0_A0_VECTOR是计时器0的CCR0的中断寄存器，TIMER0_A1_VECTOR是计时器0的CCR1-CCR4、TA的寄存器*/
/*同理定时器TA1也是分为两个TIMER1_A0_VECTOR和TIMER1_A1_VECTOR*/
#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR(void) {
   	switch(__even_in_range(TA0IV,14)){
	case TA0IV_NONE     : break;              /* No Interrupt pending */
	case TA0IV_TACCR1   :
		TA0R=0;
		temp=TA0CCR1;
		break;            /* TA0CCR1_CCIFG */
	case TA0IV_TACCR2   : break;        /* TA0CCR2_CCIFG */
	case TA0IV_TACCR3   : break;            /* TA0CCR3_CCIFG */
	case TA0IV_TACCR4   : break;           /* TA0CCR4_CCIFG */
	case TA0IV_5             : break;       		/* Reserved */
	case TA0IV_6             : break;      		 /* Reserved */
	case TA0IV_TAIFG      : break;           /* TA0IFG */
	default: break;

	}
}


void ARROW(u8 cmd){
	int i=0;
    GrContextForegroundSet(&g_sContext,  ClrBlack);
    GrContextBackgroundSet(&g_sContext, ClrWhite);
	if(cmd==1){
		for(i=0; i<7; i++){
    		Graphics_drawLineV(&g_sContext, i+2, 11+i, 23-i);
		}
		Graphics_Rectangle arrow0 = { 2, 25, 12, 86 };
		GrContextForegroundSet(&g_sContext,  ClrWhite);
		GrContextBackgroundSet(&g_sContext, ClrBlack);
        Graphics_fillRectangle(&g_sContext, &arrow0);
	}
	if(cmd==2){
		for(i=0; i<7; i++){
    		Graphics_drawLineV(&g_sContext, i+2, 31+i, 43-i);
		}
		Graphics_Rectangle arrow1 = { 2, 49, 12, 86 };
		Graphics_Rectangle arrow2 = { 2, 0, 12, 25 };
		GrContextForegroundSet(&g_sContext,  ClrWhite);
		GrContextBackgroundSet(&g_sContext, ClrBlack);
        Graphics_fillRectangle(&g_sContext, &arrow1);
        Graphics_fillRectangle(&g_sContext, &arrow2);
	}
	if(cmd==3){
		for(i=0; i<7; i++){
    		Graphics_drawLineV(&g_sContext, i+2, 51+i, 63-i);
		}
		Graphics_Rectangle arrow3 = { 2, 70, 12, 86 };
		Graphics_Rectangle arrow4 = { 2, 0, 12, 49 };
		GrContextForegroundSet(&g_sContext,  ClrWhite);
		GrContextBackgroundSet(&g_sContext, ClrBlack);
        Graphics_fillRectangle(&g_sContext, &arrow3);
        Graphics_fillRectangle(&g_sContext, &arrow4);
	}
	if(cmd==4){
		for(i=0; i<7; i++){
    		Graphics_drawLineV(&g_sContext, i+2, 71+i, 83-i);
		}
		Graphics_Rectangle arrow5 = { 2, 0, 12, 70 };
		GrContextForegroundSet(&g_sContext,  ClrWhite);
		GrContextBackgroundSet(&g_sContext, ClrBlack);
        Graphics_fillRectangle(&g_sContext, &arrow5);
	}
}

void LedDriver_Init(void){
	//LED driver
	P3DIR |=  BIT6;
	P3DIR |=  BIT5;
	P3OUT |= (BIT6+BIT5);
}

void KEY_Init(void){
	//按键外部中断
	P2DIR &= ~BIT1;
	P2REN |= BIT1;
	P2OUT |= BIT1;
	P2IES |= BIT1;
	P1DIR &= ~BIT1;
	P1REN |= BIT1;
	P1OUT |= BIT1;
	P1IES |= BIT1;
	P1IFG &= ~BIT1;
	P1IE |= BIT1;
	P2IFG &= ~BIT1;
	P2IE |= BIT1;
}

void BEEP(u8 cmd){

	P2SEL |= BIT5;                       							   // P3.5  options select
	P2DIR |= BIT5;                       						   // P3.5  output
	P2OUT|=BIT5;
	TA2CCR0 = 11;                           					   // PWM Period
	TA2CCTL2 = OUTMOD_7;                       			   // CCR5 reset/set
	TA2CCR2 = 5;                            					   // CCR5 PWM duty cycle
	TA2CTL = TASSEL__ACLK + MC_1 + TACLR;         // ACLK, up mode, clear TBR
	if(cmd){
		TA2CCR2 = 5;
	}
	else{
		P2OUT&=~BIT5;
		TA2CCR2 = 0;
	}
}


void RD_OFF(void){
	P3OUT |= BIT6;
}

void IR_OFF(void){
	P3OUT |= BIT5;
}

void RD_ON(void){
	P3OUT&=~BIT6;
}

void IR_ON(void){
	P3OUT&=~BIT5;
}

/*
*  输入：IR 和 R 的变量
*  输出：滤波后的数值
*  设计是0-8.5HZ，采样率是200HZ
*/

int  IR_Filter(int  input) {
	int y0, x2;
	//这里是更新数据
	x2 = IR_Old.x1;
	IR_Old.x1 = IR_Old.x0;
	IR_Old.x0 = input;
	//这里是计算差分方程
	y0 =IR_Old.x0 * IR_B[0] + IR_Old.x1 * IR_B[1] + x2 * IR_B[2] - IR_Old.y1 * IR_A[1] - IR_Old.y2 * IR_A[2];
	y0 = y0 / IR_A[0];
	//这里是更新数据
	IR_Old.y2 = IR_Old.y1;
	IR_Old.y1 = y0;
	//返回计算结果
	return y0;
}

void IR_reset(void) {
	IR_Old.x0 = 0;
	IR_Old.x1 = 0;
	IR_Old.y1 = 0;
	IR_Old.y2 = 0;
}
/*
*  输入：IR 和 R 的变量
*  输出：滤波后的数值
*  设计是0-8.5HZ，采样率是200HZ
*/
int  RD_Filter(int  input) {
	int y0, x2;
	//这里是更新数据
	x2 = RD_Old.x1;
	RD_Old.x1 = RD_Old.x0;
	RD_Old.x0 = input;
	//这里是计算差分方程
	y0 =RD_Old.x0 * RD_B[0] + RD_Old.x1 * RD_B[1] + x2 * RD_B[2] - RD_Old.y1 * RD_A[1] - RD_Old.y2 * RD_A[2];
	y0 = y0 / RD_A[0];
	//这里是更新数据
	RD_Old.y2 = RD_Old.y1;
	RD_Old.y1 = y0;
	//返回计算结果
	return y0;
}

void RD_reset(void) {
	RD_Old.x0 = 0;
	RD_Old.x1 = 0;
	RD_Old.y1 = 0;
	RD_Old.y2 = 0;
}

long diff(long  *group, int j) {
	if(j==0) return 0;//group[0];
	else{
		return (group[j] - group[j - 1]);
	}
}

void CLK_Init(void){
	// MCLK   25MHZ
	// SMCLK 12MHZ
	// ACLK	   32.768KHZ
	// 时钟设置
	//P1DIR |= BIT0;
	//P1OUT |= BIT0;
	SetVCore(3);
	UCSCTL3 = SELREF_2;
	UCSCTL4 |= SELA_2;
	UCSCTL5 |= DIVS_1;
	__bis_SR_register(SCG0);
	UCSCTL0 = 0x0000;
	UCSCTL1 = DCORSEL_7;
	UCSCTL2 = FLLD_0 + 762;
	__bic_SR_register(SCG0);
	__delay_cycles(782000);
	do{
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	    SFRIFG1 &= ~OFIFG;
	}while (SFRIFG1&OFIFG);
}


long  mean(long* group, u8 c, u8 l){
	long sum=0;
	u8 adjust=0;
	u8 t;
	u8 cc=0;
	cc=c;
	if(l>c)	{
		adjust = l - c;
	}
	else{
		adjust = MAX_LENGTH+l-c;
	}
	for(t=0; t<adjust-1; t++){
		sum += group[cc];
		cc=((cc+1) & 0x7F);
	}
	return (sum/adjust);
}

void SpO2str(int  num){
	char  temp[3];
	int  n;
	n=num%10;
	temp[0]=n+'0';
	num=(num-n)/10;
	n=num%10;
	temp[1]=n+'0';
	temp[2]='\0';
	spo2[0]=temp[1];
	spo2[1]=temp[0];
	spo2[2]='\0';
}

void ALARM2str(int  num){
	char  temp[3];
	int  n;
	n=num%10;
	temp[0]=n+'0';
	num=(num-n)/10;
	n=num%10;
	temp[1]=n+'0';
	temp[2]='\0';
	alarm[0]=temp[1];
	alarm[1]=temp[0];
	alarm[2]='\0';
}

void PI2str(int  num){
	char  temp[2];
	int  n;
	n=num%10;
	temp[0]=n+'0';
	temp[1]='\0';
	P[0]=temp[0];
	P[1]=temp[1];
}

void BPM2str(int  num){
	char  temp[4];
	int n;
	if(num>999)	num=999;
	if(num<0)		num=0;
	n=num%10;
	temp[0]=n+'0';
	num=(num-n)/10;
	n=num%10;
	temp[1]=n+'0';
	num=(num-n)/10;
	n=num%10;
	temp[2]=n+'0';
	temp[3]='\0';
	bpm[0]=temp[2];
	bpm[1]=temp[1];
	bpm[2]=temp[0];
	bpm[3]='\0';
	if(bpm[0]=='0')	bpm[0]=' ';
}

void choose(void){
	GrStringDraw(&g_sContext, "PPGs", AUTO_STRING_LENGTH, 16, 11 ,   TRANSPARENT_TEXT);
	GrStringDraw(&g_sContext, "BLE", AUTO_STRING_LENGTH, 16, 31,  TRANSPARENT_TEXT);
	GrStringDraw(&g_sContext, "ALARM", AUTO_STRING_LENGTH, 16, 51,  TRANSPARENT_TEXT);
	GrStringDraw(&g_sContext, "CANCEL", AUTO_STRING_LENGTH, 16, 71,  TRANSPARENT_TEXT);
}

void choosePPG0(void){
//	GrStringDraw(&g_sContext, "PPGs", AUTO_STRING_LENGTH, 16, 11 ,   TRANSPARENT_TEXT);
	ARROW(1);
}

void chooseBLE0(void){
	//GrStringDraw(&g_sContext, "BLE", AUTO_STRING_LENGTH, 16, 31,  TRANSPARENT_TEXT);
	ARROW(2);
}

void chooseALARM0(void){
	//GrStringDraw(&g_sContext, "ALARM", AUTO_STRING_LENGTH, 16, 51,  TRANSPARENT_TEXT);
	ARROW(3);
}

void chooseCANCEL0(void){
	//GrStringDraw(&g_sContext, "CANCEL", AUTO_STRING_LENGTH, 16, 71,  TRANSPARENT_TEXT);
	ARROW(4);
}

void UART_Init(void){

	P3SEL |= BIT3 + BIT4;
	UCA0CTL1 |= UCSWRST;                      																		// **Put state machine in reset**
	UCA0CTL1 |= UCSSEL__SMCLK;
	UCA0BR0 = 0x6C;                           																			//BOUND:9600
	UCA0BR1 = 0x00;
	UCA0MCTL = 0x54;
	UCA0CTL1 &= ~UCSWRST;                     																	// **Initialize USCI state machine**
//	UCA0IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt// Enable USCI_A0 RX interrupt
}

void BLE_Init(void){
	P6DIR|=BIT2;
	P6OUT|=BIT2;
	P6DIR&=~BIT3;
}

u8 BLE_CHECK(void){
	if(!(P6IN&BIT3))
		return 0;
	else
		return 1;
}

void BLE_ON(void){

	P6OUT&=~BIT2;
}

void BLE_OFF(void){

	P6OUT|=BIT2;
}

// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             	// Vector 0 - no interrupt
  case 2:                                   		// Vector 2 - RXIFG
   //while (!(UCA0IFG&UCTXIFG));    // USCI_A1 TX buffer ready?
   // UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}

/*
void ADC12_Init(void){

	ADC12CTL0 &= ~ADC12ENC;                           //关闭软件使能
	//使用内部电压，打开ADC12内核，8分频，内部参考电压2.5V
	//使用内部电压，参考位为AVCC和AVSS
	ADC12CTL0 = ADC12ON + ADC12SHT02 ;//+ ADC12MSC;// + ADC12REFON + ADC12REF2_5V;
	//ADC12MCTL0 = SREF_0;
	//使用外部电压，打开ADC12内核，8分频
	//使用外部电压，参考为为VeREF+和AVSS
	//ADC12CTL0 = ADC12ON + SHT0_8;
	//ADC12MCTL0 = SREF_2;
	//单通道单次转换，采样信号来自采样定时器
	ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0;
	//单通道多次转换，采样信号来自采样定时器
	//ADC12CTL1 = SHP + CONSEQ_2;
	ADC12MCTL0 |= ADC12INCH_0 + ADC12SREF_0   ;//+ ADC12EOS;
	//ADC12MCTL1 |= ADC12SREF_0 + ADC12INCH_1 + EOS;
	ADC12IE = BIT0;
	ADC12CTL0 |= ADC12ENC;                           		//开启软件使能
	P6SEL |= BIT0;
	//ADC12CTL0 |= ADC12SC;                   					// Start conversion

	ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
	ADC12CTL1 = ADC12SHP;                     // Use sampling timer
	ADC12IE = 0x01;                           // Enable interrupt
	ADC12CTL0 |= ADC12ENC;
	P6SEL |= 0x01;                            // P6.0 ADC option select

}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void){
	ADC12IFG &= ~BIT0;
}
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void) {
	switch (__even_in_range(UCA0IV, 4)) {
	case 0:break;                             // Vector 0 - no interrupt
	case 2:break;							  // Vector 2 - RXIFG
	case 4:break;                             // Vector 4 - TXIFG
	default: break;
	}
}
*/

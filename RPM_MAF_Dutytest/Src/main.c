/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define USE_OLED

#include <stdio.h>

#ifdef USE_OLED
#include "oled.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


extern uint8_t nonside[];



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#if 0
#define ON_STAR_RPM 1200
#define ON_STAR_DUTY 500

#define LINEAR_STAR_RPM 1600
#define LINEAR_END_RPM 2500

#define LINEAR_STAT_DUTY 750
#define LINEAR_END_DUTY 1000
#endif

#if 0
#define ON_STAR_RPM 1200
#define ON_STAR_DUTY 100

#define LINEAR_STAR_RPM 2000
#define LINEAR_END_RPM 3000

#define LINEAR_STAT_DUTY 200
#define LINEAR_END_DUTY 1000
#endif

#if 1 
// 20190512, 传感器rpm
// 启动（线性开始）1200，40%； 关1000，40%； 最高2500，100%

#define ON_STAR_RPM 1000
#define ON_STAR_DUTY 400

#define LINEAR_STAR_RPM 1200
#define LINEAR_END_RPM 2500

#define LINEAR_STAT_DUTY 400
#define LINEAR_END_DUTY 1000
#endif


//#define M2SW_ON_MAF  10
//#define M2SW_OFF_MAF 5


// 20190512， MAF 5-->8, 10-->12
//#define ON_STAR_MAF 5
#define ON_STAR_MAF 8
#define ON_STAR_MAF_DUTY 500

//#define LINEAR_STAR_MAF 10
#define LINEAR_STAR_MAF 12
#define LINEAR_END_MAF  25

#define LINEAR_STAT_MAF_DUTY 400
#define LINEAR_END_MAF_DUTY 1000


#define SYSTEM_ON_TEMP 80



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint32_t ddd;
uint32_t dd;
uint32_t d;
uint32_t rpm;
uint32_t rpm_old;



uint32_t sensor_rpm = 0;
uint32_t adc_rpm = 0;
uint32_t obd_rpm = 0;


uint32_t prescaler;
uint32_t period;
uint32_t pulse;
uint32_t clk_prescaler;


/*
double prescaler;
double period;
double pulse;
double clk_prescaler;
*/

uint32_t d1;
uint32_t d2;
uint32_t d3;

uint32_t enc_d;
uint16_t adc_d;
uint16_t m1_out_pwm_d;
uint8_t  m2_sw = 0;

//uint8_t ddd_ch[] = "123";
//uint8_t ss[20] = "123";
char ss[20] = "123";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//----------------------------------------------------------------------------
// printf to uart
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
//------------------------------------------------------------------------------

#define H_OBD_UART huart2
//#define H_OBD_UART huart3


// for uart3, elm327
uint8_t aRx3Buffer[1];      //接收缓存
uint8_t aTx3Buffer[] = "*********SENDING DATA USING USART3 with DMA***********\r\n";


uint8_t cmd_atz[]  = "AT Z\r"; // reset
uint8_t cmd_ati[]  = "AT I\r";
uint8_t cmd_010c[] = "010C\r"; // rpm
uint8_t cmd_010d[] = "010D\r"; // speed
uint8_t cmd_0105[] = "0105\r"; // Engine coolant temperature 
uint8_t cmd_0110[] = "0110\r"; // MAF
uint8_t cmd_atsp0[]  = "AT SP 0\r"; // 
uint8_t cmd_015C[] = "015C\r"; // Engine oil temperature


uint16_t m_rpm = 0;
uint16_t m_maf = 0;
uint8_t m_speed = 0;  // 0 - 255 km/h
uint8_t m_EngineOilTemperature = 0;  // Engine oil temperature
uint8_t m_EngineCoolantTemperature = 0;


uint8_t state = 0;
uint8_t vout_state = 0;
uint8_t sw_state = 0;

void delay_ms(uint16_t ms)
{
	/*
	uint16_t i,j;

	for(i=0; i<ms; i++) //2000
	{
		for(j=0;j<1650; j++)
		{

		}
	}
	*/
	HAL_Delay(ms);
}

uint8_t key_d;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == KEY1_Pin)
	{
		key_d = 1;
	}
	if(GPIO_Pin == KEY2_Pin)
	{
		key_d = 2;
	}
	if(GPIO_Pin == KEY3_Pin)
	{
		key_d = 3;
	}
	if(GPIO_Pin == KEY4_Pin)
	{
		key_d = 4;
	}
}



uint32_t duty = 0;
uint32_t freq = 0;

//volatile static uint32_t tmp1 = 0, tmp2 = 0;
uint32_t tmp1 = 0, tmp2 = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    //volatile static uint32_t tmp1 = 0, tmp2 = 0;
    
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        tmp1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);//周期
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        tmp2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);//占空比
    }
}

#define UART3_RX_BUFF_LEN 100
//#define UART3_RCV_SIZE 8

uint8_t uart3_rx_buff[UART3_RX_BUFF_LEN];
uint8_t cmd_buff[UART3_RX_BUFF_LEN];

uint8_t uart3_rx_index = 0;
uint8_t cmd_ok = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
	
  //HAL_UART_Transmit_DMA(&huart3,aRx3Buffer,1); // DM2发送出去
	//HAL_UART_Receive_DMA(&huart3,aRx3Buffer,1); // 重新DMA接收 
	
	//HAL_UART_Transmit_DMA(&H_OBD_UART,aRx3Buffer,1); // DM2发送出去
	//HAL_UART_Receive_DMA(&H_OBD_UART,aRx3Buffer,1); // 重新DMA接收 

	// test
	//HAL_UART_Transmit(&huart1,aRx3Buffer, 1, 0xFFFF);
		
	
	uart3_rx_buff[uart3_rx_index] = aRx3Buffer[0];
	if( uart3_rx_index < UART3_RX_BUFF_LEN-1) uart3_rx_index++;
	else uart3_rx_index = 0;
	
	if(uart3_rx_buff[uart3_rx_index-1] == 0x0d && uart3_rx_buff[uart3_rx_index-2] == 0x0d && cmd_ok==0)
	{
		cmd_ok = 1;
		uart3_rx_index = 0;
		for(uint8_t i = 0; i<UART3_RX_BUFF_LEN; i++)
		{
			cmd_buff[i] = uart3_rx_buff[i];
			uart3_rx_buff[i] = 0x00;
		}		
	}
	
	
}


//uint16_t hex2uint16(const char *p)
uint16_t hex2uint16(const uint8_t *p)
{
	char c = *p;
	uint16_t i = 0;
	for (uint8_t n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ' && n == 2) {
            continue;
        } else if (c < '0' || c > '9') {
			break;
        }
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

//uint8_t hex2uint8(const char *p)
uint8_t hex2uint8(const uint8_t *p)
{
	uint8_t c1 = *p;
	uint8_t c2 = *(p + 1);
	if (c1 >= 'A' && c1 <= 'F')
		c1 -= 7;
	else if (c1 >='a' && c1 <= 'f')
	    c1 -= 39;
	else if (c1 < '0' || c1 > '9')
		return 0;

	if (c2 == 0)
		return (c1 & 0xf);
	else if (c2 >= 'A' && c2 <= 'F')
		c2 -= 7;
	else if (c2 >= 'a' && c2 <= 'f')
	    c2 -= 39;
	else if (c2 < '0' || c2 > '9')
		return 0;

	return c1 << 4 | (c2 & 0xf);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	// GPIO
	HAL_GPIO_WritePin(M2OUT_SW_GPIO_Port, M2OUT_SW_Pin, GPIO_PIN_RESET);
	
	
	  ddd = 0;
		d = 0;
		rpm = 0;
	  rpm_old = 100;
	
		period = 1000-1;
		pulse = 100;
		prescaler = 60-1;

	
    d1 = 500;
		d2 = 1000;
		d3 = 3000;

	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //HAL_Delay(1000); //1000ms

#ifdef USE_OLED
	delay_ms(10);
	LCD_Init();
	delay_ms(10);
		
	//line 1
	LCD_Print(2, 0, "WM Test",TYPE16X16,TYPE8X16);

	//line 2
	LCD_Print(2, 16, "Version 0.10",TYPE16X16,TYPE8X16);
	//sprintf(ss,"%s",__DATA_);
	//LCD_Print(2, 16, ss, TYPE16X16, TYPE8X16);

	// line 3
	//LCD_Print(2, 32,  "OBD MAF:",TYPE16X16,TYPE8X16);
	sprintf(ss,"%s",__DATE__);
	LCD_Print(2, 32, ss,TYPE16X16,TYPE8X16);

	// line 4
	//LCD_Print(2, 48,  "OBD EOT:",TYPE16X16,TYPE8X16);
	sprintf(ss,"%s",__TIME__);
	LCD_Print(2, 48, ss,TYPE16X16,TYPE8X16);

#endif

	printf("\n\r\n\rWM_RPM V0.1\n\r");
	printf("Build %s %s\n\r", __TIME__, __DATE__);

	uint32_t uid[3];
	HAL_GetUID(uid);

	printf("UID:");
	for(int8_t i = 0; i < 3; i++) {
		printf("%08X", uid[i]);
	}  
	printf("\n\r");


/*
500rpm =500r/min = 500/60 Hz, === 8.333 Hz
300rpm = 300 r/min = 300/60 Hz === 5 Hz 4000?
600rpm = 600 r/min = 600/60 Hz === 10 Hz 2000?
1200rpm = 1200 r/min = 1200/60 Hz == 20 Hz   1000
1800rpm = 30 Hz  666.67?
2400rpm = 40 Hz  500?
3000rpm = 50 Hz   400
3600rpm = 60 Hz

(20000/tmp2)*60

*/

  // PWM output

	// m1_out_pwm_d
	// M1 out PWM, control the motor
	// TIM2:  Prescaler：72-1；CKD: Division by 2;  ==> 1kHz
	// Prescaler=720-1;Counter Period 2000-1 == 50Hz, read pwm ==400
	// Prescaler=720-1;Counter Period 5000-1 == 20Hz, read pwm ==1000
	// Prescaler=2880-1 and CounterPeriod=5000-1, read pwm = 4000
	m1_out_pwm_d = 0;
	
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m1_out_pwm_d);
	//HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4); //M1OUT_PWM
	
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1_out_pwm_d);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); //M1OUT_PWM


	// PWM output, use TIM3 for PWM sim.
	//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	//HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);

  // PWM input, use for get freq.
	// TIM3:
	// Prescaler 3600-1;
	// CKD: Division by 4;
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);//
  //rpm_sensor = tmp2;

//__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);

		//HAL_UART_Receive_DMA(&huart3,aRx3Buffer,1);// 启动DMA接收
		//HAL_UART_Transmit_DMA(&huart3,aTx3Buffer,sizeof(aTx3Buffer));// DMA发送数据
		
		
		//HAL_UART_Receive_DMA(&huart3,aRx3Buffer,1);// 启动DMA接收
		HAL_UART_Receive_DMA(&H_OBD_UART,aRx3Buffer,1);// 启动DMA接收

		
		
		// init elm327
		// reset
		//HAL_UART_Transmit_DMA(&huart3,cmd_atz,sizeof(cmd_atz));// DMA发送数据
		HAL_UART_Transmit_DMA(&H_OBD_UART,cmd_atz,sizeof(cmd_atz));// DMA发送数据
		delay_ms(1000);

		// set protocal auto
		//HAL_UART_Transmit_DMA(&huart3,"AT SP 0\r", 8); 
		//HAL_UART_Transmit_DMA(&huart3,cmd_atsp0,sizeof(cmd_atsp0));
		HAL_UART_Transmit_DMA(&H_OBD_UART,cmd_atsp0,sizeof(cmd_atsp0));
		delay_ms(2000);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			delay_ms(10);
		
			//----------------------------------------------------
			// get info from OBD
			//----------------------------------------------------
		  ddd++;
		  if(ddd>5)
			{
				ddd = 0;
				
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				//HAL_GPIO_TogglePin(M2OUT_SW_GPIO_Port, M2OUT_SW_Pin);
				switch(state)
				{
					case 0:
						HAL_UART_Transmit_DMA(&H_OBD_UART,cmd_010c,sizeof(cmd_010c)); // rpm
					  //printf("[rpm]");
						state = 1;
						break;
					
					case 1:
						HAL_UART_Transmit_DMA(&H_OBD_UART,cmd_010d,sizeof(cmd_010d)); // speed
						state = 2;
						break;
					
					case 2:
						HAL_UART_Transmit_DMA(&H_OBD_UART,cmd_0110,sizeof(cmd_0110)); // MAF
						state = 3;
						break;
					
					case 3:
						HAL_UART_Transmit_DMA(&H_OBD_UART,cmd_0105,sizeof(cmd_0105));  // Engine Coolant Temperature
						state = 0;
						break;
					
					case 4:
						state = 0;
						break;
					
					case 5:
						state = 0;
						break;
					
					default:
						state = 0;
						break;
				}
				//----------------------------------------------------------------------
#if 1
					printf("[");
				for(uint8_t i = 0; i<UART3_RX_BUFF_LEN; i++)
				{
					if(cmd_buff[i]==0x0d) printf("_");
					else if(cmd_buff[i]==0x00) printf(".");
					else printf("%c",cmd_buff[i]);
				}
				printf("]");
				
				printf("m_rpm=%04d ",m_rpm);
				printf("m_speed=%04d ",m_speed);
				printf("m_maf=%04d ",m_maf);
				//printf("m_EngineOilTemperature=%04d ",m_EngineOilTemperature);
				printf("m_EngineCoolantTemperature=%04d ",m_EngineCoolantTemperature);
				printf("\r");
#endif				
				//----------------------------------------------------------------------
				
			}
			
			//========================================================================
			if(cmd_ok==1)
			{
				cmd_ok = 0;

				//-----------------------------------------------------------------------------------------
				// test: info display
				//-----------------------------------------------------------------------------------------
		
		#if 0
				printf("cmd_ok: [");
				for(uint8_t i = 0; i<UART3_RX_BUFF_LEN; i++)
				{
					if(cmd_buff[i]==0x0d) printf("_");
					else if(cmd_buff[i]==0x00) printf(".");
					else printf("%c",cmd_buff[i]);
				}
				//printf("]\n\r");
				printf("]");
		#endif
				//-----------------------------------------------------------------------------------------
				// rpm: >010C.41_0C_1A_F8.. 
				//-----------------------------------------------------------------------------------------
				if(cmd_buff[0]=='>' && cmd_buff[1]=='0' && cmd_buff[2]=='1' && cmd_buff[3]=='0' && cmd_buff[4]=='C')
				{
					if(cmd_buff[6]=='4' && cmd_buff[7]=='1' && cmd_buff[9]=='0' && cmd_buff[10]=='C')
					{
						//cmd_buff[12],cmd_buff[13], __,  cmd_buff[15],cmd_buff[16],
						m_rpm = hex2uint16(&cmd_buff[12]);
						m_rpm >>= 2;
						//printf("m_rpm = %d\n\r",m_rpm);
					}
					else
					{
						m_rpm = 0;
						//printf("get rpm error\n\r");
					}
				}
				//-----------------------------------------------------------------------------------------
				// speed: >010D.41_0D_XX_XX..
				//-----------------------------------------------------------------------------------------
				if(cmd_buff[0]=='>' && cmd_buff[1]=='0' && cmd_buff[2]=='1' && cmd_buff[3]=='0' && cmd_buff[4]=='D')
				{
					if(cmd_buff[6]=='4' && cmd_buff[7]=='1' && cmd_buff[9]=='0' && cmd_buff[10]=='D')
					{
						//cmd_buff[12],cmd_buff[13], __,  cmd_buff[15],cmd_buff[16],
						m_speed = hex2uint8(&cmd_buff[12]);
						//printf("m_speed = %d\n\r",m_speed);
					}
					else
					{
						m_speed = 0;
						//printf("get speed error\n\r");
					}
				}
				//-----------------------------------------------------------------------------------------
				// MAF >0110.41_10_XX_XX..
				//-----------------------------------------------------------------------------------------
				if(cmd_buff[0]=='>' && cmd_buff[1]=='0' && cmd_buff[2]=='1' && cmd_buff[3]=='1' && cmd_buff[4]=='0')
				{
					if(cmd_buff[6]=='4' && cmd_buff[7]=='1' && cmd_buff[9]=='1' && cmd_buff[10]=='0')
					{
						//cmd_buff[12],cmd_buff[13], __,  cmd_buff[15],cmd_buff[16],
						m_maf = hex2uint16(&cmd_buff[12]);
						m_maf = m_maf / 100;
						//printf("m_maf = %d\n\r",m_maf);
					}
					else
					{
						m_maf = 0;
						//printf("get MAF error\n\r");
					}
				}
				//-----------------------------------------------------------------------------------------
				// EngineOilTemperature: >015C.41_5C_XX_XX..
				//-----------------------------------------------------------------------------------------
#if 0
				if(cmd_buff[0]=='>' && cmd_buff[1]=='0' && cmd_buff[2]=='1' && cmd_buff[3]=='5' && cmd_buff[4]=='C')
				{
					if(cmd_buff[6]=='4' && cmd_buff[7]=='1' && cmd_buff[9]=='5' && cmd_buff[10]=='C')
					{
						//cmd_buff[12],cmd_buff[13], __,  cmd_buff[15],cmd_buff[16],
						m_EngineOilTemperature = hex2uint8(&cmd_buff[12]) - 40;
						//printf("m_EngineOilTemperature = %d\n\r",m_EngineOilTemperature);
					}
					else
					{
						m_EngineOilTemperature = 0;
						//printf("get EngineOilTemperature error\n\r");
					}
				}
#endif
				//-----------------------------------------------------------------------------------------
				// EngineCoolantTemperature: >0105.41_05_XX_XX..
				//-----------------------------------------------------------------------------------------
				if(cmd_buff[0]=='>' && cmd_buff[1]=='0' && cmd_buff[2]=='1' && cmd_buff[3]=='0' && cmd_buff[4]=='5')
				{
					if(cmd_buff[6]=='4' && cmd_buff[7]=='1' && cmd_buff[9]=='0' && cmd_buff[10]=='5')
					{
						//cmd_buff[12],cmd_buff[13], __,  cmd_buff[15],cmd_buff[16],
						m_EngineCoolantTemperature = hex2uint8(&cmd_buff[12]) - 40;
						//printf("m_EngineCoolantTemperature = %d\n\r",m_EngineCoolantTemperature);
					}
					else
					{
						m_EngineCoolantTemperature = 0;
						//printf("get EngineCoolantTemperature error\n\r");
					}
				}
				//-----------------------------------------------------------------------------------------
	#if 0
				printf("m_rpm=%04d ",m_rpm);
				printf("m_speed=%04d ",m_speed);
				printf("m_maf=%04d ",m_maf);
				printf("m_EngineOilTemperature=%04d ",m_EngineOilTemperature);
				printf("m_EngineCoolantTemperature=%04d ",m_EngineCoolantTemperature);
				printf("\r");
	#endif
				//-----------------------------------------------------------------------------------------


				// clear
				for(uint8_t i = 0; i<UART3_RX_BUFF_LEN; i++)
				{
					cmd_buff[i] = 0x00;
				}

				
			}else {
				/*
				m_rpm = 0;
				m_speed = 0;
				m_maf = 0;
				m_EngineOilTemperature = 0;
				*/
			}
			//------------------------------------------------------------------------
	#if 0
					printf("[");
				for(uint8_t i = 0; i<UART3_RX_BUFF_LEN; i++)
				{
					if(cmd_buff[i]==0x0d) printf("_");
					else if(cmd_buff[i]==0x00) printf(".");
					else printf("%c",cmd_buff[i]);
				}
				printf("]");
				
				printf("m_rpm=%04d ",m_rpm);
				printf("m_speed=%04d ",m_speed);
				printf("m_maf=%04d ",m_maf);
				printf("m_EngineOilTemperature=%04d ",m_EngineOilTemperature);
				printf("m_EngineCoolantTemperature=%04d ",m_EngineCoolantTemperature);
				printf("\r");
#endif				
			//========================================================================
			// calc rpm
			//========================================================================

			// rpm from sensor
			//sensor_rpm = (20000/tmp2)*60;
			sensor_rpm = (20000*60)/tmp2;
			
			// rpm from adc
			adc_rpm = adc_d * 0.9f;
			
			// rpm from OBD
			obd_rpm = m_rpm;
			
			//----------------------------------------------------------------------------------------
			// 调节电位器，使ADC大于0，强制进行测试模式输出。调节ADC等于0，按传感器或OBD数据进行输出。
			// OBD有数据，按OBD，无数据按传感器。
			
			if( obd_rpm > 10 ) rpm = obd_rpm;
			else{
				if(sensor_rpm > 10 ) rpm = sensor_rpm;
				else                 rpm = adc_rpm;
			}
			
	#if 1
			//---------------------------------------------------------------------------------------------
			// output
			// PWM by rpm
			//---------------------------------------------------------------------------------------------

			//-------------------------------------------------------------
			// M1 PWM
			//-------------------------------------------------------------
			// ON_STAR_RPM 启动，维持到LINEAR_STAR_RPM：ON_STAR_DUTY。
			// LINEAR_STAR_RPM开始到LINEAR_END_RPM线性：LINEAR_STAT_DUTY
			// LINEAR_END_RPM以上全开
			// 
			switch(vout_state)
			{
				case 0:
					m1_out_pwm_d = 0;
					if(rpm > LINEAR_STAR_RPM ) vout_state = 2;
					break;
				case 1:
					m1_out_pwm_d = ON_STAR_DUTY;
					if( rpm > LINEAR_STAR_RPM ) vout_state = 2;
				  if( rpm < ON_STAR_RPM ) vout_state = 4;
					break;
				case 2:
					m1_out_pwm_d = (rpm-LINEAR_STAR_RPM)*(LINEAR_END_DUTY-LINEAR_STAT_DUTY)/(LINEAR_END_RPM-LINEAR_STAR_RPM) +LINEAR_STAT_DUTY;					

					if(rpm > LINEAR_END_RPM ) vout_state = 3;
				  if(rpm < LINEAR_STAR_RPM ) vout_state = 1;
					break; 
				case 3:
					m1_out_pwm_d = LINEAR_END_DUTY;
					if( rpm > LINEAR_STAR_RPM && rpm < LINEAR_END_RPM ) vout_state = 2;
				  if( rpm > ON_STAR_RPM     && rpm < LINEAR_STAR_RPM ) vout_state = 1;
				  if( rpm < ON_STAR_RPM ) vout_state = 1;
					break;
				case 4:
					m1_out_pwm_d = 0;
					if( rpm > LINEAR_STAR_RPM) vout_state = 2;
					break;
				default:
					vout_state = 0;
					break;
				
			}
			// output PWM
			
	#if 1
			//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, m1_out_pwm_d);
			//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1_out_pwm_d);

	#else // test
			dd+=10;
		  if(dd>999)
			{
				dd = 0;
				//obd_rpm += 100;
				//if(obd_rpm > 5000 ) obd_rpm = 0;

			}
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dd);
	#endif
			
	#endif


#if 0
			//---------------------------------------------------------------------------------------------
			// output
			// PWM by MAF
			//---------------------------------------------------------------------------------------------

			//-------------------------------------------------------------
			// M1 PWM
			//-------------------------------------------------------------
			// ON_STAR_RPM 启动，维持到LINEAR_STAR_RPM：ON_STAR_DUTY。
			// LINEAR_STAR_RPM开始到LINEAR_END_RPM线性：LINEAR_STAT_DUTY
			// LINEAR_END_RPM以上全开
			// 
			switch(vout_state)
			{
				case 0:
					m1_out_pwm_d = 0;
					if(m_maf > LINEAR_STAR_MAF ) vout_state = 2;
					break;
				case 1:
					m1_out_pwm_d = ON_STAR_MAF_DUTY;
					if( m_maf > LINEAR_STAR_MAF ) vout_state = 2;
				  if( m_maf < ON_STAR_MAF ) vout_state = 4;
					break;
				case 2:
					m1_out_pwm_d = (m_maf-LINEAR_STAR_MAF)*(LINEAR_END_MAF_DUTY-LINEAR_STAT_MAF_DUTY)/(LINEAR_END_MAF-LINEAR_STAR_MAF) +LINEAR_STAT_MAF_DUTY;					

					if(m_maf > LINEAR_END_MAF ) vout_state = 3;
				  if(m_maf < LINEAR_STAR_MAF ) vout_state = 1;
					break; 
				case 3:
					m1_out_pwm_d = LINEAR_END_MAF_DUTY;
					if( m_maf > LINEAR_STAR_MAF && m_maf < LINEAR_END_MAF ) vout_state = 2;
				  if( m_maf > ON_STAR_MAF     && m_maf < LINEAR_STAR_MAF ) vout_state = 1;
				  if( m_maf < ON_STAR_MAF ) vout_state = 1;
					break;
				case 4:
					m1_out_pwm_d = 0;
					if( m_maf > LINEAR_STAR_MAF) vout_state = 2;
					break;
				default:
					vout_state = 0;
					break;
				
			}
			// output PWM
			
			//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1_out_pwm_d);

	#endif
			

			
			
			
#if 0
			//-------------------------------------------------------------
			// M2 SW
			//-------------------------------------------------------------
			switch(sw_state)
			{
				case 0:
					m2_sw = 0;
					if(m_maf >= M2SW_ON_MAF ) sw_state = 2;
					break;
				
				case 1:
					m2_sw = 1;
					if( m_maf >= M2SW_ON_MAF ) sw_state = 2;
				  if( m_maf < M2SW_OFF_MAF ) sw_state = 0;
					break;
				
				case 2:
					m2_sw = 1;					
				  if( m_maf < M2SW_ON_MAF ) sw_state = 1;
				  if( m_maf < M2SW_OFF_MAF ) sw_state = 0;
					break; 

				default:
					m2_sw = 0;
					break;
			}
#endif
			
#if 0
			if(m2_sw>0){
				HAL_GPIO_WritePin(M2OUT_SW_GPIO_Port, M2OUT_SW_Pin, GPIO_PIN_SET);
				//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 900);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m_maf*60);
			}else{
				HAL_GPIO_WritePin(M2OUT_SW_GPIO_Port, M2OUT_SW_Pin, GPIO_PIN_RESET);
				//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m_maf*100);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			}
#endif
			
//-----------------------------------------------------------------------------------------------------			
// 20190512
#if 0
			if(m1_out_pwm_d>0 && m_EngineCoolantTemperature > SYSTEM_ON_TEMP) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1_out_pwm_d);
					HAL_GPIO_WritePin(M2OUT_SW_GPIO_Port, M2OUT_SW_Pin, GPIO_PIN_SET);
			}
			else
			{
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
					HAL_GPIO_WritePin(M2OUT_SW_GPIO_Port, M2OUT_SW_Pin, GPIO_PIN_RESET);

			}
#endif
			if(m1_out_pwm_d>0 ) {
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1_out_pwm_d);  // 电机
					HAL_GPIO_WritePin(M2OUT_SW_GPIO_Port, M2OUT_SW_Pin, GPIO_PIN_SET); // 电磁阀
			}
			else
			{
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // 电机
					HAL_GPIO_WritePin(M2OUT_SW_GPIO_Port, M2OUT_SW_Pin, GPIO_PIN_RESET); // 电磁阀

			}
//-----------------------------------------------------------------------------------------------------			
			
			
#if 0
			//---------------------------------------------------------------------------------------------
			// EC11 旋转编码器
			//---------------------------------------------------------------------------------------------
		  enc_d = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim4));//获取定时器的值
#endif
		
#if 1
			//---------------------------------------------------------------------------------------------
			// ADC
			//---------------------------------------------------------------------------------------------
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1,50);
			
			if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
			{
						adc_d = HAL_ADC_GetValue(&hadc1);
						//printf("PA1 Voltage : %.4f \r\n",adc_d*3.3f/4096);
			} 
#else
			adc_d = 0;
#endif
	
			
	
			//---------------------------------------------------------------------------------------------
			// OLED
			//---------------------------------------------------------------------------------------------
				
#ifdef USE_OLED
			//line 1
			LCD_Print(2, 0, "ADC rpm:",TYPE16X16,TYPE8X16);
			d = adc_rpm; //rpm;
			sprintf(ss,"%6d",d);
			LCD_Print(64, 0, ss,TYPE16X16,TYPE8X16);

			//line 2
			LCD_Print(2, 16, "OBD rpm:",TYPE16X16,TYPE8X16);
			//LCD_Print(2, 24, "总线:",TYPE16X16,TYPE8X16);
			d = m_rpm;//tmp1;//freq;//period;
			sprintf(ss,"%6d",d);
			LCD_Print(64, 16, ss, TYPE16X16, TYPE8X16);
			//LCD_Print(64, 24, ss, TYPE16X16, TYPE8X16);

			// line 3
			LCD_Print(2, 32,  "OBD MAF:",TYPE16X16,TYPE8X16);
			d = m_maf;
			sprintf(ss,"%6d",d);
			LCD_Print(64, 32, ss,TYPE16X16,TYPE8X16);

			// line 4
			//LCD_Print(2, 48,  "M DUTY:",TYPE16X16,TYPE8X16);
			LCD_Print(2, 48,  "SNR rpm:",TYPE16X16,TYPE8X16);
			//d = m1_out_pwm_d;
			d = sensor_rpm;
			sprintf(ss,"%6d",d);
			LCD_Print(64, 48, ss,TYPE16X16,TYPE8X16);
	
#endif




		//=========================================================================================================================
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, M2OUT_SW_Pin|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : M2OUT_SW_Pin PA10 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = M2OUT_SW_Pin|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin KEY3_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY4_Pin */
  GPIO_InitStruct.Pin = KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned char f_timer_TX=0;
unsigned char f_seg_timer_500ms;
unsigned char f_timer_100us =0;
unsigned char f_timer_10ms=0;

unsigned char f_timer_30ms=0;
unsigned char i,p_tx1,p_tx2;
unsigned char flag_digit_1=1;
unsigned char tx[3];
unsigned char test[6]={"HELLO\n"};
unsigned char tx2_buffer[10]={"123456789"};
unsigned char tx1_buffer[6]={"abcdef"};

unsigned char rx_buffer1[64];
unsigned char rx_buffer2[64];

unsigned char bufferEvent[64];
unsigned char d_timer_30ms;
unsigned char d_timer_TX;
unsigned char TX_delay_val =255;
unsigned char key1_data, key2_data;
unsigned char state,event;
unsigned char e_rp,e_wp;
unsigned char rx1_rp,rx1_wp;
unsigned char rx2_rp,rx2_wp;


unsigned char uart_tx1_flag,uart_tx2_flag;
unsigned char flag_state_tx1, flag_state_tx;

unsigned char digit1,digit2;
char seven_segment_table[17] = {	0b1111110,	// '0'
		    	0b0110000,	// '1'
		   	0b1101101,	// '2'
			0b1111001,	// '3'
			0b0110011,	// '4'
			0b1011011,	// '5'
			0b1011111,	// '6'
			0b1110000,	// '7'
			0b1111111,	// '8'
			0b1111011,	// '9'
			0b1111101,	// 'a'  --10
			0b0011111,	// 'b'  --11
			0b0001101,	// 'c'  --12
			0b0111101,	// 'd'  --13
			0b1101111,	// 'e'	--14
			0b1000111,	// 'f'  --15
			0b0000001 	// '-'  --16

};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void task_timer(void);
void counting_task(void);
void uart_TX2_task(void);
void uart_TX1_task(void);
void uart_RX1_task(void);
void uart_RX2_task(void);
void led_display_task(void);
void key_read_task(void);
void main_task(void);
void setEvent(unsigned char event);
unsigned char getEvent(void);
void seven_segment_driver(char input,char select_digit);
void TX_delay_decrease(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  rx1_rp=0;
  rx2_rp=0;
  rx1_wp=0;
  rx2_wp=0;

  HAL_UART_Receive_IT(&huart4, &rx_buffer1[rx1_wp], 1);
  HAL_UART_Receive_IT(&huart5, &rx_buffer2[rx2_wp], 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	  	 task_timer();
	  	     led_display_task();
	  	     key_read_task();

	  	     uart_RX2_task();
	  	     uart_RX1_task();

	  	    // main_task();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void uart_RX2_task(void)
{

	if(rx2_rp==rx2_wp) return;

	switch(rx_buffer2[rx2_rp++])
	{
		case 'a':
			digit2=10;
			break;
		case 'b':
			digit2=11;
			break;
		case 'c':
			digit2=12;
			break;
		case 'd':
			digit2=13;
			break;
		case 'e':
			digit2=14;
			break;
		case 'f':
			digit2=15;
			break;
		default:
			break;
	}

	if(rx2_rp>63){
    		rx2_rp=0;
   	 }


}


void uart_RX1_task(void)
{

	if(rx1_rp==rx1_wp) return;

	switch(rx_buffer1[rx1_rp++])
	{
		case '0':
			digit1=0;
			break;
		case '1':
			digit1=1;
			break;
		case '2':
			digit1=2;
			break;
		case '3':
			digit1=3;
			break;

		case '4':
			digit1=4;
			break;
		case '5':
			digit1=5;
			break;
		case '6':
			digit1=6;
			break;
		case '7':
			digit1=7;
			break;
		case '8':
			digit1=8;
			break;
		case '9':
			digit1=9;
			break;
		default:
			break;
	}

	if(rx1_rp>63){
    		rx1_rp=0;
   	 }

}

void led_display_task(void)
{
   if(!f_timer_100us) return;
   f_timer_100us =0;

   flag_digit_1=(~flag_digit_1)&0x1;
   if (flag_digit_1){
	   seven_segment_driver(seven_segment_table[digit1],flag_digit_1);
   }
   else{
	   seven_segment_driver(seven_segment_table[digit2],flag_digit_1);
   }

}

void task_timer(void)
{
	if(!f_timer_10ms) return;       // checking if 10 ms timer interrupt is set (10 ms), if set then do timer task
	f_timer_10ms =0;		// clear the flag to wait next interupt

	d_timer_30ms++;			// count timer for 30 ms interval
				// count timer for LED interval

	if(d_timer_30ms==3)		// checking if the count reached 30 ms
	{
		d_timer_30ms =0;	// assign "0" to repeat counting
		f_timer_30ms=1;		// Set flag to inform 30 ms timer is done counting
	}

	d_timer_TX++;
	if(d_timer_TX>=TX_delay_val)     // checking if the count reached LED interval
	{
		d_timer_TX=0;		// assign "0" to repeat counting
		f_timer_TX=1;

	}


}

void key_read_task(void)
{
	if(!f_timer_30ms) return;  		 // Checking if 30 ms counting is done
	f_timer_30ms =0;          		 // clear the flag to wait next counting

	uint8_t key_pindata = (uint8_t)(KEY1_GPIO_Port->IDR & (KEY1_Pin|KEY2_Pin));

	key1_data = key1_data<<1;      		 //Preparing to read KEY1 Input
	key1_data &= 0b00001110;
	key1_data |= ((key_pindata>>2) & 0x1);			 // Read KEY1 Input

	key2_data = key2_data<<1;		 //Preparing to read KEY2 Input
	key2_data &= 0b00001110;
	key2_data |= (key_pindata>>3);    		 // Read KEY2 Input

	if(key1_data == KEY_PRESSED)    	 // Checking if KEY1 is pressed
	{
		setEvent(EVENT_KEY1_PRESSED);    // Store the event in buffer
		uart_TX1_task();
	}

	if(key1_data == KEY_RELEASED)		//  Checking if KEY1 is released
	{
		setEvent(EVENT_KEY1_RELEASED); // Store the event in buffer
	}

	if(key2_data == KEY_PRESSED)		// Checking if KEY2 is pressed
	{
		setEvent(EVENT_KEY2_PRESSED); // Store the event in buffer
		uart_TX1_task();
	}

	if(key2_data == KEY_RELEASED)		//  Checking if KEY2 is released
	{
		setEvent(EVENT_KEY2_RELEASED); // Store the event in buffer
	}

}


void main_task(void)
{
	/*//	uint8_t led = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
	uint8_t led = (uint8_t)(KEY1_GPIO_Port->IDR & (KEY1_Pin|KEY2_Pin));

	if (led==0b1100){
	 uart_TX1_task();
	}// uart_TX2_task();
	digit1++;
	if (digit1>15){
		digit1=0;
	}

	digit2=digit1;
*/
/*
	if (e_rp!=e_wp){
		event = getEvent();   // if there is event then get the event from buffer
	}else{
		event= EVENT_NO_ENTRY;
	}
	switch(state)

	{       case STATE_IDLE:		// if KEY1 was pressed then start up counting


			switch(event)
			{
			case EVENT_KEY1_PRESSED:

				state = STATE_TX1;
				TX_delay_val=50;
				d_timer_TX=0;

				break;

			case EVENT_KEY2_PRESSED:

				state = STATE_TX2;
				TX_delay_val=50;
				d_timer_TX=0;

				break;
			}

			break;


		case STATE_TX1:	     // if KEY2 was pressed then start down counting

			uart_TX1_task();
			switch(event)
			{
			 case EVENT_TX1_TRANSMITTED:
				TX_delay_decrease();
				break;

			 case EVENT_KEY1_RELEASED:

				state = STATE_IDLE;
				break;
			}

			break;


		case STATE_TX2:	     // if KEY2 was pressed then start down counting

			uart_TX2_task();
			switch(event)
			{
			case EVENT_TX2_TRANSMITTED:
				TX_delay_decrease();
				break;

			case EVENT_KEY2_RELEASED:

				state = STATE_IDLE;
				break;

			}

			break;
	}
*/
}

void TX_delay_decrease(void){

	if (TX_delay_val==10) return;

	TX_delay_val-=5;
}


void uart_TX1_task(void)
{

	if(!f_timer_TX) return;  		 // Checking if 30 ms counting is done
	f_timer_TX =0;
	HAL_UART_Transmit(&huart4, &tx1_buffer[p_tx1++], 1, 10);
	setEvent(EVENT_TX1_TRANSMITTED);
	if(p_tx1>5){
	p_tx1=0;
	}

}

void uart_TX2_task(void)
{

	if(!f_timer_TX) return;  		 // Checking if 30 ms counting is done
	f_timer_TX =0;

	HAL_UART_Transmit(&huart5, &tx2_buffer[p_tx2++], 1, 10);
	setEvent(EVENT_TX2_TRANSMITTED);
	if(p_tx2>8){
	p_tx2=0;
	}

}

void setEvent(unsigned char event)
{
	bufferEvent[e_wp] = event;
	e_wp++;
	if (e_wp>63)
	{
		e_wp=0;
	}
}


unsigned char getEvent(void)
{
	unsigned char event = bufferEvent[e_rp];
	e_rp++;
	if (e_rp>63)
	{
		e_rp=0;
	}
 	 return event;
}

void seven_segment_driver(char input, char select_digit)
{
	uint32_t mask = S1_Pin|S2_Pin|S3_Pin|S4_Pin|S5_Pin|S6_Pin|S7_Pin;
	uint32_t val = ((uint32_t) ~input)&mask;
	if (!select_digit)
	{
		val |= (1)<<7;
	}else
	{
		val |= (1)<<8;

	}
	GPIOF->ODR = val;

}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim3)
  {
	  f_timer_100us=1;
  }
  else if(htim == &htim4)
  {
	  f_timer_10ms=1;

  }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	if (huart == &huart4)
	{
		rx1_wp++;
		HAL_UART_Receive_IT(&huart4, &rx_buffer1[rx1_wp], 1);
		 if(rx1_wp>63){
		    	rx1_wp=0;
		    }
	}
	else if(huart == &huart5)
	{
		rx2_wp++;
		HAL_UART_Receive_IT(&huart5, &rx_buffer2[rx2_wp], 1);
		 if(rx2_wp>63){
		    	rx2_wp=0;
		    }
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


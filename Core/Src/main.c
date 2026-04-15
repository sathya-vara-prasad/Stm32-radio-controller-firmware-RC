/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"usbd_hid.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint8_t reportID;
    int8_t x;
    int8_t y;
    int8_t z;
    int8_t rz;
    int8_t rx;
    int8_t ry;
    //   int8_t slider;
    uint16_t buttons;
}__attribute__((packed)) joystickreport;

// Channel calibration data (inspired by RC transmitter code)
typedef struct {
    uint16_t chLower[6];      // Minimum ADC values per axis
    uint16_t chMiddle[6];     // Center ADC values per axis
    uint16_t chUpper[6];      // Maximum ADC values per axis
    int8_t chReverse[6];      // Reverse flags
    int16_t chTrim[6];        // Fine-tune offsets
} CalibrationData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TGL1_PIN GPIO_PIN_0 // PB0
#define TGL2_PIN GPIO_PIN_1 // PB1
#define TGL3_PIN GPIO_PIN_7 //PB7
#define TGL4_PIN GPIO_PIN_8 // PB8
#define TGL5_PIN GPIO_PIN_10 // PB10
#define TGL6_PIN GPIO_PIN_11 // PB11


#define TGL_PORT GPIOB

#define BTN1_PIN GPIO_PIN_3 // PB2
#define BTN2_PIN GPIO_PIN_4 // PB3
#define BTN3_PIN GPIO_PIN_5 // PB4
#define BTN4_PIN GPIO_PIN_6 // PB5
#define BTN_PORT GPIOB

#define DEADZONE 50
#define ADC_CHANNELS 6
#define SAMPLES_PER_CHANNEL 10
#define ADC_BUFFER_SIZE (ADC_CHANNELS * SAMPLES_PER_CHANNEL)
#define EMA_ALPHA 0.15f

// SBUS Protocol Defines
#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00
#define SBUS_MIN 172
#define SBUS_MAX 1811
#define SBUS_CENTER 992
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*uint16_t raw_adc[4];                         // stores raw ADC readings
float avg_buffer[4][SMOOTH_WINDOW] = {0};    // moving average history buffer
float smoothed_adc[4] = {0};                 // smoothed ADC values
*/

uint16_t adc_buffer[ADC_BUFFER_SIZE];
uint16_t adc_filtered[ADC_CHANNELS];
float ema_values[ADC_CHANNELS] = {0};

extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t sbusFrame[25];

// Calibration data with defaults
CalibrationData_t cal = {
		.chLower = {200, 200, 200, 200, 0, 0},
		.chMiddle = {2048, 2048, 2048, 2048, 2048, 2048},
		.chUpper = {3900, 3900, 3900, 3900, 4095, 4095},
		.chReverse = {0, 0, 0, 0, 0,0},
		.chTrim = {0, 0, 0, 0, 0, 0}
		};

int16_t rcChannels[4];  // RC channel values (1000-2000)

uint16_t potentiometers[2] = {0};  // Store 3 potentiometer raw values (0-4095)
uint8_t pot_percent[2] = {0};      // Store as percentages (0-100%)

volatile uint8_t adc_ready = 0;
static bool ema_seeded = false;

int16_t data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void EncodeSBUS(uint16_t channels[16], uint8_t frame[SBUS_FRAME_SIZE]);
void SendSBUSToFlightController(uint8_t *frame);
int16_t MapToSBUS(int8_t joy_value);
void JoystickToSBUS(joystickreport *joy);

uint16_t GetMedianValue(uint16_t *buffer, uint8_t channel);
float EMAFilter(float current, float previous, float alpha);
int16_t MapADCtoRC(uint16_t adc_value, uint8_t axis);
void BuildSBUSFrame(int16_t channels[16]);
int16_t RCtoSBUS(int16_t rc_value);
void ProcessADCData(void);

int16_t JoyToSBUS(int8_t v);
int16_t PotToSBUS(uint16_t pot_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ADC_Calibration(void)
{
    if ((hadc1.Instance->CR2 & ADC_CR2_ADON) != 0) {
        hadc1.Instance->CR2 &= ~ADC_CR2_ADON;
        HAL_Delay(1);
    }
    hadc1.Instance->CR2 |= ADC_CR2_RSTCAL;
    while (hadc1.Instance->CR2 & ADC_CR2_RSTCAL);
    hadc1.Instance->CR2 |= ADC_CR2_CAL;
    while (hadc1.Instance->CR2 & ADC_CR2_CAL);
    HAL_Delay(2);
}

// Median filter implementation
uint16_t GetMedianValue(uint16_t *buffer, uint8_t channel) {
    uint16_t temp_array[SAMPLES_PER_CHANNEL];

    // Extract samples for this channel
    for (int i = 0; i < SAMPLES_PER_CHANNEL; i++) {
        temp_array[i] = buffer[channel + ADC_CHANNELS * i];
    }

    // Bubble sort
    for (int j = 0; j < SAMPLES_PER_CHANNEL - 1; j++) {
        for (int i = 0; i < SAMPLES_PER_CHANNEL - j - 1; i++) {
            if (temp_array[i] > temp_array[i + 1]) {
                uint16_t temp = temp_array[i];
                temp_array[i] = temp_array[i + 1];
                temp_array[i + 1] = temp;
            }
        }
    }

    // Return median
    if (SAMPLES_PER_CHANNEL & 1) {
        return temp_array[SAMPLES_PER_CHANNEL / 2];
    } else {
        return (temp_array[SAMPLES_PER_CHANNEL / 2 - 1] +
                temp_array[SAMPLES_PER_CHANNEL / 2]) / 2;
    }
}

// Exponential Moving Average filter
float EMAFilter(float current, float previous, float alpha) {
    return (alpha * current) + ((1.0f - alpha) * previous);
}

// Map ADC to RC values (1000-2000) with calibration
int16_t MapADCtoRC(uint16_t adc_value, uint8_t axis) {
    int raw = (int)adc_value;
    int result;

    // Apply deadzone around center
    int centered = raw - cal.chMiddle[axis];
    if (abs(centered) < DEADZONE) {
        return 1500;
    }

    // Map using calibration points
    if (raw < cal.chMiddle[axis]) {
        int range_low = cal.chMiddle[axis] - cal.chLower[axis];
        if (range_low < 100) range_low = 100;
        result = 1000 + (raw - cal.chLower[axis]) * 500 / range_low;
    } else {
        int range_high = cal.chUpper[axis] - cal.chMiddle[axis];
        if (range_high < 100) range_high = 100;
        result = 1500 + (raw - cal.chMiddle[axis]) * 500 / range_high;
    }

    // Apply trim and reverse
    result += cal.chTrim[axis];
    if (cal.chReverse[axis]) {
        result = 3000 - result;
    }

    // Clamp
    if (result < 1000) result = 1000;
    if (result > 2000) result = 2000;

    return (int16_t)result;
}

/* USER CODE BEGIN 0 */

// Convert int8_t joystick (-127 to +127) to SBUS (172-1811)
int16_t JoyToSBUS(int8_t v) {
    int32_t scaled = ((int32_t)(v + 127) * (SBUS_MAX - SBUS_MIN)) / 254 + SBUS_MIN;
    if (scaled < SBUS_MIN)  scaled = SBUS_MIN;
    if (scaled > SBUS_MAX)  scaled = SBUS_MAX;
    return (int16_t)scaled;
}

// Convert potentiometer (0-4095) to SBUS (172-1811)
int16_t PotToSBUS(uint16_t pot_value) {
    int32_t scaled = ((int32_t)pot_value * (SBUS_MAX - SBUS_MIN)) / 4095 + SBUS_MIN;
    if (scaled < SBUS_MIN)  scaled = SBUS_MIN;
    if (scaled > SBUS_MAX)  scaled = SBUS_MAX;
    return (int16_t)scaled;
}
void BuildSBUSFrame(int16_t channels[16]) {
    memset(sbusFrame, 0, sizeof(sbusFrame));
    sbusFrame[0] = SBUS_HEADER;

    uint32_t bitIndex = 0;
    for (int i = 0; i < 16; i++) {
        int16_t value = channels[i] & 0x07FF;
        for (int b = 0; b < 11; b++) {
            if (value & (1 << b)) {
                sbusFrame[1 + (bitIndex >> 3)] |= (1 << (bitIndex & 0x07));
            }
            bitIndex++;
        }
    }

    sbusFrame[23] = 0x00;
    sbusFrame[24] = SBUS_FOOTER;
}
void ProcessADCData(void) {
    uint16_t temp_filtered[ADC_CHANNELS];

    // Step 1: Filter ALL 7 channels (median + EMA)
    for (int ch = 0; ch < ADC_CHANNELS; ch++) {
        uint16_t median = GetMedianValue(adc_buffer, ch);

        if (!ema_seeded) {
            ema_values[ch] = (float)median;
        } else {
            ema_values[ch] = EMAFilter((float)median, ema_values[ch], EMA_ALPHA);
        }

        temp_filtered[ch] = (uint16_t)ema_values[ch];
    }
    ema_seeded = true;

    // Step 2: Process JOYSTICKS (channels 0-3) with crosstalk suppression
    for (int ch = 0; ch < 4; ch++) {
        int16_t deviation = (int16_t)temp_filtered[ch] - (int16_t)cal.chMiddle[ch];

        // Snap to center if very close (eliminates crosstalk)
        if (abs(deviation) < 25) {
            adc_filtered[ch] = cal.chMiddle[ch];
        } else {
            adc_filtered[ch] = temp_filtered[ch];
        }

        rcChannels[ch] = MapADCtoRC(adc_filtered[ch], ch);
    }

    // Step 3: Process POTENTIOMETERS (channels 4-6) - direct reading, no deadzone
    for (int i = 0; i < 2; i++) {
        adc_filtered[4 + i] = temp_filtered[4 + i];
        potentiometers[i] = adc_filtered[4 + i];

        // Convert to percentage (0-100%)
        pot_percent[i] = (uint8_t)((potentiometers[i] * 100) / 4095);
    }
}




// DMA conversion complete callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    adc_ready = 1;
}
/*void SendSBUSToFlightController(uint8_t *frame) {
    // Transmit 25-byte SBUS frame
	 HAL_UART_Transmit(&huart3, frame, 25, HAL_MAX_DELAY);


}*/






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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

 // char tg[] = "TX Ready\r\n";
    	             // HAL_UART_Transmit(&huart1, (uint8_t*)tg, sizeof(tg)-1, HAL_MAX_DELAY);

    	              HAL_GPIO_WritePin(invert_p_GPIO_Port, invert_p_Pin, GPIO_PIN_SET);


    	          //



    	           //   ADC_Calibration();

    	             HAL_ADCEx_Calibration_Start(&hadc1);
    	                 HAL_Delay(10);




    	                 // Start DMA in circular mode
    	                 HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
    	                 HAL_Delay(200);

    	                 for (int i = 0; i < 5; i++) {
    	                     while (!adc_ready) { }  // Wait for DMA complete
    	                     adc_ready = 0;
    	                     ProcessADCData();
    	                     HAL_Delay(20);
    	                 }


    	                 // Seed EMA and auto-calibrate center position using freshly acquired samples
    	                 ProcessADCData();
    	                 for (int i = 0; i < 4; i++) {
    	                 cal.chMiddle[i] = adc_filtered[i];
    	                 }


    	                 char cal_msg[150];
    	                 snprintf(cal_msg, sizeof(cal_msg),
    	                          "Calibrated: CH0=%d CH1=%d CH2=%d CH3=%d POT1=%d POT2=%d\r\n",
    	                          cal.chMiddle[0], cal.chMiddle[1], cal.chMiddle[2], cal.chMiddle[3],
    	                          cal.chMiddle[4], cal.chMiddle[5]);
    	              //   HAL_UART_Transmit(&huart1, (uint8_t*)cal_msg, strlen(cal_msg), HAL_MAX_DELAY);


    	                 uint32_t loop_counter = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  joystickreport last_joy = {0};  // previous state
    joystickreport joy = {0};     // current state

    joy.reportID = 1;
    last_joy.reportID = 1;

   // uint8_t idx = 0;              //smoothing buffer index
  while (1)
  {
	  // Wait for DMA complete
	        if (adc_ready) {
	            adc_ready = 0;

	            // Process ADC data with median + EMA filtering
	            ProcessADCData();



	            joy.x  = -(int8_t)(((int32_t)(rcChannels[0] - 1500) * 127) / 500);
	          	            joy.y  = (int8_t)(((int32_t)(rcChannels[1] - 1500) * 127) / 500);
	          	            joy.z  = -(int8_t)(((int32_t)(rcChannels[2] - 1500) * 127) / 500);
	          	            joy.rz = (int8_t)(((int32_t)(rcChannels[3] - 1500) * 127) / 500);


	            joy.rx     = (int8_t)((potentiometers[0] * 254 / 4095) - 127);  // Pot 1
	            joy.ry     = (int8_t)((potentiometers[1] * 254 / 4095) - 127);  // Pot 2
	           // joy.slider = (int8_t)((potentiometers[2] * 254 / 4095) - 127);  // Pot 3



	            uint8_t but1  = !HAL_GPIO_ReadPin(BTN_PORT, BTN1_PIN);
	          	            uint8_t but2  = !HAL_GPIO_ReadPin(BTN_PORT, BTN2_PIN);
	          	            uint8_t but3  = !HAL_GPIO_ReadPin(BTN_PORT, BTN3_PIN);
	          	            uint8_t but4  = !HAL_GPIO_ReadPin(BTN_PORT, BTN4_PIN);
	            uint8_t left1  = HAL_GPIO_ReadPin(TGL_PORT, TGL1_PIN);
	            uint8_t right1 = HAL_GPIO_ReadPin(TGL_PORT, TGL2_PIN);

	            uint8_t sw_left = 0;
	            uint8_t sw_center = 0;
	            uint8_t sw_right = 0;

	            if (left1 == GPIO_PIN_RESET && right1 == GPIO_PIN_SET) {
	                sw_left = 1;
	            }
	            else if (left1 == GPIO_PIN_SET && right1 == GPIO_PIN_RESET) {
	                sw_right = 1;
	            }
	            else {
	                sw_center = 1;
	            }


	            uint8_t left2  = HAL_GPIO_ReadPin(TGL_PORT, TGL3_PIN);
	            uint8_t right2 = HAL_GPIO_ReadPin(TGL_PORT, TGL4_PIN);

	            uint8_t sw_left1 = 0;
	            uint8_t sw_center1 = 0;
	            uint8_t sw_right1 = 0;

	            if (left2 == GPIO_PIN_RESET && right2 == GPIO_PIN_SET) {
	                sw_left1 = 1;
	            }
	            else if (left2 == GPIO_PIN_SET && right2 == GPIO_PIN_RESET) {
	                sw_right1 = 1;
	            }
	            else {
	                sw_center1 = 1;
	            }

	            uint8_t ts1   = !HAL_GPIO_ReadPin(TGL_PORT, TGL5_PIN);
	            uint8_t ts2   = !HAL_GPIO_ReadPin(TGL_PORT, TGL6_PIN);


	   	            joy.buttons =
	                (but1      << 0) | (but2      << 1) | (but3      << 2) | (but4      << 3) | (ts1       << 4) | (ts2       << 5) | (sw_left   << 6) |
	                (sw_center << 7) | (sw_right  << 8) | (sw_left1  << 9) | (sw_center1<< 10)| (sw_right1 << 11);


	            // Send USB HID if changed
	            bool changed = false;
	            if (abs(joy.x  - last_joy.x)  > 2) changed = true;
	            if (abs(joy.y  - last_joy.y)  > 2) changed = true;
	            if (abs(joy.z  - last_joy.z)  > 2) changed = true;
	            if (abs(joy.rz - last_joy.rz) > 2) changed = true;
	            if (abs(joy.rx - last_joy.rx) > 2) changed = true;
	            if (abs(joy.ry - last_joy.ry) > 2) changed = true;
	          //  if (abs(joy.slider - last_joy.slider) > 2) changed = true;
	            if (joy.buttons != last_joy.buttons) changed = true;

	            if (changed) {

	            	/* uint8_t *ptr = (uint8_t*)&joy;
	            	        char raw[100];
	            	        snprintf(raw, sizeof(raw),
	            	                 "USB→[%02X][%02X][%02X][%02X][%02X][%02X]\r\n",
	            	                 ptr[0], ptr[1], ptr[2], ptr[3], ptr[4],
	            	                 sizeof(joystickreport));
	            	        HAL_UART_Transmit(&huart1, (uint8_t*)raw, strlen(raw), 10);*/
	            	joy.reportID = 1;

	                USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&joy, sizeof(joystickreport));
	                memcpy(&last_joy, &joy, sizeof(joystickreport));
	            }

	            // Debug output (throttled)
	            if (loop_counter % 25 == 0) {
	            	 char buf[200];
	            	 snprintf(buf, sizeof(buf),
	            	                     "ADC:%d,%d,%d,%d | POT1=%d(%d%%) POT2=%d(%d%%) | JOY:X=%d Y=%d Z=%d RZ=%d BTN=0x%02X\r\n",
	            	                     adc_filtered[0], adc_filtered[1], adc_filtered[2], adc_filtered[3],
	            	                     potentiometers[0], pot_percent[0],
	            	                     potentiometers[1], pot_percent[1],
	            	                     joy.x, joy.y, joy.z, joy.rz, joy.buttons);
	            	          //  HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 10);
	                HAL_GPIO_TogglePin(Led_Out_GPIO_Port, Led_Out_Pin);
	            }

	            // Build and send SBUS
	            int16_t sbusChannels[16] = {0};

	            // Joysticks: Use joy values directly (bypass RC values)
	            sbusChannels[3] = JoyToSBUS(joy.x);   // Roll
	            sbusChannels[2] = JoyToSBUS(-joy.y);   // Pitch
	            sbusChannels[0] = JoyToSBUS(joy.z);   // Throttle
	            sbusChannels[1] = JoyToSBUS(-joy.rz);  // Yaw



	          	            sbusChannels[4] = PotToSBUS(potentiometers[0]);
	          	            sbusChannels[5] = PotToSBUS(potentiometers[1]);


	          	          // Channel 7: 3-way switch #1 (TGL1+TGL2)
	          	          uint8_t switch1_state = 0;
	          	          if (sw_left) switch1_state = 0;        // Left → SBUS_MIN
	          	          else if (sw_center) switch1_state = 1; // Center → SBUS_CENTER
	          	          else if (sw_right) switch1_state = 2;  // Right → SBUS_MAX
	          	          sbusChannels[6] = SBUS_MIN + (switch1_state * (SBUS_MAX - SBUS_MIN) / 2);

	          	          // Channel 8: 3-way switch #2 (TGL3+TGL4)
	          	          uint8_t switch2_state = 0;
	          	          if (sw_left1) switch2_state = 0;
	          	          else if (sw_center1) switch2_state = 1;
	          	          else if (sw_right1) switch2_state = 2;
	          	          sbusChannels[7] = SBUS_MIN + (switch2_state * (SBUS_MAX - SBUS_MIN) / 2);

	          	          // Channel 9: 2-way toggles (TGL5, TGL6) - 4 combinations
	          	          uint8_t toggle_state_ch9 = (ts1) | (ts2 << 1);
	          	          sbusChannels[8] = SBUS_MIN + (toggle_state_ch9 * (SBUS_MAX - SBUS_MIN) / 3);

	          	          // Channel 10: Momentary buttons (BTN1, BTN2) - 4 combinations
	          	          uint8_t button_state_ch10 = (but1) | (but2 << 1);
	          	          sbusChannels[9] = SBUS_MIN + (button_state_ch10 * (SBUS_MAX - SBUS_MIN) / 3);

	          	          // Channel 11: Momentary buttons (BTN3, BTN4) - 4 combinations
	          	          uint8_t button_state_ch11 = (but3) | (but4 << 1);
	          	          sbusChannels[10] = SBUS_MIN + (button_state_ch11 * (SBUS_MAX - SBUS_MIN) / 3);

	          	          // Channels 12-16: Unused (center position)
	          	          for (int i = 11; i < 16; i++) {
	          	              sbusChannels[i] = SBUS_CENTER;
	          	          }
	            BuildSBUSFrame(sbusChannels);
	           HAL_UART_Transmit(&huart1, sbusFrame, SBUS_FRAME_SIZE, 10);

	            loop_counter++;
	        }

	        HAL_Delay(8);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_Out_GPIO_Port, Led_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(invert_p_GPIO_Port, invert_p_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Led_Out_Pin */
  GPIO_InitStruct.Pin = Led_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB3 PB4 PB5 PB6
                           PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : invert_p_Pin */
  GPIO_InitStruct.Pin = invert_p_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(invert_p_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

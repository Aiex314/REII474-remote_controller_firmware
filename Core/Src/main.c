/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "st7920_driver.h"
#include "nRF24_driver.h"

#include "message_protocol.h"

#include "command_processor.h"

#include "bitmaps.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.14159265358979323
const float RADS_TO_RPM = 60/(2*PI);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef* lcd_us_timer = &htim11;

UART_HandleTypeDef* huart_dbg = &huart1;

SPI_HandleTypeDef* nrf_spi = &hspi1;
SPI_HandleTypeDef* lcd_spi = &hspi3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t POT_result = 2048;

RWP_MODE rwp_mode = RWP_MODE_HALTED;
RWP_DATA_KIND rwp_data = SENSOR_DATA;
RWP_CTRL_MODE rwp_ctrl = RWP_CTRL_LINEAR;

uint8_t control_mode;
float rpm = 0.0, pitch, roll, yaw;
float v_a, v_b, v_c, v_d, v_e, v_f, ctrl_value;


typedef enum {
  MAIN_MENU,
  MANUAL_MENU,
  AUTO_MENU,
  DEMO_MENU,
  CALIBRATION_MENU,
  HELP_MENU,
  DEBUG_MENU,
  EASTER_EGG_MENU,
  ERROR_ACKNOWLEDGE,
  CONNECTION_PENDING
} rmt_mode;

rmt_mode cur_rmt_mode  = MAIN_MENU;
rmt_mode prev_rmt_mode = MAIN_MENU;

int8_t menu_sel_index = 0;
uint16_t frame_counter = 0;

uint8_t button_state[4] = {0};

static uint8_t assistance_mode;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  static uint32_t tick_count = 0;

  // Operate the nRF24 at 100Hz
  // Also process UART at 100Hz
  if (htim == &htim4) {
    // If the nRF24 received a response packet, get and parse it.
    uint8_t status = readNRF24status();
    if ((status & 0b00100000) != 0) {
      RWP_MESSAGE_FRAME rwp_message;

      // Retrieve acknowledgement payload
      readNRF24payload(rwp_message.bytes, 32);

      if (rwp_message.control_data.data_type == SENSOR_DATA) {
    	// Get the sensor readings
        rpm   = rwp_message.sensor_data.RPM;
        pitch = rwp_message.sensor_data.pitch;
        roll  = rwp_message.sensor_data.roll;
        yaw   = rwp_message.sensor_data.yaw;

        // Get the RWP's control mode
        rwp_ctrl = rwp_message.sensor_data.ctrl_mode;

        if (getDebug() == 1) {
          char buffer[256];
          float t = (float)tick_count/100.0;
          sprintf(buffer, "%6.2f, %8.5f, %8.5f, %8.5f, %5.1f\r\n", t, yaw, pitch, roll, rpm);

          uint8_t len = strlen(buffer);

          putString((uint8_t*)buffer, len);
        }
      }
      else if (rwp_message.control_data.data_type == CONTROL_DATA) {
        v_a = rwp_message.control_data.v_a;
        v_b = rwp_message.control_data.v_b;
        v_c = rwp_message.control_data.v_c;
        v_d = rwp_message.control_data.v_d;
        v_e = rwp_message.control_data.v_e;
        v_f = rwp_message.control_data.v_f;
        ctrl_value = rwp_message.control_data.ctrl_value;

        if (getDebug() == 1) {
          char buffer[256];
          float t = (float)tick_count/100.0;
          sprintf(buffer, "%6.2f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\r\n", t, v_a, v_b, v_c, v_d, v_e, v_f, ctrl_value);

          uint8_t len = strlen(buffer);

          putString((uint8_t*)buffer, len);
        }
      }


      // Clear TX DS interrupt
      writeNRF24register(N24_STATUS, status | 0b00100000);
    }

    // On retransmission overflow, silently ignore...
    if ((status & 0b00010000) != 0) {
      writeNRF24register(N24_STATUS, status | 0b00010000);
    }

    // Set up a message frame
    RMT_MESSAGE_FRAME rmt_message;
    memset(rmt_message.bytes, 0x00, 16);

    // Put the potentiometer reading in the transmit buffer
    rmt_message.control_msg.req_mode = rwp_mode;
    rmt_message.control_msg.req_data_type = rwp_data;
    rmt_message.control_msg.pot_value = POT_result-2048;

    // Send the message
    sendNRF24payload(rmt_message.bytes, 16);


    processUART();

    ++tick_count;
  }
  // Sample and debounce the buttons at 1kHz
  // Also sample and filter the potentiometer at 1kHz
  else if (htim == &htim5) {
    // Sample the buttons
    GPIO_PinState btn1 = HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin); // Yes, the numbers disagree
    GPIO_PinState btn2 = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin); // I built the remote incorrectly...
    GPIO_PinState btn3 = HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin);
    GPIO_PinState btn4 = HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin);

    GPIO_PinState cur_states[4] = {
      btn1,
      btn2,
      btn3,
      btn4
    };

    static uint8_t button_counters[4];
    uint8_t num_pressed = 0;


    for (int i=0; i<4; i++) {
      if (cur_states[i] == GPIO_PIN_SET) button_counters[i] = 0;
      else ++num_pressed;

      if (button_counters[i] >= 5) button_state[i] = 1;
      else {
        button_state[i] = 0;
        ++button_counters[i];
      }
    }

    // Get a reading from the potentiometer
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    uint16_t result = HAL_ADC_GetValue(&hadc1);
    // Filter the result
    POT_result = 0.6*POT_result + 0.4*result;

    // Press all four buttons with knob all the way
    // to the right to get into debug mode
    if (num_pressed == 4 && POT_result < 10)
      setDebug(1);
  }
}

void btnSelect() {
  switch (cur_rmt_mode) {
    case DEBUG_MENU:
      if (rwp_mode == RWP_MODE_MANUAL) rwp_mode = RWP_MODE_AUTO;
      else rwp_mode = RWP_MODE_MANUAL;
      break;
    case ERROR_ACKNOWLEDGE:
      rwp_mode = RWP_MODE_HALTED;
      cur_rmt_mode = MAIN_MENU;
      break;
    case MAIN_MENU: {
      rmt_mode entry_mode[] = {
        MANUAL_MENU,
        AUTO_MENU,
//        DEMO_MENU,
        CALIBRATION_MENU,
        HELP_MENU,
        DEBUG_MENU,
        EASTER_EGG_MENU
      };

      cur_rmt_mode = entry_mode[menu_sel_index];
    }
    default:
      break;
  }
}

void btnBack() {
  switch (cur_rmt_mode) {
    case MAIN_MENU:
      break;
    default:
      cur_rmt_mode = MAIN_MENU;
      rwp_mode = RWP_MODE_HALTED;
      menu_sel_index = 0;
      break;
  }
}

void btnUp() {
  switch (cur_rmt_mode) {
    case DEBUG_MENU:
      if (rwp_mode == RWP_MODE_HALTED) rwp_mode = RWP_MODE_LOCK;
      else if (rwp_mode == RWP_MODE_LOCK) rwp_mode = RWP_MODE_CALIBRATION;
      else rwp_mode = RWP_MODE_HALTED;
      break;
    default:
      --menu_sel_index;
      break;
  }
}

void btnDown() {
  switch (cur_rmt_mode) {
    case DEBUG_MENU:
      if (rwp_data == CONTROL_DATA) rwp_data = SENSOR_DATA;
      else rwp_data = CONTROL_DATA;
      break;
    default:
      ++menu_sel_index;
      break;
  }

  return;
}

void btnHandler() {
  // Buttons ordered from top left to bottom right
  static uint8_t prev_state[4] = {0, 0, 0, 0};
  void (*btnFn[4])() = {&btnBack, &btnUp, &btnSelect, &btnDown};

  for (int i=0; i<4; i++) {
    if (prev_state[i] == 0 && button_state[i] == 1) {
      prev_state[i] = 1;

      btnFn[i]();
    }
    else if (button_state[i] == 0) {
      prev_state[i] = 0;
    }
  }
}


void updateDisplay() {
  // General purpose string buffer
  uint8_t buffer[32];

  // Detect button presses for UI
  static uint8_t prev_state[4] = {0};
  uint8_t btn_pressed[4] = {0};
  for (int i=0; i<4; i++) {
    btn_pressed[i] = 0;
    if (prev_state[i] == 0 && button_state[i] == 1) {
      prev_state[i] = 1;
      btn_pressed[i] = 1;
    }
    else if (button_state[i] == 0) {
      prev_state[i] = 0;
    }
  }

  // When switching remote modes, completely clear the buffer
  static rmt_mode last_rmt_mode = DEBUG_MENU;
  uint8_t new_menu = (last_rmt_mode != cur_rmt_mode);
  if (new_menu) LCDclearBuffer(0x00);
  last_rmt_mode = cur_rmt_mode;

  switch (cur_rmt_mode) {
  case DEBUG_MENU:
    // Print the button statusses
    sprintf((char*)buffer, "BTNS [%.1d%.1d%.1d%.1d]", button_state[0], button_state[1], button_state[2], button_state[3]);
    LCDdrawEmphString(buffer, 11, 0, 0);

    // Display the last potentiometer reading
    float voltage = 3.3/4096.0 * POT_result;
    sprintf((char*)buffer, "POT  %1.02fV", voltage);
    LCDdrawString(buffer, 10, 10, 0);


    if (rwp_data == SENSOR_DATA) {
      // Display the received data
      sprintf((char*)buffer, "RPM   %5d", (int)rpm);
      LCDdrawString(buffer, 11, 28, 0);

      sprintf((char*)buffer, "Y    %6.2f", yaw);
      LCDdrawString(buffer, 11, 37, 0);

      sprintf((char*)buffer, "P    %6.2f", pitch);
      LCDdrawString(buffer, 11, 46, 0);

      sprintf((char*)buffer, "R    %6.2f", roll);
      LCDdrawString(buffer, 11, 54, 0);


      LCDdrawRect(67, 27, 60, 37, 0);

      char* s = "????";
      if (rwp_ctrl == RWP_CTRL_NONE || rwp_mode != RWP_MODE_AUTO) s = "NONE";
      else if (rwp_ctrl == RWP_CTRL_LINEAR) s = "LINR";
      else if (rwp_ctrl == RWP_CTRL_SWINGUP) s = "SWUP";
      else if (rwp_ctrl == RWP_CTRL_SWINGDOWN) s = "SWDN";
      sprintf((char*)buffer, "CMODE %s", s);
      LCDdrawString(buffer, 10, 37, 68);
    }
    else {
      sprintf((char*)buffer, "CVAL %6.3f", ctrl_value);
      LCDdrawString(buffer, 11, 28, 0);

      sprintf((char*)buffer, "a_p %7.4f", v_a);
      LCDdrawString(buffer, 11, 37, 0);

      sprintf((char*)buffer, "w_p %7.4f", v_b);
      LCDdrawString(buffer, 11, 46, 0);

      sprintf((char*)buffer, "w_f %7.4f", v_c);
      LCDdrawString(buffer, 11, 54, 0);

      sprintf((char*)buffer, "PEp %6.3f", v_d);
      LCDdrawString(buffer, 10, 37, 68);

      sprintf((char*)buffer, "KEp %6.3f", v_e);
      LCDdrawString(buffer, 10, 46, 68);

      sprintf((char*)buffer, "KEf %6.3f", v_f);
      LCDdrawString(buffer, 10, 54, 68);
    }

    char* s = "????";
    if (rwp_mode == RWP_MODE_MANUAL) s = "MANL";
    else if (rwp_mode == RWP_MODE_AUTO) s = "AUTO";
    else if (rwp_mode == RWP_MODE_HALTED) s = "HALT";
    else if (rwp_mode == RWP_MODE_CALIBRATION) s = "CALI";
    else if (rwp_mode == RWP_MODE_LOCK) s = "LOCK";
    else if (rwp_mode == RWP_MODE_DEMO) s = "DEMO";
    sprintf((char*)buffer, "MODE  %s", s);
    LCDdrawString(buffer, 10, 28, 68);

    // Nice divider
    LCDdrawRect(66, 27, 1, 37, 1);
    break;
  case MAIN_MENU:
    // Header
    LCDclearBuffer(0x00);
    LCDdrawString((uint8_t*)"Menu", 4, 1, 51);
    LCDdrawRect(1, 9, 126, 1, 1);

    // Print entries
    char* menu_entries[] = {
      "Manual mode",
      "Automatic mode",
//      "Demo mode",
      "Calibration",
      "Help",
      "Debug",
      "Easter eggs :)"
    };

    // Set menu length based on what has been unlocked...
    uint8_t l = 4;
    if (getDebug()) l = 5;

    menu_sel_index = (l+menu_sel_index) % l;

    static uint8_t start_index = 0;
    if (menu_sel_index < start_index) start_index = menu_sel_index;
    while (menu_sel_index - start_index > 4) start_index++;

    for (uint8_t i=0; i<5; i++) {
      if (i == l) break;

      uint8_t idx = i+start_index;
      if (idx == menu_sel_index)
        LCDdrawEmphString((uint8_t*)menu_entries[idx], strlen(menu_entries[idx]), 10*i + 11, 1);
      else
        LCDdrawStringDeemph((uint8_t*)menu_entries[idx], strlen(menu_entries[idx]), 10*i + 11, 1);
    }

    if (l - start_index > 5) LCDdrawString("v", 1, 56, 121);
    if (start_index > 0) LCDdrawString("^", 1, 11, 121);
    break;
  case MANUAL_MENU:
    // Menu selection bounds
    menu_sel_index = (2+menu_sel_index) % 2;

    // Check whether potentiometer is centered when entering menu
    static uint8_t pot_check = 0;
    if (new_menu) {
      pot_check = 0;
      rwp_mode = RWP_MODE_HALTED;
    }
    if (pot_check == 0) {
      if (POT_result > 2028 && POT_result < 2068) {
        pot_check = 1;
        rwp_mode = RWP_MODE_MANUAL;
        LCDclearBuffer(0x00);
      }
      else {
        // Print message while potentiometer is not centered
        LCDdrawString((uint8_t*)"Please set knob to 0", 20, 28, 4);
        break;
      }
    }
    // Header
    LCDdrawString((uint8_t*)"Manual mode", 11, 1, 31);
    LCDdrawRect(1, 9, 126, 1, 1);
    LCDdrawBitmap(back_icon_bitmap, 1, 1, 8, 8);

    // Print the current control value
    float ctrl_val = (POT_result-2048)/2048.0;
    sprintf((char*)buffer, "Knob value:    % 2.03f", ctrl_val);
    LCDdrawString(buffer, 21, 11, 1);

    // Difficulty selector
    if (new_menu) assistance_mode = 0;
    const char* assistance_names[] = {
      "Easy",
      "Med.",
      "Hard",
    };
    LCDdrawString((uint8_t*)"Difficulty:", 11, 19, 1);
    if (menu_sel_index == 0) {
      LCDdrawEmphString((uint8_t*)assistance_names[assistance_mode], 4, 19, 103);

      if (btn_pressed[2] && !new_menu) assistance_mode = (1+assistance_mode)%3;
    }
    else
      LCDdrawStringDeemph((uint8_t*)assistance_names[assistance_mode], 4, 19, 103);

    break;
  case AUTO_MENU:
    // Header
    LCDdrawString((uint8_t*)"Auto mode", 9, 1, 37);
    LCDdrawRect(1, 9, 126, 1, 1);
    LCDdrawBitmap(back_icon_bitmap, 1, 1, 8, 8);

    // Instruction
	LCDdrawString("Press select to", 15, 11, 19);
	LCDdrawString("toggle Auto mode", 16, 19, 16);

	// State
	static uint8_t cur_mode = 0;
	const char* mode_names[] = {
	  "HALT",
	  "AUTO"
	};
	const RWP_MODE modes[] = {
	  RWP_MODE_HALTED,
	  RWP_MODE_AUTO
	};

    menu_sel_index = (3+menu_sel_index) % 3;

    if (new_menu) cur_mode = menu_sel_index = 0;

	LCDdrawString("State:", 6, 37, 34);
	LCDdrawString(mode_names[cur_mode], 4, 37, 76);

	if (btn_pressed[2] && !new_menu) {
	  cur_mode = 1-cur_mode;
	}

	rwp_mode = modes[cur_mode];

	rwp_data = CONTROL_DATA;

	switch (menu_sel_index) {
	  case 0:
		sprintf((char*)buffer, "P. Angle: %5.2frad  ", v_a);
		break;
	  case 1:
		sprintf((char*)buffer, "P. Speed: %5.2frad/s", v_b);
		break;
	  case 2:
		sprintf((char*)buffer, "RW.Speed: %5.2fRPM  ", v_c*RADS_TO_RPM);
		break;
	}

	LCDdrawString(buffer, 20, 46, 4);

    break;
  case CALIBRATION_MENU:
	// Is calibration complete yet?
	static uint8_t sys_calibrated = 0;
	static uint8_t calibrate_tick = 0;

	// Header
	LCDdrawString((uint8_t*)"Calibration", 11, 1, 31);
	LCDdrawRect(1, 9, 126, 1, 1);
    LCDdrawBitmap(back_icon_bitmap, 1, 1, 8, 8);

	// Instruction
	LCDdrawString("Balance pendulum in", 19, 11, 1);
	LCDdrawString("upright position, &", 19, 19, 1);
	LCDdrawString("press confirm.", 14, 28, 1);

	// After calibration, display confirmation
	LCDdrawRect(22, 54, 84, 8, 0);
	if (sys_calibrated == 0) {
	  rwp_mode = RWP_MODE_HALTED;
	  LCDdrawString("Waiting...", 10, 54, 34);

	  if (btn_pressed[2] && !new_menu) {
	    sys_calibrated = 1;
	    calibrate_tick = 10;
	  }
	}
	else if (calibrate_tick != 0) {
	  rwp_mode = RWP_MODE_CALIBRATION;

	  --calibrate_tick;
	  LCDdrawString("Calibrating...", 14, 54, 22);
	}
	else {
	  rwp_mode = RWP_MODE_HALTED;
	  LCDdrawString("Calibrated!", 11, 54, 31);

	  if (btn_pressed[2] && !new_menu) {
		sys_calibrated = 1;
		calibrate_tick = 10;
	  }
	}
	break;
  case HELP_MENU:
	LCDdrawString((uint8_t*)"Help", 4, 1, 52);
	LCDdrawRect(1, 9, 126, 1, 1);
	LCDdrawBitmap(back_icon_bitmap, 1, 1, 8, 8);

    menu_sel_index = (4+menu_sel_index) % 4;

    if (new_menu) menu_sel_index = 0;

	uint8_t* bitmap_array[] = {
	  help_1,
	  help_2,
	  help_3,
	  help_4
	};

	LCDdrawBitmap(bitmap_array[menu_sel_index], 1, 10, 128, 53);

	break;
  case ERROR_ACKNOWLEDGE:
    // Display the error
    LCDdrawEmphString("RWIP fault:", 11, 1, 1);

    uint8_t err_idx = rwp_mode-0x80;
    const char* err_msgs[] = {
      "Pendulum Overspeed",
      "Impact detected"
    };
    uint8_t msg_len = strlen(err_msgs[err_idx]);
    LCDdrawString(err_msgs[err_idx], msg_len, 28, 64-3*msg_len);

    LCDdrawRect(56, 51, 16, 12, 1);
    LCDdrawRect(57, 52, 14, 11, 0);
    LCDdrawString("OK", 2, 53, 58);
    break;
  default:
    LCDdrawString((uint8_t*)"Bad menu ID", 11, 1, 1);
    break;
  }

  // Update screen.
  LCDupdateFromBuffer();

  // Update frame counter, for anything that wants it
  ++frame_counter;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_Base_Start(&htim11);

  // Initialize the LCD
  LCDinit();
  LCDsetGraphicMode(1);
  LCDclearBuffer(0);
  LCDdrawBootScreen();
  LCDupdateFromBuffer();

  // Initialize the nRF24
  NRF24init();

  // Start the nRF24 timer
  HAL_TIM_Base_Start_IT(&htim4);

  // Start sampling the buttons
  HAL_TIM_Base_Start_IT(&htim5);

  HAL_Delay(500);

  LCDclearBuffer(0);
  LCDupdateFromBuffer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t x = 0;
  while (1)
  {
    // UI refreshes at 20Hz
    btnHandler();
    updateDisplay();
    HAL_Delay(50);


	// Cool demo :)
//    TAKE THESE OUT OF LOOP!
//	  uint8_t x = 0;
//	  uint8_t y = 0;
//	  float t   = 0.0;
//    KEEP THESE IN LOOP!
//    LCDdrawRect(x, y, 5*6, 8, 0);
//    x = 40 + floor(32*sin(1*t));
//    y = 20 + floor(16*sin(2*t));
//    LCDdrawString((uint8_t*)"COOL!", 5, y, x);
//    LCDupdateFromBuffer();
//
//    t += 0.1;
//    HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0xffff;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLINK_Pin */
  GPIO_InitStruct.Pin = BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLINK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_1_Pin BTN_2_Pin BTN_3_Pin BTN_4_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin|BTN_2_Pin|BTN_3_Pin|BTN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CSN_Pin NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin|NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Stepper.h"
#include "HolonomicDrive3.h"

#include "MessageRecomposer.h"
#include "ChampiCan.h"

#include "ChampiState.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "msgs_can.pb.h"
#include "can_ids.hpp"

#include "CUSTOM_LIB_SPARKFUN.h"

#include <stdlib.h>
#include <string.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYS_CORE_CLOCK_HZ 170000000.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Stepper stepper0;
Stepper stepper1;
Stepper stepper2;

HolonomicDrive3 holo_drive;

ChampiCan champi_can;
MessageRecomposer msg_recomposer_cmd_vel;
MessageRecomposer msg_recomposer_config;

ChampiState champi_state;

QwiicOTOS myOtos(&hi2c1, 0x17);
bool needTrackingSensorResetAndCalibration = true;

// On le déclare ici au contraire des autres buffers, car il va servir tout le temps.
uint8_t buffer_encode_tx_vel[30]; // todo 30, c'est large, on peut peut-être réduire.

bool new_config_received = false;
uint32_t time_last_cmd_vel = -1; // ms
uint32_t cmd_vel_timeout; // ms/ set by the configuration message
#define CMD_VEL_TIMEOUT_MAX 1000 // ms. Used to avoid a too long timeout (bad configuration)


uint32_t last_time_loop = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern "C"
{
int _write(int file, char *ptr, int len)
{
   for (int DataIdx = 0; DataIdx < len; DataIdx++)
//        ITM_SendChar(*ptr++);
//   	HAL_UART_Transmit(&huart2, (uint8_t*)ptr++, 1, HAL_MAX_DELAY);
   return len;
}

}

void setup();

void loop();

void on_receive_cmd_vel(const std::string &proto_msg);

void transmit_vel(Vel vel);

void on_receive_config(const std::string &proto_msg);

void transmit_ret_config(msgs_can_BaseConfig ret_config);

void Error_Handler_CAN_ok();

void wait_tx_ok();

void set_loop_freq(int hz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



// ================================== HELPER FUNCTIONS =================================

/**
 * @brief Little helper function to set the frequency of the loop
 * @param hz : the frequency of the loop
 */
void set_loop_freq(int hz) {
    htim6.Instance->ARR = SYS_CORE_CLOCK_HZ / (htim6.Instance->PSC + 1) / hz;
}


// ===================================== CALLBACKS =====================================

/**
  * @brief  Timer callback for main routine.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
//        loop();
    }
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

    // Attention !! Quand on met un breakpoint dans cette fonction, on ne reçoit plus que 2 messages au lieu du
    // bon nombre.

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        /* Retrieve Rx messages from RX FIFO0 */
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
            champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_CAN_RX);
            Error_Handler_CAN_ok();
        }
        /* Handle Interesting messages
         * Pour le moment, on n'utilise pas de mutex ou de choses comme ça, donc il faut faire attention
         * à ne pas modifier trop de variables partagées, et de priviligier la modifcation de variables
         * de 32 bits ou moins (pour que leur modification soit une opération atomique)
         * */

        if (RxHeader.Identifier == CAN_ID_BASE_CMD_VEL) {
            msg_recomposer_cmd_vel.add_frame(RxData, RxHeader.DataLength);

            if (msg_recomposer_cmd_vel.check_if_new_full_msg()) {
                std::string proto_msg = msg_recomposer_cmd_vel.get_full_msg();
                on_receive_cmd_vel(proto_msg);

            }
        }
        else if (RxHeader.Identifier == CAN_ID_BASE_SET_CONFIG) {
            msg_recomposer_config.add_frame(RxData, RxHeader.DataLength);

            if (msg_recomposer_config.check_if_new_full_msg()) {
                std::string proto_msg = msg_recomposer_config.get_full_msg();
                on_receive_config(proto_msg);
            }
        }
        else if (RxHeader.Identifier == CAN_ID_BASE_RESET) {
            // reset. TODO améliorer ça pour ne plus avoir à reset
            NVIC_SystemReset();
        }
        else if (RxHeader.Identifier == CAN_ID_RESET_AND_CALIBRATE_TRACKING_SENSOR) {
          needTrackingSensorResetAndCalibration = true;
        }
    }
}








// ===================================== FUNCTIONS RX/TX ============================================

/**
 * @brief Function to handle the received message from the CAN bus: it decodes the message and updates the
 * velocity command.
 * @param proto_msg : the received message (protobuf encoded)
 */
void on_receive_cmd_vel(const std::string &proto_msg) {

    // Allocate space for the decoded message.
    msgs_can_BaseVel ret_cmd_vel = msgs_can_BaseVel_init_zero;
    // Create a stream that reads from the buffer.
    pb_istream_t stream_ret = pb_istream_from_buffer((const unsigned char *) proto_msg.c_str(), proto_msg.size());
    // Now we are ready to decode the message.
    if (!pb_decode(&stream_ret, msgs_can_BaseVel_fields, &ret_cmd_vel)) {
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_PROTO_DECODE);
        Error_Handler_CAN_ok();
    }

    // Use message
    Vel cmd_vel = {ret_cmd_vel.x, ret_cmd_vel.y, ret_cmd_vel.theta};
    holo_drive.set_cmd_vel(cmd_vel);

    // Update time_last_cmd_vel
    time_last_cmd_vel = HAL_GetTick();
}

/**
 * @brief Function to send the current velocity of the robot on the CAN bus.
 * @param vel : the current velocity of the robot
 */
void transmit_vel(Vel vel) {

    // Init message
    msgs_can_BaseVel vel_proto = msgs_can_BaseVel_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer_encode_tx_vel, sizeof(buffer_encode_tx_vel));

    // Fill message
    vel_proto.x = vel.x;
    vel_proto.y = vel.y;
    vel_proto.theta = vel.theta;
    vel_proto.has_x = true;
    vel_proto.has_y = true;
    vel_proto.has_theta = true;

    // Encode message
    bool status = pb_encode(&stream, msgs_can_BaseVel_fields, &vel_proto);
    size_t message_length = stream.bytes_written;

    // Check for errors
    if (!status) {
        // TODO on peut récupérer un message d'erreur avec PB_GET_ERROR(&stream))
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_PROTO_ENCODE);
        Error_Handler_CAN_ok();
    }

    // Send
    if (champi_can.send_msg(CAN_ID_BASE_CURRENT_VEL, (uint8_t *) buffer_encode_tx_vel, message_length) != 0) {
        /* Transmission request Error */
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_CAN_TX);
        Error_Handler_CAN_ok();
    }
}

void transmitTrackingPoseAndStd(msgs_can_TrackingSensorData_StatusType status, Pose2D pose, Pose2D std) {
    // Init message
    msgs_can_TrackingSensorData tracking_data_proto = msgs_can_TrackingSensorData_init_zero;
    msgs_can_TrackingSensorStd tracking_std_proto = msgs_can_TrackingSensorStd_init_zero;

    uint8_t buff[60];
    uint8_t buff2[60];
    pb_ostream_t stream = pb_ostream_from_buffer(buff, sizeof(buff));
    pb_ostream_t stream2 = pb_ostream_from_buffer(buff2, sizeof(buff2));

    // Fill message
    tracking_data_proto.status = status;
    tracking_data_proto.pose_x_mm = pose.x;
    tracking_data_proto.pose_y_mm = pose.y;
    tracking_data_proto.theta_rad = pose.h;
    tracking_data_proto.has_status = true;
    tracking_data_proto.has_pose_x_mm = true;
    tracking_data_proto.has_pose_y_mm = true;
    tracking_data_proto.has_theta_rad = true;

    tracking_std_proto.pose_x_std = std.x;
    tracking_std_proto.pose_y_std = std.y;
    tracking_std_proto.theta_std = std.h;
    tracking_std_proto.has_pose_x_std = true;
    tracking_std_proto.has_pose_y_std = true;
    tracking_std_proto.has_theta_std = true;

    // Encode message
    bool ok = pb_encode(&stream, msgs_can_TrackingSensorData_fields, &tracking_data_proto);
    ok = ok && pb_encode(&stream2, msgs_can_TrackingSensorStd_fields, &tracking_std_proto);

    size_t message_length = stream.bytes_written;
    size_t message_length2 = stream2.bytes_written;

    // Check for errors
    if (!ok) {
        // TODO on peut récupérer un message d'erreur avec PB_GET_ERROR(&stream))
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_PROTO_ENCODE);
        Error_Handler_CAN_ok();
    }

    // Send
    if (champi_can.send_msg(CAN_ID_TRACKING_SENSOR_DATA, (uint8_t *) buff, message_length) != 0) {
        /* Transmission request Error */
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_CAN_TX);
        Error_Handler_CAN_ok();
    }
    if (champi_can.send_msg(CAN_ID_TRACKING_SENSOR_STD, (uint8_t *) buff2, message_length2) != 0) {
        /* Transmission request Error */
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_CAN_TX);
        Error_Handler_CAN_ok();
    }
}

void on_receive_config(const std::string &proto_msg) {

    // Allocate space for the decoded message.
    msgs_can_BaseConfig ret_config = msgs_can_BaseConfig_init_zero;
    // Create a stream that reads from the buffer.
    pb_istream_t stream_ret = pb_istream_from_buffer((const unsigned char *) proto_msg.c_str(), proto_msg.size());
    // Now we are ready to decode the message.
    if (!pb_decode(&stream_ret, msgs_can_BaseConfig_fields, &ret_config)) {
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_PROTO_DECODE);
        Error_Handler_CAN_ok();
    }

    // Check if the message is valid
    if (ret_config.has_base_radius && ret_config.has_wheel_radius && ret_config.has_max_accel && ret_config.has_cmd_vel_timeout) {
        // Set the configuration
        holo_drive.set_config(ret_config.max_accel, ret_config.wheel_radius, ret_config.base_radius);
        cmd_vel_timeout = (uint32_t) (ret_config.cmd_vel_timeout * 1000.0); // s to ms
        if (cmd_vel_timeout > CMD_VEL_TIMEOUT_MAX) {
            cmd_vel_timeout = CMD_VEL_TIMEOUT_MAX;
        }
        // Transmit it back to acknowledge the reception
        transmit_ret_config(ret_config);

        new_config_received = true;
    }
}

void transmit_ret_config(msgs_can_BaseConfig ret_config) {
    // Init stream
    uint8_t buff[30]; // todo 30, c'est large, on peut peut-être réduire.
    pb_ostream_t stream = pb_ostream_from_buffer(buff, sizeof(buff));

    // Encode message
    bool status = pb_encode(&stream, msgs_can_BaseConfig_fields, &ret_config);
    size_t message_length = stream.bytes_written;

    // Check for errors
    if (!status) {
        // TODO on peut récupérer un message d'erreur avec PB_GET_ERROR(&stream))
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_PROTO_ENCODE);
        Error_Handler_CAN_ok();
    }

    // Send
    if (champi_can.send_msg(CAN_ID_BASE_RET_CONFIG, (uint8_t *) buff, message_length) != 0) {
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_CAN_TX);
        Error_Handler_CAN_ok();
    }
}

/**
 * @brief Fonction qui attend que le l'envoi de données sur le CAN fonctionne. Ca envoie un message de test
 * à répétition jusqu'à ce que ça fonctionne.
 * Also blinks the built-in LED at 5 Hz.
 * TODO replace BASE_TEST by status message ?
 */
void wait_tx_ok() {
    uint8_t buff[20] = {0}; // We need a big message to fill the FIFO

    // Send a message to test if the can bus works (at least 1 node up)
    uint32_t ret = champi_can.send_msg(CAN_ID_BASE_TEST, (uint8_t *) buff, 20);

    if(ret==0){
        return;
    }

    // If we get an error, retry doesn't work sometimes. So we reset the stm to try again. Also blink the led 10Hz

    // blink the built-in LED for 1s
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(Built_in_LED_GREEN_GPIO_Port, Built_in_LED_GREEN_Pin);
        HAL_Delay(100);
    }

    // Beep
    htim17.Instance->CCR1 = 1000;
    HAL_Delay(200);
    htim17.Instance->CCR1 = 0;

    // Then reset the stm
    NVIC_SystemReset();

}

/**
 * @brief Error handler we call when CAN might still work.
 * It blinks the built-in LED at 1Hz AND sends status on CAN bus.
 */
void Error_Handler_CAN_ok() {

    // Stop robot
    holo_drive.set_cmd_vel(Vel{0, 0, 0});

    // Blink the built-in LED at 1Hz
    uint32_t last_time = HAL_GetTick();
    while (true) {
        holo_drive.spin_once_motors_control(); // Stop the robot (respect acceleration limits)
        champi_state.spin_once();
        HAL_Delay(10); // 10ms required to match the main loop frequency (for control)

        if (HAL_GetTick() - last_time > 500) {
            last_time = HAL_GetTick();
            htim17.Instance->CCR1 == 10000 ? htim17.Instance->CCR1 = 0 : htim17.Instance->CCR1 = 10000; // Beeper
            HAL_GPIO_TogglePin(Built_in_LED_GREEN_GPIO_Port, Built_in_LED_GREEN_Pin); // The built-in LED
        }
    }
}




// ==================================== EMERGENCY_STOP ============================================

/**
 * @brief Function to check if the emergency stop is pressed.
 * @return true if the emergency stop is pressed, false otherwise.
 */

bool is_emergency_stop_pressed()
{
    return HAL_GPIO_ReadPin(EMERGENCY_STOP_GPIO_Port, EMERGENCY_STOP_Pin) == GPIO_PIN_RESET;
}



// ========================================= TIRETTE ================================================

/**
 * @brief Function to check if the tirette has just been pulled (low to high transition).
 * @return true if the tirette is pulled, false otherwise.
 */

bool is_tirette_pulled() {
	static bool first_time = true;
    static GPIO_PinState last_state = GPIO_PIN_RESET;
	if(first_time) {
		last_state = HAL_GPIO_ReadPin(TIRETTE_GPIO_Port, TIRETTE_Pin);
		first_time = false;
		return false;
	}

    GPIO_PinState current_state = HAL_GPIO_ReadPin(TIRETTE_GPIO_Port, TIRETTE_Pin);

    if (current_state == GPIO_PIN_SET && last_state == GPIO_PIN_RESET) {
        last_state = current_state;
        return true;
    }

    last_state = current_state;
    return false;
}

void send_can_tirette_pulled() {

    // Send message containing 1 byte
    uint8_t buff[1] = {0};
    champi_can.send_msg(CAN_ID_TIRETTE_START, buff, 1);
}







// ===================================== SETUP AND LOOP ============================================


/**
 * @brief Setup function.
 */
void setup() {
	printf("starting setup...\n");
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

    while (!myOtos.isConnected()) {
    	printf("otos not connected\n");
        HAL_Delay(1000);
        htim17.Instance->CCR1 == 10000 ? htim17.Instance->CCR1 = 0 : htim17.Instance->CCR1 = 10000;  // beeper
        HAL_GPIO_TogglePin(Built_in_LED_GREEN_GPIO_Port, Built_in_LED_GREEN_Pin); // The built-in LED
    }

    stepper0 = Stepper(htim8, TIM_CHANNEL_1, GPIOA, GPIO_PIN_4);
    stepper1 = Stepper(htim1, TIM_CHANNEL_1, GPIOA, GPIO_PIN_0);
    stepper2 = Stepper(htim15, TIM_CHANNEL_1, GPIOA, GPIO_PIN_1);

    holo_drive = HolonomicDrive3(stepper0, stepper1, stepper2);

    champi_can = ChampiCan(&hfdcan1);
    msg_recomposer_cmd_vel = MessageRecomposer();
    msg_recomposer_config = MessageRecomposer();

    if (champi_can.start() != 0) {
        // TODO: On a jamais rencontré cette erreur.
        Error_Handler();
    }

    // This is required: when the Raspberry Pi starts up, transmit CAN frames returns error.
    wait_tx_ok();

    champi_state = ChampiState(&champi_can, 500);

    champi_state.report_status(msgs_can_Status_StatusType_INIT, msgs_can_Status_ErrorType_NONE);

    // Wait for the configuration message (blink 5Hz)
    while (!new_config_received) {
        HAL_Delay(200);
        htim17.Instance->CCR1 == 10000 ? htim17.Instance->CCR1 = 0 : htim17.Instance->CCR1 = 10000;  // beeper
        HAL_GPIO_TogglePin(Built_in_LED_GREEN_GPIO_Port, Built_in_LED_GREEN_Pin); // The built-in LED
        // Send status to the CAN bus regularly
        champi_state.spin_once();
    }


    bool ok = myOtos.selfTest();
    if (! ok) {
    	// ERROR WITH OTOS
        transmitTrackingPoseAndStd(msgs_can_TrackingSensorData_StatusType::msgs_can_TrackingSensorData_StatusType_ERROR, Pose2D(), Pose2D()); // update status
        HAL_Delay(1000000000); // TODO better
    }

    myOtos.setAngularScalar(1.07);
    myOtos.setLinearScalar(0.992);

    Pose2D offset = {0, 0.049844, -M_PI/2.0};
    myOtos.setOffset(offset);

    // Switch led ON to indicate that the configuration is done
    htim17.Instance->CCR1 == 0; // Beeper
    HAL_GPIO_WritePin(Built_in_LED_GREEN_GPIO_Port, Built_in_LED_GREEN_Pin, GPIO_PIN_SET);

    champi_state.report_status(msgs_can_Status_StatusType_OK, msgs_can_Status_ErrorType_NONE);

    // We got everything, start the main loop
//    set_loop_freq(20);
//    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief Main loop.
 */
void loop() {



	if (needTrackingSensorResetAndCalibration)
	{
		myOtos.calibrateImu();
		HAL_Delay(10);
		myOtos.resetTracking();
		HAL_Delay(10);
		needTrackingSensorResetAndCalibration = false;
//		printf("RESETTING TRACKING\n");
	}

	// Obtenir la position actuelle
	Pose2D otosPose = myOtos.getPosition();
	Pose2D otosStd = myOtos.getPositionStdDev();
//	printf("%d",(int)otosPose.x*1000);
//	printf("\t");
//	printf("%d",(int)otosPose.y*1000);
//	printf("\t");
//	printf("%d",(int)otosPose.h);
//	printf("\n");
	transmitTrackingPoseAndStd(msgs_can_TrackingSensorData_StatusType::msgs_can_TrackingSensorData_StatusType_OK, otosPose, otosStd);
	// TODO plutot transmit la pose à interval régulier, ou quand on recoit une nouvelle pose du capteur


    // Check if the command velocity is too old
    if (time_last_cmd_vel != -1 && (HAL_GetTick() - time_last_cmd_vel > cmd_vel_timeout)) {
        // Report the error
        champi_state.report_status(msgs_can_Status_StatusType_ERROR, msgs_can_Status_ErrorType_CMD_VEL_TIMEOUT);
        Error_Handler_CAN_ok();
    }

    holo_drive.spin_once_motors_control();

    transmit_vel(holo_drive.get_current_vel());

    champi_state.spin_once();

    // Execute the following every 500ms
    static uint32_t last_time = HAL_GetTick();
    if (HAL_GetTick() - last_time < 200) {
        return;
    }
    last_time = HAL_GetTick();

    // check emergency stop. If pressed, set enable pin to high
    if (is_emergency_stop_pressed()) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    // Check if the tirette is pulled. If it is, send CAN message
    if (is_tirette_pulled()) {
        send_can_tirette_pulled();
    }

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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

    setup();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    double freq = 100; // Hz

    while (true) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    	uint32_t diff_time  = HAL_GetTick() - last_time_loop;
    	if (diff_time/1000. >= 1./freq)
    	{
        	last_time_loop = HAL_GetTick();
    		loop();
    	}

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
    /*
     * Cette fonction est appelée lorsqu'il y a une erreur grave. On pourrait réfléchir à des comportements
     * pour résoudre les problèmes, mais je pense que le mieux pour l'instant, c'est de tout bloquer pour
     * être sûr de voir le problème.
     */
    while (true) {
        // Blink 1Hz
        HAL_GPIO_TogglePin(Built_in_LED_GREEN_GPIO_Port, Built_in_LED_GREEN_Pin);
        htim17.Instance->CCR1 == 10000 ? htim17.Instance->CCR1 = 0 : htim17.Instance->CCR1 = 10000; // Beeper
        HAL_Delay(1000);
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

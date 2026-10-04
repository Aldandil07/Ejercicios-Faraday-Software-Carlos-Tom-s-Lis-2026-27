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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp3.h"
#include "math.h"
#include "h3lis331dl_reg.h"
#include "stm32f4xx_hal.h"
#include "lwgps.h"
#include "ism330bx_reg.h"
#include "ekfprototipo2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern I2C_HandleTypeDef hi2c1; //barómetro y giroscpio/IMU
extern I2C_HandleTypeDef hi2c2; //acelerómetro
extern I2C_HandleTypeDef hi2c3; //gps

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP390_I2C_ADDR (0x76 << 1)
#define H3LIS331DL_I2C_ADD (0x18 << 1)
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#define TESEO_I2C_ADDR      (0x3A << 1) // Dirección I2C shifted
#define RX_BUFFER_SIZE      256
#define ISM330BX_I2C_ADD ISM330BX_I2C_ADD_H
#define ALPHA 0.98f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;


/* USER CODE BEGIN PV */
uint32_t current_time = 0;
uint32_t last_time_process = 0;

/*Barómetro*/
uint32_t intervalo_bar = 500;
uint32_t last_time_bar = 0;
struct bmp3_dev dev;
struct bmp3_data data = { 0 };
struct bmp3_settings settings = { 0 };
uint16_t settings_sel;
uint8_t dev_addr = (BMP3_ADDR_I2C_PRIM << 1);
float presion_hpa = 0.0f;
float temperatura_c = 0.0f;

/*Acelerómetro*/
uint32_t intervalo_accel = 10;
uint32_t last_time_accel = 0;
float accel_g[3];
float altitud_m = 0.0f;
float a_total = 0.0f;


/*Sensor gnss*/
uint32_t intervalo_gps = 100;
uint32_t last_time_gps = 0;
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t gps_rx_buffer[128];
lwgps_t hgps;
float longitud;
float latitud;
float altitud;

/*Giroscopio*/
uint32_t intervalo_imu = 10;
uint32_t last_time_imu = 0;

stmdev_ctx_t dev_ctx;
int16_t data_raw_angular_rate[3];
float angular_rate_mdps[3];
uint8_t whoamI;

//De cara al ekf
struct RawSensorData raw_sensor_data = {0};
AltitudeEstimate_t altitude_estimate = {0.0f, 0.0f};
uint8_t barometer_ready = 0;
uint8_t barometer_updated = 0;
uint8_t accelerometer_ready = 0;
uint8_t altitude_ekf_initialized = 0;


/*Resumen de datos:
GPS: longitud, latitud, altitud.
Barómetro: altitud_m.
Acelerómetro: accel_g[3], a_total.
Giroscopio: inc.pitch, inc.roll e inc.yaw.
El yaw es relativo y deriva sin una referencia magnética.

*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);

HAL_StatusTypeDef TeseoV_Read_I2C(uint8_t *pData, uint16_t Size) {
    // Transmisión/recepción del flujo continuo NMEA desde el búfer del Teseo V
    return HAL_I2C_Master_Receive(&hi2c3, TESEO_I2C_ADDR, pData, Size, 100);
}

// Función para calcular la altitud en metros
float calcular_altitud(float presion_hpa, float presion_nivel_mar_hpa) {
//1013.25f
return 44330.0f * (1.0f - powf(presion_hpa / presion_nivel_mar_hpa, 0.190295f));
}


//aceleración total:
float calcular_aceleracion_total(float ax, float ay, float az)
{
    return sqrtf((ax * ax) + (ay * ay) + (az * az));
}

//pitch roll:
typedef struct {
    float pitch;
    float roll;
    float yaw;
} Inclinacion_t;

Inclinacion_t angulos = {0.0f, 0.0f, 0.0f}; // Inicialización de la estructura de inclinación

Inclinacion_t calcular_inclinacion_fusion(float ax, float ay, float az, 
                                          float gx, float gy, float gz, float dt, 
                                          Inclinacion_t inc_prev)
{
    Inclinacion_t inc;

    // 1. Cálculo de Pitch y Roll estáticos por acelerómetro (en grados)
    float pitch_acc = atan2f(ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEG;
    float roll_acc  = atan2f(ay, sqrtf((ax * ax) + (az * az))) * RAD_TO_DEG;

    // 2. Filtro Complementario (Integración del giroscopio + Acelerómetro)
    // gx, gy y gz deben estar en grados por segundo (°/s)
    inc.pitch = ALPHA * (inc_prev.pitch + gy * dt) + (1.0f - ALPHA) * pitch_acc;
    inc.roll  = ALPHA * (inc_prev.roll  + gx * dt) + (1.0f - ALPHA) * roll_acc;

    // El yaw no puede corregirse con el acelerómetro: se integra el giroscopio Z.
    inc.yaw = inc_prev.yaw + gz * dt;
    inc.yaw = fmodf(inc.yaw + 180.0f, 360.0f);
    if (inc.yaw < 0.0f) {
        inc.yaw += 360.0f;
    }
    inc.yaw -= 180.0f;

    return inc;
}

void procesar_datos_cohete(float dt)
{
  // 1. Obtener la aceleración total
  a_total = calcular_aceleracion_total(accel_g[0], accel_g[1], accel_g[2]);

  // 2. Obtener la inclinación respecto a la vertical
  angulos = calcular_inclinacion_fusion(accel_g[0], accel_g[1], accel_g[2], 
                                         angular_rate_mdps[0] / 1000.0f,
                                         angular_rate_mdps[1] / 1000.0f,
                                         angular_rate_mdps[2] / 1000.0f,
                                         dt, angulos);

  raw_sensor_data.acelerometro.accel_g[0] = accel_g[0];
  raw_sensor_data.acelerometro.accel_g[1] = accel_g[1];
  raw_sensor_data.acelerometro.accel_g[2] = accel_g[2];
  raw_sensor_data.acelerometro.a_total = a_total;
  raw_sensor_data.inc.pitch = angulos.pitch;
  raw_sensor_data.inc.roll = angulos.roll;
  raw_sensor_data.inc.yaw = angulos.yaw;

  if (barometer_ready && accelerometer_ready) {
    if (!altitude_ekf_initialized) {
      if (ekfprototipo2_init(&raw_sensor_data) != 0) {
        Error_Handler();
      }
      altitude_ekf_initialized = 1;
    }

    if (ekfprototipo2_update(&raw_sensor_data, dt, barometer_updated,
                             &altitude_estimate) != 0) {
      Error_Handler();
    }
    barometer_updated = 0;
  }
}


/* USER CODE BEGIN PFP */
// Wrappers de I2C/SPI requeridos por el driver de ST
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/*Giroscopio/IMU*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Write((I2C_HandleTypeDef*)handle, ISM330BX_I2C_ADD, reg, 
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  HAL_I2C_Mem_Read((I2C_HandleTypeDef*)handle, ISM330BX_I2C_ADD, reg, 
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

/*Barómetro*/


/* Función adaptadora de LECTURA I2C para la librería de Bosch */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  uint8_t dev_addr = *(uint8_t*)intf_ptr;
  if (HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 100) == HAL_OK)
  {
      return BMP3_OK;
  }
  return BMP3_E_COMM_FAIL;
}

/* Función adaptadora de ESCRITURA I2C para la librería de Bosch */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  uint8_t dev_addr = *(uint8_t*)intf_ptr;
  if (HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, 100) == HAL_OK)
  {
      return BMP3_OK;
  }
  return BMP3_E_COMM_FAIL;
}

/* Función adaptadora de RETARDO para la librería de Bosch */
void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
  // Convertimos microsegundos a milisegundos mínimos para la HAL
  uint32_t ms = period / 1000;
  if (ms == 0) ms = 1;
  HAL_Delay(ms);
}

/*Funciones rw para acelerómetro*/
int32_t stm32_i2c_write(void *handle, uint8_t reg, const uint8_t *bufr, uint16_t len) {
  HAL_I2C_Mem_Write((I2C_HandleTypeDef*)handle, H3LIS331DL_I2C_ADD, reg | 0x80, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufr, len, 1000);
  return 0;
}

int32_t stm32_i2c_read(void *handle, uint8_t reg, uint8_t *bufr, uint16_t len) {
  HAL_I2C_Mem_Read((I2C_HandleTypeDef*)handle, H3LIS331DL_I2C_ADD, reg | 0x80, I2C_MEMADD_SIZE_8BIT, bufr, len, 1000);
  return 0;
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
  lwgps_init(&hgps);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  //IMU:
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg  = platform_read;
  dev_ctx.handle    = &hi2c1;

  ism330bx_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != ISM330BX_ID) {
      // Error de comunicación con el sensor
      Error_Handler();
  }

  ism330bx_reset_set(&dev_ctx, PROPERTY_ENABLE);
  uint8_t rst;
  do {
      ism330bx_reset_get(&dev_ctx, &rst);
  } while (rst);

  ism330bx_gy_data_rate_set(&dev_ctx, ISM330BX_GY_ODR_AT_120Hz);
  ism330bx_gy_full_scale_set(&dev_ctx, ISM330BX_2000dps);

  //Barómetro:
  // 1. Asignar las funciones de interfaz a la estructura del barómetro
  dev.intf = BMP3_I2C_INTF;
  dev.read = bmp3_i2c_read;
  dev.write = bmp3_i2c_write;
  dev.delay_us = bmp3_delay_us;
  dev.intf_ptr = &dev_addr;

  //acelerómetro
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = stm32_i2c_write;
  dev_ctx.read_reg  = stm32_i2c_read;
  dev_ctx.handle    = &hi2c2;

  // 2. Inicializar el driver del BMP390
  int8_t rslt = bmp3_init(&dev);

  if (rslt == BMP3_OK)
  {
    // 3. Configurar el barómetro (activar presión, temperatura y modo Normal)
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_4X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;

    uint16_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;
      
    bmp3_set_sensor_settings(settings_sel, &settings, &dev);
      
    // Establecer en modo de funcionamiento continuo (Normal Mode)
    settings.op_mode = BMP3_MODE_NORMAL;
    bmp3_set_op_mode(&settings, &dev);
  }
  
  //WHO AM I acelerómetro
  uint8_t whoamI = 0;
    h3lis331dl_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != H3LIS331DL_ID) {
        // Fallo de comunicación en I2C2
        Error_Handler();
    }

  // Configuración inicial del acelerómetro
    h3lis331dl_full_scale_set(&dev_ctx, H3LIS331DL_100g);
    h3lis331dl_data_rate_set(&dev_ctx, H3LIS331DL_ODR_100Hz);

    int16_t data_raw[3];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    current_time = HAL_GetTick();
    if (current_time - last_time_bar >= intervalo_bar) { //bar
    last_time_bar = current_time;
    // Leer datos de presión y temperatura compensados de bar.
      int8_t rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);

      if (rslt == BMP3_OK)
      {
          presion_hpa = data.pressure / 100.0f; // Convertir Pa a hPa
          temperatura_c = data.temperature;      // Grados Celsius
          altitud_m = calcular_altitud(presion_hpa, 1013.25f);
          raw_sensor_data.barometro.altitud_m = altitud_m;
          barometer_ready = 1;
          barometer_updated = 1;
      }
    }
    if(current_time - last_time_accel >= intervalo_accel) { //acelerómetro
      last_time_accel = current_time;
      uint8_t ready = 0;
      h3lis331dl_flag_data_ready_get(&dev_ctx, &ready);
      if (ready)
      {
        int16_t data_raw[3];
        h3lis331dl_acceleration_raw_get(&dev_ctx, data_raw);
        accel_g[0] = h3lis331dl_from_fs100_to_mg(data_raw[0]) / 1000.0f;
        accel_g[1] = h3lis331dl_from_fs100_to_mg(data_raw[1]) / 1000.0f;
        accel_g[2] = h3lis331dl_from_fs100_to_mg(data_raw[2]) / 1000.0f;
        accelerometer_ready = 1;
      }
    }
    // 1. Leer bloque de bytes desde el STA8135GA (por ejemplo, vía I2C3)
    if (current_time - last_time_gps >= intervalo_gps) {
      last_time_gps = current_time;
      if (HAL_I2C_Master_Receive(&hi2c3, TESEO_I2C_ADDR, gps_rx_buffer, sizeof(gps_rx_buffer), 100) == HAL_OK) {
        // 2. Pasar los datos recibidos al analizador
        lwgps_process(&hgps, gps_rx_buffer, sizeof(gps_rx_buffer));

        // 3. Verificar si hay un fijado de posición válido (Fix)
        if (hgps.is_valid)
        {
          latitud  = hgps.latitude;
          longitud = hgps.longitude;
          altitud  = hgps.altitude;
          raw_sensor_data.gps.latitud = latitud;
          raw_sensor_data.gps.longitud = longitud;
          raw_sensor_data.gps.altitud = altitud;

          // Usar coordenadas para telemetría o registro
        }
      }
    }
    if (current_time - last_time_imu >= intervalo_imu) {
      last_time_imu = current_time;
      ism330bx_reg_t reg;
      ism330bx_status_reg_get(&dev_ctx, &reg.status_reg);
      if (reg.status_reg.gda) {
          ism330bx_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
          angular_rate_mdps[0] = ism330bx_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
          angular_rate_mdps[1] = ism330bx_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
          angular_rate_mdps[2] = ism330bx_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
      }
    }
    if (current_time - last_time_process >= 250) { //cada 0.25 segundos
      float dt_process = (current_time - last_time_process) / 1000.0f;
      last_time_process = current_time;
      procesar_datos_cohete(dt_process);
    }
  }
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

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

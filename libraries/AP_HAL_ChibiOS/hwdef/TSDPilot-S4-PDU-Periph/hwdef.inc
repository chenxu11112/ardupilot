
# MCU class and specific type
MCU STM32F4xx STM32F405xx

# bootloader starts firmware at 64k
FLASH_RESERVE_START_KB 64
FLASH_SIZE_KB 1024

# store parameters in pages 2 and 3
STORAGE_FLASH_PAGE 2
define HAL_STORAGE_SIZE 15360

# board ID for firmware load
APJ_BOARD_ID 10201

# setup build for a peripheral firmware
env AP_PERIPH 1

define STM32_ST_USE_TIMER 5
define CH_CFG_ST_RESOLUTION 32

# enable watchdog

# crystal frequency
OSCILLATOR_HZ 8000000

define HAL_HAVE_PIXRACER_LED
PB15 LED_RED   OUTPUT GPIO(10)
PB14 LED_GREEN OUTPUT GPIO(11)
PB13 LED_BLUE  OUTPUT GPIO(12)
define HAL_GPIO_A_LED_PIN 10
define HAL_GPIO_B_LED_PIN 11
define HAL_GPIO_C_LED_PIN 12

STDOUT_SERIAL SD4
STDOUT_BAUDRATE 57600

define HAL_NO_GPIO_IRQ
define SERIAL_BUFFERS_SIZE 512
define PORT_INT_REQUIRED_STACK 64

# debug 
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# avoid timer and RCIN threads to save memory
define HAL_NO_RCIN_THREAD
define HAL_USE_RTC FALSE
define DMA_RESERVE_SIZE 0
define HAL_DISABLE_LOOP_DELAY
define HAL_NO_MONITOR_THREAD

define HAL_DEVICE_THREAD_STACK 2048

# ---------------------- CAN bus -------------------------
# enable CAN support
PB8 CAN1_RX CAN1
PB9 CAN1_TX CAN1

CAN_ORDER 1

# ----------------------  UART   -------------------------
# order of UARTs
SERIAL_ORDER OTG1 EMPTY EMPTY EMPTY

PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# USART4
PC10 UART4_TX UART4 SPEED_HIGH NODMA
PC11 UART4_RX UART4 SPEED_HIGH NODMA

# ------------------ BATTERY Monitor -----------------------
define HAL_PERIPH_ENABLE_BATTERY

define HAL_USE_ADC TRUE
define STM32_ADC_USE_ADC1 TRUE
PC2 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC3 BATT_CURRENT_SENS ADC1 SCALE(1)

# Set the Default Battery Monitor Type to be Off
define AP_BATT_MONITOR_MAX_INSTANCES 1

define HAL_BATT_MONITOR_DEFAULT 4

define HAL_BATT_VOLT_PIN 13
define HAL_BATT_VOLT_SCALE 21.0

define HAL_BATT_CURR_PIN 12
define HAL_BATT_CURR_SCALE 50.0

# -------------------- Buzzer+NeoPixels --------------------
define HAL_PERIPH_ENABLE_RC_OUT
define HAL_PERIPH_ENABLE_NOTIFY
define HAL_BARO_ALLOW_INIT_NO_BARO

# enable ESC control
define HAL_SUPPORT_RCOUT_SERIAL 1
define HAL_WITH_ESC_TELEM 1

# allow for reboot command for faster development
define HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT 0

define AP_PERIPH_HAVE_LED TRUE

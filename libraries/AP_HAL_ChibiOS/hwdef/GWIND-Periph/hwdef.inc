# hw definition file for processing by chibios_pins.py

# MCU class and specific type
MCU STM32G441 STM32G441xx

# board ID for firmware load
APJ_BOARD_ID 1046

# setup build for a peripheral firmware
env AP_PERIPH 1

# assume the 128k flash part
FLASH_SIZE_KB 128

# bootloader starts firmware at 20k + 4k (STORAGE_FLASH)
FLASH_RESERVE_START_KB 22

# store parameters in pages 
STORAGE_FLASH_PAGE 9
define HAL_STORAGE_SIZE 800

# ChibiOS system timer
define CH_CFG_ST_FREQUENCY 100000
define CH_CFG_ST_TIMEDELTA 0

# crystal frequency
OSCILLATOR_HZ 8000000


# order of UARTs
SERIAL_ORDER
define HAL_USE_UART FALSE
define HAL_NO_UARTDRIVER TRUE

define HAL_USE_I2C FALSE

define HAL_USE_ADC FALSE
define STM32_ADC_USE_ADC1 FALSE

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# enable CAN support
PA11 CAN1_RX CAN1
PA12 CAN1_TX CAN1


define HAL_NO_MONITOR_THREAD
define HAL_NO_LOGGING
define HAL_NO_GPIO_IRQ
define HAL_NO_RCIN_THREAD
define HAL_NO_RCOUT_THREAD
define HAL_NO_TIMER_THREAD

define PORT_INT_REQUIRED_STACK 64

define HAL_USE_RTC FALSE
# define DISABLE_SERIAL_ESC_COMM TRUE

define DMA_RESERVE_SIZE 0

define HAL_DEVICE_THREAD_STACK 0x200

# MAIN_STACK is stack for ISR handlers
MAIN_STACK 0x300

# PROCESS_STACK controls stack for main thread
PROCESS_STACK 0xA00

define HAL_DISABLE_LOOP_DELAY

define AP_PARAM_MAX_EMBEDDED_PARAM 128

# keep ROMFS uncompressed as we don't have enough RAM
# to uncompress the bootloader at runtime
env ROMFS_UNCOMPRESSED True

# disable stack checking to reduce flash cost
define CH_DBG_ENABLE_STACK_CHECK FALSE

# reduce the number of CAN RX Buffer
define HAL_CAN_RX_QUEUE_SIZE 32
define HAL_CAN_POOL_SIZE 6000


# disable dual GPS and GPS blending to save flash space
define GPS_MAX_RECEIVERS 1
define GPS_MAX_INSTANCES 1
define HAL_COMPASS_MAX_SENSORS 1
define HAL_GYROFFT_ENABLED 0

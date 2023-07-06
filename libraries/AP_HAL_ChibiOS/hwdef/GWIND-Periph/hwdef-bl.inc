# hw definition file for processing by chibios_pins.py

# MCU class and specific type
MCU STM32G431 STM32G431xx

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 18

# assume 128k flash part
FLASH_SIZE_KB 128

# reserve some space for params
APP_START_OFFSET_KB 4

# board ID for firmware load
APJ_BOARD_ID 1046

# setup build for a peripheral firmware
env AP_PERIPH 1

# crystal frequency, no crystal for now
OSCILLATOR_HZ 8000000

# order of UARTs
SERIAL_ORDER
define HAL_USE_UART FALSE

define HAL_USE_SERIAL FALSE

define HAL_NO_GPIO_IRQ

define DMA_RESERVE_SIZE 0

MAIN_STACK 0x800
PROCESS_STACK 0x800

define HAL_DISABLE_LOOP_DELAY

# enable CAN support
PA11 CAN1_RX CAN1
PA12 CAN1_TX CAN1

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD


# use a small bootloader timeout
define HAL_BOOTLOADER_TIMEOUT 3000



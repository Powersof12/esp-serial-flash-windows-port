#define SERIAL_FLASHER_DEBUG_TRACE				(false)
#define SERIAL_FLASHER_INTERFACE_UART			(1)
#define INITIAL_UART_BAUDRATE					(115200)
#define HIGHER_UART_BAUDRATE					(921600)		// 0 = Do not change rates, 230400, 460800, 921600
#define SERIAL_FLASHER_RESET_HOLD_TIME_MS		(100)
#define SERIAL_FLASHER_BOOT_HOLD_TIME_MS		(50)
#define SERIAL_FLASHER_WRITE_BLOCK_RETRIES		(3)

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))



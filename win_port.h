//**********************************************************************************************************************
//Copy Rights : (C) 2025 Flagler Engineering LLC
//**********************************************************************************************************************
#ifndef MODULE_WIN_PORT_
#define MODULE_WIN_PORT_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <windows.h>
#include "esp_loader_io.h"

#define HAL_Delay								Sleep
#define HAL_GetTick								GetTickCount

#ifdef __cplusplus
extern "C" {
#endif

void PRINTF(const char *str);

// Windows Serial Port Functions
void serial_port_set_device_name(const char* device);
int serial_port_open(uint32_t baud_rate);
int serial_port_change_baudrate(int baudrate);
void serial_port_flush(void);
void set_dtr(int high_or_low);
void set_rts(int high_or_low);
int serial_port_write(uint8_t* buffer, uint32_t size);
SSIZE_T serial_port_read(uint8_t* buffer, uint32_t size, uint32_t timeout);
void serial_port_close(void);

#ifdef __cplusplus
}
#endif

#endif

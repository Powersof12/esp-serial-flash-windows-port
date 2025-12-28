//**********************************************************************************************************************
//Copy Rights : (C) 2025 Flagler Engineering LLC
//**********************************************************************************************************************
#include "win_port.h"
#include "config.h"

static HANDLE ourPortHandle;
static char com_port_device[MAX_PATH];

static DWORD s_time_end;

//**********************************************************************************************************************
//**********************************************************************************************************************
#if SERIAL_FLASHER_DEBUG_TRACE
static void transfer_debug_print(const uint8_t* data, uint16_t size, bool write)
{
    static bool write_prev = false;

    if (write_prev != write) {
        write_prev = write;
        PRINTF("\n--- %s ---\n", write ? "WRITE" : "READ");
    }

    for (uint32_t i = 0; i < size; i++) {
        PRINTF("%02x ", data[i]);
    }
}
#endif

//**********************************************************************************************************************
//**********************************************************************************************************************
esp_loader_error_t loader_port_write(const uint8_t* data, uint16_t size, uint32_t timeout)
{
    int err = serial_port_write((uint8_t*)data, size);

    if (err == 0) 
    {
#if SERIAL_FLASHER_DEBUG_TRACE
        transfer_debug_print(data, size, true);
#endif
        return ESP_LOADER_SUCCESS;
    }
    //else if (err == TIMEOUT) 
    //{
    //    return ESP_LOADER_ERROR_TIMEOUT;
    //}
    else 
    {
        return ESP_LOADER_ERROR_FAIL;
    }
}

//**********************************************************************************************************************
//**********************************************************************************************************************
esp_loader_error_t loader_port_read(uint8_t* data, uint16_t size, uint32_t timeout)
{
    int err = (int)serial_port_read(data, size, timeout);

    if (err >= 0)
    {
#if SERIAL_FLASHER_DEBUG_TRACE
        transfer_debug_print(data, size, false);
#endif
        return ESP_LOADER_SUCCESS;
    }

    return ESP_LOADER_ERROR_TIMEOUT;
}

//**********************************************************************************************************************
//**********************************************************************************************************************
int loader_port_reset_target(void)
{
    char msg[1024];
    serial_port_flush();

    set_dtr(1);
    set_rts(0);
    HAL_Delay(SERIAL_FLASHER_RESET_HOLD_TIME_MS);
    set_rts(1);

    serial_port_read((uint8_t*)msg, 1024 - 1, 100);
    if (strstr(msg, "SPI_FAST_FLASH_BOOT"))
    {
        PRINTF("WiFi is in Normal Mode\n");
        return(0);
    }

    return(-1);
}

//**********************************************************************************************************************
//**********************************************************************************************************************
int loader_port_enter_bootloader(void)
{
    char msg[1024];
    serial_port_flush();

    set_dtr(1);
    set_rts(0);
    HAL_Delay(SERIAL_FLASHER_RESET_HOLD_TIME_MS);
    set_dtr(0);
    set_rts(1);
    HAL_Delay(SERIAL_FLASHER_BOOT_HOLD_TIME_MS);
    set_dtr(1);

    serial_port_read((uint8_t*)msg, 1024 - 1, 100);
    if (strstr(msg, "DOWNLOAD"))
    {
        PRINTF("WiFi is in Download Mode\n");
        return(0);
    }

    return(-1);
}

//**********************************************************************************************************************
//**********************************************************************************************************************
void loader_port_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

//**********************************************************************************************************************
//**********************************************************************************************************************
void loader_port_start_timer(uint32_t ms)
{
    s_time_end = HAL_GetTick() + ms;
}

//**********************************************************************************************************************
//**********************************************************************************************************************
uint32_t loader_port_remaining_time(void)
{
    int32_t remaining = s_time_end - HAL_GetTick();
    return (remaining > 0) ? (uint32_t)remaining : 0;
}

//**********************************************************************************************************************
//**********************************************************************************************************************
void loader_port_debug_print(const char* str)
{
    PRINTF(str);
}

//**********************************************************************************************************************
//**********************************************************************************************************************
esp_loader_error_t loader_port_change_transmission_rate(uint32_t baudrate)
{
    return (serial_port_change_baudrate(baudrate));
}

//************************************************
// Windows Serial Port Functions below this marker
//************************************************
void serial_port_set_device_name(const char* device)
{
    sprintf_s(com_port_device, MAX_PATH, "\\\\.\\%s", device);      // Required for Com Ports above 9 ???
    // "\\\\.\\COM10"
    //printf("%s\n", com_port_device);
}

//**********************************************************************************************************************
// Opens the specified serial port, configures its timeouts, and sets its
// baud rate.  Returns a handle on success, or INVALID_HANDLE_VALUE on failure.
//**********************************************************************************************************************
int serial_port_open(uint32_t baud_rate)
{
    HANDLE port = CreateFileA(com_port_device, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (port == INVALID_HANDLE_VALUE)
    {
        ourPortHandle = NULL;
        PRINTF("COMM ERR, Invalid Handle");
        return(-1);
    }
    ourPortHandle = port;

    // Flush away any bytes previously read or written.
    serial_port_flush();

    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    BOOL success = SetCommTimeouts(port, &timeouts);
    if (!success)
    {
        PRINTF("COMM ERR, Setting Timeouts");
        serial_port_close();
        return(-2);
    }

    // Set the baud rate and other options.
    DCB state = { 0 };
    state.DCBlength = sizeof(DCB);

    GetCommState(port, &state);

    state.BaudRate = baud_rate;
    state.ByteSize = 8;
    state.Parity = NOPARITY;
    state.StopBits = ONESTOPBIT;

    // Disable all flow control:
    state.fOutxCtsFlow = FALSE; // No CTS output flow control
    state.fOutxDsrFlow = FALSE; // No DSR output flow control
    state.fDtrControl = DTR_CONTROL_ENABLE;
    state.fRtsControl = RTS_CONTROL_ENABLE;
    state.fInX = FALSE;         // No XON/XOFF for input
    state.fOutX = FALSE;        // No XON/XOFF for output

    success = SetCommState(port, &state);
    if (!success)
    {
        PRINTF("COMM ERR, Setting Comm State");
        serial_port_close();
        return(-3);
    }

    return(0);
}

//**********************************************************************************************************************
//**********************************************************************************************************************
int serial_port_change_baudrate(int baudrate)
{
    if (ourPortHandle != NULL)
    {
        serial_port_close();
    }

    serial_port_open(baudrate);

    serial_port_flush();

    return 0;
}

//**********************************************************************************************************************
//**********************************************************************************************************************
void serial_port_flush(void)
{
    if (ourPortHandle != NULL)
    {
        FlushFileBuffers(ourPortHandle);
    }
}

//**********************************************************************************************************************
// DTR is inverted logic
//**********************************************************************************************************************
void set_dtr(int high_or_low)
{
    if (ourPortHandle != NULL)
    {
        if (!high_or_low)
        {
            EscapeCommFunction(ourPortHandle, SETDTR);
        }
        else
        {
            EscapeCommFunction(ourPortHandle, CLRDTR);
        }
    }
}

//**********************************************************************************************************************
// RTS is inverted logic
//**********************************************************************************************************************
void set_rts(int high_or_low)
{
    if (ourPortHandle != NULL)
    {
        if (!high_or_low)
        {
            EscapeCommFunction(ourPortHandle, SETRTS);
        }
        else
        {
            EscapeCommFunction(ourPortHandle, CLRRTS);
        }
    }
}

//**********************************************************************************************************************
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
//**********************************************************************************************************************
int serial_port_write(uint8_t* buffer, uint32_t size)
{
    if (ourPortHandle != NULL)
    {
        DWORD written;
        BOOL success = WriteFile(ourPortHandle, buffer, (DWORD)size, &written, NULL);
        if ((success) && (written == size))
        {
            return 0;
        }
    }

    return(-1);
}

//**********************************************************************************************************************
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
//**********************************************************************************************************************
SSIZE_T serial_port_read(uint8_t* buffer, uint32_t size, uint32_t timeout)
{
    if (ourPortHandle != NULL)
    {
        DWORD received;

        bool success = ReadFile(ourPortHandle, buffer, (DWORD)size, &received, NULL);
        if ((success) && (received > 0))
        {
            return received;
        }

        // Let's try a short sleep
        Sleep(20);

        success = ReadFile(ourPortHandle, buffer, (DWORD)size, &received, NULL);
        if ((success) && (received > 0))
        {
            return received;
        }
        else
        {
            Sleep(timeout);
            success = ReadFile(ourPortHandle, buffer, (DWORD)size, &received, NULL);
            if ((success) && (received > 0))
            {
                return received;
            }
        }
    }

    return(-1);
}

//**********************************************************************************************************************
// Close the Serial Port
//**********************************************************************************************************************
void serial_port_close(void)
{
    if (ourPortHandle != NULL)
    {
        CloseHandle(ourPortHandle);
    }

    ourPortHandle = NULL;
}

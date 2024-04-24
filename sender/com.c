#include <stdio.h>
#include <stdint.h>
#include <windows.h>
 
void print_error(const char* context)
{
    DWORD error_code = GetLastError();
    char buffer[512];
    DWORD size = FormatMessageA(
        FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_MAX_WIDTH_MASK,
        NULL, error_code, MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US),
        buffer, sizeof(buffer), NULL
    );
    if (size == 0) 
    { 
        buffer[0] = 0;
    }
    fprintf(stderr, "%s: %s\n", context, buffer);
}
 
// Opens the specified serial port, configures its timeouts, and sets its
// baud rate.  Returns a handle on success, or INVALID_HANDLE_VALUE on failure.
HANDLE open_serial_port(const char* device, uint32_t baud_rate)
{
    HANDLE port = CreateFileA(
        device, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL
    );
    if (port == INVALID_HANDLE_VALUE)
    {
        print_error(device);
        return INVALID_HANDLE_VALUE;
    }
    
    // Flush away any bytes previously read or written.
    BOOL success = FlushFileBuffers(port);
    if (!success)
    {
        print_error("Failed to flush serial port");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    
    // Configure read and write operations to time out after 100 ms.
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    
    success = SetCommTimeouts(port, &timeouts);
    if (!success)
    {
        print_error("Failed to set serial timeouts");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    
    // Set the baud rate and other options.
    DCB state = {0};
    state.DCBlength = sizeof(DCB);
    state.BaudRate = baud_rate;
    state.ByteSize = 8;
    state.Parity = NOPARITY;
    state.StopBits = ONESTOPBIT;
    success = SetCommState(port, &state);
    if (!success)
    {
        print_error("Failed to set serial settings");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    
    return port;
}
 
int write_port(HANDLE port, uint8_t * buffer, size_t size)
{
    DWORD written;
    BOOL success = WriteFile(port, buffer, size, &written, NULL);
    if (!success)
    {
        print_error("Failed to write to port");
        return -1;
    }
    if (written != size)
    {
        print_error("Failed to write all bytes to port");
        return -1;
    }
    return 0;
}
 
SSIZE_T read_port(HANDLE port, uint8_t * buffer, size_t size)
{
    DWORD received;
    BOOL success = ReadFile(port, buffer, size, &received, NULL);
    if (!success)
    {
        print_error("Failed to read from port");
        return -1;
    }
    return received;
}


int main(int argc, char *argv[])
{
    char* device = NULL;
    char* kernel_fpath = NULL;
    if(argc != 3)
    {

    }

    const char* device = "\\COM3";
    uint32_t baud_rate = CBR_115200;
    
    HANDLE port = open_serial_port(device, baud_rate);
    if (port == INVALID_HANDLE_VALUE) 
    { 
        printf("Invalid handle");
        return 1;
    }
    
    CloseHandle(port);
    return 0;
}
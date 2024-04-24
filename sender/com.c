#include <stdio.h>
#include <stdint.h>
#include <windows.h>
 
void print_error(const char * context)
{
    DWORD error_code = GetLastError();
    char buffer[256];
    DWORD size = FormatMessageA(
        FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_MAX_WIDTH_MASK,
        NULL, error_code, MAKELANGID(LANG_ENGLISH, SUBLANG_ENGLISH_US),
        buffer, sizeof(buffer), NULL);
    if (size == 0) { buffer[0] = 0; }
    fprintf(stderr, "%s: %s\n", context, buffer);
}
 
// Opens the specified serial port, configures its timeouts, and sets its
// baud rate.  Returns a handle on success, or INVALID_HANDLE_VALUE on failure.
HANDLE open_serial_port(const char * device, uint32_t baud_rate)
{
    HANDLE port = CreateFileA(device, GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
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
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
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
 
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
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
 
// Sets the target, returning 0 on success and -1 on failure.
//
// For more information about what this command does, see the "Set Target"
// command in the "Command reference" section of the Jrk G2 user's guide.
int jrk_set_target(HANDLE port, uint16_t target)
{
    if (target > 4095) { target = 4095; }
    uint8_t command[2];
    command[0] = 0xC0 + (target & 0x1F);
    command[1] = (target >> 5) & 0x7F;
    return write_port(port, command, sizeof(command));
}
 
// Gets one or more variables from the Jrk (without clearing them).
// Returns 0 for success, -1 for failure.
int jrk_get_variable(HANDLE port, uint8_t offset, uint8_t * buffer, uint8_t length)
{
    uint8_t command[] = { 0xE5, offset, length };
    int result = write_port(port, command, sizeof(command));
    if (result) { return -1; }
    SSIZE_T received = read_port(port, buffer, length);
    if (received < 0) { return -1; }
    if (received != length)
    {
        fprintf(stderr, "read timeout: expected %u bytes, got %lld\n",
        length, (int64_t)received);
        return -1;
    }
    return 0;
}
 
// Gets the Target variable from the jrk or returns -1 on failure.
int jrk_get_target(HANDLE port)
{
    uint8_t buffer[2];
    int result = jrk_get_variable(port, 0x02, buffer, sizeof(buffer));
    if (result) { return -1; }
    return buffer[0] + 256 * buffer[1];
}
 
// Gets the Feedback variable from the jrk or returns -1 on failure.
int jrk_get_feedback(HANDLE port)
{
    // 0x04 is the offset of the feedback variable in the "Variable reference"
    // section of the Jrk user's guide.  The variable is two bytes long.
    uint8_t buffer[2];
    int result = jrk_get_variable(port, 0x04, buffer, sizeof(buffer));
    if (result) { return -1; }
    return buffer[0] + 256 * buffer[1];
}
 
int main()
{
    // Choose the serial port name.  If the Jrk is connected directly via USB,
    // you can run "jrk2cmd --cmd-port" to get the right name to use here.
    // COM ports higher than COM9 need the \\.\ prefix, which is written as
    // "\\\\.\\" in C because we need to escape the backslashes.
    const char * device = "\\\\.\\COM7";
    
    // Choose the baud rate (bits per second).  This does not matter if you are
    // connecting to the Jrk over USB.  If you are connecting via the TX and RX
    // lines, this should match the baud rate in the Jrk's serial settings.
    uint32_t baud_rate = 9600;
    
    HANDLE port = open_serial_port(device, baud_rate);
    if (port == INVALID_HANDLE_VALUE) { return 1; }
    
    int feedback = jrk_get_feedback(port);
    if (feedback < 0) { return 1; }
    
    printf("Feedback is %d.\n", feedback);
    
    int target = jrk_get_target(port);
    if (target < 0) { return 1; }
    printf("Target is %d.\n", target);
    
    int new_target = (target < 2048) ? 2248 : 1848;
    printf("Setting target to %d.\n", new_target);
    int result = jrk_set_target(port, new_target);
    if (result) { return 1; }
    
    CloseHandle(port);
    return 0;
}
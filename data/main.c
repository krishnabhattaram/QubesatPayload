#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void print_data_to_uart(unsigned *data, int len);

int main() {
    int len = 60;
    unsigned data[60];
    for (unsigned i = 0; i < len; i++) {
        data[i] = i;
    }

    print_data_to_uart(data, len);

    return 0;
}

void print_data_to_uart(unsigned *data, int len) {
    unsigned MAX_INTS_PER_TRANSMIT = 40;  // pulled this out my ass ngl

    char buf[500];  // Create a buffer for the formatted string
    char uart_buf[500];

    for (unsigned printed = 0; printed < len;
         printed += MAX_INTS_PER_TRANSMIT) {
        for (unsigned i = 0; i < 500; i++) buf[i] = 0;
        for (unsigned i = 0; i < 500; i++) uart_buf[i] = 0;

        int l = 0;  // Keep track of the length of the formatted string
        int n;  // Keep track of the number of characters added to the formatted
                // string

        // Add each array element to the formatted string
        unsigned to_transmit = MAX_INTS_PER_TRANSMIT;
        if (len - printed < MAX_INTS_PER_TRANSMIT) {
            to_transmit = len - printed;
        }
        for (unsigned i = printed; i < printed + to_transmit; i++) {
            n = sprintf(buf + l, "%d\r\n", data[i]);
            if (n < 0 || l + n >= sizeof(buf)) {
                // Error handling: buffer overflow or snprintf error
                return;
            }
            l += n;
        }

        int uart_buf_len = (l > 500) ? 500 : l;
        memcpy(uart_buf, buf, uart_buf_len);

        // Print the formatted string
        // HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len,
        //                   HAL_MAX_DELAY);

        printf("%s", uart_buf);
    }
}
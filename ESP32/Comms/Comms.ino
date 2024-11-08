#include "driver/uart.h"
#include "esp_intr_alloc.h"


uint8_t numDigits = 0;
uint8_t RecievedFlag = 0;
uint8_t CFGFlag = 0;
uint8_t SendFlag = 0;
uint8_t rx_buffer[2];
uint8_t tx_buffer[20];

// Define UART parameters
const int uartTXPin = 13;  // Replace with your TX pin
const int uartRXPin = 12;  // Replace with your RX pin
const uart_port_t uart_num = UART_NUM_2; // Using Serial2 (UART2)

// Queue to hold UART events
static QueueHandle_t uart_queue;

void IRAM_ATTR uart_event_task(void *pvParameters);
void IRAM_ATTR handleInterrupt();
void setup() {
  // Initialize the serial monitor for debugging
  Serial.begin(115200);

  // Set GPIO input pin mode
  pinMode(11, INPUT);

  // UART2 configuration
  const uart_config_t uart_config = {
    .baud_rate = 2400,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };

  // Install the UART driver
  uart_param_config(uart_num, &uart_config);
  uart_set_pin(uart_num, uartTXPin, uartRXPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num, 256, 256, 10, &uart_queue, 0);

  // Send two bytes: 102 and 10 (only once in setup)
  
  // Create a task that will handle UART events (interrupt)
  xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

  gpio_pullup_dis(GPIO_NUM_12);
}

void loop() {

  if(digitalRead(11))
  {
    delay(2000);
    tx_buffer[0] = 102;
    tx_buffer[1] = 0;
    uart_write_bytes(uart_num, (const char *)tx_buffer, 2);
    delay(500);
  }

  if(RecievedFlag)
  {
    neopixelWrite(48, 255, 0, 0);
    delay(1000);
    tx_buffer[0] = 103;
    tx_buffer[1] = 1;
    uart_write_bytes(uart_num, (const char *)tx_buffer, 2);
    RecievedFlag = 0;
    neopixelWrite(48, 255, 0, 255);
    delay(500);
  }

  if (SendFlag)
  {
    delay(1000);
    neopixelWrite(48, 255, 255, 0);
    tx_buffer[0] = 104;
    tx_buffer[1] = 7;
    tx_buffer[2] = 5;
    tx_buffer[3] = 7;
    tx_buffer[4] = 6;
    uart_write_bytes(uart_num, (const char *)tx_buffer, 2);
    SendFlag = 0;
    delay(500);
  }

}

void IRAM_ATTR uart_event_task(void *pvParameters) {
  neopixelWrite(48, 255, 255, 255);
  uart_event_t event;
  uint8_t dtmp[2];
  uint8_t seq102_1_Counter = 0; // Counter to track received bytes for 102, 1 sequence
  uint8_t seq103_0_Counter = 0; // Counter to track received bytes for 103, 0 sequence
  
  for (;;) {
    // Waiting for UART event.
    if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
      switch (event.type) {
        // UART data reception event
        case UART_DATA:
          // Read one byte from UART
          if (uart_read_bytes(uart_num, dtmp, 2, portMAX_DELAY) > 0) {
            // Check sequence 102, 1
            Serial.println(dtmp[0]);
            Serial.println(dtmp[1]);
            
          }
          break;

        // Handle other UART events if needed
        default:
          break;
      }
    }
  }
}

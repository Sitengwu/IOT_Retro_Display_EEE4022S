#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "IOT RETRO DISPLAY";  // WiFi name
const char *password = "123456789";      // WiFi password

// MQTT Broker
const char *mqtt_server = "192.168.14.90";
const char *topic = "esp32";
const int mqtt_port = 1883;
// const char *mqtt_username = "";
// const char *mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);

HardwareSerial SerialPort(2);  // Use Serial2 for ESP32-S3

const int RX_PIN = 12;               // RX pin for UART communication
const int TX_PIN = 13;               // TX pin for UART communication
volatile uint8_t dataBuffer[2];      // Buffer to store incoming 2 bytes
volatile bool dataReceived = false;  // Flag to indicate 2 bytes have been received
volatile bool digitReady = false;
uint8_t numDigits = 0;
int number = 0;
uint8_t txBuffer[10];

uint8_t red = 0;
uint8_t green = 0;
uint8_t blue = 0;

unsigned long previousMillis = 0;

void IRAM_ATTR onUartRx() {
  static uint8_t byteCount = 0;

  // Check if data is available
  while (SerialPort.available()) {
    dataBuffer[byteCount++] = SerialPort.read();

    // Check if we have received 2 bytes
    if (byteCount == 2) {
      dataReceived = true;
      byteCount = 0;  // Reset for the next pair of bytes
    }
  }
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("Connecting to MQTT server...\n");
    if (client.connect(client_id.c_str())) {
      Serial.println("Public MQTT broker connected");
      client.subscribe(topic);
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }

  SerialPort.begin(2400, SERIAL_8N1, RX_PIN, TX_PIN);

  // Attach the UART RX interrupt
  SerialPort.onReceive(onUartRx);
}

void setup_wifi() {
  delay(10);
  // Connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message: ");
  Serial.println(message);
  Serial.println("-----------------------");

  // Split the string by commas
  char messageArray[message.length() + 1];
  message.toCharArray(messageArray, message.length() + 1);  // Convert String to char array

  int values[10];  // Array to store int values
  int index = 0;   // Index for the int array

  char *token = strtok(messageArray, ",");
  while (token != NULL && index < 10) {
    values[index++] = atoi(token);  // Convert token to int and store in array
    token = strtok(NULL, ",");
  }

  // Print the stored int values
  Serial.println("Stored Values:");
  for (int i = 0; i < index; i++) {
    Serial.println(values[i]);  // Print each int value
  }

  if (values[0] == 105) {
    txBuffer[0] = 105;

    for (int i = 1; i < sizeof(txBuffer); i++) {
      txBuffer[i] = 0;
    }

    number = values[1];
    int temp = number;
    uint8_t count = 1;
    while (temp > 0) {
      txBuffer[count] = temp % 10;
      temp = temp / 10;
      count++;
    }

    if (digitReady) {
      SerialPort.write(txBuffer, numDigits + 1);
      Serial.println("Sent 105 + digits");
    }
  } else if (values[0] == 104) {
    red = values[1];
    green = values[2];
    blue = values[3];
  }
}

void loop() {

  client.loop();

  unsigned long currentMillis = millis();

  // Non-blocking delay
  if (currentMillis - previousMillis >= 2000) {
    previousMillis = currentMillis;
    if (analogRead(11) > 100) {
      digitReady = false;
      Serial.println("CFG HIGH");
      txBuffer[0] = 102;
      txBuffer[1] = 0;
      SerialPort.write(txBuffer, 2);
      Serial.println("Sent 102 and 0");
    }
    // Process data if MQTT bytes have been received
    if (dataReceived) {
      dataReceived = false;  // Reset flag
      if ((dataBuffer[0] > 0) && (dataBuffer[1] == 102)) {
        txBuffer[0] = 103;
        txBuffer[1] = dataBuffer[0];
        numDigits = dataBuffer[0];
        SerialPort.write(txBuffer, 2);
        Serial.printf("Sent 103 and %d\n", dataBuffer[0]);
      } else if ((dataBuffer[0] == 102) && (dataBuffer[1] > 0)) {
        txBuffer[0] = 103;
        txBuffer[1] = dataBuffer[1];
        numDigits = dataBuffer[1];
        SerialPort.write(txBuffer, 2);
        Serial.printf("Sent 103 and %d\n", dataBuffer[1]);
      } else if (((dataBuffer[0] == 0) && (dataBuffer[1] == 103)) || ((dataBuffer[0] == 103) && (dataBuffer[1] == 0))) {
        txBuffer[0] = 104;
        txBuffer[1] = 104;
        txBuffer[2] = red;
        txBuffer[3] = blue;
        txBuffer[4] = green;
        txBuffer[5] = 104;
        txBuffer[6] = 104;
        SerialPort.write(txBuffer, 7);
        Serial.println("Sent 104");
      } else if (((dataBuffer[0] == 104) && (dataBuffer[1] == 104)) || ((dataBuffer[0] > -1) && (dataBuffer[1] == 104))) {
        digitReady = true;
        txBuffer[0] = 105;

        for (int i = 1; i < sizeof(txBuffer); i++) {
          txBuffer[i] = 0;
        }
        int temp = number;
        uint8_t count = 1;
        while (temp > 0) {
          txBuffer[count] = temp % 10;
          temp = temp / 10;
          count++;
        }

        SerialPort.write(txBuffer, numDigits + 1);
        Serial.println("Sent 105 + digits");
      }
      // Do something with the 2-byte data
      Serial.print("Received bytes: ");
      Serial.print(dataBuffer[0]);
      Serial.print(" ");
      Serial.println(dataBuffer[1]);
    }
  }
}
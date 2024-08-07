#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "HELIOS300";      // WiFi name
const char *password = "123456789";  // WiFi password

// MQTT Broker
const char *mqtt_server = "192.168.0.104";
const char *topic = "esp32";
const int mqtt_port = 1883;
// const char *mqtt_username = "";
// const char *mqtt_password = "";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("Connecting to MQTT server...\n");
    if (client.connect(client_id.c_str())) 
    {
      Serial.println("Public MQTT broker connected");
      client.subscribe(topic);
    } 
    else 
    {
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
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
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

void loop() {
  // put your main code here, to run repeatedly:
  client.loop();
}

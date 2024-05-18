#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Configurações do Wi-Fi
const char* ssid = "VIVOFIBRA-0AA0";          // Substitua por sua SSID
const char* password = "a123zsraa";     // Substitua por sua senha

// Configurações do Broker MQTT
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

// Nome do tópico para publicação
const char* topic = "esp32/speed";

// Pino para o LED (muitos modelos ESP32 usam GPIO2 para o LED embutido)
const int ledPin = 2;  // Pode ser outro pino dependendo do modelo

bool wifi_connected = false;
bool mqtt_connected = false;
bool led_state = LOW; // Começa desligado

// Criar objetos para Wi-Fi e MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Função para conectar ao Wi-Fi
void setup_wifi() {
  delay(10);
  Serial.println("Conectando ao Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  wifi_connected = true;
  Serial.println("");
  Serial.print("Conectado ao Wi-Fi: ");
  Serial.println(WiFi.localIP());
}

// Função para conectar ao broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Tentando conectar ao MQTT...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);  // Gerar ID único para o cliente

    if (client.connect(clientId.c_str())) {
      mqtt_connected = true; // Conectado ao MQTT com sucesso
      Serial.println("Conectado ao MQTT");
    } else {
      Serial.print("Falha, rc=");
      Serial.print(client.state());
      Serial.println(" Tentando novamente em 5 segundos");
      delay(5000);  // Esperar antes de tentar novamente
    }
  }
}

bool        is_inst_stride_len_present = true;                              /**< True if Instantaneous Stride Length is present in the measurement. */
bool        is_total_distance_present = true;                               /**< True if Total Distance is present in the measurement. */
bool        is_running = false;                                             /**< True if running, False if walking. */
uint16_t    inst_speed = 0;                                                 /**< Instantaneous Speed. */
uint8_t     inst_cadence = 0;                                               /**< Instantaneous Cadence. */
uint16_t    inst_stride_length = 50;                                        /**< Instantaneous Stride Length. */
uint32_t    total_distance = 0;
float       total_distance_float = 0;

unsigned long last_interrupt_time = 0;
unsigned long last_step_time = 0;
float       distance_per_mark = 0.3; // 30 cm in meters
byte        fakePos[1] = {1};

#define RSCService BLEUUID((uint16_t)0x1814)
BLECharacteristic RSCMeasurementCharacteristics(BLEUUID((uint16_t)0x2A53), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A5D), BLECharacteristic::PROPERTY_READ);
BLEDescriptor RSCDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));

// Buffer for moving average calculation
#define SPEED_BUFFER_SIZE 10
uint16_t speed_buffer[SPEED_BUFFER_SIZE] = {0};
uint8_t speed_buffer_index = 0;

unsigned long last_speed_update_time = 0;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      // Track connection state
    };

    void onDisconnect(BLEServer* pServer) {
      // Track connection state
    }
};

void InitBLE() {
  BLEDevice::init("JG device");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pRSC = pServer->createService(RSCService);

  pRSC->addCharacteristic(&RSCMeasurementCharacteristics);
  RSCDescriptor.setValue("Rate from 0 to 200");
  RSCMeasurementCharacteristics.addDescriptor(&RSCDescriptor);
  RSCMeasurementCharacteristics.addDescriptor(new BLE2902());

  pRSC->addCharacteristic(&sensorPositionCharacteristic);
  sensorPositionDescriptor.setValue("Position 0 - 6");
  sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);

  pServer->getAdvertising()->addServiceUUID(RSCService);

  pRSC->start();
  pServer->getAdvertising()->start();
}

void IRAM_ATTR sensorTrigger() {
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 50) { // Debounce time
    unsigned long msec_between_steps = interrupt_time - last_step_time;
    last_step_time = interrupt_time;

    // Calculate instantaneous speed
    float mps = distance_per_mark / (msec_between_steps / 1000.0);
    uint16_t new_speed = (uint16_t)((256 / 3.6) * mps);

    // Update the speed buffer
    speed_buffer[speed_buffer_index] = new_speed;
    speed_buffer_index = (speed_buffer_index + 1) % SPEED_BUFFER_SIZE;

    is_running = true;

    // Inverter o estado do LED
    led_state = !led_state;
    // Atualizar o estado do LED
    digitalWrite(ledPin, led_state);

    last_interrupt_time = millis(); // Record the time of last trigger for debounce
  }
}


void setup() {
  setup_wifi();  // Conectar ao Wi-Fi
  client.setServer(mqtt_server, mqtt_port);  // Configurar broker MQTT
  pinMode(ledPin, OUTPUT);  // Configurar o pino do LED como saída
  Serial.begin(115200);
  pinMode(25, INPUT);
  attachInterrupt(digitalPinToInterrupt(25), sensorTrigger, RISING);
  InitBLE();
  pinMode(ledPin, OUTPUT);
}

void loop() {
  unsigned long current_time = millis();

  // Check if 500 ms have passed since the last speed update
  if (current_time - last_speed_update_time >= 1000) {
    last_speed_update_time = current_time;

    if (current_time - last_interrupt_time > 1000) {
      // Reset speed if no sensor trigger for 1000 ms
      inst_speed = 0;
      is_running = false;
    } else {
      // Calculate the average speed
      uint32_t speed_sum = 0;
      for (uint8_t i = 0; i < SPEED_BUFFER_SIZE; i++) {
        speed_sum += speed_buffer[i];
      }
      inst_speed = speed_sum / SPEED_BUFFER_SIZE;
    }

    // Update BLE characteristic value
    byte charArray[10] = {
      3,  // Flags (value 3 means running)
      (unsigned byte)inst_speed, (unsigned byte)(inst_speed >> 8), // Speed
      (unsigned byte)inst_cadence, // Cadence
      (unsigned byte)inst_stride_length, (unsigned byte)(inst_stride_length >> 8), // Stride length
      (unsigned byte)total_distance, (unsigned byte)(total_distance >> 8), (unsigned byte)(total_distance >> 16), (unsigned byte)(total_distance >> 24) // Total distance
    };

    RSCMeasurementCharacteristics.setValue(charArray, 10);
    RSCMeasurementCharacteristics.notify();
    sensorPositionCharacteristic.setValue(fakePos, 1);
  }

  // Turn off LED if the sensor hasn't been triggered recently
  if (millis() - last_interrupt_time > 100) {
    digitalWrite(2, HIGH);
  }

   if (!client.connected()) {
    reconnect();  // Se desconectado, tentar reconectar
  }
  
  client.loop();  // Manter a conexão ativa
  
  
  // Publicar a velocidade no tópico
  String message = "Wi-Fi: " + String(wifi_connected ? "Conectado" : "Desconectado") + ", MQTT: " + String(mqtt_connected ? "Conectado" : "Desconectado") + ", Speed: " + String(inst_speed/50) + " km/h";
  Serial.println("Publicando: " + message);
  client.publish(topic, message.c_str());

  delay(1000);  // Tempo restante antes de enviar a próxima mensagem

}

// lab5_ex1.cpp
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Servo.h>

// --- PIN ASSIGNMENTS ---
const int RED_PIN = 26, 
GREEN_PIN = 27, 
BLUE_PIN = 14, 
YELLOW_PIN = 12;
const int BUTTON_PIN = 25, 
LIGHT_SENSOR_PIN = 33, 
SERVO_PIN = 5, 
BUZZER_PIN = 32;

Servo myServo = Servo();

hd44780_I2Cexp lcd;
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

char ssid []= "Wokwi-GUEST";
char password []= "";


// MQTT Broker settings
const char* mqtt_broker = "mqtt.iotserver.uz";
const int mqtt_port = 1883;
const char* mqtt_user = "userTTPU";
const char* mqtt_password = "mqttpass";

const char* mqtt_topic_pub    = "ttpu/Jaloliddin/lab5/out";
const char* mqtt_topic_sub    = "ttpu/Jaloliddin/lab5/in";

const char* mqtt_topic_button = "ttpu/Jaloliddin/lab5/button";
const char* mqtt_topic_servo_speed = "ttpu/Jaloliddin/lab5/servo";
const char* mqtt_topic_sub_buzzer_freq = "ttpu/Jaloliddin/lab5/buzzer";
const char* mqtt_topic_lightsensor = "ttpu/Jaloliddin/lab5/light";


WiFiClient espClient;
PubSubClient mqtt_client(espClient);

//-----------------------------------------------
//Declare Task Handle
TaskHandle_t ServoTaskHandle = NULL;
TaskHandle_t BuzzerTaskHandle = NULL;
TaskHandle_t ButtonTaskHandle = NULL;

TaskHandle_t LightSensorTaskHandle = NULL;
TaskHandle_t LCDTaskHandle = NULL;
SemaphoreHandle_t mqttMutex;



double servo_speed = 10.0; 
const float servo_ke = 0.0; 
int buzzer_frequency = 1000;
int lightValue = 0;
String mqttMessage = "";


// --- PROTOTYPES ---
void connectWiFi();
void connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void ServoTask(void *parameter);  //Task function for controlling Servo motor
void BuzzerTask(void *parameter); //Task function for controlling Buzzer
void ButtonTask(void *parameter); //Task function for controlling Button
void LightSensorTask(void *parameter);  //Task function for controlling Light sensor
void LCDTask(void *parameter);    //Task function for controlling LCD display




// ================================================================
// SETUP 
void setup() {
  Serial.begin(115200);
  mqttMutex = xSemaphoreCreateMutex();
  
  // Configure button 
  pinMode(BUTTON_PIN, INPUT);

   // Configure servo
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_PIN, 0);

  // Configure Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(2, 0, 16);
  ledcAttachPin(BUZZER_PIN, 2);

  lcd.begin(16, 2);

  connectWiFi();

  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);

  // Tasks created with names from your requirement
  xTaskCreatePinnedToCore(ServoTask,  "ServoTask", 10000, NULL, 1, &ServoTaskHandle, 1);
  xTaskCreatePinnedToCore(BuzzerTask, "BuzzerTask", 2048, NULL, 1, &BuzzerTaskHandle, 1);
  xTaskCreatePinnedToCore(ButtonTask, "ButtonTask", 4096, NULL, 4, &ButtonTaskHandle, 1);
  xTaskCreatePinnedToCore(LightSensorTask,  "LightSensorTask", 4096, NULL, 2, &LightSensorTaskHandle, 1);
  xTaskCreatePinnedToCore(LCDTask,    "LCDTask", 2048, NULL, 1, &LCDTaskHandle, 1);
}

// ================================================================
// Loop 
void loop() {

    // Connect to MQTT Broker
  connectMQTT();

    //process incoming MQTT messages
  mqtt_client.loop();

  vTaskDelay(100 / portTICK_PERIOD_MS);

  //Block the mutex before using  mqtt
  if (xSemaphoreTake(mqttMutex,(TickType_t)10) == pdTRUE)
  {
    mqtt_client.loop();
    xSemaphoreGive(mqttMutex);
  }
  else {
    Serial.println("Failed to take MQTT mutex in loop");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

// ================================================================
// CONNECT TO WIFI
void connectWiFi() {
  Serial.println("Connecting to WiFi ...");
  WiFi.begin("Wokwi-GUEST", "");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: "); 
  Serial.println(WiFi.localIP());
}

// ================================================================
// Function to connect/reconnect to MQTT broker
void connectMQTT() {
  while (!mqtt_client.connected()) {
    Serial.println("Connecting to MQTT broker");
    String client_id = "esp32-client-" + String(WiFi.macAddress());
    if (mqtt_client.connect(client_id.c_str(), "userTTPU", "mqttpass")) {
      Serial.println("Connected to MQTT broker!");

      // Subscribe to topic
      mqtt_client.subscribe(mqtt_topic_sub);
      Serial.print("Subscribed to topic: "); 
      Serial.println(mqtt_topic_sub);

      // Subscribe to servo speed topic
      mqtt_client.subscribe(mqtt_topic_servo_speed);
      Serial.print("Subscribed to topic: "); 
      Serial.println(mqtt_topic_servo_speed);

      // Subscribe to buzzer frequency topic
      mqtt_client.subscribe(mqtt_topic_sub_buzzer_freq);
      Serial.print("Subscribed to topic: "); 
      Serial.println(mqtt_topic_sub_buzzer_freq);

      // Subscribe to button topic
      mqtt_client.subscribe(mqtt_topic_button);
      Serial.print("Subscribed to topic: "); 
      Serial.println(mqtt_topic_button);

      // Requirement: Published message to topic
      mqtt_client.publish(mqtt_topic_pub, "Hello from ESP32 MQTT Client");
      Serial.print("Published message to topic: "); 
      Serial.println(mqtt_topic_pub);
    }
     else {
      Serial.print("Failed to connect, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

// ================================================================
// Callback function for received MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
 
  // Convert payload to String
  String msg = "";
  for (unsigned int i=0; i<length; i++) msg += (char)payload[i];
  
  // Specific Log Format
  Serial.print("Message received on topic: ");
  String topicSTR = String(topic);
  Serial.println(topic);

  Serial.print("Message content: ");
  Serial.println(msg);
  Serial.println("-------");
  String msgStr = String(msg);
  
  if (topicSTR == String(mqtt_topic_servo_speed)) {
    double new_speed = msgStr.toDouble();
    if (new_speed > 0.0 && new_speed <= 100.0) {
      servo_speed = new_speed;
      Serial.print("Updated servo speed to: ");
      Serial.println(servo_speed);
    }
    else {
      Serial.println("Invalid servo speed received.");
    }
  }

    else if (topicSTR == String(mqtt_topic_sub_buzzer_freq)) {
      int new_freq = msgStr.toInt();
      if (new_freq >= 0 && new_freq <= 10000) {
        buzzer_frequency = new_freq;
        Serial.print("Updated buzzer frequency to: ");
        Serial.println(buzzer_frequency);
      }
      else {
        Serial.println("Invalid buzzer frequency received.");
      }
    }
    
    if (topicSTR == String(mqtt_topic_sub)) {
    Serial.print("Received message for display: ");
    Serial.println(msgStr);
  }
  else if (String(topic) == mqtt_topic_sub) mqttMessage = msg;
}

// ================================================================
// FREE RTOS TASKS (Updated Serial Labels)
// ================================================================

void ServoTask(void *parameter)
{
  float start_degree,target_degree;

  float current_degree;

  float step_degree = 1.0;

  Serial.println("Servo Task started");
  myServo.write(SERVO_PIN, 0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  start_degree = 0.0;
  target_degree = 180;

  while (1) {
    
    current_degree = start_degree;
    while ((current_degree < target_degree && target_degree > start_degree)
           || (current_degree > target_degree && target_degree < start_degree) ) {

       if (target_degree > start_degree){
         step_degree = 1.0 * servo_speed * 0.1;
       }
       else {
         step_degree = -1.0 * servo_speed * 0.1;
       }
      // update current degree
      current_degree += step_degree;
      myServo.write(SERVO_PIN, current_degree);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
      vTaskDelay(5000 / portTICK_PERIOD_MS);

      // swap start and target degrees
      float temp = start_degree;
      start_degree = target_degree;
      target_degree = temp;
  }
}

void BuzzerTask(void *parameter) {
  Serial.println("Buzzer Task started");
  while (1) {
    if (buzzer_frequency >= 0)
    {
      ledcWriteTone(2, buzzer_frequency);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ledcWriteTone(2, 0);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void ButtonTask(void *parameter) 
{
  Serial.println("Button Task started");
  int previous_state = LOW;
  int current_state = LOW;
    while (1) {
        int current_state = digitalRead(BUTTON_PIN);
        if (current_state == HIGH && previous_state == LOW) {
          Serial.println("Button pressed!");
            if (xSemaphoreTake(mqttMutex, (TickType_t)10) == pdTRUE) {
                mqtt_client.publish(mqtt_topic_button, "Button Pressed!");
                xSemaphoreGive(mqttMutex);
            }
            else {
              Serial.println("Failed to take MQTT mutex in ButtonTask");
            }
        }
        previous_state = current_state;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void LightSensorTask(void *parameter) 
{
  Serial.println("Light Sensor Task started");
  while (1) {
    lightValue = analogRead(LIGHT_SENSOR_PIN);
    Serial.print("Light Sensor Value: ");
    Serial.println(lightValue);

    if (xSemaphoreTake(mqttMutex,(TickType_t)100) == pdTRUE) {
      String light_str= String(lightValue);
      mqtt_client.publish(mqtt_topic_lightsensor, light_str.c_str());
      xSemaphoreGive(mqttMutex);
    }
    else {
      Serial.println("Failed to take MQTT mutex in LightSensorTask");
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void LCDTask(void *parameter) {
  while (true) {
    lcd.clear();
    
    // --- Line 1: S: (servo), L: (light), B: (buzzer) ---
    lcd.setCursor(0, 0);
    lcd.print("S:"); 
    // Fix: Pass SERVO_PIN to the read function
    lcd.print((int)myServo.read(SERVO_PIN)); 
    
    lcd.print(" L:"); 
    lcd.print(lightValue);
    
    lcd.print(" B:"); 
    lcd.print(buzzer_frequency);

    // --- Line 2: Message from MQTT ---
    lcd.setCursor(0, 1);
    if (mqttMessage.length() > 0) {
      lcd.print(mqttMessage.substring(0, 16));
    } else {
      lcd.print("Waiting MQTT...");
    }

    // Requirement: Update every 2 seconds
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
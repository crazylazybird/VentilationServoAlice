#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// ================== НАСТРОЙКИ СЕТИ ==================
const char* WIFI_SSID     = "Anton";
const char* WIFI_PASSWORD = "11223360";

// ================== MQTT (Dealgate) ==================
// См. профиль Dealgate: Host/Port/User/Pass/ClientID
// Host рекомендуют доменное имя: mqtt.dealgate.ru
// Port обычно 1883 (проверь в профиле)
const char* MQTT_HOST     = "mqtt.dealgate.ru";
const uint16_t MQTT_PORT  = 1883;

const char* MQTT_USER     = "ghost101";
const char* MQTT_PASS     = "1234567890";

// В профиле/устройстве Dealgate укажи тот же ClientID
const char* MQTT_CLIENT_ID = "alice_servo_c3_01";

// Last Will (по желанию, для диагностики в Dealgate)
const char* LWT_TOPIC = "servo/lwt";
const char* LWT_OFF   = "OFF";
const char* LWT_ON    = "ON";

// ================== MQTT ТОПИКИ УМЕНИЙ ==================
// Сконфигурируй в карточке устройства Dealgate те же топики
const char* POWER_SET_TOPIC   = "servo/power/set";   // приём ON/OFF
const char* POWER_STATE_TOPIC = "servo/power/state"; // публикация ON/OFF (retained)

const char* ANGLE_SET_TOPIC   = "servo/angle/set";   // приём 0..100 (%)
const char* ANGLE_STATE_TOPIC = "servo/angle/state"; // публикация 0..100 (retained)

// ================== СЕРВО ==================
const int SERVO_PIN   = 4;      // GPIO4 (ESP32-C3)
const int SERVO_MIN_US = 500;   // калибруй под свой сервопривод
const int SERVO_MAX_US = 2500;

const int MOVE_STEP_DEG = 2;    // шаг плавного движения (градусы)
const int MOVE_STEP_MS  = 10;   // задержка между шагами (мс)

// ================== ГЛОБАЛЬНЫЕ ==================
WiFiClient wifi;
PubSubClient mqtt(wifi);
Servo servo;

bool servoEnabled = false;   // OFF по умолчанию (detached)
int currentAngleDeg = 0;     // 0..180

// ================== УТИЛИТЫ ==================
int percentToDeg(int percent) {
  percent = constrain(percent, 0, 100);
  // линейная шкала: 100% -> 180°
  long deg = lroundf(percent * 1.8f);
  return constrain((int)deg, 0, 180);
}

int degToPercent(int deg) {
  deg = constrain(deg, 0, 180);
  long p = lroundf(deg / 1.8f);
  return constrain((int)p, 0, 100);
}

void attachIfNeeded() {
  if (!servoEnabled || !servo.attached()) {
    servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servoEnabled = true;
  }
}

void detachIfNeeded() {
  if (servo.attached()) servo.detach();
  servoEnabled = false;
}

void moveServoSmoothToDeg(int targetDeg) {
  targetDeg = constrain(targetDeg, 0, 180);
  attachIfNeeded();

  int step = (targetDeg > currentAngleDeg) ? MOVE_STEP_DEG : -MOVE_STEP_DEG;
  while (currentAngleDeg != targetDeg) {
    int next = currentAngleDeg + step;
    if ((step > 0 && next > targetDeg) || (step < 0 && next < targetDeg)) next = targetDeg;
    currentAngleDeg = next;
    servo.write(currentAngleDeg);
    delay(MOVE_STEP_MS);
  }
}

// ================== ПУБЛИКАЦИИ СОСТОЯНИЯ ==================
void publishPowerState() {
  mqtt.publish(POWER_STATE_TOPIC, servoEnabled ? "ON" : "OFF", true /*retained*/);
}

void publishAngleState() {
  char payload[8];
  snprintf(payload, sizeof(payload), "%d", degToPercent(currentAngleDeg));
  mqtt.publish(ANGLE_STATE_TOPIC, payload, true /*retained*/);
}

void publishAllStates() {
  publishPowerState();
  publishAngleState();
}

// ================== CALLBACK ==================
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Скопируем payload в строку (терминируем)
  char msg[64];
  unsigned int n = min(length, (unsigned int)(sizeof(msg) - 1));
  memcpy(msg, payload, n);
  msg[n] = '\0';

  // POWER: ON/OFF
  if (strcmp(topic, POWER_SET_TOPIC) == 0) {
    if (strcasecmp(msg, "ON") == 0) {
      attachIfNeeded();
      publishPowerState();
      // не двигаем — оставляем последний угол
    } else if (strcasecmp(msg, "OFF") == 0) {
      detachIfNeeded();
      publishPowerState();
    }
    return;
  }

  // ANGLE: 0..100 (%)
  if (strcmp(topic, ANGLE_SET_TOPIC) == 0) {
    // парсим целое 0..100
    int percent = atoi(msg);
    percent = constrain(percent, 0, 100);
    int targetDeg = percentToDeg(percent);

    // Если OFF — сначала attach
    attachIfNeeded();
    moveServoSmoothToDeg(targetDeg);
    publishAngleState();
    publishPowerState(); // на всякий случай
    return;
  }
}

// ================== СЕТЬ ==================
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi: connecting to %s\n", WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - start > 30000) {
      Serial.println("\nWiFi: retry");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      start = millis();
    }
  }
  Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

void ensureMqtt() {
  if (mqtt.connected()) return;

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  // LWT (завещание)
  mqtt.connect(
    MQTT_CLIENT_ID,
    MQTT_USER, MQTT_PASS,
    LWT_TOPIC, 0, true, LWT_OFF
  );

  if (mqtt.connected()) {
    // подписки
    mqtt.subscribe(POWER_SET_TOPIC);
    mqtt.subscribe(ANGLE_SET_TOPIC);

    // онлайн-метка
    mqtt.publish(LWT_TOPIC, LWT_ON, true /*retained*/);

    // опубликуем начальное состояние
    publishAllStates();

    Serial.println("MQTT: connected");
  } else {
    Serial.printf("MQTT: connect failed (state=%d)\n", mqtt.state());
  }
}

// ================== SETUP / LOOP ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Старт: угол 0°, OFF (detached)
  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo.write(currentAngleDeg);
  delay(200);
  detachIfNeeded();

  ensureWifi();
  ensureMqtt();
}

void loop() {
  ensureWifi();
  ensureMqtt();
  mqtt.loop();
}

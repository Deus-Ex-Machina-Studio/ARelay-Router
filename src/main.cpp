#include <Arduino.h>
#include "ESP8266WiFi.h"
#include "ESP8266Ping/src/ESP8266Ping.h"

#include "Timer.h"

#define SSID "KPKO5" // Robostart
#define PASSWORD "Le02062057shy" // 25028325
#define HOST "yandex.ru"
#define RELAY_PIN D2 // Порт Реле
#define LED_PIN D4 // Порт светодиода ESP8266
#define DELAY_PIN 600000 // Пауза для ping'а
#define DELAY_RELOAD 10000 // Пауза для перезагрузки
#define DELAY_TO_OFF_WIFI 2000 // Пауза для ожидания полного выключения роутера
#define DELAY_TO_ON_WIFI 60000 // Пауза для ожидания запуска точки доступа на роутере
#define ERRORS_COUNT 2 // Максимальное кол-во возможных ошибок при ping'е

bool b_lastPing = false;
bool b_ping = false;
bool b_wasReload = false;
byte i_errorCount = 0;

void ConnectToWifi() {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("*");
    }
    
    Serial.println("");
    Serial.println("WiFi connection Successful");
    Serial.print("The IP Address of ESP8266 Module is: ");
    Serial.println(WiFi.localIP());
}

void CheckConnection() {
    bool b_currentPing = Ping.ping(HOST, 1);
    
    if (!b_currentPing) {
        i_errorCount++;
    } else {
        i_errorCount = 0;
        b_ping = b_currentPing;
    }

    if (i_errorCount >= ERRORS_COUNT) b_ping = b_currentPing;

    b_lastPing = b_currentPing;

    digitalWrite(LED_PIN, !b_ping);
}

void Reload() {
    if (!b_ping) {
        digitalWrite(RELAY_PIN, false);
        delay(DELAY_TO_OFF_WIFI);
        digitalWrite(RELAY_PIN, true);
        delay(DELAY_TO_ON_WIFI);
        if (WiFi.status() != WL_CONNECTED) ConnectToWifi();
    }
}

Timer pingTimer = Timer(DELAY_PIN, [] {
    CheckConnection();
});

Timer reloadTimer = Timer(DELAY_RELOAD, [] {
    Reload();
});

void setup(void) { 
    Serial.begin(115200);

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(D4, LOW);

    ConnectToWifi();
    
    b_lastPing = false;
    b_ping = true;
}

void loop() {
    pingTimer.Update();
    reloadTimer.Update();
}

/*
#define SSID "ModelFactory"
#define PASSWORD "mf2014org"
#define RELAY_PIN D2
#define ERRORS_COUNT 2

bool b_lastPing = false;
bool b_ping = false;
byte i_errorCount = 0;

void ConnectToWifi() {
    // Connect to WiFi
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("*");
    }
    
    Serial.println("");
    Serial.println("WiFi connection Successful");
    Serial.print("The IP Address of ESP8266 Module is: ");
    Serial.println(WiFi.localIP());
}

void setup(void) { 
    Serial.begin(115200);

    ConnectToWifi();
    
    pinMode(RELAY_PIN, OUTPUT);

    b_lastPing = WiFi.status();
    b_ping = b_lastPing;

    digitalWrite(RELAY_PIN, b_ping);
}

void loop() {
    bool b_currentPing = (WiFi.status() == WL_CONNECTED);
    
    if (!b_currentPing) {
        i_errorCount++;
    } else {
        i_errorCount = 0;
    }

    if (i_errorCount >= ERRORS_COUNT) b_ping = b_currentPing;
    
    b_lastPing = b_currentPing;
    
    Serial.println(b_ping);
    if (!b_ping) {
        digitalWrite(RELAY_PIN, b_ping);
        delay(500);
        digitalWrite(RELAY_PIN, !b_ping);
        delay(500);
        ConnectToWifi();
    }
    
    delay(2000);
}
*/
/*
#define SSID "ModelFactory"
#define PASSWORD "mf2014org"
#define HOST "yandex.ru"
#define RELAY_PIN D2
#define ERRORS_COUNT 2

bool b_lastPing = false;
bool b_ping = false;
byte i_errorCount = 0;

void ConnectToWifi() {
    // Connect to WiFi
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("*");
    }
    
    Serial.println("");
    Serial.println("WiFi connection Successful");
    Serial.print("The IP Address of ESP8266 Module is: ");
    Serial.println(WiFi.localIP());
}

void setup(void) { 
    Serial.begin(115200);

    ConnectToWifi();
    
    pinMode(RELAY_PIN, OUTPUT);

    b_lastPing = true;
    b_ping = b_lastPing;

    digitalWrite(D2, HIGH);
}

void loop() {
    bool b_currentPing = Ping.ping(HOST, 1);
    
    if (!b_currentPing) {
        i_errorCount++;
    } else {
        i_errorCount = 0;
    }

    if (i_errorCount >= ERRORS_COUNT) b_ping = b_currentPing;
    
    b_lastPing = b_currentPing;
    
    Serial.println(b_ping);
    if (!b_ping) {
        digitalWrite(RELAY_PIN, b_ping);
        delay(500);
        digitalWrite(RELAY_PIN, !b_ping);
        delay(100);
        ConnectToWifi();
    }
    
    delay(100);
}

*/
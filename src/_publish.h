#include <MqttConnector.h>

extern int relayPinState;
extern MqttConnector *mqtt;

extern int relayPin;
extern int LED_PIN;

extern float shuntvoltage, busvoltage, loadvoltage;
extern double current_mA, current_A, sumCurrent;
// extern float power_mW = 0, power_W = 0;
// extern uint32_t pevTime = 0, pevPrint = 0;
// extern uint32_t powerCount = 0;
// extern double powerSum = 0;
// extern double powerAverage = 0;
// extern double powerHour = 0, powerWattHour, powerKilosWattHour;

extern char myName[];

static void readSensor();

extern String DEVICE_NAME;
extern int PUBLISH_EVERY;

void register_publish_hooks()
{
  strcpy(myName, DEVICE_NAME.c_str());
  mqtt->on_prepare_data_once([&](void) {
    Serial.println("initializing sensor...");
  });

  mqtt->on_before_prepare_data([&](void) {
    readSensor();
  });

  mqtt->on_prepare_data([&](JsonObject *root) {
    JsonObject &data = (*root)["d"];
    JsonObject &info = (*root)["info"];
    data["myName"] = myName;
    data["millis"] = millis();
    if (busvoltage > 0 && busvoltage < 12)
    {
      data["voltage"] = busvoltage;
    }
    if (current_mA > 0)
    {
      data["current_mA"] = current_mA;
    }
    // data["power_mW"] = power_mW;
    data["relayState"] = relayPinState;
    data["updateInterval"] = PUBLISH_EVERY;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  },
                        PUBLISH_EVERY);
  mqtt->on_after_prepare_data([&](JsonObject *root) {
    /**************
      JsonObject& data = (*root)["d"];
      data.remove("version");
      data.remove("subscription");
    **************/
  });

  mqtt->on_published([&](const MQTT::Publish &pub) {
    Serial.println("Published.");
  });
}

static void readSensor()
{
  // perform reading sensor
  Serial.println("Perform reading sensor...");

  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  Serial.print("Voltage     : ");
  Serial.print(busvoltage);
  Serial.println(" V");
  Serial.print("Current mA  : ");
  Serial.print(current_mA);
  Serial.println(" mA");
  // Serial.print("Power mW    : ");
  // Serial.print(power_mW, 6);
  // Serial.println(" mWS");
  // Serial.print("Power sum   : ");
  // Serial.print(powerSum, 6);
  // Serial.println(" mWS");
}

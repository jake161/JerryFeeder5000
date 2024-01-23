//This example demonstrates the ESP RainMaker with a custom device
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"

#define DEFAULT_MOISTURE_LEVEL 0
const char *service_name = "JERRY_1";
const char *pop = "a1d3s2";


//This is all setup BS
//GPIO for sensor reading
#if CONFIG_IDF_TARGET_ESP32C3
static int sensorPin= A0;
static int gpio_reset = D9;
#else
//GPIO for virtual device
static int sensorPin = A0;
static int gpio_reset = D9;
#endif

int sensorValue=0;


// The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
// But, you can also define custom devices using the 'Device' base class object, as shown here
static Device *my_device = NULL;
//This is the initialization of what I care about^

//This is some rainmaker Bull
// WARNING: sysProvEvent is called from a separate FreeRTOS task (thread)!
void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32S2
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
        printQR(service_name, pop, "softap");
#else
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
        printQR(service_name, pop, "ble");
#endif
        break;
    case ARDUINO_EVENT_PROV_INIT:
        wifi_prov_mgr_disable_auto_stop(10000);
        break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
        wifi_prov_mgr_stop_provisioning();
        break;
    default:;
    }
}
//Ignore this^


void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    const char *device_name = device->getDeviceName();
    const char *param_name = param->getParamName();

    // if (strcmp(param_name, "Power") == 0) {
    //     Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
    //     dimmer_state = val.val.b;
    //     (dimmer_state == false) ? digitalWrite(gpio_dimmer, LOW) : digitalWrite(gpio_dimmer, HIGH);
    //     param->updateAndReport(val);
    // } else if (strcmp(param_name, "Level") == 0) {
    //     Serial.printf("\nReceived value = %d for %s - %s\n", val.val.i, device_name, param_name);
    //     param->updateAndReport(val);
    // }
}

void setup()
{
    Serial.begin(115200);
    pinMode(sensorPin, INPUT);

    Node my_node;
    my_node = RMaker.initNode("ESP RainMaker Node");
    my_device = new Device("MoistureSensor", "custom.device.moisture_sensor");
    //The juicy bits :)

    if (!my_device) {
        return;
    }
    //Create custom dimmer device
    my_device->addNameParam();
    my_device->assignPrimaryParam(my_device->getParamByName(ESP_RMAKER_DEF_POWER_NAME));
    //This requires some further investigation ^ :D


    //Create and add a custom level parameter
    Param moisture_param("Moisture", "custom.param.moisture", value(DEFAULT_MOISTURE_LEVEL), PROP_FLAG_READ | PROP_FLAG_WRITE);
    moisture_param.addBounds(value(0), value(100), value(1));
    moisture_param.addUIType(ESP_RMAKER_UI_TEXT);
    my_device->assignPrimaryParam(my_device->getParamByName("Moisture")); //Why does this work?
    //I like this a lot :)))))

    my_device->addCb(write_callback);

    //Add custom dimmer device to the node
    my_node.addDevice(*my_device);

    //This is optional
    RMaker.enableOTA(OTA_USING_TOPICS);
    //If you want to enable scheduling, set time zone for your region using setTimeZone().
    //The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html
    // RMaker.setTimeZone("Asia/Shanghai");
    // Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
    RMaker.enableTZService();

    RMaker.enableSchedule();

    RMaker.enableScenes();

    RMaker.start();

    WiFi.onEvent(sysProvEvent);  // Will call sysProvEvent() from another thread.
#if CONFIG_IDF_TARGET_ESP32S2
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#else
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#endif
}

void loop()
{
    if (digitalRead(gpio_reset) == LOW) { //Push button pressed

        // Key debounce handling
        delay(100);
        int startTime = millis();
        while (digitalRead(gpio_reset) == LOW) {
            delay(50);
        }
        int endTime = millis();

        if ((endTime - startTime) > 10000) {
            // If key pressed for more than 10secs, reset all
            Serial.printf("Reset to factory.\n");
            RMakerFactoryReset(2);
        } else if ((endTime - startTime) > 3000) {
            Serial.printf("Reset Wi-Fi.\n");
            // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
            RMakerWiFiReset(2);
        } else {
            //Monkeying Around
            sensorValue = analogRead(sensorPin);
            Serial.printf("Moisture Value @ " + sensorValue);
            if (my_device) {
                my_device->updateAndReportParam("Moisture", sensorValue);
            }
        }
    }
    delay(100);
}
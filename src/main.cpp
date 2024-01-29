// This example demonstrates the ESP RainMaker with a custom device
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"

#define DEFAULT_POWER_MODE true
#define DEFAULT_MOISTURE_MSG "Starting Up"
#define DEFAULT_CALIBRATION_LEVEL 0
#define DEFAULT_MOISTURE_READ 0
const char *service_name = "PROV_1234";
const char *pop = "abcd1234";

const bool DEBUGMSG = false;

// GPIO for push button
static int gpio_0 = 9;
static int gpio_moisture = A1;


int moisture_state = 0;
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 10000;
int moisture_calibration = 0;

// Function Prototype
void updateMoisture();

// The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
// But, you can also define custom devices using the 'Device' base class object, as shown here
static Device *my_device = NULL;

// WARNING: sysProvEvent is called from a separate FreeRTOS task (thread)!
void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id)
    {
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

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    const char *device_name = device->getDeviceName();
    const char *param_name = param->getParamName();

    // example for updating params on the fly
    //  if (strcmp(param_name, "Power") == 0)
    //  {
    //      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
    //      dimmer_state = val.val.b;
    //      (dimmer_state == false) ? digitalWrite(gpio_dimmer, LOW) : digitalWrite(gpio_dimmer, HIGH);
    //      param->updateAndReport(val);
    //  }
    if (strcmp(param_name, "Calibration Level") == 0)
    {
        Serial.printf("\nReceived value = %d for %s - %s\n", val.val.i, device_name, param_name);
        param->updateAndReport(val);
        moisture_calibration = val.val.i;
        if (DEBUGMSG == true)
        {
            Serial.println("Calibration Point: " + String(moisture_calibration)); // for DEBUGMSG
        }
        updateMoisture();
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(gpio_0, INPUT);
    pinMode(gpio_moisture, INPUT);
    analogRead(gpio_moisture);

    Node my_node;
    my_node = RMaker.initNode("ESP RainMaker Node");
    my_device = new Device("JerryFeeder5000", "custom.device.jerry", &gpio_moisture);
    if (!my_device)
    {
        return;
    }

    // Create custom device
    my_device->addNameParam();

    Param moisture_param("Moisture", "custom.param.moisture", value(DEFAULT_MOISTURE_MSG), PROP_FLAG_READ);
    moisture_param.addUIType(ESP_RMAKER_UI_TEXT);
    my_device->addParam(moisture_param);
    my_device->assignPrimaryParam(my_device->getParamByName("Moisture"));

    Param data_param("Moisture Reading", "custom.param.data", value(DEFAULT_MOISTURE_READ), PROP_FLAG_READ);
    moisture_param.addUIType(ESP_RMAKER_UI_TEXT);
    my_device->addParam(data_param);

    Param calibration_param("Calibration Level", "custom.param.cal", value(DEFAULT_CALIBRATION_LEVEL), PROP_FLAG_READ | PROP_FLAG_WRITE);
    calibration_param.addUIType(ESP_RMAKER_UI_SLIDER);
    calibration_param.addBounds(value(0), value(4000), value(1));
    my_device->addParam(calibration_param);

    my_device->addCb(write_callback);

    // Add custom device to the node
    my_node.addDevice(*my_device);

    // This is optional
    RMaker.enableOTA(OTA_USING_TOPICS);
    // If you want to enable scheduling, set time zone for your region using setTimeZone().
    // The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html
    //  RMaker.setTimeZone("Asia/Shanghai");
    //  Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
    RMaker.enableTZService();

    RMaker.enableSchedule();

    RMaker.enableScenes();

    RMaker.start();

    WiFi.onEvent(sysProvEvent); // Will call sysProvEvent() from another thread.
#if CONFIG_IDF_TARGET_ESP32S2
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#else
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

    startMillis = millis();
}

void loop()
{
    if (digitalRead(gpio_0) == LOW)
    { // Push button pressed

        // Key debounce handling
        delay(100);
        int startTime = millis();
        while (digitalRead(gpio_0) == LOW)
        {
            delay(50);
        }
        int endTime = millis();

        if ((endTime - startTime) > 10000)
        {
            // If key pressed for more than 10secs, reset all
            Serial.printf("Reset to factory.\n");
            RMakerFactoryReset(2);
        }
        else if ((endTime - startTime) > 3000)
        {
            Serial.printf("Reset Wi-Fi.\n");
            // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
            RMakerWiFiReset(2);
        }
        else
        {
            moisture_state = analogRead(gpio_moisture);
            Serial.printf("\nSet moisture wet calibration value to: \"%i\" \n", moisture_state);
            moisture_calibration = moisture_state;
            my_device->updateAndReportParam("Calibration Level", moisture_calibration);
            updateMoisture();
        }
    }

    currentMillis = millis();                  // get the current "time" (actually the number of milliseconds since the program started)
    if (currentMillis - startMillis >= period) // test whether the period has elapsed
    {
        updateMoisture();
        startMillis = currentMillis; // IMPORTANT to save the start time
    }
    delay(100);
}

void updateMoisture()
{
    moisture_state = analogRead(gpio_moisture);
    if (moisture_state > moisture_calibration)
    {
        my_device->updateAndReportParam("Moisture", "Needs a drink (✖╭╮✖)");
        my_device->updateAndReportParam("Moisture Reading", moisture_state);
        if (DEBUGMSG == true)
        {
            Serial.println("Moisture Read: " + String(moisture_state)); // For DEBUGMSG
        }
    }
    else
    {
        my_device->updateAndReportParam("Moisture", "Saturated ;)");
        my_device->updateAndReportParam("Moisture Reading", moisture_state);
        if (DEBUGMSG == true)
        {
            Serial.println("Moisture Read: " + String(moisture_state)); // For DEBUGMSG
        }
    }
}
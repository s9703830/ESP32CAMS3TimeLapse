#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
//#include <WiFiEspClient.h>
//#include <WiFiEsp.h>
#include <PubSubClient.h>
#include <time.h>
#include <SPI.h>
#include <Wire.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_camera.h"
#include "esp_spi_flash.h"
#include "esp_sntp.h"
#include "esp32-hal.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "Base64.h"
#include "mbedtls/base64.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include "camera_pins.h" // CAMERA_MODEL_AI_THINKER defined in camera_pins.h

#include <Int64String.h>

#include "ThingsBoard.h" // Include THingsBoard IoT lib for MQTT etc.

#include <Adafruit_Sensor.h>
//#include <Adafruit_I2CDevice.h>
//#include <Adafruit_TSL2591.h>
#include <DHT.h>
#include <DHT_U.h>

// See guide for details on sensor wiring and usage:
// https://learn.adafruit.com/dht/overview
// https://github.com/adafruit/DHT-sensor-library
#define DHTTYPE DHT11     // DHT 11
#define DHTPIN 13   // GPIO16 Digital pin connected to the DHT sensor 
DHT dht(DHTPIN, DHTTYPE);
DHT_Unified dht_u(DHTPIN, DHTTYPE);

// I2C START Pin definition etc.
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15
#define I2C_CLOCK 100000 // 100000 = 100kHz, 400000 = 400kHz
#define I2C_ADDR_TSL25911_LIGHT_SENSOR 0x29 // already defaulted to this so this define is not used
#define I2C_ADDR__UV_SENSOR 0x29
#define I2C_ADDR_OLED_SSD1306_128X64 0x3C
//TwoWire i2cBus = TwoWire(0);
//Adafruit_I2CDevice i2c_dev = Adafruit_I2CDevice();
//Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
// I2C END


// AWS MQTT START
#define AWS_MQTT_TOPIC_DHT11_SENSOR "hrcos82greenhouse-dht11"
#define AWS_MQTT_TOPIC_LUX_SENSOR "hrcos82greenhouse-dht11"  // TODO 
#define AWS_MQTT_TOPIC_OXYGEN_SENSOR "hrcos82greenhouse-dht11" // TODO 
#define AWS_MQTT_TOPIC_WATER_SENSOR "hrcos82greenhouse-dht11" // TODO 
// AWS MQTT END

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define DEFAULT_FRAMESIZE FRAMESIZE_UXGA


#ifndef WIFI_SSID
#define WIFI_SSID "MOBISOFT-2.4G"
#define WIFI_PASS "1singapoer4"
#endif

// Initialize the WiFi Ethernet client object
// WiFiEspClient espClient;
//WiFiClient espClient;
WiFiClientSecure espClient;

// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
PubSubClient awsMQTTClient(espClient); //  AWS MQTT Client

char thingsboardServer[] = "thingsboard.markushaywood.net"; // "YOUR_THINGSBOARD_HOST_OR_IP";
#define TOKEN "d3kS0YQsuLrxnCgrscO0"


#define GMT_TIME_OFFSET 3600*2 // measure in seconds GMT+1 = 3600 GMT=2=3600*2
#define GMT_TIME_DLS_OFFSET 0 // Day light savings offset i s0 for ZA
#define TIME_ZONE "SAST-2,M3.5.0/02,M10.5.0/03" // Set with your time zone https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv

#define S3_API_GW_BASEURL "https://vavi8xg2d2.execute-api.us-east-1.amazonaws.com/prod/"

// Define the AWS CA Root certificate as we upload to AWS services
// https://docs.platformio.org/en/latest/platforms/espressif32.html#embedding-binary-data
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_src_certs_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_src_certs_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_src_certs_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_src_certs_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_src_certs_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_src_certs_private_pem_key_end");

// WiFI credentials
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;	
const char* ntpServer = "pool.ntp.org"; //"time.google.com"; //
bool internet_connected = false;
String MAC = ""; //String(WiFi.macAddress());

const char* mqtt_server = "a17cw65b0eoxkx-ats.iot.us-east-1.amazonaws.com"; // Relace with your MQTT END point
const int   mqtt_port = 8883;


time_t now;    // this is the epoch
tm timeInfo;         // the structure tm holds time information in a more convenient way
String timeEpoch =  ""; //String(timeClient.getEpochTime());
String timeEpochms =  ""; //String(timeClient.getEpochTime());


bool connect_wifi() {
  int connAttempts = 0;
  bool connected = false;

  // disconnect if connection possibly already exists, before connecting
  if(internet_connected) WiFi.disconnect();

  espClient.setCACert((char*)aws_root_ca_pem_start);
  espClient.setCertificate((char*)certificate_pem_crt_start);
  espClient.setPrivateKey((char*)private_pem_key_start);

  Serial.println(F("-------------------- WiFi CONNECTING --------------------------"));
  Serial.println("\r\nConnecting to: " + String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED ) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.print(".");
    if (connAttempts > 10) { // try to connect 10 times with a 1 second delay before giving up
       connected =  false;
       Serial.println("\r\nConnecting to: " + String(ssid) + "FAILED!!");
    }
    connAttempts++;
  }
  Serial.println();
  if(WiFi.status() == WL_CONNECTED ){
    connected = true;    
    MAC = WiFi.macAddress();
    Serial.println(F("WiFI Internet connected"));
  }
  Serial.println(F("---------------------------------------------------------------"));
  internet_connected = connected;
  return connected;
}

void printESP32ChipInfo() {
  Serial.println();
  Serial.println(F("------------------- ESP32 CAM Chip Info ------------------------"));
  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  Serial.printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
          chip_info.cores,
          (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
          (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  Serial.printf("silicon revision %d, ", chip_info.revision);

  Serial.printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
          (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();
  fflush(stdout);
}

void restart_system() {
  Serial.println();
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println("Restarting System now!");
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();
  if(internet_connected) WiFi.disconnect();
  fflush(stdout);
  esp_restart();
}

void cbSyncTime(struct timeval *tv)  // callback function to show when NTP was synchronized
{
  Serial.println(F("NTP time synched"));
}

// https://techtutorialsx.com/2021/09/01/esp32-system-time-and-sntp/
// https://werner.rothschopf.net/microcontroller/202103_arduino_esp32_ntp_en.htm
void sync_time_sntp() {
  sntp_set_sync_interval(12 * 60 * 60 * 1000UL); // 12 hours
  sntp_set_time_sync_notification_cb(cbSyncTime);  // set a Callback function for time synchronization notification
  //configTime(GMT_TIME_OFFSET, GMT_TIME_DLS_OFFSET, ntpServer); we are using setenv so set to 0,0,ntpServer
  configTime(0, 0, ntpServer);
  setenv("TZ", 	TIME_ZONE, 1); 
  tzset();
}

int64_t xx_time_get_time() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

void refreshTime() {
  Serial.println();
  Serial.println(F("------------------------- refreshTime  -----------------------"));

  time(&now); // read the current time
  localtime_r(&now, &timeInfo); // update the structure tm with the current time

  struct timeval time_now;
  gettimeofday(&time_now, NULL);
  int64_t  msecs_time = (time_now.tv_sec * 1000LL) + (time_now.tv_usec / 1000LL);

  timeEpochms = String(int64String(msecs_time)); 
  Serial.println("refreshTime: timeEpochms : " + timeEpochms);
  timeEpoch = String(timeInfo.tm_year + 1900) + "-" + String(timeInfo.tm_mon + 1) + "-" + String(timeInfo.tm_mday) + "-" +  String(timeInfo.tm_hour) + "-" + String(timeInfo.tm_min) + "-" +  String(timeInfo.tm_sec);
  Serial.println(F("--------------------------------------------------------------"));
  Serial.println();
}

void printTime() {
  Serial.println();
  Serial.println(F("------------------------- printTime ---------------------------"));
  //refreshTime(); // refresh in main loop to keep time the same for all sensor and photo readings
  Serial.print(F("year:"));
  Serial.print(timeInfo.tm_year + 1900); // years since 1900
  Serial.print(F("\tmonth:"));
  Serial.print(timeInfo.tm_mon + 1); // January = 0 (!)
  Serial.print(F("\tday:"));
  Serial.print(timeInfo.tm_mday); // day of month
  Serial.print(F("\thour:"));
  Serial.print(timeInfo.tm_hour); // hours since midnight 0-23
  Serial.print(F("\tmin:"));
  Serial.print(timeInfo.tm_min); // minutes after the hour 0-59
  Serial.print(F("\tsec:"));
  Serial.print(timeInfo.tm_sec); // seconds after the minute 0-61*
  Serial.print(F("\twday"));
  Serial.print(timeInfo.tm_wday); // days since Sunday 0-6
  if (timeInfo.tm_isdst == 1) // Daylight Saving Time flag
    Serial.print(F("\tDST"));
  else
    Serial.print(F("\tstandard"));
  Serial.println();
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();
} // end printTime() 


void dht_sensor_init() {
  // Initialize device.
  dht.begin();
  // Set delay between sensor readings based on sensor details.
  //delayMS = sensor.min_delay / 1000;
} // end dht_sensor_init()

void dht_sensor_info_print() {
  Serial.println();
  Serial.println(F("----------------------- dht_sensor_info_print ---------------------------"));
  Serial.println(F("DHTxx Unified Sensor info print"));
  // Print temperature sensor details.
  sensor_adafruit_t sensor;
  dht_u.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht_u.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();
} // end dht_sensor_info_print()

void dht_sensor_read_print() {
  Serial.println();
  Serial.println(F("----------------------- dht_sensor_read_print ---------------------------"));
  Serial.println(F("DHTxx Unified Sensor read measurements and print"));

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();
  float heatindex = -100.0;

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  else {
    heatindex = dht.computeHeatIndex(temperature, humidity, false);
    if(isnan(heatindex)) {
      Serial.print(F("Failed to calculate Heat Index!"));
    } else {
      Serial.print(F("Heat Index: "));
      Serial.print(heatindex);
      Serial.println(F("°C"));
      //if ( tb.connected() ) tb.sendTelemetryFloat("heatindex", heatindex);
      // {"ts":1451649600512, "values":{"key1":"value1", "key2":"value2"}}
      String telemetryJSON = "{\"ts\":"+ timeEpochms +", \"values\":{\"temperature\":\"" + temperature + "\", \"humidity\":\"" + humidity + "\", \"heatindex\":\"" + heatindex + "\"}}";
      Serial.println("Sending MQTT sensor data :> " + telemetryJSON);
      if ( tb.connected() ) tb.sendTelemetryJson(telemetryJSON.c_str());
      if ( awsMQTTClient.connected() ) awsMQTTClient.publish(AWS_MQTT_TOPIC_DHT11_SENSOR, telemetryJSON.c_str());
    }
  }

  if (isnan(temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(temperature);
    Serial.println(F("°C"));
    //if ( tb.connected() ) tb.sendTelemetryFloat("temperature", temperature);
  }
  // Get humidity event and print its value.
    if (isnan(humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.println(F("%"));
    //if ( tb.connected() ) tb.sendTelemetryFloat("humidity", humidity);
  }
 
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();  
} // end dht_sensor_read_print()

/*
void displayTSL2591SensorDetails(void)
{
  Serial.println();
  Serial.println(F("----------------------- displayTSL2591SensorDetails ---------------------------"));

  sensor_adafruit_t sensor;
  tsl2591.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}


//    Configures the gain and integration time for the TSL2591
void configureTSL2591Sensor(void)
{
  Serial.println();
  Serial.println(F("------------------- configureTSL2591Sensor ---------------------"));

  if (tsl2591.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
  } 
  else 
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl2591.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  // Display the gain and integration time for reference sake   
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl2591.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl2591.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println();

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}

//    Shows how to perform a basic read on visible, full spectrum or
//    infrared light (returns raw 16-bit ADC values)
void simpleReadTSL2591Sensor(void)
{
  Serial.println();
  Serial.println(F("------------------- simpleReadTSL2591Sensor ---------------------"));
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  uint16_t x = tsl2591.getLuminosity(TSL2591_VISIBLE);
  //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("Luminosity: "));
  Serial.println(x, DEC);
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}

//    Show how to read IR and Full Spectrum at once and convert to lux
void advancedReadTSL2591Sensor(void)
{
  Serial.println();
  Serial.println(F("------------------- advancedReadTSL2591Sensor ---------------------"));
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl2591.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl2591.calculateLux(full, ir), 6);
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println(); 
}
*/

void init_camera() {
  Serial.println();
  Serial.println(F("----------------------- init_camera ---------------------------"));

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    Serial.printf("PSRAM was found");
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    Serial.printf("PSRAM was NOT found");
  }

  Serial.println();

  // Power Cycle the camera before init
  digitalWrite(PWDN_GPIO_NUM, LOW); // Power Down Camera
  delay(10);
  digitalWrite(PWDN_GPIO_NUM, HIGH); // Power Up Camera
  delay(10);
  esp_err_t err = esp_camera_init(&config); // camera init
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //Set camera image quality settings
  s->set_framesize(s, DEFAULT_FRAMESIZE); // Set in defines above
  if (s->id.PID == OV2640_PID) {
  //s->set_ae_level(s,1);
  //s->set_vflip(s, 1);
  //s->set_hmirror(s, 1);
  }

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();
} // end init_camera()


static esp_err_t take_photo(camera_fb_t * fb)
{
  Serial.println();
  Serial.println(F("--------------------- Taking photo... -------------------------"));

  fb = NULL; // Frame Buffer pointer clear
  esp_err_t res = ESP_OK; // error result

  // Turns on the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  //pinMode(4, INPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  //rtc_gpio_hold_en(GPIO_NUM_4);
  //delay(1000);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  fb = esp_camera_fb_get(); // obtain Frame Buffer pointer
  if (fb) {
    Serial.println("Camera capture success.");
    res = ESP_OK;
  }else{
    Serial.println("Camera capture failed!");
    res = ESP_FAIL;
  }

  //esp_camera_fb_return(fb); // Return the frame buffer to be reused again.

  vTaskDelay(500 / portTICK_PERIOD_MS);
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  //rtc_gpio_hold_en(GPIO_NUM_4); // prevent pin config from changing when going to sleep mode
  //delay(1000);
  //vTaskDelay(1000 / portTICK_PERIOD_MS);

  Serial.println(F("--------------------------------------------------------------"));
  Serial.println();

  return res;
} // end take_photo()


esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
  static char *output_buffer;  // Buffer to store response of http request from event handler
  static int output_len;       // Stores number of bytes read

  switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
      Serial.println("HTTP_EVENT_ERROR");
      //ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
      break;
    case HTTP_EVENT_ON_CONNECTED:
      Serial.println("HTTP_EVENT_ON_CONNECTED");
      //ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
      break;
    case HTTP_EVENT_HEADER_SENT:
      Serial.println("HTTP_EVENT_HEADER_SENT");
      //ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
      break;
    case HTTP_EVENT_ON_HEADER:
      Serial.println();
      Serial.printf("HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
      //ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
      break;
    case HTTP_EVENT_ON_DATA:
      Serial.println();
      Serial.printf("HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
      if (!esp_http_client_is_chunked_response(evt->client)) {
        // Write out data
        // printf("%.*s", evt->data_len, (char*)evt->data);
      }
      break;
    case HTTP_EVENT_ON_FINISH:
      Serial.println("");
      Serial.println("HTTP_EVENT_ON_FINISH");
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
      break;
    case HTTP_EVENT_DISCONNECTED:
      Serial.println("HTTP_EVENT_DISCONNECTED");
           ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
      break;
  }
  return ESP_OK;
} // _http_event_handler


String getS3UploadURL() {
//  refreshTime();
  String post_url = S3_API_GW_BASEURL + MAC + "/" + timeEpoch; // Location where images are POSTED
  return post_url;
}


static esp_err_t take_send_photo() {
  esp_err_t err_ret = ESP_OK;

  String post_url2 = getS3UploadURL();
  Serial.print("S3_API_GW_URL : ");
  Serial.print(post_url2);
  char post_url3[post_url2.length() + 1];
  post_url2.toCharArray(post_url3, sizeof(post_url3));

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;

//  res = take_photo(fb);

  // Turns on the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  //pinMode(4, INPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  //rtc_gpio_hold_en(GPIO_NUM_4);
  //delay(1000);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  fb = esp_camera_fb_get(); // obtain Frame Buffer pointer
  if (fb) {
    Serial.println("Camera capture success.");
    res = ESP_OK;
  }else{
    Serial.println("Camera capture failed!");
    res = ESP_FAIL;
  }

  //esp_camera_fb_return(fb); // Return the frame buffer to be reused again.

  vTaskDelay(500 / portTICK_PERIOD_MS);
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);


  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_client.html
  esp_http_client_handle_t http_client;  
  esp_http_client_config_t config_client = {0};  
  config_client.url = post_url3;
  config_client.event_handler = _http_event_handler;
  config_client.cert_pem = (char*)aws_root_ca_pem_start;
  config_client.method = HTTP_METHOD_POST;
  config_client.auth_type  = HTTP_AUTH_TYPE_NONE;
  config_client.transport_type  = HTTP_TRANSPORT_OVER_SSL;

  Serial.println("esp_http_client_init");
  http_client = esp_http_client_init(&config_client);


  int image_buf_size = 4000 * 1000;                                                  
  uint8_t *image = (uint8_t *)ps_calloc(image_buf_size, sizeof(char));

  //taskYIELD();

  size_t length=fb->len;
  size_t olen;

  Serial.print(F("length is: "));
  Serial.println(length);


  int err1 = mbedtls_base64_encode(image, image_buf_size, &olen, fb->buf, length);

  Serial.println(F("esp_http_client_set_post_field"));
  esp_http_client_set_post_field(http_client, (const char *)fb->buf, fb->len);
  
  Serial.println(F("esp_http_client_set_header"));
  esp_http_client_set_header(http_client, "Content-Type", "image/jpg");
  esp_http_client_set_header(http_client, "x-api-key", "GlSOIZLY9jaxTq0LdG9x7aRYHVLGZFyl5GL34EIr");
   
  //esp_task_wdt_reset();  //Reset the watchdog

  Serial.println(F("esp_http_client_perform"));
  err_ret = esp_http_client_perform(http_client);
  if (err_ret == ESP_OK) {
    Serial.println(F("esp_http_client_get_status_code: "));
    Serial.println(esp_http_client_get_status_code(http_client));
  }


  Serial.println(F("esp_http_client_cleanup"));
  esp_http_client_cleanup(http_client);

  Serial.println(F("esp_camera_fb_return"));
  esp_camera_fb_return(fb);

  Serial.println(F("take_send_photo(): DONE"));

  return err_ret;
} // end take_send_photo()


void reconnectThingsBoard() {
  // Loop until we're reconnected
  while (!tb.connected()) {
    Serial.print(F("Connecting to ThingsBoard node ..."));
    // Attempt to connect (clientId, username, password)
    if ( tb.connect(thingsboardServer, TOKEN) ) {
      Serial.println(F("[DONE]"));
    } else {
      Serial.print(F("[FAILED]"));
      Serial.println(F(" : retrying in 5 seconds"));
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
} // reconnectThingsBoard()


void callbackAWSMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("Message arrived ["));
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnectAWSMQTT() {
 // Loop until we're reconnected
  while (!awsMQTTClient.connected()) {
    Serial.print(F("Attempting AWS MQTT connection..."));
    awsMQTTClient.setServer(mqtt_server, mqtt_port);
    awsMQTTClient.setCallback(callbackAWSMQTT);
    // Create a random client ID
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX); // TODO, set more specific and controlled MQTT client id
    // Attempt to connect
    if (awsMQTTClient.connect(clientId.c_str())) {
      Serial.println(F("connected"));
      // Once connected, publish an announcement...
      awsMQTTClient.publish("ei_out", "HRCOS81Greeenhouse1 saying hello world!");
      // ... and resubscribe
      awsMQTTClient.subscribe("ei_in");
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(awsMQTTClient.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
} // end reconnectAWSMQTT()


void i2c_setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  //Wire.begin(I2C_SDA, I2C_SCL, 100000);
  //I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  //i2cBus.begin((int)I2C_SDA, (int)I2C_SCL, (uint32_t)I2C_CLOCK); 
}

void setup() {
  // put your setup code here, to run once:
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200); //  Start the serial port for terminal communication

  printESP32ChipInfo(); // print the chip info
 
  connect_wifi(); // Connect to WiFi

  sync_time_sntp(); // Sync time with time server

  init_camera(); // Initialise the Camera

  //pinMode(DHTPIN, INPUT_PULLUP);
  dht_sensor_init(); // Initilaise the DHT11 Temperature and Humitiy sensor
  dht_sensor_info_print();

  //i2c_setup(); // setup i2c bus
  //configureTSL2591Sensor(); // Initilaise TSL2591 Light Sensor
  //displayTSL2591SensorDetails();

  // SD_MMC.begin("/sdcard", true) // https://randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/

} // end setup()


void loop() {
  // put your main code here, to run repeatedly:

  vTaskDelay(30000 / portTICK_PERIOD_MS); // allow 30 seconds time for WiFi connect and time server sync etc. before starting main loop.

  for (int i = 288; i >= 0; i--) { // loop for 288 * 5 minutes = 24 hours and then reboot, just in case
      //Serial.printf("Restarting in %d seconds...\n", i);
      Serial.printf("Taking photo no: %d and sensor data reading in 5 minutes ...\n", i);

      if ( !tb.connected() ) { // connect to THingsBoard if not connected yet
        //reconnectThingsBoard(); // TODO, make secure connections to THingsBoard
      }
      
      if( !awsMQTTClient.connected() ) {
        reconnectAWSMQTT(); // ensure AWS MQTT connection stay active
      }

      refreshTime(); // refresh time to ensure latest time is submitted with sensor readings
      dht_sensor_read_print();
      //advancedReadTSL2591Sensor();
      printTime();
      take_send_photo();
      printTime();

      tb.loop(); // Loop the ThingsBoard API for procesing
      awsMQTTClient.loop(); // Loop the AWS MQQT client APi for processing

      vTaskDelay(300000 / portTICK_PERIOD_MS); // sleep for 5 minutes before taking next photo and sensor readings
  }
  restart_system();

} // end loop



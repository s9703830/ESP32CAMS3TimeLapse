#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
//#include <SPI.h>
//#include <Wire.h>


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
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "camera_pins.h" // CAMERA_MODEL_AI_THINKER defined in camera_pins.h
#include <Int64String.h>


// AWS MQTT TOPICS
#define AWS_MQTT_TOPIC_GREENHOUSE_SENSORS_LISTEN "hrcos82greenhouse_listen" // The greenhouse controller listens on this Topic for incoming messages
#define AWS_MQTT_TOPIC_GREENHOUSE_SENSORS_SEND "hrcos82greenhouse_send" // The greenhouse controller sends messages on this Topic to AWS IOT Core

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define DEFAULT_FRAMESIZE FRAMESIZE_UXGA

#ifndef WIFI_SSID
#define WIFI_SSID "MOBISOFT-2.4G"
#define WIFI_PASS "1singapoer4"
//#define WIFI_SSID "haywoodm-2.4G"
//#define WIFI_PASS "54RM85M3"
#endif

// Initialize the WiFi Ethernet client object
WiFiClientSecure espClient;

// Initialize AWS Core MQTT instance
PubSubClient awsMQTTClient(espClient); //  AWS MQTT Client


#define GMT_TIME_OFFSET 3600*2 // measure in seconds GMT+1 = 3600 GMT=2=3600*2
#define GMT_TIME_DLS_OFFSET 0 // Day light savings offset is 0 for ZA
#define TIME_ZONE "SAST-2,M3.5.0/02,M10.5.0/03" // Set with your time zone https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv

#define S3_API_GW_BASEURL "https://vavi8xg2d2.execute-api.us-east-1.amazonaws.com/prod/"

// Define the AWS CA Root certificate to use when we communicate AWS services
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
const char* ntpServer = "pool.ntp.org"; //"time.google.com"; // NTP Time Server to use for time synch
bool internet_connected = false;
String MAC = ""; // String(WiFi.macAddress());

const char* mqtt_server = "a17cw65b0eoxkx-ats.iot.us-east-1.amazonaws.com"; // MQTT endpoint to use for MQTT messaging
const int   mqtt_port = 8883; // TCP port to use for MQTT messaging


#define CMD_RECIEVED_BUFFER_LENGTH 3072  // length of the Serial Command recieve buffer from RaspBerry Pi , 3kB
char   cmdReceivedBuffer[CMD_RECIEVED_BUFFER_LENGTH];  // store the Serial CMD
int16_t   cmdReceivedBufferIndex;


time_t now;   // this is the epoch
tm timeInfo;   // the structure tm holds time information in a more convenient way
String timeEpoch =  ""; // String(timeClient.getEpochTime());
String timeEpochms =  ""; // String(timeClient.getEpochTime());


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
} // connect_wifi()


void callbackAWSMQTT(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("[MQTT Message arrived]:=>TOPIC:"));
  Serial.print(topic);
  Serial.print(":==>");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnectAWSMQTT() {
 // Loop until we're reconnected
  awsMQTTClient.setBufferSize(4096); // 4kb buffer size
  while (!awsMQTTClient.connected()) {
    Serial.print(F("Attempting AWS MQTT connection..."));
    awsMQTTClient.setServer(mqtt_server, mqtt_port);
    awsMQTTClient.setCallback(callbackAWSMQTT);
    // Create a random client ID
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX); // TODO, set more specific and controlled MQTT client id
    // Attempt to connect
    if (awsMQTTClient.connect(clientId.c_str())) {
      Serial.println(F("AWS MQTT connected"));
      // Once connected, publish an announcement...
      awsMQTTClient.publish(AWS_MQTT_TOPIC_GREENHOUSE_SENSORS_SEND, "{\"HRCOS81Greeenhouse1\":\" AWS MQTT Reconnected!\"}");
      // ... and resubscribe
      awsMQTTClient.subscribe(AWS_MQTT_TOPIC_GREENHOUSE_SENSORS_LISTEN);
    } else {
      Serial.print(F("AWS MQTT connect Failed, rc="));
      Serial.print(awsMQTTClient.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
  }
} // end reconnectAWSMQTT()


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
  Serial.println(F("[RESTARTING SYSTEM]"));
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
  sntp_set_sync_interval(168 * 60 * 60 * 1000UL); // synch with NTP every 7 days = 168 hours
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



void sendTelemetry(String telemetryJSON) {
  Serial.println();
  Serial.println(F("---------------- sendTelemetry to AWS IoT Core ----------------"));

  if( !awsMQTTClient.connected() ) {
    reconnectAWSMQTT(); // ensure AWS MQTT connection is active before sending telemetry, reconnect if not
  }

  Serial.println("[AWS Sending MQTT sensor data ]:=> " + telemetryJSON);
  if ( awsMQTTClient.connected() ) awsMQTTClient.publish(AWS_MQTT_TOPIC_GREENHOUSE_SENSORS_SEND, telemetryJSON.c_str());

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();   
} // sendTelemetry()




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
    Serial.println();
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    Serial.printf("PSRAM was NOT found");
    Serial.println();
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
      Serial.println();
      //ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
      break;
    case HTTP_EVENT_ON_DATA:
      Serial.println();
      Serial.printf("HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
      Serial.println();
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
  Serial.println(post_url2);
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


String serialRxStr = Serial.readString();  //read until timeout

byte cmdParse(String *pStr)
{
    byte modeIndex = 0;

    if(pStr->startsWith("[MQTT send JSON]=:>")){
        modeIndex = 1;
        Serial.println("RECEIVED SERIAL CMD :==>" + *pStr);  
        refreshTime(); // refresh time to ensure latest time is submitted with sensor readings
        String jsonStr = "{\"ts\":"+ timeEpochms + ", \"tsp\":\""+ timeEpoch + "\"" +pStr->substring(25);  // inject the timestamp and strip of the command header to get the Telemetry JSON payload
        Serial.println("RECEIVED SERIAL CMD :==>" + jsonStr);          
        sendTelemetry(jsonStr); // 
        take_send_photo(); // Take and Upload Photo to AWS S3 together with data metrics recieved from PICO
    }else if(pStr->startsWith("[GET TIME EPOCH MS]=:>")){
        modeIndex = 2;
        Serial.println("[GET TIME EPOCH MS]=:>" + timeEpochms);  
    }else if(pStr->startsWith("CALPH")){
        modeIndex = 1;
    }
    return modeIndex;

}

//void serialEvent() {
void serialEventProcess() {  

  if(Serial.available() > 0){
    Serial.println();
    Serial.println(F("---------------------- serialEvent --------------------------"));

    serialRxStr = Serial.readString();  //read until timeout
    serialRxStr.trim();  // remove any \r \n whitespace at the end of the String
    Serial.println("serialRxStr :> " + serialRxStr);  
    cmdParse(&serialRxStr);

  Serial.println(F("---------------------------------------------------------------"));
  Serial.println();  

  }

} // serialEvent()


void setup() {
  // put your setup code here, to run once:
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200); //  Start the serial port for terminal communication
  Serial.setTimeout(1000); //  Serial read timeout 1000ms

  //printESP32ChipInfo(); // print the chip info
 
  connect_wifi(); // Connect to WiFi

  sync_time_sntp(); // Sync time with time server

  init_camera(); // Initialise the Camera

  // SD_MMC.begin("/sdcard", true) // https://randomnerdtutorials.com/esp32-cam-ai-thinker-pinout/

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

} // end setup()


void loop() {
  // put your main code here, to run repeatedly:

  delay(30000); // allow 30 seconds time for WiFi connect and time server sync etc. before starting main loop.

  static unsigned long timepoint1 = millis(); // WiFi connect check and NTP time sync timeout
  static unsigned long timepoint2 = timepoint1; // reboot time
  while(true) {
      if(millis()-timepoint1>300000U){  //time interval: 5 minutes
        
        timepoint1 = millis();

        if(WiFi.status() != WL_CONNECTED ){ // ensure WiFi is connected and reconnect if not
          connect_wifi();
        }

        refreshTime(); // refresh time to ensure latest time is submitted with sensor readings
        //printTime();
        //take_send_photo();
        //printTime();
      } // timepoint1

      if(millis()-timepoint2>86400000U){  //time interval: 24 hours        
        timepoint2 = millis();        
        restart_system(); // do a restart every 24 hours 
      } // timepoint2
      
      if( !awsMQTTClient.connected() ) {
          reconnectAWSMQTT(); // ensure AWS MQTT connection stay active
      }

      serialEventProcess(); // Read Serial command and action it.
      awsMQTTClient.loop(); // Loop the AWS MQQT client APi for processing
  }

} // end loop



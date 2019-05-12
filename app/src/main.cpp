#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "mgos.h"
#include "mgos_mqtt.h"
#ifdef MGOS_HAVE_WIFI
#include "mgos_wifi.h"
#endif
#include "mgos_debug.h"
#include "mgos_i2c.h"
#include "mgos_rpc.h"
#include "mgos_shadow.h"
#include "mgos_bme280.h"  //bme280 sensor
#include "mgos_gpio.h"
#include "mgos_arduino_adafruit_tsl25721.h"  //tsl25721 sensor
#include "mgos_sgp30.h"  //sgp30 sensor


//LED GPIOS 5,17,16
#define USER_BUTTON_GPIO 14 //middle button
#define LED_RED_GPIO 5// cannot use 12  //GPIO number for LED
#define LED_GREEN_GPIO 17
#define LED_BLUE_GPIO 16
#define GPIO0 0 
#define BME208_ADDRESS 0x76//0x77
#define ENABLE_SENSORS_GPIO 27  //enables/disables all sensors


//struct mgos_i2c* i2c = NULL;
struct mgos_bme280 *bme; //bme280 structure
TSL25721 *tsl; //tsl25721 class
const char *state_json;  //shadow state data
struct mgos_sgp30_data* sgp; //sgp30 data structure
uint32_t SensorFlags;
#define SENSOR_FLAGS_FOUND_BME280  0x00000001
#define SENSOR_FLAGS_FOUND_SGP30   0x00000002
#define SENSOR_FLAGS_FOUND_TSL25721 0x00000004
//uint32_t EnableAP;
uint32_t LEDOption; //current state of RGB LED

extern const float vapor[];  //vapor pressure for water for each degree Celsius in kPascals


static void mqtt_timer_cb(void *arg) {
  char topic[100];//,json[256];
  float temperature,humidity, pressure;

 
  
  //snprintf(topic, sizeof(topic), "/devices/%s/events",mgos_sys_config_get_device_id());
  snprintf(topic, sizeof(topic), "readings");

 // char buf[8];
//  int x = mgos_gpio_toggle(mgos_sys_config_get_board_led3_pin());
//  mgos_gpio_toggle(LED_RED_GPIO);
  //LOG(LL_INFO, ("Pin: %s, published: %s x %d", mgos_gpio_str(pin, buf),
  //              res ? "yes" : "no", x));


  
//  bool val = mgos_gpio_toggle(mgos_sys_config_get_board_led1_pin());
//  LOG(LL_INFO, ("%s uptime: %.2lf, RAM: %lu, %lu free", val ? "Tick" : "Tock",
//                mgos_uptime(), (unsigned long) mgos_get_heap_size(),
//                (unsigned long) mgos_get_free_heap_size()));
//  (void) arg;

//  LOG(LL_INFO, ("Temperature: %f C",mgos_bme280_read_temperature(bme)));
//  LOG(LL_INFO, ("Humidity: %f %%RH",mgos_bme280_read_humidity(bme)));
//  LOG(LL_INFO, ("Pressure: %f Pa",mgos_bme280_read_pressure(bme)));

//  LOG(LL_INFO,("{Temperature: %f}", bme->getTemperature()));
//  LOG(LL_INFO,("{Hum: %f}", bme->getHumidity()));

  //get sensor data
  
  //BME280 sensor
  if (SensorFlags&SENSOR_FLAGS_FOUND_BME280) {
    temperature=mgos_bme280_read_temperature(bme);
    humidity=mgos_bme280_read_humidity(bme);
    pressure=mgos_bme280_read_pressure(bme);
  } else {
    temperature=0;
    humidity=0;
    pressure=0;
  }
    
  LOG(LL_INFO, ("Temperature: %f C",temperature));
  LOG(LL_INFO, ("Humidity: %f %%RH",humidity));
  LOG(LL_INFO, ("Pressure: %f Pa",pressure));

//  bool res = mgos_mqtt_pubf(topic, 0, false /* retain */,
//                            "{Temperature: %f, Humidity: %f, Pressure: %f}",
//                            temperature,humidity,pressure);

//  bool res = mgos_mqtt_pubf(topic, 0, false /* retain */,
//                            "{total_ram: %lu, free_ram: %lu}",
//                            (unsigned long) mgos_get_heap_size(),
//                            (unsigned long) mgos_get_free_heap_size());


  //TSL25721 Sensor 
  uint16_t x=0;
  uint32_t lum=0,lux=0;
  uint16_t ir=0, full=0;  

  if (SensorFlags&SENSOR_FLAGS_FOUND_TSL25721) {
    x= mgos_tsl25721_getLuminosity(tsl,TSL25721_VISIBLE);     
    //x = tsl.getLuminosity(TSL25721_FULLSPECTRUM);
    //x = tsl.getLuminosity(TSL25721_INFRARED);
    lum = mgos_tsl25721_getFullLuminosity(tsl);
    ir = lum >> 16;
    full = lum & 0xFFFF;
    lux=mgos_tsl25721_calculateLux(tsl,full, ir);
  } else {
    x=0;
    ir=0;
    full=0;
  }  //if (SensorFlags&SENSOR_FLAGS_FOUND_TSL25721) { 
  
  LOG(LL_INFO, ("Lux: %d",lux));

  //SGP30 sensor
  uint32_t tvoc=0,co2=0;
  if (SensorFlags&SENSOR_FLAGS_FOUND_SGP30) {
    //todo: Set baseline (from Sensirion Gas Sensors SGP30 Driver Integration Step-by-Step Guide, p3:)
    //If no stored baseline is available after initializing the baseline algorithm, 
    //the sensor has to run for 12 hours until the baseline
    //can be stored. This will ensure an optimal behavior for subsequent startups. 
    //Reading out the baseline prior should be avoided
    //unless a valid baseline is restored first. Once the baseline is properly initialized or restored, 
    //the current baseline value should be stored approximately once per hour. While the sensor is off, 
    //baseline values are valid for a maximum of seven days.

    //on power up, the CO2 sensor needs to run for 12 hours until the baseline can be stored. 
    //so report "calibrating" until baseline is stored

    if (!mgos_sgp30_read(sgp)) {
      tvoc=mgos_sgp30_data_get_tvoc(sgp);
      co2=mgos_sgp30_data_get_co2(sgp);
    } else {
      LOG(LL_INFO, ("Error: mgos_sgp30_read() failed"));    
    }
  } else { 
    tvoc=0;
    co2=0;
  }//if (SensorFlags&SENSOR_FLAGS_FOUND_TSL25721) {
  LOG(LL_INFO, ("Luminosity: %d",x));
  LOG(LL_INFO, ("IR: %d\t\tFull: %d\t\tVisible: %d",ir,full,full-ir));
  LOG(LL_INFO, ("tvoc=%d ppb\t\tco2=%d ppm",tvoc,co2));  
  //LOG(LL_INFO, ("Full: %d",full));
  //LOG(LL_INFO, ("Visible: %d",full-ir));

  //calculate Vapor Pressure Deficit
  float SVP=0.0,VPD=0.0;
  

  if (SensorFlags&SENSOR_FLAGS_FOUND_BME280) {
    //Determine the Saturated Vapor Pressure (SVP) for the current Temperature (in kPascals)
    //Tetens equation: (from p13 of  Monteith, J.L., and Unsworth, M.H. 2008. Principles of Environmental Physics. Third Ed. AP, Amsterdam. http://store.elsevier.com/Principles-of-Environmental-Physics/John-Monteith/isbn-9780080924793/)
    //see also p.337 in appendix for table for Saturation Vapor Pressure at various Temperatures.
    //more info: https://en.wikipedia.org/wiki/Tetens_equation and http://biomet.ucdavis.edu/conversions/HumCon.pdf
    //0C= 273.2K ()
    //equation where temp is in K: SVP=0.61078*powf(2.718281828,17.27*(temperature-273.2)/(temperature-36));  //kPascals
    SVP=0.61078*powf(2.718281828,17.27*temperature/(237.2+temperature));  //kPascals
    

    //Calculate the Vapor Pressure Deficit (VPD): the saturated vapor pressure minus the actual vapor pressure
    VPD=((100.0-humidity)/100.0)*SVP;

    LOG(LL_INFO, ("SVP=%f kPa\t\tVPD=%f kPa",SVP,VPD));  

  } //if (SensorFlags&SENSOR_FLAGS_FOUND_BME280) {




  //snprintf(json, sizeof (json),"{Temperature: %f, Humidity: %f, Pressure: %f,Luminosity: %d,IR: %d,Full %d,Visible %d,Lux %d,TVOC: %d,CO2: %d}",
  //                          temperature,humidity,pressure,x,ir,full,full-ir,lux,tvoc,co2);

  //bool res = mgos_mqtt_pubf(topic,MG_MQTT_QOS(1),false,json);
  bool res = mgos_mqtt_pubf(topic, 0, false /* retain */,
                            "{DeviceID: \"%s\",Temperature: %f, Humidity: %f, Pressure: %f,Luminosity: %d,IR: %d,Full: %d,Visible: %d,Lux: %d,TVOC: %d,CO2: %d,VPD: %f}",
                           mgos_sys_config_get_device_id(),temperature,humidity,pressure,x,ir,full,full-ir,lux,tvoc,co2,VPD);

} //static void led_timer_cb(void *arg) {

static void net_cb(int ev, void *evd, void *arg) {
  switch (ev) {
    case MGOS_NET_EV_DISCONNECTED:
      LOG(LL_INFO, ("%s", "Net disconnected"));
      break;
    case MGOS_NET_EV_CONNECTING:
      LOG(LL_INFO, ("%s", "Net connecting..."));
      break;
    case MGOS_NET_EV_CONNECTED:
      LOG(LL_INFO, ("%s", "Net connected"));
      break;
    case MGOS_NET_EV_IP_ACQUIRED:
      LOG(LL_INFO, ("%s", "Net got IP address"));
      break;
  }

  (void) evd;
  (void) arg;
}  //static void net_cb(int ev, void *evd, void *arg) {

#ifdef MGOS_HAVE_WIFI
static void wifi_cb(int ev, void *evd, void *arg) {
  switch (ev) {
    case MGOS_WIFI_EV_STA_DISCONNECTED:
      LOG(LL_INFO, ("WiFi STA disconnected %p", arg));
      break;
    case MGOS_WIFI_EV_STA_CONNECTING:
      LOG(LL_INFO, ("WiFi STA connecting %p", arg));
      break;
    case MGOS_WIFI_EV_STA_CONNECTED:
      LOG(LL_INFO, ("WiFi STA connected %p", arg));

      break;
    case MGOS_WIFI_EV_STA_IP_ACQUIRED:
      LOG(LL_INFO, ("WiFi STA IP acquired %p", arg));
      //if got IP then disable AP mode unless user has enabled
      if (!mgos_sys_config_get_apmode()) {  //user did not set apmode on
        if (mgos_sys_config_get_wifi_ap_enable()) { //ap mode is enabled 
          //disable access point mode
          LOG(LL_INFO, ("Disabling AP Mode"));
          struct mgos_config_wifi_ap ap_cfg;
          memcpy(&ap_cfg, mgos_sys_config_get_wifi_ap(), sizeof(ap_cfg));   //read in existing wifi config
          ap_cfg.enable = false;
          mgos_sys_config_set_wifi_ap_enable(false);
          save_cfg(&mgos_sys_config, NULL);
          if (mgos_wifi_setup_ap(&ap_cfg)) {
            LOG(LL_ERROR, ("AP off"));
          } else {
            LOG(LL_ERROR, ("Disable AP failed"));
          }
  	    //mgos_system_restart(0);
        } //if (mgos_sys_config_get_wifi_ap_enable()) { //ap mode is enabled 
       } //if (!mgos_sys_config_get_apmode()) {  //user did not set apmode on

      break;
    case MGOS_WIFI_EV_AP_STA_CONNECTED: {
      struct mgos_wifi_ap_sta_connected_arg *aa =
          (struct mgos_wifi_ap_sta_connected_arg *) evd;
      LOG(LL_INFO, ("WiFi AP STA connected MAC %02x:%02x:%02x:%02x:%02x:%02x",
                    aa->mac[0], aa->mac[1], aa->mac[2], aa->mac[3], aa->mac[4],
                    aa->mac[5]));
      break;
    }
    case MGOS_WIFI_EV_AP_STA_DISCONNECTED: {
      struct mgos_wifi_ap_sta_disconnected_arg *aa =
          (struct mgos_wifi_ap_sta_disconnected_arg *) evd;
      LOG(LL_INFO,
          ("WiFi AP STA disconnected MAC %02x:%02x:%02x:%02x:%02x:%02x",
           aa->mac[0], aa->mac[1], aa->mac[2], aa->mac[3], aa->mac[4],
           aa->mac[5]));

      //re-enable AP mode?     
      break;
    }
  }
  (void) arg;
}
#endif /* MGOS_HAVE_WIFI */


static void button_USER_cb(int pin, void *arg) {

    LOG(LL_INFO, ("button_USER_cb"));

  //Toggle LED

//  int x = mgos_gpio_toggle(mgos_sys_config_get_board_led3_pin());
	//mgos_gpio_toggle(LED_RED_GPIO);
	switch(LEDOption) {
		case 0:
			mgos_gpio_write(LED_RED_GPIO,1);
			mgos_gpio_write(LED_GREEN_GPIO,0);
			mgos_gpio_write(LED_BLUE_GPIO,0);
			LEDOption++;
			break;
		case 1:	
			mgos_gpio_write(LED_RED_GPIO,0);
			mgos_gpio_write(LED_GREEN_GPIO,1);
			mgos_gpio_write(LED_BLUE_GPIO,0);
			LEDOption++;
			break;
		case 2:	
			mgos_gpio_write(LED_RED_GPIO,0);
			mgos_gpio_write(LED_GREEN_GPIO,0);
			mgos_gpio_write(LED_BLUE_GPIO,1);
			LEDOption++;
			break;
		case 3:
			mgos_gpio_write(LED_RED_GPIO,1);
			mgos_gpio_write(LED_GREEN_GPIO,1);
			mgos_gpio_write(LED_BLUE_GPIO,0);
			LEDOption++;
			break;
		case 4:	
			mgos_gpio_write(LED_RED_GPIO,1);
			mgos_gpio_write(LED_GREEN_GPIO,0);
			mgos_gpio_write(LED_BLUE_GPIO,1);
			LEDOption++;
			break;
		case 5:	
			mgos_gpio_write(LED_RED_GPIO,0);
			mgos_gpio_write(LED_GREEN_GPIO,1);
			mgos_gpio_write(LED_BLUE_GPIO,1);
			LEDOption++;
			break;
		case 6:	
			mgos_gpio_write(LED_RED_GPIO,1);
			mgos_gpio_write(LED_GREEN_GPIO,1);
			mgos_gpio_write(LED_BLUE_GPIO,1);
			LEDOption=0;
			break;
	}; //switch 

  if (!mgos_gpio_read(GPIO0)) {
    LOG(LL_INFO, ("toggle apmode"));
//    LOG(LL_INFO, ("setting apmode"));
    //printf("apmode, %d\n", mgos_sys_config_get_apmode());
    if (mgos_sys_config_get_apmode()) {
        mgos_sys_config_set_apmode(0);
    } else {
		mgos_sys_config_set_apmode(1);
    }
    LOG(LL_INFO, ("apmode=%d\n", mgos_sys_config_get_apmode()));

    //save the apmode setting
    save_cfg(&mgos_sys_config, NULL); //writes conf9.json


    struct mgos_config_wifi_ap ap_cfg;
    if (mgos_sys_config_get_apmode()) {
      memcpy(&ap_cfg, mgos_sys_config_get_wifi_ap(), sizeof(ap_cfg));
      ap_cfg.enable = true;
	  sprintf(ap_cfg.ssid,"MendelSensors");
      LOG(LL_INFO, ("AP SSID: %s",ap_cfg.ssid));
      //mgos_sys_config_set_wifi_ap_enable(true);
      save_cfg(&mgos_sys_config, NULL);
      if (mgos_wifi_setup_ap(&ap_cfg)) {
        LOG(LL_INFO, ("AP on"));
      } else {
        LOG(LL_ERROR, ("Enable AP failed"));
      }
    } else { //if (mgos_sys_config_get_apmode()) {
		LOG(LL_INFO, ("AP off"));  //most likely need restart here or something more- does not seem to be working
	} //if (mgos_sys_config_get_apmode()) {
	  
  } //  if (!mgos_gpio_read(GPIO0)) {

	
  (void) arg;
} //static void button_cb(int pin, void *arg) {

#if 0 	
static void button_cb(int pin, void *arg) {
  char topic[100];
  float temperature,humidity, pressure;

    LOG(LL_INFO, ("button_cb"));

  //LOG(LL_INFO, ("Pin: %s, published: %s x %d", mgos_gpio_str(pin, buf),
  //              res ? "yes" : "no", x));


  LOG(LL_INFO, ("test"));
//if user button and GPIO00 are both pressed (voltage=0) then go into AP mode
  if (mgos_gpio_read(GPIO0)) {
     LOG(LL_INFO, ("GPIO0 is high"));
  //#if 0   
  //  snprintf(topic, sizeof(topic), "/devices/%s/events",
  //           mgos_sys_config_get_device_id());

    //get sensor data
    if (SensorFlags&SENSOR_FLAGS_FOUND_BME280) {
      temperature=mgos_bme280_read_temperature(bme);
      humidity=mgos_bme280_read_humidity(bme);
      pressure=mgos_bme280_read_pressure(bme);
    } else {
      temperature=0;
      humidity=0;
      pressure=0;
    }
      
    LOG(LL_INFO, ("Temperature: %f C",temperature));
    LOG(LL_INFO, ("Humidity: %f %%RH",humidity));
    LOG(LL_INFO, ("Pressure: %f Pa",pressure));

  //  bool res = mgos_mqtt_pubf(topic, 0, false /* retain */,
  //                            "{Temperature: %f, Humidity: %f, Pressure: %f}",
  //                            temperature,humidity,pressure);

  //  bool res = mgos_mqtt_pubf(topic, 0, false /* retain */,
  //                            "{total_ram: %lu, free_ram: %lu}",
  //                            (unsigned long) mgos_get_heap_size(),
  //                            (unsigned long) mgos_get_free_heap_size());


    uint16_t x=0;
    uint32_t lum=0,lux=0;
    uint16_t ir=0, full=0;
    
    if (SensorFlags&SENSOR_FLAGS_FOUND_TSL25721) {
	  
      mgos_tsl25721_getStatus(tsl);	    
	    
      x= mgos_tsl25721_getLuminosity(tsl,TSL25721_VISIBLE);     
      //x = tsl.getLuminosity(TSL25721_FULLSPECTRUM);
      //x = tsl.getLuminosity(TSL25721_INFRARED);



      lum = mgos_tsl25721_getFullLuminosity(tsl);

      ir = lum >> 16;
      full = lum & 0xFFFF;

      lux=mgos_tsl25721_calculateLux(tsl,full, ir);

    } else {
      x=0;
      ir=0;
      full=0;
    }  //if (SensorFlags&SENSOR_FLAGS_FOUND_TSL25721) { 
    
    LOG(LL_INFO, ("Lux: %d",lux));

    uint32_t tvoc=0,co2=0;
    if (SensorFlags&SENSOR_FLAGS_FOUND_SGP30) {
      if (!mgos_sgp30_read(sgp)) {
        tvoc=mgos_sgp30_data_get_tvoc(sgp);
        co2=mgos_sgp30_data_get_co2(sgp);
      } else {
        LOG(LL_INFO, ("Error: mgos_sgp30_read() failed"));    
      }
    } else { 
      tvoc=0;
      co2=0;
    }//if (SensorFlags&SENSOR_FLAGS_FOUND_TSL25721) {
    LOG(LL_INFO, ("Luminosity: %d",x));
    LOG(LL_INFO, ("IR: %d\t\tFull: %d\t\tVisible: %d",ir,full,full-ir));
    LOG(LL_INFO, ("tvoc=%d ppb\t\tco2=%d ppm",tvoc,co2));  
    //LOG(LL_INFO, ("Full: %d",full));
    //LOG(LL_INFO, ("Visible: %d",full-ir));

    


    //calculate Vapor Pressure Deficit
    float SVP=0.0,VPD=0.0;

    if (SensorFlags&SENSOR_FLAGS_FOUND_BME280) {
      //Determine the Saturated Vapor Pressure (SVP) for the current Temperature (in kPascals)
      //Tetens equation: (from p13 of  Monteith, J.L., and Unsworth, M.H. 2008. Principles of Environmental Physics. Third Ed. AP, Amsterdam. http://store.elsevier.com/Principles-of-Environmental-Physics/John-Monteith/isbn-9780080924793/)
      //see also p.337 in appendix for table for Saturation Vapor Pressure at various Temperatures.
      //more info: https://en.wikipedia.org/wiki/Tetens_equation and http://biomet.ucdavis.edu/conversions/HumCon.pdf
      //0C= 273.2K ()
      //equation where temp is in K: SVP=0.61078*powf(2.718281828,17.27*(temperature-273.2)/(temperature-36));  //kPascals
      SVP=0.61078*powf(2.718281828,17.27*temperature/(237.2+temperature));  //kPascals
      

      //Calculate the Vapor Pressure Deficit (VPD): the saturated vapor pressure minus the actual vapor pressure
      VPD=((100.0-humidity)/100.0)*SVP;

      LOG(LL_INFO, ("SVP=%f kPa\t\tVPD=%f kPa",SVP,VPD));  

    } //if (SensorFlags&SENSOR_FLAGS_FOUND_BME280) {


    //publish to "readings" topic  
    bool res = mgos_mqtt_pubf("readings", 0, false /* retain */,
                              "{DeviceID: \"%s\",Temperature: %f, Humidity: %f, Pressure: %f,Luminosity: %d,IR: %d,Full: %d,Visible: %d,Lux: %d,TVOC: %d,CO2: %d,VPD: %f}",
                            mgos_sys_config_get_device_id(),temperature,humidity,pressure,x,ir,full,full-ir,lux,tvoc,co2,VPD);


    
  #if 0 
    bool res = mgos_mqtt_pubf(topic, 0, false /* retain */,
                              "{total_ram: %lu, free_ram: %lu, Temperature: %f, Humidity: %f, Pressure: %f}",
                              (unsigned long) mgos_get_heap_size(),
                              (unsigned long) mgos_get_free_heap_size(),
                              temperature,humidity,pressure);
  #endif

  //  bool res = mgos_mqtt_pubf(topic, 0, false /* retain */,
  //                            "{total_ram: %lu, free_ram: %lu}",
  //                            (unsigned long) mgos_get_heap_size(),
  //                            (unsigned long) mgos_get_free_heap_size());


  } else { 

    //if USER_BUTTON (GPIO14 and GPIO0) are both 0, put ESP32 into AP mode enable

//    LOG(LL_INFO, ("toggle apmode"));
    LOG(LL_INFO, ("setting apmode"));
    //printf("apmode, %d\n", mgos_sys_config_get_apmode());
    if (mgos_sys_config_get_apmode()) {
        mgos_sys_config_set_apmode(0);
    } else {
      mgos_sys_config_set_apmode(1);
    }
    LOG(LL_INFO, ("apmode=%d\n", mgos_sys_config_get_apmode()));

    //save the apmode setting
    save_cfg(&mgos_sys_config, NULL); //writes conf9.json


    struct mgos_config_wifi_ap ap_cfg;
    if (mgos_sys_config_get_apmode()) {
      memcpy(&ap_cfg, mgos_sys_config_get_wifi_ap(), sizeof(ap_cfg));
      ap_cfg.enable = true;
      LOG(LL_INFO, ("AP SSID: %s",ap_cfg.ssid));
      //mgos_sys_config_set_wifi_ap_enable(true);
      save_cfg(&mgos_sys_config, NULL);
      if (mgos_wifi_setup_ap(&ap_cfg)) {
        LOG(LL_INFO, ("AP on"));
      } else {
        LOG(LL_ERROR, ("Enable AP failed"));
      }
    } //if (mgos_sys_config_get_apmode()) {
  } //if mos_gpio_read(GPIO0)

  (void) arg;
} //static void button_cb(int pin, void *arg) {
#endif 

static void shadow_cb(int ev, void *evd, void *arg) {

  struct mg_str *event_data;
  float temperature,humidity,pressure;
  char json[512];
  struct json_token t;
  int i,value;

  switch(ev) {
    case MGOS_SHADOW_CONNECTED:
      LOG(LL_INFO, ("Got MGOS_SHADOW_CONNECTED"));

//  LOG(LL_INFO, ("Temperature: %f C",mgos_bme280_read_temperature(bme)));
//  LOG(LL_INFO, ("Humidity: %f %%RH",mgos_bme280_read_humidity(bme)));
//  LOG(LL_INFO, ("Pressure: %f Pa",mgos_bme280_read_pressure(bme)));

      //mgos_shadow_get();
      if (SensorFlags&SENSOR_FLAGS_FOUND_BME280) {
        temperature=mgos_bme280_read_temperature(bme);
        humidity=mgos_bme280_read_humidity(bme);
        pressure=mgos_bme280_read_pressure(bme);
      } else {
        temperature=0;
        humidity=0;
        pressure=0;
      }
      sprintf(json,"{ \"LED\": \"0\",\"Temperature\": \"%f\",\"Humidity\": \"%f\",\"Pressure\": \"%f\"}",temperature,humidity,pressure);
      mgos_shadow_update(0,json);

    break;
    case MGOS_SHADOW_UPDATE_DELTA:
      LOG(LL_INFO, ("Got MGOS_SHADOW_UPDATE_DELTA"));
      event_data = (struct mg_str *)evd;  //get key
      memcpy(json,event_data->p,event_data->len);
      json[event_data->len]=0; //terminate string
      LOG(LL_INFO, ("data: '%s'",json));
      printf("Parsing array: %.*s\n", event_data->len, event_data->p);
      if (json_scanf(event_data->p, event_data->len, "{LED: %d}", &value)) {
          LOG(LL_INFO, ("Got LED delta"));          
          sprintf(json,"{\"LED\": \"%d\"}",value);  
          printf("Sending: '%s'",json);      
          mgos_shadow_update(0,json); //update state to clear delta
          mgos_gpio_write(LED_RED_GPIO,value);                // according to the delta
      }  //if (json_scanf(
/*
      for (i = 0; json_scanf_array_elem(event_data->p, event_data->len, "", i, &t) > 0; i++) {
        printf("Index %d, token [%.*s]\n", i, t.len, t.ptr);
        if (json_scanf(t.ptr, t.len, "{LED: %d}", &value)) {
          LOG(LL_INFO, ("Got LED delta"));          
          sprintf(json,"{LED:\"%d\"}",value);        
          mgos_shadow_update(0,json); //update state to clear delta
        }  //if (json_scanf(
      } //for i
      */

//      sprintf(json,"{\"%d\"}",event_data->p);

    break;
    default:
      //LOG(LL_INFO, ("Got Shadow Message %d",ev));
      LOG(LL_INFO, ("Got Shadow Message %s",mgos_shadow_event_name(ev))); 
    break;
  };  //switch
//  mgos_shadow_update(0, const char *state_json);

} //static void shadow_cb(int ev, void *evd, void *arg) {


enum mgos_app_init_result mgos_app_init(void) {
 
  int res;
  float temperature,humidity,pressure;
  char buf[8];

	LEDOption=0;  //RGB LED status
	
	//set RGB LED pins to output
  	mgos_gpio_set_mode(LED_RED_GPIO, MGOS_GPIO_MODE_OUTPUT);
	mgos_gpio_set_mode(LED_GREEN_GPIO, MGOS_GPIO_MODE_OUTPUT);
	mgos_gpio_set_mode(LED_BLUE_GPIO, MGOS_GPIO_MODE_OUTPUT);

	//turn on green LED
	mgos_gpio_write(LED_RED_GPIO,0);  
	mgos_gpio_write(LED_GREEN_GPIO,1);  
	mgos_gpio_write(LED_BLUE_GPIO,0);  


	//set user button handler
/*
	//	int btn_pin = mgos_sys_config_get_board_btn1_pin();
//	if (btn_pin >= 0) {
	enum mgos_gpio_pull_type btn_pull;
	enum mgos_gpio_int_mode btn_int_edge;
	if (mgos_sys_config_get_board_btn1_pull_up()) {
	  btn_pull = MGOS_GPIO_PULL_UP;
	  btn_int_edge = MGOS_GPIO_INT_EDGE_NEG;
	} else {
	  btn_pull = MGOS_GPIO_PULL_DOWN;
	  btn_int_edge = MGOS_GPIO_INT_EDGE_POS;
	}

	btn_pull = MGOS_GPIO_PULL_UP;
	btn_int_edge = MGOS_GPIO_INT_EDGE_NEG;

//	mgos_gpio_set_button_handler(USER_BUTTON_GPIO, btn_pull, btn_int_edge, 20, button_USER_cb, NULL);
  } //  if (btn_pin >= 0) {
*/
  mgos_gpio_set_button_handler(USER_BUTTON_GPIO, MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_NEG, 20, button_USER_cb, NULL);

/*
  // Blink built-in LED every second 
  int led_pin = mgos_sys_config_get_board_led1_pin();
  if (led_pin >= 0) {
    LOG(LL_INFO, ("LED pin %s", mgos_gpio_str(led_pin, buf)));
    mgos_gpio_set_mode(led_pin, MGOS_GPIO_MODE_OUTPUT);
    mgos_set_timer(1000, MGOS_TIMER_REPEAT, led_timer_cb, NULL);
  }
  
  mgos_gpio_set_mode(mgos_sys_config_get_board_led3_pin(),
                     MGOS_GPIO_MODE_OUTPUT);
  mgos_gpio_write(mgos_sys_config_get_board_led3_pin(), 0);
*/
    //call timer cb every 10minutes
    mgos_set_timer(600000, MGOS_TIMER_REPEAT, mqtt_timer_cb, NULL);
    //mgos_set_timer(60000, MGOS_TIMER_REPEAT, mqtt_timer_cb, NULL);

#if 0 
  /* Publish to MQTT on button press */
  int btn_pin = mgos_sys_config_get_board_btn1_pin();
  if (btn_pin >= 0) {
    enum mgos_gpio_pull_type btn_pull;
    enum mgos_gpio_int_mode btn_int_edge;
    if (mgos_sys_config_get_board_btn1_pull_up()) {
      btn_pull = MGOS_GPIO_PULL_UP;
      btn_int_edge = MGOS_GPIO_INT_EDGE_NEG;
    } else {
      btn_pull = MGOS_GPIO_PULL_DOWN;
      btn_int_edge = MGOS_GPIO_INT_EDGE_POS;
    }
    LOG(LL_INFO, ("Button pin %s, active %s", mgos_gpio_str(btn_pin, buf),
                  (mgos_sys_config_get_board_btn1_pull_up() ? "low" : "high")));
    mgos_gpio_set_button_handler(btn_pin, btn_pull, btn_int_edge, 20, button_cb,
                                 NULL);
  } //  if (btn_pin >= 0) {
#endif 

  /* Network connectivity events */
  mgos_event_add_group_handler(MGOS_EVENT_GRP_NET, net_cb, NULL);

#ifdef MGOS_HAVE_WIFI
  mgos_event_add_group_handler(MGOS_WIFI_EV_BASE, wifi_cb, NULL);
#endif

  mgos_event_add_group_handler(MGOS_SHADOW_BASE, shadow_cb, NULL);
  

  SensorFlags=0; //clear sensor found flags

  //enable all sensors
  LOG(LL_INFO, ("Enabling sensors"));
  mgos_gpio_set_mode(ENABLE_SENSORS_GPIO, MGOS_GPIO_MODE_OUTPUT);
  //mgos_gpio_write(ENABLE_SENSORS_GPIO,1);  
  mgos_gpio_write(ENABLE_SENSORS_GPIO,0);  
  
  LOG(LL_INFO, ("create bme"));
  
  //check bme device id
  bme=mgos_bme280_i2c_create(BME208_ADDRESS);
  temperature=0;
  humidity=0;
  if (bme) {
      LOG(LL_INFO, ("bme created"));
      SensorFlags|=SENSOR_FLAGS_FOUND_BME280;
      temperature=mgos_bme280_read_temperature(bme);
      humidity=mgos_bme280_read_humidity(bme);
      pressure=mgos_bme280_read_pressure(bme);

      LOG(LL_INFO, ("Temperature: %f C",temperature));
      LOG(LL_INFO, ("Humidity: %f %%RH",humidity));
      LOG(LL_INFO, ("Pressure: %f Pa",pressure));

  } else {
      LOG(LL_INFO, ("bme create failed"));
  }


  //initialize GPIO0 pin (used with User pin GPIO14 to enable AP mode)
  mgos_gpio_set_mode(GPIO0,MGOS_GPIO_MODE_INPUT); //set AP pin to input
  mgos_gpio_set_pull(GPIO0,MGOS_GPIO_PULL_UP); //pull up

  tsl=mgos_tsl25721_create(TSL25721_ADDR_FLOAT); 
  if (tsl) {
    if (mgos_tsl25721_begin(tsl)) {
        LOG(LL_INFO, ("tsl begin ok"));
        SensorFlags|=SENSOR_FLAGS_FOUND_TSL25721;
	    
	 mgos_tsl25721_getStatus(tsl);
//      if (mgos_tsl25721_enable(tsl)) {
//        if (tsl->enable()) {
 //       LOG(LL_INFO, ("tsl enable ok"));
          // You can change the gain on the fly, to adapt to brighter/dimmer light situations
          //tsl.setGain(TSL25721_GAIN_0X);         // set no gain (for bright situtations)
          mgos_tsl25721_setGain(tsl,TSL25721_GAIN_16X);      // set 16x gain (for dim situations)
		  //mgos_tsl25721_setGain(tsl,TSL25721_GAIN_120X);      // set 16x gain (for dim situations)

          // Changing the integration time gives you a longer time over which to sense light
          // longer timelines are slower, but are good in very low light situtations!
          //mgos_tsl25721_setIntegrationTime(tsl,TSL25721_INTEGRATIONTIME_27MS);  // shortest integration time (bright light)
		  mgos_tsl25721_setIntegrationTime(tsl,TSL25721_INTEGRATIONTIME_2MS);  // shortest integration time (bright light)
          //tsl.setTiming(TSL25721_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
          //tsl.setTiming(TSL25721_INTEGRATIONTIME_402MS);  // longest integration time (dim light)

          // Simple data read example. Just read the infrared, fullspecrtrum diode 
          // or 'visible' (difference between the two) channels.
          // This can take 13-402 milliseconds! Uncomment whichever of the following you want to read
          uint16_t x = mgos_tsl25721_getLuminosity(tsl,TSL25721_VISIBLE);     
          //uint16_t x = tsl.getLuminosity(TSL25721_FULLSPECTRUM);
          //uint16_t x = tsl.getLuminosity(TSL25721_INFRARED);
          
          LOG(LL_INFO, ("Luminosity: %d",x));

          // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
          // That way you can do whatever math and comparisons you want!
          uint32_t lum = mgos_tsl25721_getFullLuminosity(tsl);
          uint16_t ir, full;
          ir = lum >> 16;
          full = lum & 0xFFFF;

          LOG(LL_INFO, ("IR: %d\t\tFull: %d\t\tVisible: %d",ir,full,full-ir));
          //LOG(LL_INFO, ("Full: %d",full));
          //LOG(LL_INFO, ("Visible: %d",full-ir));
          LOG(LL_INFO, ("Lux: %d",(mgos_tsl25721_calculateLux(tsl,full, ir))));

//      } else {
//        LOG(LL_INFO, ("Error: tsl.begin failed"));
//      } //if (mgos_tsl25721_enable(tsl))
    } else {
      LOG(LL_INFO, ("Error: tsl25721 begin failed"));
    } //tsl.begin     
  } else {
      LOG(LL_INFO, ("Error: mgos_tsl25721_create() failed"));
  } //if (tsl) {


  //SGP30 gas sensor
    if (mgos_sgp30_setup()) {
      LOG(LL_INFO, ("mgos_sgp30_setup() success"));
      SensorFlags|=SENSOR_FLAGS_FOUND_SGP30;

      //on power up, the CO2 sensor needs to run for 12 hours until the baseline can be stored. 
      //so report "calibrating" until baseline is stored

      //Calculate absolute humidity from relative humidity
      double tempnum,abshumidity;
      tempnum = pow(2.718281828, (17.67 * temperature) / (temperature + 243.5));
      abshumidity= (6.112 * tempnum * humidity * 2.1674) / (273.15 + temperature);
      //Set absolute humidity
      if ((mgos_sgp30_set_absolute_humidity((int)abshumidity))==0) {
        LOG(LL_INFO, ("set absolute humidity on sgp30 to %d",(uint32_t)abshumidity));
      } else {
        LOG(LL_INFO, ("warning: failed to set absolute humidity (%d) on sgp30",(uint32_t)abshumidity));
      }
      sgp=mgos_sgp30_data_create();
      if (sgp) {
        if (!mgos_sgp30_read(sgp)) {
          int tvoc=mgos_sgp30_data_get_tvoc(sgp);
          int co2=mgos_sgp30_data_get_co2(sgp);
          LOG(LL_INFO, ("tvoc=%d ppb\t\tco2=%d ppm",tvoc,co2));  
        } else {
          LOG(LL_INFO, ("Error: mgos_sgp30_read() failed"));    
        }
      } else {
        LOG(LL_INFO, ("Error: mgos_sgp30_data_create() failed"));  
      } //if (sgp)

  } else {
      LOG(LL_INFO, ("Error: mgos_sgp30_setup() failed"));
  } //if (mgos_sgp30_setup())

	//EnableAP=0; //used to know if user wants AP mode enabled

  return MGOS_APP_INIT_SUCCESS;
} //enum mgos_app_init_result mgos_app_init(void) {


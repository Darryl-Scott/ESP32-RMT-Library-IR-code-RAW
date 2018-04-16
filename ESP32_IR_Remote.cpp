
 /* Copyright (c) 2018 Darryl Scott. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 * 
 * Based on the Code from Neil Kolban: https://github.com/nkolban/esp32-snippets/blob/master/hardware/infra_red/receiver/rmt_receiver.c
 * Based on the Code from pcbreflux: https://github.com/pcbreflux/espressif/blob/master/esp32/arduino/sketchbook/ESP32_IR_Remote/ir_demo/ir_demo.ino
 * Based on the Code from Xplorer001: https://github.com/ExploreEmbedded/ESP32_RMT
 */
/* This is a simple code to receive and then save IR received by an IR sensor connected to the ESP32 and then resend it back out via an IR LED  
 * Reason that I created this I could not find any complete examples of IR send and Received in the RAW format, I am hoping this code can then be used
 * to build on the existing IR libraries out there.
 */

#include "Arduino.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"



#ifdef __cplusplus
}
#endif

#include "ESP32_IR_Remote.h"


#define roundTo 50 //rounding microseconds timings
#define MARK_EXCESS 220 //tweeked to get the right timing
#define SPACE_EXCESS 190 //tweeked to get the right timing
#define rmt_item32_TIMEOUT_US  5000   /*!< RMT receiver timeout value(us) */

// Clock divisor (base clock is 80MHz)
#define CLK_DIV 80

// Number of clock ticks that represent 10us.  10 us = 1/100th msec.
#define TICK_10_US (80000000 / CLK_DIV / 100000)

static RingbufHandle_t ringBuf;



ESP32_IRrecv::ESP32_IRrecv() {
}


void ESP32_IRrecv::ESP32_IRrecvPIN(int recvpin, int port) {
  if (recvpin>=GPIO_NUM_0 && recvpin<GPIO_NUM_MAX) {
    gpionum = recvpin;
  } else {
    gpionum = (int)GPIO_NUM_22;
  }
  if (port>=RMT_CHANNEL_0 && port<RMT_CHANNEL_MAX) {
    rmtport = port;
  } else {
    rmtport = (int)RMT_CHANNEL_0;
  }
}


void ESP32_IRrecv::ESP32_IRrecvPIN(int recvpin) {
  ESP32_IRrecvPIN(recvpin,(int)RMT_CHANNEL_0);
}

void ESP32_IRrecv::ESP32_IRsendPIN(int sendpin) {
  ESP32_IRrecvPIN(sendpin,(int)RMT_CHANNEL_0);
}

void ESP32_IRrecv::ESP32_IRsendPIN(int sendpin, int port) {
  
  if (sendpin>=GPIO_NUM_0 && sendpin<GPIO_NUM_MAX) {
    gpionum = sendpin;
  } else {
    gpionum = (int)GPIO_NUM_22;
  }
  if (port>=RMT_CHANNEL_0 && port<RMT_CHANNEL_MAX) {
    rmtport = port;
  } else {
    rmtport = (int)RMT_CHANNEL_0;
  }
}

void ESP32_IRrecv::initReceive() {
  rmt_config_t config;
  config.rmt_mode = RMT_MODE_RX;
  config.channel = (rmt_channel_t)rmtport;
  config.gpio_num = (gpio_num_t)gpionum;
  gpio_pullup_en((gpio_num_t)gpionum);
  config.mem_block_num = 7; //how many memory blocks 64 x N (0-7)
  config.rx_config.filter_en = 1;
  config.rx_config.filter_ticks_thresh = 100; // 80000000/100 -> 800000 / 100 = 8000  = 125us
  config.rx_config.idle_threshold = TICK_10_US * 100 * 20;
  config.clk_div = CLK_DIV;
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 1000, 0));
  rmt_get_ringbuf_handle(config.channel, &ringBuf);
  rmt_rx_start(config.channel, 1);
}


void ESP32_IRrecv::initSend(){
  rmt_config_t config;
  config.channel = (rmt_channel_t)rmtport;
  config.gpio_num = (gpio_num_t)gpionum;
  config.mem_block_num = 7;//how many memory blocks 64 x N (0-7)
  config.clk_div = CLK_DIV;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_duty_percent = 50;
  config.tx_config.carrier_freq_hz = 38000;
  config.tx_config.carrier_level = (rmt_carrier_level_t)1;
  config.tx_config.carrier_en = 1;
  config.tx_config.idle_level = (rmt_idle_level_t)0;
  config.tx_config.idle_output_en = true;
  config.rmt_mode = (rmt_mode_t)0;//RMT_MODE_TX;
  rmt_config(&config);
  rmt_driver_install(config.channel, 0, 0);//19     /*!< RMT interrupt number, select from soc.h */
}


void ESP32_IRrecv::sendIR(unsigned int data[], int IRlength){
     rmt_config_t config;
     config.channel = (rmt_channel_t)rmtport;
     //build item
     size_t size = (sizeof(rmt_item32_t) * IRlength);
     rmt_item32_t* item = (rmt_item32_t*) malloc(size); //allocate memory
     memset((void*) item, 0, size); //wipe current data in memory
     int i = 0;
     int x = 0;
     int offset = 0;
     Serial.print("Sending.....:");
     while(x < IRlength) {
            Serial.print(data[x]);Serial.print(",");Serial.print(data[x+1]);Serial.print(",");
            //    To build a series of waveforms. 
            buildItem(item[i],data[x],data[x+1]);
            x=x+2;
            i++;
     }
     Serial.println();
     //To send data according to the waveform items.
     rmt_write_items(config.channel, item, IRlength, true);
     //Wait until sending is done.
     rmt_wait_tx_done(config.channel,1);
     //before we free the data, make sure sending is already done.
     free(item);
}


void ESP32_IRrecv::stopIR(){
  rmt_config_t config;
  config.channel = (rmt_channel_t)rmtport;
  rmt_rx_stop(config.channel);
  Serial.print("Uninstalling..");
  Serial.print("Port : ");Serial.println(config.channel);
  rmt_driver_uninstall(config.channel);
}


void ESP32_IRrecv::getDataIR(rmt_item32_t item, unsigned int* datato, int index) {
  unsigned int lowValue = (item.duration0) * (10 / TICK_10_US)-SPACE_EXCESS;
  lowValue = roundTo*round((float)lowValue/roundTo);
  Serial.print(lowValue);Serial.print(",");
  datato[index] = lowValue;
  unsigned int highValue = (item.duration1) * (10 / TICK_10_US)+MARK_EXCESS;
  highValue = roundTo * round((float)highValue/roundTo);
  Serial.print(highValue);Serial.print(",");
  datato[index+1] = highValue;
}

void ESP32_IRrecv::buildItem(rmt_item32_t &item,int high_us,int low_us){
  item.level0 = 1;
  item.duration0 = (high_us / 10 * TICK_10_US);
  item.level1 = 0;
  item.duration1 = (low_us / 10 * TICK_10_US);
}


void ESP32_IRrecv::decodeRAW(rmt_item32_t *data, int numItems, unsigned int* datato) {
    int x =0;
    for (int i=0; i<numItems; i++) {  
      getDataIR(data[i],datato,x);
      x=x+2;
    }
    Serial.println();
}


int ESP32_IRrecv::readIR(unsigned int* data,int maxBuf) {
  RingbufHandle_t rb = NULL;
  rmt_config_t config;
  config.channel = (rmt_channel_t)rmtport;
  int command = 0;
  rmt_get_ringbuf_handle(config.channel, &rb);
  while(rb) {
            size_t itemSize = 0;
            rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &itemSize, (TickType_t)rmt_item32_TIMEOUT_US);//portMAX_DELAY);
            int numItems = itemSize / sizeof(rmt_item32_t);
            if( numItems == 0 ){return 0;}
            int i;
            Serial.print("Found num of Items :");Serial.println(numItems*2-1);
            memset(data,0,maxBuf);
            Serial.print("Raw IR Code :");
            decodeRAW(item, numItems, data);
            vRingbufferReturnItem(ringBuf, (void*) item);
            return (numItems*2-1);
  }
  vTaskDelete(NULL);
  return 0;
}


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


#ifndef ESP32_IR_REMOTE_H_
#define ESP32_IR_REMOTE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp32-hal.h"
#include "esp_intr.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "freertos/semphr.h"
#include "soc/rmt_struct.h"

#ifdef __cplusplus
}
#endif

class ESP32_IRrecv {
  public:
    ESP32_IRrecv();
    void ESP32_IRrecvPIN (int recvpin);
    void ESP32_IRsendPIN (int sendpin);
    void ESP32_IRrecvPIN (int recvpin, int port);
    void ESP32_IRsendPIN (int sendpin, int port);
    void initReceive();
    void initSend();
    void stopIR();
    int readIR(unsigned int* data, int maxBuf);
    void sendIR(unsigned int data[], int IRlength);

  private:
    int gpionum;
    int rmtport;
    void decodeRAW(rmt_item32_t *data, int numItems, unsigned int* datato);
    void getDataIR(rmt_item32_t item, unsigned int* datato, int index);
    void buildItem(rmt_item32_t &item,int high_us,int low_us);  
};

#endif /* ESP32_IR_REMOTE_H_ */

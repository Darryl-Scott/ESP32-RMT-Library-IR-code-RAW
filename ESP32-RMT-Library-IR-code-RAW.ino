
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

 
#include "ESP32_IR_Remote.h"

const int RECV_PIN = 17; // pin on the ESP32
const int SEND_PIN = 2; // pin on the ESP32


ESP32_IRrecv irrecv;

#define SendIRxTimes 5



unsigned int IRdata[1000]; //holding IR code in ms

void setup() {
  Serial.begin(115200);
  irrecv.ESP32_IRrecvPIN(RECV_PIN,0);//channel 0 so it can use the full memory of the channels
  Serial.println("Initializing...");
  irrecv.initReceive();
  Serial.println("Init complete");
  Serial.println("Send an IR to Copy");
}


void loop() {
  int codeLen=0;
  codeLen=irrecv.readIR(IRdata,sizeof(IRdata));
  if (codeLen > 6) { //ignore any short codes
    Serial.print("RAW code length: ");
    Serial.println(codeLen);
    irrecv.stopIR(); //uninstall the RMT channel so it can be reused for Receiving IR or send IR
    irrecv.ESP32_IRsendPIN(SEND_PIN,0);//channel 0 so it can use the full memory of the channels
    irrecv.initSend(); //setup the ESP32 to send IR code
    delay(1000);
    for(int x = 0;x<SendIRxTimes;x++){ //send IR code that was recorded
      irrecv.sendIR(IRdata,codeLen);
      delay(1000);
    }
    codeLen=0;
    irrecv.stopIR(); //uninstall the RMT channel so it can be reused for Receiving IR or send IR
    irrecv.ESP32_IRrecvPIN(RECV_PIN,0); //channel 0 so it can use the full memory of the channels
    irrecv.initReceive(); //setup the ESP32 to Receive IR code
    
  }
}  




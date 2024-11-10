#ifndef __WIFI__
#define __WIFI__
#define ESP8266

#ifdef ESP8266
#include <ESP8266WiFi.h>
#elif ESP32
#include <WiFi.h>
#endif
#include "ESPNowW.h"

extern uint8_t receiver_mac[] ;
extern struct S_Message_t;
void Send_Messages(S_Message_t *data, u8 *peerMac);
S_Message_t Receive_Message(uint8_t *macAddr, uint8_t *data, int dataLen);
#endif
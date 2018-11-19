/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>

#include "keys.h"

#define BUILTIN_LED 25 // ?? different pin_arduino.h !!

// the OLED used
// pins defined in TTGO variant, pins_arduino.h
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST);

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

typedef struct state {
	uint16_t total_snd = 0; /**< total packets send */
	uint16_t total_rcv = 0; /**< total packets received (excluding ack) */
	bool gps = false; /**< is GPS fixed ? */
	int8_t rssi = 0; /**< last RSSI received value */
	int8_t snr = 0; /**< last snr received value */
	uint8_t ant = 0; /**< number of last seen antennas */

} state_t;

state_t curState;

/** display current cnx state
 * @param state string to be displayed
 */
void displayState (const char* state) {
	u8x8.clearLine (2);
	u8x8.drawString (0,2,state);
	Serial.println(state);
}

/** display number of packets send & received
 */
void displayStats (state_t* st) {
	char l[17];
	uint8_t n = 4;

	/* packets */
	sprintf (l, "s: %5d r:%5d", st->total_snd, st->total_rcv);
	l[16] = 0;
	u8x8.clearLine (n);
	u8x8.drawString (0,n++, l);
	// RSSI & SNR
	sprintf (l, "RSSI%4d SNR%4d", st->rssi, st->snr);
	l[16] = 0;
	u8x8.clearLine (n);
	u8x8.drawString (0,n++, l);
	// gps & antenas
	sprintf (l, "GPS: %s  ANT:%3d", (st->gps)?"OK":"KO", st->ant);
	l[16] = 0;
	u8x8.clearLine (n);
	u8x8.drawString (0,n++, l);

}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            displayState ("EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            displayState ("EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            displayState("EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            displayState("EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            displayState ("EV_JOINING");
            break;
        case EV_JOINED:
            displayState ("EV_JOINED");
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            displayState("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            displayState("EV_REJOIN_FAILED");
            break;
        case EV_TXCOMPLETE:
            displayState("EV_TXCOMPLETE");
				curState.total_snd++;
				  curState.rssi = LMIC.rssi;
				  curState.snr = LMIC.snr;
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
				  curState.total_rcv++;
            }
				displayStats (&curState);
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            displayState("EV_LOST_TSYNC");
            break;
        case EV_RESET:
            displayState("EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            displayState("EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            displayState("EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            displayState("EV_LINK_ALIVE");
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            displayState("EV_TXSTART");
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

	// init oled screen
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.drawString(0, 1, "LoRaWAN TL1 OTAA"); 

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

#ifdef CFG_eu868
	Serial.println("EU");
#endif
	// LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	// // Disable link check validation
	// LMIC_setLinkCheckMode(0);
   //  // TTN uses SF9 for its RX2 window.
   //  LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);


    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
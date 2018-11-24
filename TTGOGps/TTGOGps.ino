/* read gps sentences & display position on screen
 */

#include "minmea.h"
#include <U8x8lib.h>

#define GPS_TX 23 // gps tx to uart rx
#define GPS_RX 22 // gps rx to uart tx

#define LINE_LEN 64 // max length of gps line

HardwareSerial gps(1);

// the OLED used
// pins defined in TTGO variant, pins_arduino.h
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST);


void setup (void) {
	gps.begin (9600, SERIAL_8N1, GPS_TX, GPS_RX);
	Serial.begin (115200);
	Serial.println ("Go...");

	// init oled screen
	u8x8.begin();
	u8x8.setFont(u8x8_font_chroma48medium8_r);
	u8x8.drawString(0, 1, "TTGO GPS"); 

}

/** read line from serial port
 * this is a blocking call until \n is found or buffer is full !
 * @param ser serial port to read from
 * @param line buffer for data (will be \n\0 terminated or \0 terminated if buffer is full)
 * @param len lenght of buffer
 * @return number of read bytes
 */
uint16_t readline (HardwareSerial ser, char* line, uint16_t len) {
	char* l = line;
	line[len-1] = 0;
	*l = 0;

	// wait for data
	while (! ser.available()) {
		delay(10);
	}
	
	while (1) { // until \n
		if (ser.available()) {
			*l = ser.read();
			if (*l == 13) {
				// CR, do not increment
			} else if (*l == 10) {
				// LF, end of line
				*(l+1) = 0;
				return l-line;
			} else {
				l++;
				if (l-line >= len) {
					// end of buffer !
					*l = 0;
					return len;
				}
			}
		}
	}
	return l-line;

}

float latitude = -1.0;
float longitude = -1.0;
int fix_quality = -1;
int satellites_tracked = -1;

void display (void) {
	char l[17];
	sprintf (l, "Fix:%d Sat:%d", fix_quality, satellites_tracked);
	l[16] = 0;
	u8x8.clearLine (3);
	u8x8.drawString (0, 3, l);
	sprintf (l, "lat:%f", latitude);
	l[16] = 0;
	u8x8.clearLine (4);
	u8x8.drawString (0, 4, l);
	sprintf (l, "lon:%f", longitude);
	l[16] = 0;
	u8x8.clearLine (5);
	u8x8.drawString (0, 5, l);

}

void loop (void) {
	static char line[MINMEA_MAX_LENGTH];

	readline (gps, line, MINMEA_MAX_LENGTH);
	switch (minmea_sentence_id(line, false)) {
		case MINMEA_SENTENCE_RMC: {
			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, line)) {
				// latitude valid and changed? apply a threshold
				float new_latitude = minmea_tocoord(&frame.latitude);
				if((new_latitude != NAN) && (abs(new_latitude - latitude) > 0.001)) {
					latitude = new_latitude;
					Serial.printf("New latitude: %f\n", latitude);
					display();
				}
				// longitude valid and changed? apply a threshold
				float new_longitude = minmea_tocoord(&frame.longitude);
				if((new_longitude != NAN) && (abs(new_longitude - longitude) > 0.001)) {
					longitude = minmea_tocoord(&frame.longitude);
					Serial.printf("New longitude: %f\n", longitude);
					display();
				}
			}
		}
		break;
		case MINMEA_SENTENCE_GGA: {
			struct minmea_sentence_gga frame;
			if (minmea_parse_gga(&frame, line)) {
				// fix quality changed?
				if(frame.fix_quality != fix_quality) {
					fix_quality = frame.fix_quality;
					Serial.printf("New fix quality: %d\n", fix_quality);
					display();
				}
			}
		}
		break;
		case MINMEA_SENTENCE_GSV: {
			struct minmea_sentence_gsv frame;
			if (minmea_parse_gsv(&frame, line)) {
				// number of satellites changed?
				if(frame.total_sats != satellites_tracked) {
					satellites_tracked = frame.total_sats;
					Serial.printf("New satellites tracked: %d\n", satellites_tracked);
					display();
				}
			}
		}
		break;
		default:
			// Serial.print ("unknown sentence ");
			// Serial.print(line);
			break;
	}
}

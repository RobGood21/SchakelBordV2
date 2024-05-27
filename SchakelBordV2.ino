/*
 Name:		SchakelBordV2.ino
 Created:	4/25/2024 10:43:35 AM
 Author:	Rob Antonisse

 Schakelbord, een op arduino gebaseerde DCC centrale specifiek voor aansturen van accessories als wissels en seinen met DCC.



*/


//libraries
#include <NmraDcc.h>
#include <EEPROM.h>


//display 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//fastled
#include <FastLED.h>

CRGB pixels[64]; //uitgaan van alle schakelaars krijgen een pixel

Adafruit_SSD1306 dp(128, 64, &Wire, -1); //constructor display  takes program 9598bytes; memory  57bytes

#define TrueBit OCR2A = 115 //pulsduur true bit
#define FalseBit OCR2A = 230 //pulsduur false  bit

//constructors
NmraDcc  Dcc;

//public variables
//byte shiftbyte[2]; //de bytes die in de shiftregisters worden geschoven
byte shiftcount=0; //welke van de 10 switch lijnen wordt getest
byte comlast[10]; //Laatst bekende stand van de schakelaar
unsigned long slowtime;


void setup() {
	//Serial.begin(9600); //takes program 1846bytes; memory 340bytes

	//Display
	dp.begin(SSD1306_SWITCHCAPVCC, 0x3C);

	//DCC
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder per decoderadres

	//Fastled
	FastLED.addLeds<WS2812, 8, RGB>(pixels, 9);

	//Pinnen
	DDRC &= ~(15 << 0);
	PORTC |= (15 << 0); //pullups to A0~A3


	DDRD &= ~(240 << 0);
	PORTD |= (240 << 0); //pull ups 4~7 (D4~D7)

	DDRB |= (62 << 0); //pins 9,10,11,12,13 as outputs


	//init
	Init();

}
void Init() {
	//shiftregisters 1 maken

	PORTB |= (1 << 3); //serial pin hoog

	for (byte i = 0; i < 16; i++) {
		PINB |= (1 << 2); PINB |= (1 << 2); //schuif 16x een 1 in de shiftregisters
	}
	PINB |= (1 << 1); PINB |= (1 << 1); //latch in shiftregisters


	for (byte i = 0; i < 10; i++) {
		comlast[i] = 0xFF;
	}



	DP_write("www.wisselmotor.nl", 10, 28, 1);
}

void loop() {
	Dcc.process();

	//slowevent counter
	if (millis() - slowtime > 10) {
		slowtime = millis();
		Shift();
	}
}


void notifyDccAccTurnoutBoard(uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower) {
}

//switches
void Shift() {
	GPIOR0 ^= (1 << 0);
	if (GPIOR0 & (1 << 0)) {
	SW_exe();
	}
	else {

		if (shiftcount == 0) {
			PORTB &= ~(1 << 3); //Serial pin laag
			PINB |= (1 << 2); PINB |= (1 << 2); //schuif alles in de shiftregisters 1 positie op
			PINB |= (1 << 1); PINB |= (1 << 1); //latch data in de shiftregisters
			PORTB |= (1 << 3); //serial pin weer hoog
		}
		else {
			PINB |= (1 << 2); PINB |= (1 << 2); //schuif alles in de shiftregisters 1 positie op
			PINB |= (1 << 1); PINB |= (1 << 1); //latch data in de shiftregisters
		}
	}
	//shiftcount ++;
	//if (shiftcount > 9) shiftcount = 0;
}


//display
void DP_write(String _txt, byte _x, byte _y, byte _size) {
	//Om het display voor debuggen te gebruiken
	dp.clearDisplay();
	dp.setCursor(_x, _y);
	dp.setTextColor(1); //=white
	dp.setTextSize(_size);
	dp.print(_txt);
	dp.display();
}

void DP_debug(byte _nummer, bool _onoff) {
	//geeft een cijfer op het display
	dp.clearDisplay();
	dp.setCursor(20, 10);
	dp.setTextColor(1); //=white
	dp.setTextSize(2);
	dp.print(_nummer); dp.print("   ");
	if (_onoff) {
		dp.print("Aan");
	}
	else {
	dp.print("Uit");
	}
	dp.display();
}


//switches
void SW_exe() {
	byte _changed = 0;
	byte _com[2];

	_com[0] = PINC;
	_com[1] = PIND;

	_com[0] &= ~(240 << 0); //clear bits 7~4 bit0=com1 1=com2 2=com3 3=com4
	_com[1] &= ~(15 << 0); //clear bits 0~3 bit7=com5 6=com6 5=com7 4=com8

	byte _coms = _com[0] + _com[1];
	//kijken of de stand is veranderd 
	_changed = comlast[shiftcount] ^ _coms;
	if (_changed > 0) {

			DP_debug(shiftcount, true);	


		for (byte i = 0; i < 8; i++) {
			if (_changed & (1 << i)) {
				if (_coms & (1 << i)) {
					SW_off(i);
				}
				else {
					SW_on(i);
				}
			}
		}
	}
	comlast[shiftcount] = _coms; //

	shiftcount++;
	if (shiftcount > 9) shiftcount = 0;

}

void SW_on(byte _sw) {
	_sw += shiftcount * 8;
	DP_debug(_sw, true);
}
void SW_off(byte _sw) {
	_sw += shiftcount * 8;
	DP_debug(_sw, false);
}
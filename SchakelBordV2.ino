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

//defines
//Encoder
#define TrueBit OCR2A = 115 //pulsduur true bit
#define FalseBit OCR2A = 230 //pulsduur false  bit

CRGB pixels[64]; //uitgaan van alle schakelaars krijgen een pixel

Adafruit_SSD1306 dp(128, 64, &Wire, -1); //constructor display  takes program 9598bytes; memory  57bytes

#define TrueBit OCR2A = 115 //pulsduur true bit
#define FalseBit OCR2A = 230 //pulsduur false  bit

//constructors
NmraDcc  Dcc;

//General purpose registers
//GPIOR0 bit0=toggle shiftproces bit1=toggle encoder ISR timer 2


//structs
struct DccBuffer {
	byte reg;
	byte repeat;
	byte adres;
};
DccBuffer buffer[10];





//public variables
//byte shiftbyte[2]; //de bytes die in de shiftregisters worden geschoven
byte shiftcount = 0; //welke van de 10 switch lijnen wordt getest
byte comlast[10]; //Laatst bekende stand van de schakelaar
unsigned long slowtime;
//encoder
byte count_preample;
byte count_byte;
byte count_bit;
byte dcc_fase = 0;
byte dcc_data[6]; //bevat te verzenden DCC bytes, current DCC commando
byte dcc_aantalBytes; //aantal bytes current van het DCC commando

void setup() {
	//Serial.begin(9600); //takes program 1846bytes; memory 340bytes

	//Display
	dp.begin(SSD1306_SWITCHCAPVCC, 0x3C);

	//DCC
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder per decoderadres

	//Fastled
	FastLED.addLeds<WS2812, 8, RGB>(pixels, 9);

	//testen timer 0

	//encoder, interrupts
		//interrupt register settings
	//TCCR2A – Timer/Counter Control Register A, timer 2 used for generate DCC pulses
	TCCR2A = 0x00; //clear register
	//TCCR2A |= (1 << 6);//Toggle OC2A on Compare Match, niet bij schakelbord pin is in gebruik
	TCCR2A |= (1 << 1); //CTC mode clear timer at compare match, WGM21
	TCCR2B = 2; //set register timer 2 prescaler 8
	TIMSK2 |= (1 << 1);
	TrueBit;

	//Pinnen
	DDRC &= ~(15 << 0);
	PORTC |= (15 << 0); //pullups to A0~A3
	DDRD &= ~(240 << 0);
	DDRD &= ~(1 << 3); PORTD |= (1 << 3); //pin kort as input met pullup
	PORTD |= (240 << 0); //pull ups 3~7 (D4~D7)
	DDRB |= (62 << 0); //pins 9,10,11,12,13 as outputs

	PORTB |= (1 << 5); //set pin 13 hoog DCC enabled??

	//initialise
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

unsigned long teller;
byte teken;

ISR(TIMER2_COMPA_vect) {
	//cli(); //V401

	GPIOR0 ^= (1 << 1);
	PINB |= (1 << 4);

	if (~GPIOR0 & (1 << 1)) {
		//bepaal volgende bit
		switch (dcc_fase) {
		case 0: //niks doen alleen 1 bits zenden 	
			TrueBit;
			break;

		case 1: //preample zenden
			count_preample++;
			if (count_preample > 22) {
				count_preample = 0;
				dcc_fase = 2;
				FalseBit;
				count_bit = 7;
				count_byte = 0;
			}
			break;

		case 2: //send dcc_data
			//MSB first; LSB last
			if (count_bit < 8) {
				if (dcc_data[count_byte] & (1 << count_bit)) { //als het [countbit] van het byte[countbyte] waar is dan>> 
					TrueBit;
				}
				else {
					FalseBit;
				}
				count_bit--;
			}
			else { //count_bit 8 or more
				count_bit = 7;
				if (count_byte < dcc_aantalBytes) {
					count_byte++; //next byte
					FalseBit;
				}
				else { //command send reset	
					dcc_fase = 0;
					TrueBit;
				}
			}
			break;
		}
	}
	else { 
		//testen kortsluiting
		if (PINB & (1 << 5)) { //als dcc enabled is hoog
			if (PIND & (1 << 3))PORTB &= ~(1 << 5); //als pin3 hoog is is er geen spanning op de DCC outputs, kan alleen door een sluiting
		}
	}
}

void DCC_exe() {

	//idee, voor de test maak een looplicht op de dcc monitor


	//Command ophalen uit de buffer
	//aan de hand van het command de dcc_date bytes vullen
	//Aantal bytes in dcc_aantalbytes
	//dcc fase naar 1
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

void DP_debug(byte _n1, byte _n2) {
	//geeft een cijfer op het display
	dp.clearDisplay();
	dp.setCursor(20, 10);
	dp.setTextColor(1); //=white
	dp.setTextSize(2);
	dp.print(_n1); dp.print("   ");
	dp.print(_n2);
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
		for (byte i = 0; i < 8; i++) {
			if (_changed & (1 << i)) {
				if (_coms & (1 << i)) {
					SW_div(i, false);
				}
				else {
					SW_div(i, true);
				}
			}
		}
	}
	comlast[shiftcount] = _coms; //
	shiftcount++;
	if (shiftcount > 9) shiftcount = 0;

}

void SW_div(byte _sw, bool _onoff) {
	//zet de schakelaar mutaties om naar 11 on-board switches. En naar schakelmutaties verdeeld over 16 'decoders'
	//elk met 4 channels. Alles met een ingedrukt en losgelaten trigger. scrollen van de 8 on-board switches nog niet gemaak. 29mei2024


	byte _ch = 0;
	byte _dec = 0;
	switch (shiftcount) {
	case 0: //buttons DCC; com; ind
		SW_button(_sw, _onoff);
		break;
	case 1: //buttons S1~S8
		//hier program button of verwijzing naar de decoder/channel waar de buttons aan zijn gekoppeld, bij in bedrijf.
		if (_sw > 3) {
			_ch = 14 - _sw;
		}
		else {
			_ch = _sw + 3;
		}
		SW_button(_ch, _onoff);
		break;
		//*************
	default:
		if (_sw < 4) {
			_dec = shiftcount - 2;
		}
		else {
			_dec = shiftcount + 6;
			_sw = 7 - _sw;
		}
		break;
	}

	if (shiftcount > 1)SW_conn(_dec, _sw, _onoff);
}
void SW_button(byte _button, bool _onoff) {
	//verwerkt de drukknoppen op de module
	switch (_button) {
	case 0:
		if(_onoff)PORTB ^= (1 << 5); //DCC on off
		break;
	}



	DP_debug(_button, _onoff);
}
void SW_conn(byte _dec, byte _chan, bool _onoff) {
	//Schakelaars opgedeeld in 16 groepen van 4 _dec=de groep, _ch 0~3 channel in de groep, _onoff = knop ingedrukt of losgelaten

	//debug
	dp.clearDisplay();
	dp.setCursor(10, 30);
	dp.setTextSize(2);
	dp.print(_dec); dp.print("  "); dp.print(_chan); dp.print("  "); dp.print(_onoff);
	dp.display();
}

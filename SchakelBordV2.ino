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

//#include <FastLED.h>
//CRGB pixels[32]; //uitgaan van alle schakelaars 1 pixel kost 192 bytes, in combi met Serial (usb) loopt de boel vast.


//defines
//Encoder
#define TrueBit OCR2A = 115 //pulsduur true bit 115
#define FalseBit OCR2A = 230 //pulsduur false  bit 230

#define Aantalbuffers 10
#define Aantalrepeats 4

#define Aantalpreamples 10


Adafruit_SSD1306 dp(128, 64, &Wire, -1); //constructor display  takes program 9598bytes; memory  57bytes

//constructors
//NmraDcc  Dcc;

//General purpose registers
	//GPIOR0 bit0=toggle shiftproces bit1=toggle encoder ISR timer 2, 
	// bit 2=halfbit 1; bit 3=halfbit 0
	// 
	// 
	// 
//GPIOR1  bit0=keuze program, bit1=keuze program bit7=factory reset bevesting; bit 6 reset adres false of reset all true
//GPIOR2 te gebruiken als private variable binnen een void



//structs
struct DccBuffer {
	byte delay;
	byte repeat;
	byte data[2]; //alleen standaard accessories 0=adresbyte 1=instruction byte 2=checksum
};
DccBuffer buffer[Aantalbuffers];


struct Dekoder {
	byte reg;
	//bit0-1 timing 0=continue 1=0.25 2=0.5 3=1sec
	//bit2-3 dual/mono mode B00/B01=dual B10=channels 1-2 B11=channels 3-4
	//bit4 switch aan/uit flipflop of momentary 
	//bit5 invert ports
	//bit6 nc
	//bit7 hoog adres 256~511

	byte adres;
	byte stand; //0~3 stand 4~7 false=stand niet bekend true is stand bekend
};
Dekoder dekoder[16];

String txt;
String txtbovenbalk;

//public variables
//byte shiftbyte[2]; //de bytes die in de shiftregisters worden geschoven
byte shiftcount = 0; //welke van de 10 switch lijnen wordt getest
byte comlast[10]; //Laatst bekende stand van de schakelaar
unsigned long slowtime;
//encoder
byte count_preample;
byte count_byte;
byte count_bit;
byte buffercount;
byte dccfase = 0;
byte dccdata[6]; //bevat te verzenden DCC bytes, current DCC commando
byte dccaantalBytes; //aantal bytes current van het DCC commando
byte switchgroup[2]; //welke groups zijn gekoppeld aan S1~S8 EEPROM 10, in theorie is hier 1 byte te winnen
byte dcctimer; //om een 10ms event te maken voor de puls of wisselstraat timers
byte lastbutton = 1; //welke van de S1~S8 is het laatst ingedrukt
byte lastset = 0; //welke switch groep van 4 is het laatste ingedrukt
byte cursor = 0; byte submenu = 0;

//decoder
unsigned long RXtime;
byte RXbitcount = 0;
byte RXbytecount = 0;
byte RXdata[3]; //hier worden de ontvangen adresbyte, instructie byte en checksum tijdelijk  opgeslagen
byte RXpreample = 0;
byte RXfase = 0;

//temps
unsigned int counttrue = 0;
unsigned int countfalse = 0;
bool lastbit = false;
unsigned int countfout = 0;
unsigned int counttemp = 0;

//byte temp;
unsigned long teller;
//byte teken;

void setup() {
	Serial.begin(9600); //takes program 1846bytes; memory 340bytes
	//Display

	dp.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	//DCC
	//Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	//Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder per decoderadres


	//Fastled  waarschijnlijk niet te combieeen
	//FastLED.addLeds<WS2812, 8, RGB>(pixels, 9); 
	//testen timer 0

	//encoder, interrupts
		//interrupt register settings
	//TCCR2A – Timer/Counter Control Register A, timer 2 used for generate DCC pulses
	TCCR2A = 0x00; //clear register
	//TCCR2A |= (1 << 6);//Toggle OC2A on Compare Match, niet bij schakelbord pin is in gebruik
	TCCR2A |= (1 << 1); //CTC mode clear timer at compare match, WGM21
	TCCR2B = 2; //set register timer 2 prescaler 8
	TIMSK2 |= (1 << 1);

	//decoder interrupt settings
	//EICRA |= (3 << 0); //set bit ISC01 en bit ISC00, interrupt 1 (pin2) op rising edge
	//EIMSK |= (1 << 0); //set bit 0 INT0, enable int0

	  // Zet de pin change interrupt in voor pin 2 (PCINT18, PCIE2 groep)
	PCICR |= (1 << PCIE2); // Schakel pin change interrupt in voor groep PCIE2 (PCINT[23:16])
	PCMSK2 |= (1 << PCINT18); // Schakel pin change interrupt in voor pin 2 (PCINT18)

	TrueBit;

	//Pinnen
	DDRC &= ~(15 << 0);
	PORTC |= (15 << 0); //pullups to A0~A3
	DDRD &= ~(240 << 0);
	DDRD &= ~(1 << 3); PORTD |= (1 << 3); //pin kort as input met pullup
	PORTD |= (240 << 0); //pull ups 3~7 (D4~D7)
	DDRB |= (62 << 0); //pins 9,10,11,12,13 as outputs
	PORTB |= (1 << 5); //set pin 13 hoog DCC enabled??

	//Lees EEPROM
	Eepromread();

	//initialise
	Init();
}

void Eepromread() {

	byte _default = EEPROM.read(1);
	delay(100);

	if (_default > 1) {

		//default waardes
		switchgroup[0] = 0; switchgroup[1] = 1;

		for (byte i = 0; i < 16; i++) {
			dekoder[i].adres = i + 1;
			dekoder[i].reg = 0;

		}
		Eepromwrite();

	}
	else {
		//waardes terug lezen
		switchgroup[0] = EEPROM.read(10);
		switchgroup[1] = EEPROM.read(11);

		for (byte i = 0; i < 16; i++) {
			dekoder[i].adres = EEPROM.read(200 + (i * 4) + i);
			dekoder[i].reg = EEPROM.read(201 + (i * 4) + i);
		}
	}
}

void Eepromwrite() {

	EEPROM.update(1, 0);

	for (byte i = 0; i < 2; i++) {
		EEPROM.update(10 + i, switchgroup[i]);
	}

	for (byte i = 0; i < 16; i++) {
		EEPROM.update(200 + (i * 4) + i, dekoder[i].adres);
		EEPROM.update(201 + (i * 4) + i, dekoder[i].reg);
	}
}

void Factory() {
	byte _adres = 0; byte _newadres = 0; bool _hoogadres = false;

	if (GPIOR1 & (1 << 6)) { //factory
		for (int i = 0; i < EEPROM.length(); i++) {
			EEPROM.update(i, 0xFF);
		}
		setup();
	}
	else { //alleen adressen
		_adres = dekoder[0].adres;
		if (dekoder[0].reg & (1 << 7))_hoogadres = true;
		for (byte i = 1; i < 16; i++) {
			_newadres = _adres + i;
			if (_newadres == 0) {
				if (dekoder[i].reg & (1 << 7))break; //hoogst mogelijk adres bereikt
				_hoogadres = true;
			}
			dekoder[i].adres = _newadres;
			if (_hoogadres)dekoder[i].reg |= (1 << 7); else dekoder[i].reg &= ~(1 << 7);

			//Serial.print("hoogadres:"); Serial.print(_hoogadres); Serial.print("   "); Serial.println(_newadres);
		}

		Eepromwrite();
		cursor = 0;
		GPIOR1 = 0;
		DP_bedrijf();
	}
}
void Init() {
	//shiftregisters 1 maken
	GPIOR0 = 0; GPIOR1 = 0; GPIOR2 = 0; //reset general purpose registers
	cursor = 0;
	lastbutton = 1;

	PORTB |= (1 << 3); //serial pin hoog
	for (byte i = 0; i < 16; i++) {
		PINB |= (1 << 2); PINB |= (1 << 2); //schuif 16x een 1 in de shiftregisters
	}
	PINB |= (1 << 1); PINB |= (1 << 1); //latch in shiftregisters
	for (byte i = 0; i < 10; i++) {
		comlast[i] = 0xFF;
	}

	dp.clearDisplay();
	DP_bedrijf();
}
void loop() {
	//slowevent counter
	if (millis() - slowtime > 1) {  //clock van 2ms 
		slowtime = millis();
		Shift();
		if (dccfase == 0) DCC_exe();
		DCC_timer();

		//counttemp++;
		//if (counttemp > 1000) { //2 seconden			
		//	counttemp = 0;
		//	Serial.print(" true:  "); Serial.print(counttrue);
		//	counttrue = 0;
		//	Serial.print("    false:  ");
		//	Serial.print(countfalse);
		//	countfalse = 0;
		//	Serial.print("  fout:");
		//	Serial.println(countfout);
		//	countfout = 0;
		//}
	}
}

ISR(PCINT2_vect) {
	unsigned int  _time;
	//half bits lezen
	_time = micros() - RXtime;
	RXtime = micros();
	if (_time > 48 && _time < 64) { //48<>64 beste keuze
		if (GPIOR0 & (1 << 2)) {
			RX(true);
			GPIOR0 &= ~(1 << 2); //reset halfbit flag 1
		}
		GPIOR0 |= (1 << 2); //set halfbit flag 1
		GPIOR0 &= ~(1 << 3); //reset halfbit flag 0 (een 1 halfbit is net ontvangen.)
	}
	else if (_time > 100 && _time < 200) {
		if (GPIOR0 & (1 << 3)) {
			RX(false);
			GPIOR0 &= ~(1 << 3); //reset halfbit flag 0
		}
		GPIOR0 |= (1 << 3); //set halfbit flag 0
		GPIOR0 &= ~(1 << 2); //reset halfbit flag 1 (een 0 halfbit is net ontvangen.)
	}
	else {
		//ontvangst is fout, geen true en geen false bit, mag niet voorkomen een pauze in de pinchange die niet in de twee periodes past, total reset dus nodig
		RX_reset();
	}
}

void RX(bool _bit) {
	switch (RXfase) {
	case 0: //preample aftellen
		if (_bit) {
			RXpreample++;
			if (RXpreample > RXpreample)RXfase = 1;
		}
		else {
			RXpreample = 0; //opnieuw gaan tellen
		}
		break;

	case 1: //preample ontvangen wacht op een false (start) bit
		if (_bit) { //idle bit ontvangen

		}
		else { //Start bit ontvangen 
			RXfase = 2;
		}
		break;

	case 2: //startbit ontvangen

		break;
	}
}

void RX_reset() {
	//faillure of na ontvangst command alles resetten
	//bits data ontvangst reset
	GPIOR0 &= ~(1 << 3); //reset half bit flag 0
	GPIOR0 &= ~(1 << 2); //reset half bit flag 1
	//reset data RX proces
	RXdata[0] = 0;
	RXdata[1] = 0;
	RXdata[2] = 0;
	RXpreample = 0;
	RXfase = 0;
}

ISR(TIMER2_COMPA_vect) {
	//cli(); //V401
	GPIOR0 ^= (1 << 1);
	PINB |= (1 << 4);

	if (~GPIOR0 & (1 << 1)) {
		//bepaal volgende bit
		switch (dccfase) {
		case 0: //niks doen alleen 1 bits zenden 	
			TrueBit;
			break;

		case 1: //preample zenden
			count_preample++;
			if (count_preample > 22) { //22
				//Serial.println(count_preample);
				count_preample = 0;
				dccfase = 2;
				FalseBit;
				count_bit = 7;
				count_byte = 0;
			}
			break;

		case 2: //send dcc_data
			//MSB first; LSB last
			if (count_bit < 8) {
				if (dccdata[count_byte] & (1 << count_bit)) { //als het [countbit] van het byte[countbyte] waar is dan>> 
					TrueBit;
				}
				else {
					FalseBit;
				}
				count_bit--;
			}
			else { //count_bit 8 or more
				count_bit = 7;
				if (count_byte < dccaantalBytes) {
					count_byte++; //next byte
					FalseBit;
				}
				else { //command send reset	
					dccfase = 0;
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
void DCC_command(byte _dec, byte _chan, bool _onoff) { //_onoff knop ingedrukt of losgelaten
	//maakt een command in de (command)buffers
	//TIJDELIJK FF de offs eruit


	byte _adres = dekoder[_dec].adres; //(adres 0~255)	

	//bit4 true=switch aan/uit flipflop of false=momentary 
	if (dekoder[_dec].reg & (1 << 4)) {
		//momentary
		if (_onoff) {
			//knop ingedrukt
			dekoder[_dec].stand |= (1 << _chan + 4);
			dekoder[_dec].stand |= (1 << _chan);
		}
		else {
			//knop losgelaten
			dekoder[_dec].stand &= ~(1 << _chan);
		}
	}
	else {
		//aan/uit
		if (!_onoff)return; //Met een _onoff=false niks verder doen

		//stand omschakelen bij dual recht/afslaand bij single onoff
		dekoder[_dec].stand |= (1 << _chan + 4); //zet deze flag voor 'stand is bekend'  na powerup
		dekoder[_dec].stand ^= (1 << _chan); //Toggle stand

	}


	//hier de feitelijk command maken in de command buffers
	//vrije buffer zoeken
	byte _buffer = DCC_findbuffer();
	buffer[_buffer].data[0] = 128; // adresbyte B10 00 0000
	buffer[_buffer].data[1] = 240; //B11110000 
	//adres 
	if (dekoder[_dec].reg & (1 << 7))buffer[_buffer].data[1] &= ~(1 << 6); //adresses 512~1024
	if (_adres > 127) { _adres -= 128; buffer[_buffer].data[1] &= ~(1 << 5); }
	if (_adres > 63) { _adres -= 64; buffer[_buffer].data[1] &= ~(1 << 4); }
	if (_adres > 31) { _adres -= 32; buffer[_buffer].data[0] |= (1 << 5); }
	if (_adres > 15) { _adres -= 16; buffer[_buffer].data[0] |= (1 << 4); }
	if (_adres > 7) { _adres -= 8; buffer[_buffer].data[0] |= (1 << 3); }
	if (_adres > 3) { _adres -= 4; buffer[_buffer].data[0] |= (1 << 2); }
	if (_adres > 1) { _adres -= 2; buffer[_buffer].data[0] |= (1 << 1); }
	if (_adres > 0) { buffer[_buffer].data[0] |= (1 << 0); }

	////single(1-2 3-4) of dual(1-4); 	// .reg  bit2-3 dual/mono mode B00/B01=dual B10=channels 1-2 B11=channels 3-4
	if (dekoder[_dec].reg & (1 << 2)) {
		if (dekoder[_dec].stand & (1 << _chan))buffer[_buffer].data[1] |= (1 << 3);
		if (dekoder[_dec].reg & (1 << 3)) {
			//single mode channels 3&4 (let op _chan is nu de knop)
			buffer[_buffer].data[1] |= (1 << 2);
		}
		switch (_chan) { //_chan is hier de knop
		case 0:
			//channel =0   port=false				
			break;
		case 1:
			//channel = 0 port=true
			buffer[_buffer].data[1] |= (1 << 0);
			break;
		case 2:
			//channel =1 port=false
			buffer[_buffer].data[1] |= (1 << 1);  //set bit 1 channel 1
			break;
		case 3:
			//channel=1 port=true 
			buffer[_buffer].data[1] |= (1 << 1);  //set bit 1 channel 1
			buffer[_buffer].data[1] |= (1 << 0);
			break;
		}

	}
	else { //Dual mode

		buffer[_buffer].data[1] += _chan << 1;

		//port, stand bit 5=invert   false is afslaand
		//onderstaand berekend de stand die moet worden verzonden, het werkt, weet nu al niet meer waarom...
		bool _xor = false;
		bool _s = dekoder[_dec].stand & (1 << _chan);
		bool _v = ~dekoder[_dec].reg & (1 << 5);
		_xor = _s ^ _v;
		//_xor= (dekoder[_dec].stand & (1 << _chan)) ^ (dekoder[_dec].reg & (1 << 5)); //dit kan de compiler niet aan...
		if (_xor == false) buffer[_buffer].data[1] |= (1 << 0);
		buffer[_buffer].data[1] |= (1 << 3);  //set onoff bit
		//*******************************************
	}

	//overige 
	buffer[_buffer].repeat = Aantalrepeats;
	buffer[_buffer].delay = 0;


	GPIOR2 = 0; //reset private temp variable
	//continue of puls, bij puls een tweede buffer aanmaken; 	.reg bit0 - 1 timing 0 = continue 1 = 0.25 2 = 0.5 3 = 1sec
	if (dekoder[_dec].reg & (1 << 0)) GPIOR2 |= (1 << 0);
	if (dekoder[_dec].reg & (1 << 1))GPIOR2 |= (1 << 1);

	if (GPIOR2 > 0) {
		//puls de extra buffer aanmaken
		byte _bufferoff = DCC_findbuffer(); //vrije buffer zoeken

		buffer[_bufferoff].data[0] = buffer[_buffer].data[0];
		buffer[_bufferoff].data[1] = buffer[_buffer].data[1];
		buffer[_bufferoff].data[1] &= ~(1 << 3); //clear onoff bit
		buffer[_bufferoff].repeat = Aantalrepeats;

		switch (GPIOR2) {
		case 1: //0.25s
			buffer[_bufferoff].delay = 25;
			break;
		case 2: //0.5s
			buffer[_bufferoff].delay = 50;
			break;
		case 3: //1s
			buffer[_bufferoff].delay = 100;
			break;
		}
	}
}
byte DCC_findbuffer() {
	byte _buffer;
	//buffers eerst zoeken op 0 repeats , daarna op 1 repeats om aantal buffers te verdubbelen voordat er geen buffer wordt gevonden. 
	for (byte repeats = 0; repeats < 3; repeats++) {
		for (byte i = 0; i < Aantalbuffers; i++) {
			if (buffer[i].repeat == 0) {
				return i;
			}
		}
	}
	//Geen vrije buffer gevonden hoogste buffer overschrijven
	return (Aantalbuffers - 1);
}
void DCC_timer() {
	dcctimer++;
	if (dcctimer > 4) {  //clock van 10ms

		dcctimer = 0;
		for (byte i = 0; i < Aantalbuffers; i++) {
			if (buffer[i].delay > 0) {
				buffer[i].delay--;
			}
		}
	}
}
void DCC_exe() {
	//zoekt een command om te verzenden, buffers allemaal om de beurt testen, niet telkens de eerste of de laatste
	byte _count = buffercount;
	do {
		if (buffer[_count].repeat > 0 && buffer[_count].delay == 0) { //command buffer gevonden
			buffer[_count].repeat--;
			dccdata[0] = buffer[_count].data[0];
			dccdata[1] = buffer[_count].data[1];
			dccdata[2] = buffer[_count].data[0] ^ buffer[_count].data[1]; //checksum			
			dccaantalBytes = 2; //waarom 2?
			dccfase = 1;
			buffercount = _count;
			return; //beindig het proces
		}
		_count++;
		if (_count > Aantalbuffers - 1)_count = 0;
	} while (_count != buffercount);  //loop eindigt ook als geen te verzenden command is gevonden
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
//display temp/debug
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
	byte _group;
	byte _channel;
	switch (_button) {
	case 0:
		if (_onoff)PORTB ^= (1 << 5); //DCC on off
		break;

	case 1: //program algemene dingen
		if (_onoff) {
			GPIOR1 &= ~(1 << 1);
			GPIOR1 ^= (1 << 0);

			if (GPIOR1 & (1 << 0)) {
				DP_common();
			}
			else {
				dp.clearDisplay();
				DP_bedrijf();
			}
			Eepromwrite();
			cursor = 0;
		}
		break;

	case 2: //program inc individueel
		if (_onoff) {
			GPIOR1 &= ~(1 << 0);
			GPIOR1 ^= (1 << 1);

			if (GPIOR1 & (1 << 1)) {
				DP_single();
			}
			else {
				dp.clearDisplay();
				DP_bedrijf();
			}

			Eepromwrite();
			cursor = 0;
		}
		break;

	default: //3~12		
		if (GPIOR1 & (1 << 0)) {
			//common prorgram mode
			SW_common(_button, _onoff);
		}

		else if (GPIOR1 & (1 << 1)) {
			if (_onoff == false)return; //alleen indrukken verwerken
			_group = 0;
			if (lastbutton > 4)_group = 1;
			//single programmode
			switch (_button) {
			case 3: //S1 
				//dec cursor
				if (cursor > 0)cursor--;
				break;
			case 4: //S2
				//inc cursor
				if (cursor < 5)cursor++;
				break;
			case 5: //S3
				//dec value
				switch (cursor) {
				case 0: //kiezen decoder op de knoppen S1~S8
					if (switchgroup[_group] > 0) switchgroup[_group]--;
					break;
				case 1: //adres instellen
					if (dekoder[switchgroup[_group]].adres > 1) {
						dekoder[switchgroup[_group]].adres--;
					}
					else { //dus 1 of 0
						if (dekoder[switchgroup[_group]].reg & (1 << 7)) {
							if (dekoder[switchgroup[_group]].adres == 1) {
								dekoder[switchgroup[_group]].adres = 0;
							}
							else {
								dekoder[switchgroup[_group]].reg &= ~(1 << 7);
								dekoder[switchgroup[_group]].adres = 255;
							}
						}
					}
					break;
				case 2: //dec timing instellen
					if (dekoder[switchgroup[_group]].reg & (1 << 0)) { //bit0 =true
						dekoder[switchgroup[_group]].reg &= ~(1 << 0); //clear bit
					}
					else if (dekoder[switchgroup[_group]].reg & (1 << 1)) { //bit1=true
						dekoder[switchgroup[_group]].reg &= ~(1 << 1); //clear bit1
						dekoder[switchgroup[_group]].reg |= (1 << 0); //set bit 0
					}
					break;
				case 3: //dual of single mode 
					if (dekoder[switchgroup[_group]].reg & (1 << 3)) { //bit 3 , dan 3-4 terug naar 1-2
						dekoder[switchgroup[_group]].reg &= ~(1 << 3); //reset bit 3
					}
					else { //was hier dus al 1-2 terug naar dual mode
						dekoder[switchgroup[_group]].reg &= ~(1 << 2); //reset bit 2
					}
					break;
				case 4: //schakel functie of moment bit 4
					dekoder[switchgroup[_group]].reg &= ~(1 << 4);
					break;
				case 5: //invert de port afslaand en rechtdoor bit 5
					dekoder[switchgroup[_group]].reg &= ~(1 << 5);
					break;
				}
				break;

			case 6: //S4
				//inc value
				switch (cursor) {
				case 0://dec groep keuze					
					if (switchgroup[_group] < 15) switchgroup[_group]++;
					break;

				case 1: //adres
					//Serial.println(dekoder[switchgroup[_group]].adres);

					if (dekoder[switchgroup[_group]].adres == 255) {
						if (dekoder[switchgroup[_group]].reg & (1 << 7))return; //max aantal adressen bereikt
						dekoder[switchgroup[_group]].adres = 0;
						dekoder[switchgroup[_group]].reg |= (1 << 7);
					}
					else {
						dekoder[switchgroup[_group]].adres++;
					}

					//if(dekoder[switchgroup[_group]].adres <254)  dekoder[switchgroup[_group]].adres++;					

					break;
				case 2: //inc timing
					if (~dekoder[switchgroup[_group]].reg & (1 << 0)) { //bit0 =false
						dekoder[switchgroup[_group]].reg |= (1 << 0); //set bit 0
					}
					else if (~dekoder[switchgroup[_group]].reg & (1 << 1)) { //bit1=false
						dekoder[switchgroup[_group]].reg |= (1 << 1); //set bit1
						dekoder[switchgroup[_group]].reg &= ~(1 << 0); //clear bit
					}
					break;
				case 3: //single or dual mode
					if (dekoder[switchgroup[_group]].reg & (1 << 2)) { //mono mode al ingeschakeld
						dekoder[switchgroup[_group]].reg |= (1 << 3); //set bit 3 naar mono mode 3-4
					}
					else { //staat hier in dual mode 1-4
						dekoder[switchgroup[_group]].reg |= (1 << 2); //set bit 2 naar single mode 1-2						
					}
					break;
				case 4: //onoff of moment schakelaars bit4 van reg
					dekoder[switchgroup[_group]].reg |= (1 << 4);
					break;
				case 5: //invert ports bit  5
					dekoder[switchgroup[_group]].reg |= (1 << 5);
					break;
				}
				break;
			case 9: //knop S7
				switch (cursor) {
				case 1:
					if (dekoder[switchgroup[_group]].adres - 10 > 1)
					{
						dekoder[switchgroup[_group]].adres -= 10;
					}
					else {
						if (dekoder[switchgroup[_group]].reg & (1 << 7)) {
							dekoder[switchgroup[_group]].reg &= ~(1 << 7);
							dekoder[switchgroup[_group]].adres = 255;
						}
						else {
							dekoder[switchgroup[_group]].adres = 1;
						}
					}
					break;
				}
				break;
			case 10: //knop S8
				switch (cursor) {
				case 1: //adres +10	
					if (dekoder[switchgroup[_group]].adres + 10 > 255) {
						if (dekoder[switchgroup[_group]].reg & (1 << 7)) {
							dekoder[switchgroup[_group]].adres = 255;
						}
						else {
							dekoder[switchgroup[_group]].reg |= (1 << 7);
							dekoder[switchgroup[_group]].adres = 0;
						}
					}
					else {
						dekoder[switchgroup[_group]].adres += 10;
					}

					break;
				}
				break;
			}
			DP_single();
		}
		else {
			//*************************bedrijfsmode
			SW_bedrijf(_button, _onoff);
		}
		break;
	}
}

void SW_common(byte _button, bool _onoff) {
	//common programmode
	if (_onoff == false)return; //alleen on knop acties

	switch (_button) {
	case 3: //S1
		if (cursor > 0) {
			cursor--;
			GPIOR1 &= ~(192 << 0);
		}
		break;
	case 4: //S2
		if (cursor < 5) { cursor++; 	GPIOR1 &= ~(192 << 0); }
		break;

	case 5: //S3
		switch (cursor) {
		case 0: //Wisselstraat
			break;
		case 2:
			break;
		case 3: //toggle tussen dcc rest of all reset
			GPIOR1 ^= (1 << 6);
			break;
		}
		break;

	case 6: //S4
		switch (cursor) {
		case 0: //wisselstraat
			break;
		case 2:
			break;
		case 3: //toggle tussen dcc rest of all reset
			GPIOR1 ^= (1 << 6);
			break;
		}
		break;
	case 9:
		if (GPIOR1 & (1 << 7)) {
			Factory();
		}
		break;
	case 10:
		GPIOR1 ^= (1 << 7);
		break;
	}
	DP_common();
}

void SW_bedrijf(byte _button, bool _onoff) {

	byte _channel;
	byte _group;
	_button -= 2;
	if (_onoff) lastbutton = _button; //memory last button, alleen in bedrijfsmode en alleen het indrukken
	_channel = _button;
	if (_channel > 4) {
		_channel -= 4;
		_group = switchgroup[1];
	}
	else {
		_group = switchgroup[0];
	}

	DCC_command(_group, _channel - 1, _onoff); //channel = hier 1~4


	if (_onoff)TXT_bovenbalk(_button, _group, _channel);			//alleen bij indrukken knop tonen in de bovenbalk, niet het loslaten
	DP_bedrijf();
}
void TXT_bovenbalk(byte _button, byte _group, byte _channel) {
	txtbovenbalk = "";
	txtbovenbalk += "<";
	if (_button == 0)txtbovenbalk += "c"; else txtbovenbalk += _button;
	txtbovenbalk += "> ";
	dp.setCursor(10, 1);
	txtbovenbalk += "S[";
	txtbovenbalk += _group + 1; //tonen als 1~16
	txtbovenbalk += "-";
	txtbovenbalk += _channel;
	txtbovenbalk += "]";
}
void SW_conn(byte _dec, byte _chan, bool _onoff) {
	//Schakelaars opgedeeld in 16 groepen van 4 _dec=de groep, _ch 0~3 channel in de groep, _onoff = knop ingedrukt of losgelaten
	lastset = _dec;


	TXT_bovenbalk(0, _dec, _chan + 1);
	//DCCcommando hier maken
	DCC_command(_dec, _chan, _onoff);
	DP_bedrijf();
}
//displays
void DP_bedrijf() {
	byte _set = 0; byte _stand; byte _aantalregels = 3;
	dp.clearDisplay();
	DP_bovenbalk();


	if (lastset == switchgroup[0] || lastset == switchgroup[1])_aantalregels = 2;


	for (byte x = 0; x < 4; x++) {
		for (byte y = 0; y < _aantalregels; y++) {

			switch (y) {
			case 0: //Bovenste rij switchgroup 0
				_set = switchgroup[0];
				break;
			case 1: //switchgroup 1
				_set = switchgroup[1];
				break;
			case 2: //last set
				_set = lastset;
				break;
			}

			if (dekoder[_set].stand & (1 << (x + 4))) {
				if (dekoder[_set].stand & (1 << x)) {
					_stand = 2; //afslaand of aan
				}
				else {
					_stand = 1; //recht of uit
				}
			}
			else {
				_stand = 0;
			}
			switch (_stand) {
			case 0: //niet bekend
				dp.drawRect(42 + x * 20, 17 + y * 16, 10, 10, 1);
				break;
			case 1: //recht of uit
				dp.drawCircle(46 + x * 20, 21 + y * 16, 6, 1);
				break;
			case 2: //afslaand of aan
				dp.fillCircle(46 + x * 20, 21 + y * 16, 6, 1);
				break;

			}
		}
	}
	//labels
	txt = ">";
	dp.setCursor(2, 14);
	dp.setTextColor(1);
	dp.setTextSize(2);
	txt += switchgroup[0] + 1;
	//txt += "]";
	dp.print(txt);
	txt = ">"; txt += switchgroup[1] + 1; //txt += "]";
	dp.setCursor(2, 30);
	dp.print(txt);
	if (_aantalregels > 2) {
		txt = ">";
		txt += lastset + 1;
		dp.setCursor(2, 46);
		dp.print(txt);
	}

	dp.display();
}
void DP_bovenbalk() {
	dp.fillRect(0, 0, 128, 10, 1);
	dp.setCursor(1, 1);
	dp.setTextColor(0);
	dp.setTextSize(1);
	dp.print(txtbovenbalk);
	//dp.display();  wordt gedaan in aanroepende functie DP_bedrijf
}
void DP_single() {
	//tekent de single instellingen
		//tekent scherm in bedrijf
	byte _group = 0;
	unsigned int _adres;
	byte _mode;
	byte _color = 1;
	if (lastbutton > 4)_group++;
	dp.clearDisplay();
	dp.setCursor(1, 1);
	dp.setTextSize(2);
	dp.setTextColor(1);

	byte y1; byte y2;
	if (_group == 0) {
		y1 = 5; y2 = 12;
	}
	else {
		y1 = 12; y2 = 5;
	}

	for (byte i = 0; i < 4; i++) {
		dp.fillCircle(5 + (i * 8), y1, 2, 1);
		dp.drawCircle(5 + (i * 8), y2, 2, 1);
	}
	dp.setCursor(42, 1);
	dp.print(F("Set"));

	if (cursor == 0) {
		dp.fillRect(90, 0, 30, 16, 1);
		dp.setTextColor(0);
	}
	dp.setCursor(100, 1);
	dp.print(switchgroup[_group] + 1);
	dp.setTextColor(1);

	if (cursor < 4) {

		//**********************adres
		dp.setCursor(1, 19);
		dp.print(F("Adres"));

		if (cursor == 1) {
			dp.fillRect(65, 18, 60, 16, 1);
			dp.setTextColor(0);
		}
		dp.setCursor(70, 19);

		_adres = dekoder[switchgroup[_group]].adres;
		if (dekoder[switchgroup[_group]].reg & (1 << 7))_adres += 256;
		_adres = 1 + (4 * (_adres - 1));  //dcc adres van eerste channel in het decoderadres

		dp.print(_adres);
		dp.setTextColor(1);

		//*************timing puls
		if (cursor == 2) {
			dp.fillRect(1, 37, 63, 20, 1);
			_color = 0;
		}
		else {
			//dp.drawRect(55, 37, 63, 20, 1);
		}
		_mode = dekoder[switchgroup[_group]].reg;
		_mode &= ~(B11111100 << 0); //clear bits 7~3
		dp.setCursor(4, 39);
		dp.setTextColor(_color);
		//dp.setTextSize(1);
		switch (_mode) {
		case 0: //continu
			dp.fillRect(15, 45, 19, 3, _color);
			break;
		case 1: //puls 250ms
			dp.print(0.25);
			break;
		case 2: //puls 500ms
			dp.print(0.5);
			break;
		case 3: //puls 1sec
			dp.print(1);
			break;
		}
		if (_mode > 0)dp.print("s");

		//******************************* 4 channels (omzetten) 2 channels (aan en uit) 1~4 of 1~2 of 3~4
		//bit 2 van reg false=1-2 true=3-4 bit 3 van reg false is 1-4 (omzetten) true=aan/uit van channels
		dp.setTextColor(1);
		if (cursor == 3) {
			dp.fillRect(65, 37, 62, 20, 1);
			dp.setTextColor(0);
		}
		txt = "[";
		if (dekoder[switchgroup[_group]].reg & (1 << 2)) { //single mode
			if (dekoder[switchgroup[_group]].reg & (1 << 3)) {
				//channel 3&4
				txt += "3-4]";
			}
			else {
				txt += "1-2]";
			}
		}
		else {
			//dual mode 
			txt += "1-4]";
		}

		dp.setCursor(65, 39);
		dp.print(txt);

		//**************************
	}
	else {  //cursor >3 

		//bit 4 knop latch of ingedrukt aan, niet ingedrukt uit, moment schakeling. 
		_color = 1;
		dp.fillRect(0, 18, 128, 49, 0);

		if (cursor == 4) {
			dp.fillRect(1, 20, 40, 20, 1);
			_color = 0;
		}
		dp.fillCircle(12, 30, 5, _color);
		if (~dekoder[switchgroup[_group]].reg & (1 << 4)) { //default false aan/uit
			dp.fillCircle(28, 30, 5, _color);
		}

		//bit 5 invert ports rechtdoor<>afslaand

		if (cursor == 5) {
			dp.fillRect(60, 20, 40, 20, 1);
			dp.setTextColor(0);
		}
		if (dekoder[switchgroup[_group]].reg & 1 << 5) {
			dp.setCursor(62, 22);
			dp.print(F("inv"));
		}

	}
	dp.display(); //ververs het display
}
void DP_common() {
	byte  _color = 0;
	//tekend de common instellingen
	dp.clearDisplay();
	switch (cursor) {
	case 0: //Wisselstraat
		dp.setCursor(5, 25);
		dp.setTextColor(1);
		dp.setTextSize(1);
		dp.print("Wisselstraat");
		break;

	case 1:
		break;
	case 3: //******************Factory reset
		dp.fillRect(0, 0, 128, 14, 1);
		dp.setCursor(10, 2);
		dp.setTextColor(0);
		dp.setTextSize(1);
		dp.println("Reset");

		if (GPIOR1 & (1 << 7)) {
			dp.fillRect(5, 20, 81, 40, 0);
			if (GPIOR1 & (1 << 6))	txt = "Alles?"; else  txt = "Adressen?";
			dp.setTextSize(2);
			dp.setTextColor(1);
			dp.setCursor(7, 22);
			dp.print(txt);
		}
		else {
			if (~GPIOR1 & (1 << 6)) { //alleen dcc adresses
				dp.fillRect(5, 20, 80, 20, 1);
			}
			else { //factory reset
				dp.fillRect(5, 40, 80, 20, 1);
			}
			dp.setTextSize(2);

			if (GPIOR1 & (1 << 6)) dp.setTextColor(1); else dp.setTextColor(0);
			dp.setCursor(7, 22);
			dp.print("Adres");
			if (GPIOR1 & (1 << 6)) dp.setTextColor(0); else dp.setTextColor(1);
			dp.setCursor(7, 42);
			dp.print("Alles");


		}

		break;
	}
	dp.display();
}





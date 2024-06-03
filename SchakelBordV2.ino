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
// #include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//fastled

//#include <FastLED.h>
//CRGB pixels[32]; //uitgaan van alle schakelaars 1 pixel kost 192 bytes, in combi met Serial (usb) loopt de boel vast.


//defines
//Encoder
#define TrueBit OCR2A = 115 //pulsduur true bit
#define FalseBit OCR2A = 230 //pulsduur false  bit

Adafruit_SSD1306 dp(128, 64, &Wire, -1); //constructor display  takes program 9598bytes; memory  57bytes


#define TrueBit OCR2A = 115 //pulsduur true bit
#define FalseBit OCR2A = 230 //pulsduur false  bit

//constructors
NmraDcc  Dcc;

//General purpose registers
	//GPIOR0 bit0=toggle shiftproces bit1=toggle encoder ISR timer 2, 
//GPIOR1  bit0=keuze program, bit1=keuze program bit7=factory reset bevesting



//structs
struct DccBuffer {
	byte reg;
	byte repeat;
	byte adres;
};
DccBuffer buffer[10];


struct Dekoder {
	byte reg;
	//bit6 = groot adres 256~512
	//bit7=factory reset, reserved bit, als 1 dan automatisch adressen naar default waarde instellen.
	byte adres;
};
Dekoder dekoder[16];


String txt;

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
byte switchgroup[2]; //welke groups zijn gekoppeld aan S1~S8 EEPROM 10, in theorie is hier 1 byte te winnen
byte lastbutton = 1; //welke van de S1~S8 is het laatst ingedrukt
byte cursor = 0; byte submenu = 0;

//byte temp;
//unsigned long teller;
//byte teken;

void setup() {

	Serial.begin(9600); //takes program 1846bytes; memory 340bytes
	//Display
	dp.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	//DCC
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); //bit7 true maakt accessoire decoder, bit6 false geeft decoder per decoderadres


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
	for (byte i = 0; i < 2; i++) {
		switchgroup[i] = EEPROM.read(10 + i);
		if (switchgroup[i] > 20)switchgroup[i] = i; //default dus 0 en 1, tonen als 1 en 2 
	}
	for (byte i = 0; i < 16; i++) {
		//struct decoder 4 bytes 
		dekoder[i].adres = EEPROM.read(200 + (i * 4) + i);
		dekoder[i].reg = EEPROM.read(201 + (i * 4) + i);
		if (EEPROM.read(0)==0xFF) {
			//default waarde instellen
			dekoder[i].adres = i;
			EEPROM.update(200 + (i * 4) + i, i);
			dekoder[i].reg = 0;
			EEPROM.update(201 + (i * 4) + i, dekoder[i].reg);
			EEPROM.update(0, 0);
		}
	}
}
void Eepromwrite() {
	for (byte i = 0; i < 16; i++) {
		EEPROM.update(200 + (i * 4) + i, dekoder[i].adres);
		EEPROM.update(201 + (i * 4) + i, dekoder[i].reg);
	}
}
void Factory() {
	//reset EEPROM

	DP_write("factory",5,20,2);

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

	DP_bedrijf();
}
void loop() {
	Dcc.process();

	//slowevent counter
	if (millis() - slowtime > 10) {
		slowtime = millis();
		Shift();
	}
}

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
void DP_bovenbalk() {
	dp.fillRect(0, 0, 128, 10, 1);
	dp.setCursor(1, 1);
	dp.setTextColor(0);
	dp.setTextSize(1);
	dp.print(txt);

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
				DP_bedrijf();
			}
			Eepromwrite();
			cursor = 0;
		}
		break;

	default: //3~12		
		if (GPIOR1 & (1 << 0)) {
			//common programmode
			if (_onoff == false)return; //alleen on knop acties
			switch (_button) {
			case 3: //S1
				if (cursor > 1)cursor--;
				break;
			case 4: //S2
				if (cursor < 5)cursor++;
				break;

			case 5: //S3
				switch (cursor) {
				case 0: //factory reset
					GPIOR1 &= ~(1 << 7); 
					break;
				}
				break;

			case 6: //S4
				switch (cursor) {
				case 0: //factory
					if (GPIOR1 & (1 << 7)) {
						Factory();
					}
					else {
					 GPIOR1 |= (1 << 7);
					}
					break;
				}
				break;

			}
			DP_common();
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
					dekoder[switchgroup[_group]].adres--;
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
					dekoder[switchgroup[_group]].adres++;
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
			}
			DP_single();
		}
		else {

			//*************************bedrijfsmode
			_button -= 2;
			lastbutton = _button; //memory last button, alleen in bedrijfsmode

			_channel = _button;
			//ophalen welke group

			if (_channel > 4) {
				_channel -= 4;
				_group = switchgroup[1];
			}
			else {
				_group = switchgroup[0];
			}

			if (_onoff) { 			//alleen bij indrukken knop tonen in de bovenbalk, niet het loslaten
				txt = "";
				txt += "<"; txt += _button;
				txt += "> ";
				dp.setCursor(10, 1);
				txt += "S[";
				txt += _group + 1; //tonen als 1~16
				txt += "-";
				txt += _channel;
				txt += "]";
				//txt = F("nu komt switch en group");
				DP_bovenbalk();
			}
		}
		break;
	}
}
void SW_conn(byte _dec, byte _chan, bool _onoff) {
	//Schakelaars opgedeeld in 16 groepen van 4 _dec=de groep, _ch 0~3 channel in de groep, _onoff = knop ingedrukt of losgelaten

	//schakel opties: 
	//debug
	dp.clearDisplay();
	dp.setCursor(10, 30);
	dp.setTextSize(2);
	dp.print(_dec); dp.print("  "); dp.print(_chan); dp.print("  "); dp.print(_onoff);
	dp.display();
}
void DP_bedrijf() {
	//tekent scherm in bedrijf
	dp.clearDisplay();
	dp.setCursor(10, 30);
	dp.setTextSize(2);
	dp.setTextColor(1);
	dp.print("Bedrijf");
	dp.display();
}
void DP_single() {
	//tekent de single instellingen
		//tekent scherm in bedrijf
	byte _group = 0;
	byte _adres;
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
		//Hier moet nog meer komen bij scrollen en hoge adressen 255 en meer
		_adres = (dekoder[switchgroup[_group]].adres * 4) + 1;  //dcc adres van eerste channel in het decoderadres
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
		if (dekoder[switchgroup[_group]].reg & 1<<5) {
			dp.setCursor(62, 22);
			dp.print(F("inv"));
		}

	}
	dp.display(); //ververs het display
}
void DP_common() {
	//tekend de common instellingen
	dp.clearDisplay();
	dp.fillRect(0, 0, 128, 10, 1);

	//******************Factory reset
	if (cursor == 0) //(dit opschuiven later)
		dp.setCursor(5, 25);
	dp.setTextColor(1);
	dp.setTextSize(2);

	if (GPIOR1 & (1 << 7)) {
		dp.print(F("Sure?"));
	}
	else {
	 dp.print(F("Reset? "));
	}
	dp.display();
}





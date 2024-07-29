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
//CRGB pixels[64]; //uitgaan van alle schakelaars 1 pixel kost 192 bytes, in combi met Serial (usb) loopt de boel vast.


//defines
//Encoder
#define TrueBit OCR2A = 115 //pulsduur true bit 115
#define FalseBit OCR2A = 230 //pulsduur false  bit 230

#define AantalDatabytes 6
#define Aantalbuffers 30
#define Aantalrepeats 4
#define Aantalpreamples 12
#define Aantalwisselstraatsets 2
#define AantalWisselstraatacties 8

#define txt_alles "alles"
#define txt_adres "adres"



Adafruit_SSD1306 dp(128, 64, &Wire, -1); //constructor display  takes program 9598bytes; memory  57bytes

//constructors
//NmraDcc  Dcc;

//General purpose registers
	//GPIOR0 bit0=toggle shiftproces bit1=toggle encoder ISR timer 2, 
	// bit 2=halfbit 1; bit 3=halfbit 0
	// 
	// 
	// 
//GPIOR1  
// bit0=keuze program, 
// bit1=keuze program 
// bit4=een schakelaar, request update smartleds
// bit 6 reset adres false of reset all true
// bit7=factory reset bevesting; 
// 
//GPIOR2 te gebruiken als private variable binnen een void



//structs
struct DccBuffer {
	int16_t delay;
	byte repeat;
	byte data[2]; //alleen standaard 3-bytes packets  0=adresbyte 1=instruction byte (2=checksum)
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


//wisselstraten
byte wss; //welke wisselstraatset van 4 wisselstraten ?
byte actiecount;
byte WS[Aantalwisselstraatsets]; //bit0,1 knop 1~4    bit2,3,4,5 knopset 0~15  bit7=actief 
//[gekozen wisselstraatset][4]=aantal knoppen in de knop set [8]=aantalacties in een wisselstraat
byte WSactie[Aantalwisselstraatsets][4][AantalWisselstraatacties]; //bit0=stand(port) bit1,2 channel bit 3,4,5,6 knopset 0~15 bit7=aan/uit, actief
byte WSdelayfactor;
//byte WSdelay; //periode tussen twee acties in een wisselstraat, voorlopig even instellen in init()

String txt;
String txtbovenbalk;

//public variables
byte Reg; //EEprom 5
//bit0= smartleds true RGB false=GRB 

byte kleur[3]; //smartled 0=r 1=g 2=b
byte lh; //led helderheid
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
byte lastbutton = 1; //welke van de S1~S8 is het laatst ingedrukt,
//lastbutton wordt ook gebruikt in SW_common en DC_common nog niet zeker of dit kan.

byte lastset = 0; //welke switch groep van 4 is het laatste ingedrukt
byte cursor = 0; byte para = 0;
//CV programmer
byte CVreg;
// bit0=loco(false) of accessory(true; default) 
//bit1 true animatie aan, false animatie uit
int16_t CVadres = 1;
int16_t CVnum = 0;  //10bits 
byte CVval = 0; //8bits
byte CVtimer = 0; //timer voor een animatie


//decoder
unsigned long RXtime;
byte RXbitcount = 0;
byte RXbytecount = 0;
byte RXdata[3]; //hier worden de ontvangen adresbyte, instructie byte en checksum tijdelijk  opgeslagen
byte RXpreample = 0;
byte RXfase = 0;
byte Pixcount = 0; //verversen als nodig van de smartleds 



//temps
//unsigned int counttrue = 0;
//unsigned int countfalse = 0;
//bool lastbit = false;
//unsigned int countfout = 0;
//unsigned int counttemp = 0;

//byte temp;
unsigned long teller;
byte temp;
void setup() {
	//Serial.begin(9600); //takes program 1846bytes; memory 340bytes Alleen tijdens debugging dus.	 
	//Display
	dp.begin(SSD1306_SWITCHCAPVCC, 0x3C);


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
	DDRB |= (63 << 0); //pins 8,9,10,11,12,13 as outputs

	PORTB |= (1 << 5); //set pin 13 hoog DCC enabled??

	//Lees EEPROM
	Eepromread();

	//initialise
	Init();
}
void Eepromread() {
	byte _pos;
	Reg = EEPROM.read(5);
	lh = EEPROM.read(6); if (lh > 10)lh = 3;
	WSdelayfactor = EEPROM.read(7);
	if(WSdelayfactor>20)WSdelayfactor =3 ; //factor in stappen van 100ms dus default=300ms

	CVreg = 0xFF; //ook even een vaste waarde, misschien blijkt later opslag nodig te zijn.

	//wisselstraten 
	for (byte i = 0; i < Aantalwisselstraatsets; i++) {
		WS[i] = EEPROM.read(250 + i);
		if (WS[i] == 0xFF) WS[i] = 0;

		for (byte c = 0; c < 4; c++) { //c=channel, knop van de gekozen knopset die de wisselstraat bedient.
			for (byte a = 0; a < AantalWisselstraatacties; a++) {

				WSactie[i][c][a] = EEPROM.read(300 + a + (c * AantalWisselstraatacties) + (i * 4 * AantalWisselstraatacties));

				if (WSactie[i][c][a] == 0xFF) WSactie[i][c][a] = 0;
			}
		}
	}

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
		//alle (decoder)waardes terug lezen
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
	EEPROM.update(5, Reg); //default =0xFF
	EEPROM.update(6, lh); //led helderheid
	EEPROM.update(7, WSdelayfactor); //pauze in de wisselstraten

	for (byte i = 0; i < 2; i++) {
		EEPROM.update(10 + i, switchgroup[i]);
	}

	for (byte i = 0; i < 16; i++) {
		EEPROM.update(200 + (i * 4) + i, dekoder[i].adres);
		EEPROM.update(201 + (i * 4) + i, dekoder[i].reg);
	}

	//wisselstraten
	for (byte i = 0; i < Aantalwisselstraatsets; i++) {
		EEPROM.update(250 + i, WS[i]);

		for (byte c = 0; c < 4; c++) { //c=channel, knop van de gekozen knopset die de wisselstraat bedient.
			for (byte a = 0; a < AantalWisselstraatacties; a++) {
				EEPROM.update(300 + (a + (c * AantalWisselstraatacties) + (i * 4 * AantalWisselstraatacties)), WSactie[i][c][a]);
				if (WSactie[i][c][a] > 0) {
				}
			}
		}
	}
}

void Factory() {
	byte _adres = 0; byte _newadres = 0; bool _hoogadres = false;

	if (GPIOR1 & (1 << 6)) { //factory

		dp.clearDisplay();
		dp.setCursor(20, 25);
		dp.setTextColor(1);
		dp.setTextSize(2);
		dp.print("Factory");
		dp.display();

		for (int i = 0; i < EEPROM.length(); i++) {
			EEPROM.update(i, 0xFF);
		}

		delay(1000);

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
	PixShow();
}
void loop() {
	//slowevent counter
	if (millis() - slowtime > 1) {  //clock van 2ms 
		slowtime = millis();
		Shift();

		if (dccfase == 0) DCC_exe();
		DCC_timer();
		CV_timer();
		//update smartleds
		Pixcount++;
		if (Pixcount == 0) {
			if (GPIOR1 & (1 << 4)) {
				PixShow();
				GPIOR1 &= ~(1 << 4);
			}
		}

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

		if (GPIOR0 & (1 << 2)) { //is halfbit 1 flag gezet
			RX(true); //true bit ontvangen
			GPIOR0 &= ~(1 << 2); //reset halfbit flag 1
		}
		else {
			GPIOR0 |= (1 << 2); //set halfbit flag 1
			GPIOR0 &= ~(1 << 3); //reset halfbit flag 0 (een 1 halfbit is net ontvangen.)
		}
	}

	else if (_time > 100 && _time < 200) {
		if (GPIOR0 & (1 << 3)) {
			RX(false);
			GPIOR0 &= ~(1 << 3); //reset halfbit flag 0
		}
		else {
			GPIOR0 |= (1 << 3); //set halfbit flag 0
			GPIOR0 &= ~(1 << 2); //reset halfbit flag 1 (een 0 halfbit is net ontvangen.)
		}

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

			if (RXpreample > Aantalpreamples) {
				RXfase = 1;
				RXpreample = 0;
			}
		}
		else {
			RXpreample = 0;
		}
		break;

	case 1: //preample ontvangen wacht op een false (start) bit
		if (!_bit) {  //Start bit ontvangen 
			RXfase = 2;
			RXbitcount = 7;
		}
		break;

	case 2: //startbit ontvangen, databyte ontvangen
		if (_bit)  RXdata[0] |= (1 << RXbitcount); //databyte 

		if (RXbitcount > 0) {
			RXbitcount--;
		}
		else { //dus RXbitcount ==0
			RXfase = 3;
		}
		break;

	case 3:
		if (_bit) { //kan niet hier moet een nul bit komen
			RX_start();
		}
		else {
			RXfase = 4;
			RXbitcount = 7;
		}
		break;

	case 4: //ontvangen instructie byte
		if (_bit)  RXdata[1] |= (1 << RXbitcount);  //instructie byte

		if (RXbitcount > 0) {
			RXbitcount--;
		}
		else { //dus RXbitcount == 0
			RXfase = 5;
		}
		break;


	case 5:
		if (_bit) {
			//kan niet hier moet een false bit komen
			RX_start();
		}
		else {
			RXfase = 6;
			RXbitcount = 7;
		}
		break;


	case 6:
		if (_bit)  RXdata[2] |= (1 << RXbitcount);  //checksum byte

		if (RXbitcount > 0) {
			RXbitcount--;
		}
		else { //dus RXbitcount == 0
			RXfase = 7;
		}
		break;


	case 7:
		//3x databyte ontvangen nu een true bit als afsluiting, (voorlopig) alleen 3 bytes packets verwerken.
		if (_bit) RX_command(); //verwerk command, packet.
		RX_start();  //einde command
		break;
	}
}
void RX_reset() {
	//faillure of na ontvangst command alles resetten
	//bits data ontvangst reset
	GPIOR0 &= ~(1 << 3); //reset half bit flag 0
	GPIOR0 &= ~(1 << 2); //reset half bit flag 1
	//reset data RX proces

	RX_start();
}
void RX_start() {

	RXdata[0] = 0;
	RXdata[1] = 0;
	RXdata[2] = 0;

	RXpreample = 0;
	RXfase = 0;
}
void RX_command() {
	//verwerkt een ontvangen packet 
	//filters

	if (RXdata[0] < 255) { //idle packet filter

		byte _buffer = DCC_findbuffer(); //zoek een vrije buffer
		buffer[_buffer].data[0] = RXdata[0];
		buffer[_buffer].data[1] = RXdata[1];
		buffer[_buffer].repeat = 1;
		buffer[_buffer].delay = 0;

		//for (byte i = 0; i < 3; i++) {
		//	Serial.print(RXdata[i], BIN); Serial.print("    ");
		//}
		//Serial.println("");
	}
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

//void WS_exe() {
//	byte _temp; byte _dec; byte _chan; bool _stand;
//	//byte WSactie[Aantalwisselstraatsets][4][8]; //bit0=stand(port) bit1,2 channel bit 3,4,5,6 knopset 0~15 
//	//byte WS[Aantalwisselstraatsets]; //bit0,1 knop 1~4    bit2,3,4 knopset 0~15  bit7=actief 
//	for (byte actie = 0; actie < 8; actie++) {
//		//bepaal welke knopset (_dec)
//		//bepaal welke knop in de set
//		//set stand deze knop, zoals bepaald in de actie
//		//stuur DCC command, adres(en puls en invert) uit bepaalde knopset en knop.
//	}
//}

void DCC_command(byte _dec, byte _chan, bool _onoff) { //_onoff knop ingedrukt of losgelaten
	//maakt een command in de (command)buffers
	//TIJDELIJK FF de offs eruit
	bool _straat = false;
	byte _set = 0; byte _wss = 0;
	//hier komt dus de opdracht binen van alle in bedrijf schakelingen.
	//hier bepalen of het een individueel schakeling of een wisselstraat schakeling bepaald en daarnaar handelen.
	//ook de signalering zit hier achter
	//de bovenbalk ' welke knop is ingedrukt' aanduiding werkt nog wel. 
	//Wisselstraat of single bediening?

	//check of de knopset gebruikt wordt als wisselstraat set;  //bit0,1 knop 1~4    bit2,3,4 knopset 0~15  bit7=actief 
	for (byte ws = 0; ws < Aantalwisselstraatsets; ws++) {

		if (WS[ws] & (1 << 7)) {
			_set = WS[ws];
			_set = _set << 2; _set = _set >> 4; //isolate bit2,3,4,5, knopset

			if (_dec == _set) {
				_straat = true;
				_wss = ws;
				break; //knop is van een straat, for loop verlaten
			}
		}
	}

	if (_straat) {
		if (_onoff)DCC_Straat(_dec, _chan, _wss); //alleen als knop ingedrukt is. Bij false geen actie verder
	}
	else {
		WS_reset(_dec, _chan); //reset de indicatie van de straat als een actie apart  wordt geschakeld
		DCC_Single(_dec, _chan, _onoff);
	}
}

void Command_adres(byte _buffer, byte _dec) {
	//bepaald het adres
	byte _adres = dekoder[_dec].adres;

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

}
void WS_reset(byte _dec, byte _chan) {
	//onderstaand ingewikkeld verhaal, eenvoudig om alle onderdelen met elkaar te verwarren, beste laat alle text staan.
	//_dec en _chan zijn hier de ingedrukte knop.
	//getest wordt of deze knop met zijn onderliggende verwijzing naar een accessory deel uitmaakt
	//van een wisselstraatset. Dan moet de signalering van de wisselstraatset naar uit, omdat een van de 'wissels'
	//omgezet is is immers de wisselstraat stand van de wissels niet meer goed. 
	//23juli2024 volgens mij werkt het goed zo. 
	//Een andere optie zou kunnen zijn om bij verversen van de smartleds te testen of de stand overeenkomt met de stand als
	//aangegeven in de wisselstraat als een accessory daarin is opgenomen. Dit lijkt met nog complexer. Verder niet naar gekeken.

	byte _actie; byte _actieset; byte _actiechan; byte _set;
	//Als deze single bediening is voor een knop/channel die ook opgenomen is in een actie van wisselstraat 
	//dan moet de stand van die wisselstraat naar 'uit' worden gezet.

	for (byte wss = 0; wss < Aantalwisselstraatsets; wss++) { //teset alle wisselstraatsets
		if (WS[wss] & (1 << 7)) { //wisselstraatset is aan, actief   
			for (byte c = 0; c < 4; c++) { //c=channel, test de 4 knoppen in de knopset toegewezen aan de wisselstraatset
				//uiteindelijke doel is de smartled uit te zetten indien nodig
				//als de smartled al uit is is het hele volgende circus daarom niet nodig
				_set = WS[wss]; //bepaal de knopset en channel waarvan de stand naar uit moet, //bit0,1 knop 1~4    bit2,3,4,5 knopset 0~15  bit7=actief 
				_set = _set << 2, _set = _set >> 4; //isoleer decoder , knopset

				if (dekoder[_set].stand & (1 << c)) { //de set is dus actief

					for (byte a = 0; a < AantalWisselstraatacties; a++) { //a=test de acties onder de knop c in de set aangewezen in de wisselstraat
						_actie = WSactie[wss][c][a]; //bit0=stand(port) bit1,2 channel bit 3,4,5,6 knopset 0~15 bit7=aan/uit
						if (_actie & (1 << 7)) { //actie is aan, actief
							_actieset = _actie << 1; _actieset = _actieset >> 4; //isoleer de knopset van deze actie
							_actiechan = _actie << 5; _actiechan = _actiechan >> 6; //isoleer bit 1,2 channel,knop uit de knopset, in de actie

							if (_dec == _actieset && _chan == _actiechan) { //test of de ingedrukt knop(set) overeenkomt met de knopset in de actie.

								//	_set = WS[wss]; //bepaal de knopset en channel waarvan de stand naar uit moet, //bit0,1 knop 1~4    bit2,3,4,5 knopset 0~15  bit7=actief 
								//	_set = _set << 2, _set = _set >> 4; //isoleer decoder , knopset
								dekoder[_set].stand &= ~(1 << c); //reset de stand van de knop ingesteld op deze wisselstraat						}
							}
						}
					}
				}		 //for acties		
			}  //for channel even versimpeld om uit te vogelen waarom alleen c=0 werkt...(opgelost c moest worden gebruikt in: dekoder[_set].stand &= ~(1 << c); niet _actiechan
		}
	}
}

void DCC_Single(byte _dec, byte _chan, bool _onoff) {
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

	//vrije buffer zoeken
	byte _buffer = DCC_findbuffer();

	//buffer[_buffer].data[0] = 128; // adresbyte B10 00 0000
	//buffer[_buffer].data[1] = 240; //B11110000 
	////adres 
	//if (dekoder[_dec].reg & (1 << 7))buffer[_buffer].data[1] &= ~(1 << 6); //adresses 512~1024
	//if (_adres > 127) { _adres -= 128; buffer[_buffer].data[1] &= ~(1 << 5); }
	//if (_adres > 63) { _adres -= 64; buffer[_buffer].data[1] &= ~(1 << 4); }
	//if (_adres > 31) { _adres -= 32; buffer[_buffer].data[0] |= (1 << 5); }
	//if (_adres > 15) { _adres -= 16; buffer[_buffer].data[0] |= (1 << 4); }
	//if (_adres > 7) { _adres -= 8; buffer[_buffer].data[0] |= (1 << 3); }
	//if (_adres > 3) { _adres -= 4; buffer[_buffer].data[0] |= (1 << 2); }
	//if (_adres > 1) { _adres -= 2; buffer[_buffer].data[0] |= (1 << 1); }
	//if (_adres > 0) { buffer[_buffer].data[0] |= (1 << 0); }

	Command_adres(_buffer, _dec);

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
	DCC_puls(_dec, _buffer, 0);
}

void DCC_puls(byte _dec, byte _buffer, int16_t _extradelay) {
	//maakt een off command, merk op geldt weer voor de hele set switches
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
		buffer[_bufferoff].delay = _extradelay;
		switch (GPIOR2) {
		case 1: //0.25s
			buffer[_bufferoff].delay += 25;
			break;
		case 2: //0.5s
			buffer[_bufferoff].delay += 50;
			break;
		case 3: //1s
			buffer[_bufferoff].delay += 100;
			break;
		}
	}
}
void DCC_Straat(byte _dec, byte _chan, byte _wss) {
	byte _adres; byte _reg; byte _actie;
	//LETOP geen invert op een wisselstraat
	//hier komt de ingedrukte knop binnen, _dec is de knop set, _chan is de knop, _wss de gevonden wisselstraat
	// de verwijzing van de knoppen op de module (S1~S8) is hier al in verwerkt. dus het is de verwezen knop die hier binnenkomt.
	//in DCC straat is bepaald dat het om een wisselstraat bediening gaat. De niet-wisselstraat bedieningen gaan naar
	//DCC_single

	// Volgorde belangrijk, eerst de acties, daarna pas de smartleds omzetten. Anders zal een actie (wisselomleggen) van de wissels waarvan de gegevens (adres enzo) van de 
	// knopgroep worden gehaald die gebruikt wordt om de wisselstraat set om te leggen direct weer de wisselstraat uitzetten. 
	// dit alles om het omleggen van de wissels waarvan de gegeven in de knopset zitten die gebruikt wordt voor de wisselstraat toch te kunnen gebruiken.
	// //anders zou het aantal accessoirs van 64 vermindert worden met het aantal wisselstraten, dat is ongewenst. INstellen is wel een extra truck voor nodig.

	for (byte a = 0; a < AantalWisselstraatacties; a++) {
		//hier _wss wisselstraatset gebruiken, niet _dec die verwijst naar de knopset die de wisselstraat activeert.
		//_dec en _chan zijn verwijzingen naar de dekoder, knopset en channel (1~4) van de knop die de wisselstraat activeert
		//_actiedec en _actiechan verwijst naar de knopset en chammel die in de actie is bepaald, de wissels die in de straat moeten worden omgezet.

		_actie = WSactie[_wss][_chan][a];
		if (_actie & (1 << 7)) { 	//bit0=stand(port) bit1,2 channel bit 3,4,5,6 knopset 0~15 bit7=aan/uit

			//	actie is actief, uitvoeren
			//dekoder[wss].adres;	//dekoder[wss].reg; 	//bit0-1 timing 0=continue 1=0.25 2=0.5 3=1sec; 	//bit2-3 dual/mono mode B00/B01=dual B10=channels 1-2 B11=channels 3-4; 	//bit4 switch aan/uit flipflop of momentary 
			//bit5 invert ports; 	//bit6 nc; 	//bit7 hoog adres 256~511
			//stand instellen //bit 0 van het WS byte

			byte _buffer = DCC_findbuffer(); //zoek een vrije buffer
			//decoder bepalen van de accessoire aangewezen in de actie
			byte _actiedec = _actie;
			_actiedec = _actiedec << 1; _actiedec = _actiedec >> 4; //clear bits 7,2,1,0 isolate de decoder, de knopset
			Command_adres(_buffer, _actiedec); //bepaal het adres			
			buffer[_buffer].data[1] |= (1 << 3); //Stuur een on-command		

			if (_actie & (1 << 0))buffer[_buffer].data[1] |= (1 << 0);	//port (recht of afslaand) bepalen en instellen toevoegen aan het command

			//channel van de actie bepalen
			byte _actiechan = _actie; _actiechan = _actiechan << 5; _actiechan = _actiechan >> 6; //clear bits 7,2,1,0 isolate het channel
			_actiechan = _actiechan << 1;
			buffer[_buffer].data[1] += _actiechan;

			buffer[_buffer].repeat = Aantalrepeats;
			//delay instellen
			buffer[_buffer].delay = a * (WSdelayfactor*10); //tijd in stappen van 10ms
			DCC_puls(_actiedec, _buffer, buffer[_buffer].delay); //maakt tweede off command na aflopen delay

			//stand van de actie instellen 
			_actiechan = _actiechan >> 1;
			if (_actie & (1 << 0)) {
				dekoder[_actiedec].stand |= (1 << _actiechan + 4); dekoder[_actiedec].stand |= (1 << _actiechan);
			}
			else {
				dekoder[_actiedec].stand |= (1 << _actiechan + 4); dekoder[_actiedec].stand &= ~(1 << _actiechan);
			}
			//eventuele andere wisselstraten 'uit' zetten als deze straat een wissel uit deze andere set ook omlegd.
			WS_reset(_actiedec, _actiechan);
		}
	}
	//knop die wisselstraat bedient "aan" zetten. Uit zetten gebeurt door omzetten van een van de wissels in de groep, vanaf een andere knop.
	dekoder[_dec].stand |= (1 << _chan + 4);
	dekoder[_dec].stand |= (1 << _chan);
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
//smartleds

bool iswisselstraat(byte _set) {  //dec, chan
	//bool _result = false;
	byte _ws = 0;
	for (byte wss = 0; wss < Aantalwisselstraatsets; wss++) { //bit0,1 knop 1~4    bit2,3,4 knopset 0~15  bit7=actief (wisselstraat byte)
		if (WS[wss] & (1 << 7)) { //is de geteste wisselstraatset actief, aan?
			_ws = WS[wss]; //dekoder, switchset isoleren
			_ws = _ws << 2; _ws = _ws >> 4;
			if (_set == _ws) return true;
		}
	}
	return false;
}
void Kleur(byte _nummer) {
	switch (_nummer) {
	case 0: //grijs
		kleur[0] = 0;
		kleur[1] = 0;
		kleur[2] = 0;
		break;
	case 1: //rood
		kleur[0] = 1+lh*20;
		kleur[1] = 0;
		kleur[2] = 0;
		break;
	case 2: //groen
		kleur[0] = 0;
		kleur[1] = 1+lh*20;
		kleur[2] = 0;
		break;
	case 3: //paars
		kleur[0] = 1+lh*15;
		kleur[1] = 0;
		kleur[2] = 1+lh*16;
		break;
	case 4: //geel(zwak)
		kleur[0] = 1+lh*5;
		kleur[1] = 1+lh*5;
		kleur[2] = 0;
		break;
	}
	//omwisselen groen en rood als ingesteld
	if (Reg & (1 << 0)) { //RGB of GRB
		byte _temp = kleur[0];
		kleur[0] = kleur[1];
		kleur[1] = _temp;
	}
}

void PixShow() {
	//stuurt de WS281x pixels aan op de leds aansluiting
	byte _kleur; //1=grijs 2=rood 3=groen
	byte _pix;

	for (byte d = 0; d < 16; d++) { //d=decoder 
		bool _wisselstraat = iswisselstraat(d); //bepaal of knop set een wisselstraat aanstuurt.
		byte _kleur;
		for (byte c = 0; c < 4; c++) { //c=channel


			if (dekoder[d].stand & (1 << (c + 4))) { //stand bekend
				if (dekoder[d].stand & (1 << c)) { //stand aan
					_kleur = 1; //rood
					if (_wisselstraat) _kleur = 3;
				}
				else { //stand uit
					_kleur = 2; //groen
					if (_wisselstraat)_kleur = 4;
				}
			}
			else { //stand niet bekend
				_kleur = 0; //grijs
				if (_wisselstraat)_kleur = 4; //geel
			}
			Kleur(_kleur);
			//kleur vertalen en versturen 3 bytes 24bits
			for (byte color = 0; color < 3; color++) { //de 3 bytes die de kleur bepalen 1 voor 1
				_pix = kleur[color];

				//byte _pix nu versturen
				for (byte i = 7; i < 8; i--) {
					if (_pix & (1 << i)) { //true bit
						Pixone();
					}
					else { //false bit
						Pixzero();
					}
				}
			}
		}
	}
}

void Pixone() {
	cli();
	PINB |= (1 << 0);
	asm volatile(
		"rjmp . + 0\n\t"
		"rjmp . + 0\n\t"
		"rjmp . + 0\n\t"
		"rjmp . + 0\n\t"
		);
	PINB |= (1 << 0);
	PORTB &= ~(1 << 0);
	asm volatile(
		//	"rjmp . + 0\n\t"
		//	"rjmp . + 0\n\t"
		//	"rjmp . + 0\n\t"
		//	"rjmp . + 0\n\t"
		"nop\n\t"
		);
	sei();
}

void Pixzero() {
	cli();
	PINB |= (1 << 0);
	//asm volatile(
	//"nop\n\t"
	////"rjmp . + 0\n\t"
	//	);
	PINB |= (1 << 0);
	PORTB &= ~(1 << 0);
	//asm volatile(
	//	//"rjmp . + 0\n\t"
	//	//"rjmp . + 0\n\t"
	//	//"rjmp . + 0\n\t"
	//	//"rjmp . + 0\n\t"
	//	//"rjmp . + 0\n\t"
	//	"rjmp . + 0\n\t"
	//	"nop\n\t"
	//	);
	sei();
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
	GPIOR1 |= (1 << 4); //zet flag voor update smartleds
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
	case 0: //KNOP DCC
		if (_onoff)PORTB ^= (1 << 5); //DCC on off
		break;

	case 1: //KNOP program algemene dingen

		if (_onoff) {
			GPIOR1 &= ~(1 << 1);
			GPIOR1 ^= (1 << 0);

			if (GPIOR1 & (1 << 0)) {
				DP_common();
			}
			else {
				dp.clearDisplay();
				DP_bedrijf();
				Eepromwrite();
				cursor = 0;
				para = 0;
				actiecount = 0;
			}
		}
		break;

	case 2: //KNOP program inc individueel
		if (_onoff) {
			GPIOR1 &= ~(1 << 0);
			GPIOR1 ^= (1 << 1);

			if (GPIOR1 & (1 << 1)) {
				DP_single();
			}
			else {
				dp.clearDisplay();
				DP_bedrijf();
				Eepromwrite();
				cursor = 0;
			}
		}
		break;

	default: //KNOPPEN S1~S8 (3~12)
		if (GPIOR1 & (1 << 0)) {
			//common program mode
			SW_common(_button, _onoff);
		}

		else if (GPIOR1 & (1 << 1)) { //individueel program mode
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
	byte _temp; byte _set = 0; // byte _knop = 0;
	if (_onoff == false)return; //alleen on knop acties

	switch (_button) {
	case 3: //KNOP 1
		if (cursor > 0) {
			cursor--;
			GPIOR1 &= ~(192 << 0); //reset bit 7,6
		}
		break;
	case 4: //KNOP 2
		if (cursor < 3) { cursor++; 	GPIOR1 &= ~(192 << 0); } //reset bits 7,6
		break;

	case 5: //SWITCH, knop 3
		switch (cursor) {
		case 1: //CV verzenden
			switch (para) {
			case 1: //adres
				if ((CVadres > 10)) {
					CVadres -= 10;
				}
				else {
					if (CVadres == 1) {
						CVadres = 0; //maakt een BROADCAST command mogelijk
					}
					else {
						CVadres = 1;
					}
				}
				break;
			case 2: //CVnummer
				if (CVnum > 9) { //CVnum loopt van 0~1023 maar wordt getoond en gecommuniceerd in handleidingen als 1~1024
					CVnum -= 10;
				}
				else {
					CVnum = 0;
				}
				break;
			case 3: //CVval
				if (CVval > 9) {
					CVval -= 10;
				}
				else {
					CVval = 0;
				}
				break;
			}
			break;
		case 3: // Resets, toggle tussen dcc rest of all reset
			GPIOR1 ^= (1 << 6);
			break;
		}
		break;

	case 6: //S4 knop 4
		switch (cursor) {
		case 1: //CVinstellen
			switch (para) {
			case 1: //adres instellen inc x 10
				if (CVreg & (1 << 0)) { //acc
					if ((CVadres + 10) > 511) {
						CVadres = 511;
					}
					else {
						CVadres += 10;
					}
				}
				else { //loc
					if ((CVadres + 10) > 127) {
						CVadres = 127;
					}
					else {
						CVadres += 10;
					}
				}
				break;
			case 2: //CVnummer
				if (CVnum < 1013) {
					CVnum += 10;
				}
				else {
					CVnum = 1023;
				}
				break;
			case 3: //CVval
				if (CVval < 245) {
					CVval += 10;
				}
				else {
					CVval = 255;
				}
				break;
			}
			break;
		case 3: //toggle tussen dcc rest of all reset
			GPIOR1 ^= (1 << 6);
			break;
		}
		break;
	case 7:  //KNOP 5
		//if (para > 0)para--; //is meer program geheugen waarom??? heeft van doen met code optimalisatie

		switch (cursor) {
		case 0: //Wisselstraat
			if (para > 0)para--;
			break;
		case 1: //CV
			if (para > 0)para--;
			break;
		case 2: //instellingen
			if (para > 0)para--; //verplaatst cursor
			break;
		}
		break;

	case 8: //KNOP 6
		switch (cursor) {
		case 0: //wisselstraat
			if (para < 8)para++;  //aantal arameters in het instellen van de wisselstraten
			break;
		case 1: //CV
			if (para < 4)para++;
			break;
		case 2: //algemene instellingen
			if (para < 2)para++; //volgende parameter
			break;
		}
		break;

	case 9: //KNOP 7
		switch (cursor) {
		case 0: //wisselstraten
			switch (para) {
			case 0:
				if (wss > 0)wss--;
				break;
			case 1:  //wisselstraat set uitzetten
				WS[wss] &= ~(1 << 7);
				break;
			case 2: //keuze knopset die voor straat wordt gebruikt
				_set = WS[wss];
				_set = _set << 2; _set = _set >> 4;
				if (_set > 0)_set--;
				WS[wss] &= ~(60 << 0);
				_set = _set << 2;
				WS[wss] += _set;
				_set = _set >> 2;

				break;
			case 3: //keuze knop in deze set (1~4)
				lastbutton = WS[wss];
				lastbutton &= ~(252 << 0);
				if (lastbutton > 0) lastbutton--;
				WS_knop(wss, lastbutton);
				break;

				//tweede regel
			case 4: //keuze actie in volgorde voor deze knop
				if (actiecount > 0) actiecount--;
				break;
			case 5:
				WSactie[wss][lastbutton][actiecount] &= ~(1 << 7); //actie uit
				break;
			case 6: //keuze knopset dec.
				_temp = WSactie[wss][lastbutton][actiecount];
				_temp = _temp << 1; _temp = _temp >> 4;
				if (_temp > 0)_temp--;
				_temp = _temp << 3;
				WSactie[wss][lastbutton][actiecount] &= ~(120 << 0);
				WSactie[wss][lastbutton][actiecount] += _temp;
				break;
			case 7: //keuze knop in de set 1~4
				_temp = WSactie[wss][lastbutton][actiecount];
				_temp = _temp >> 1;
				_temp &= ~(252 << 0);
				if (_temp > 0)_temp--;
				WSactie[wss][lastbutton][actiecount] &= ~(6 << 0);//reset bits 1 en2 
				_temp = _temp << 1;
				WSactie[wss][lastbutton][actiecount] += _temp;
				break;
			case 8: //Stand rechtdoor
				WSactie[wss][lastbutton][actiecount] &= ~(1 << 0); //zet stand rechtdoor
				break;

			}
			break;
		case 1: //CV  //CV (reminder: indrukken knop 7 hier....)
			switch (para) {
			case 0: //loc of accessory
				CVreg &= ~(1 << 0); //loc
				CVadres = 3; //is default value for DCC decoders
				CVnum = 0;
				CVval = 3;
				break;
			case 1: //adres
				if (CVadres > 1)CVadres--;
				break;
			case 2:
				if (CVnum > 0)CVnum--;
				break;
			case 3:
				if (CVval > 0)CVval--;
				break;
			}

			break;
		case 2: //instellingen
			switch (para) {
			case 0: //instellen RGB of GRB
				Reg |= (1 << 0);
				break;
			case 1:
				if (lh > 0)lh--;
				break;
			case 2: //pauze in de wisselstraten
				if (WSdelayfactor > 0)WSdelayfactor--;
				break;
			}
			break;
		case 3:
			if (GPIOR1 & (1 << 7)) {
				Factory();
				break;
			}
		}
		break;

	case 10:  //KNOP 8
		switch (cursor) {
		case 0: //wisselstraat
			switch (para) {
			case 0:
				if (wss < Aantalwisselstraatsets - 1) wss++;
				break;
			case 1: //wisselstraatset aanzetten
				WS[wss] |= (1 << 7);
				break;
			case 2: //Keuze knopset voor deze wisselstraat
				_set = WS[wss];
				_set = _set << 2; _set = _set >> 4;
				if (_set < 15)_set++;

				WS[wss] &= ~(60 << 0);
				_set = _set << 2;
				WS[wss] += _set;
				_set = _set >> 2;
				break;

			case 3: //keuze knop binnen deze gekozen knopset voor de straat inc
				lastbutton = WS[wss];
				lastbutton &= ~(252 << 0);
				if (lastbutton < 3)lastbutton++;
				WS_knop(wss, lastbutton);
				break;

				//***** Tweede regel, de acties
			case 4: //keuze actie 1~8
				if (actiecount < 7) actiecount++;
				break;
			case 5: //actie aanzetten
				WSactie[wss][lastbutton][actiecount] |= (1 << 7);
				break;
			case 6: //keuze knop set 1~16 inc.
				_temp = WSactie[wss][lastbutton][actiecount];
				_temp = _temp << 1; _temp = _temp >> 4;
				if (_temp < 15)_temp++;

				//actiebyte aanpassen
				WSactie[wss][lastbutton][actiecount] &= ~(120 << 0); //reset bits 6~3
				_temp = _temp << 3;
				WSactie[wss][lastbutton][actiecount] += _temp;
				break;
			case 7: //keuze knop 1~4 in knopset
				_temp = WSactie[wss][lastbutton][actiecount];
				_temp = _temp >> 1;
				_temp &= ~(252 << 0); //resets, clear bit 7~2
				if (_temp < 3)_temp++;
				//actiebyte aanpassen
				WSactie[wss][lastbutton][actiecount] &= ~(6 << 0); //reset bits 1 en 2
				_temp = _temp << 1;
				WSactie[wss][lastbutton][actiecount] += _temp;
				break;
			case 8: //zet stand afslaand
				WSactie[wss][lastbutton][actiecount] |= (1 << 0);
				break;

			}
			break;
			////**********************************************CV
		case 1: //CV (reminder: indrukken knop 8 hier....)
			switch (para) {
			case 0:
				CVreg |= (1 << 0); //accessories
				CVadres = 1;
				CVnum = 0;
				CVval = 0;
				break;
			case 1: //adres
				if (CVreg & (1 << 0)) { //accessory
					if (CVadres < 511)CVadres++;
				}
				else { //mfd loc alleen kort adres mogelijk
					if (CVadres < 127)CVadres++;
				}
				break;
			case 2: //CVnum
				if (CVnum < 1023)CVnum++;
				break;
			case 3: //CVval
				if (CVval < 255)CVval++;
				break;
			case 4:  //verzenden CV
				CV_exe();
				break;
			}
			break;
		case 2: //instellen smartleds
			switch (para) {
			case 0: //RGB of GRB
				Reg &= ~(1 << 0);
				break;
			case 1: //lh=led helderheid
				if (lh < 9)lh++;
				break;
			case 2:
				if (WSdelayfactor < 19)WSdelayfactor++;
				break;
			}
			break;
		case 3:
			GPIOR1 ^= (1 << 7);
			break;
		}
		break;  //voor case 10 
	}
	if (CVreg & (1 << 1)) DP_common(); //niet als bevestiging CV zenden aan is.
}

void CV_exe() {
	byte _bytecount = 1; int16_t _adres;
	CV_ani();//display aanpassen om duidelijk te maken dat er iets wordt verzonden
	//Onlogisch om verzendingen uit de buffers te verwachten als je met CV's bezig bent.
	//wel is via de dcc-in ontvangen commands mogelijk, dit misschien blokkeren met een extra bit
	//in CVreg maar eerst maar eens zonder.

	for (byte i = 0; i < AantalDatabytes; i++) { //clear data bytes te verzenden
		dccdata[i] = 0;
	}
	//invullen databytes
	_adres = CVadres;
	if (CVreg & (1 << 0)) { //accessoire

		//aantal databytes=adres-instruction(1AAA0000)-CV1-CV2-CV3-checksum =6 aantalbytes=5 (0,1,2,3,4,5,)
		dccaantalBytes = 5;

		dccdata[0] |= (1 << 7);
		//3 extra adres bytes mogelijk
		dccdata[1] |= (240 << 0); //set bit 7,6,5,4 adresbytes in one complement
		//dccdata[1] bites 0,1,2,3 blijven 0000 alleen gehele decoder instelbaar CDDD
		if (_adres > 255) {
			dccdata[1] &= ~(1 << 6);
			_adres -= 256;
		}
		if (_adres > 127) {
			dccdata[1] &= ~(1 << 5);
			_adres -= 128;
		}
		if (_adres > 63) {
			dccdata[1] &= ~(1 << 4);
			_adres -= 64;
		}
		_bytecount++;
	}
	else { //loc
		//aantaldatabytes=adres-CV1-CV2-CV3-checksum =5 aantalbytes=4
		dccaantalBytes = 4;
		//alleen kort adres locs geen tweede (instructie)byte, direct door naar de CV bytes
	}
	dccdata[0] += _adres; //adres optellen bij eerste byte
	//merk op de accessories hebben een byte meer dan locs, bij accessory byte opschuiven
	//eerste CV byte 111 GGVV GG=(B)11 Write Byte VV=bit10,9 van het CV 10-bits
	dccdata[_bytecount] = B11101100;
	_adres = CVnum;
	if (_adres > 511) {
		dccdata[_bytecount] |= (1 << 1);
		_adres -= 512;
	}
	if (_adres > 255) {
		dccdata[_bytecount] |= (1 << 0);
		_adres -= 256;
	}
	_bytecount++; //volgende 2e CV byte AAAAAAAA 
	dccdata[_bytecount] = _adres;
	_bytecount++; // volgende 3e CV byte VVVVVVVV (value)
	dccdata[_bytecount] = CVval;
	_bytecount++; //volgende byte checksum
	_adres = dccdata[0]; //_adres hier even als temp byte gebruiken.
	for (byte i = 1; i < _bytecount; i++) {
		_adres = _adres ^ dccdata[i];
	}
	dccdata[_bytecount] = _adres;

	//start verzending
	count_preample = 0;
	dccfase = 1;

	//28jul2024 in een ideale wereld zou het nu moeten werken.
}

void CV_ani() {
	CVreg &= ~(1 << 1);
	CVtimer = 0;
	dp.fillCircle(64, 38, 24, 1);
	dp.setCursor(56, 24); dp.setTextColor(0); dp.setTextSize(4); dp.print(">"); //kost 60bytes program mem
	dp.display();
}
void CV_timer() {
	//timed een animatie anders zie je als gebruiker niet dat je een CV aan het versturen bent
	//called om de 2ms
	if (~CVreg & (1 << 1)) {
		if (CVtimer < 250) {
			CVtimer++;
		}
		else {
			CVreg |= (1 << 1);
			DP_common();
		}
	}
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
	byte  _temp = 0; //temp variable
	//tekend de common instellingen
	byte _set = 0; //byte _knop = 0;

	dp.clearDisplay();
	dp.fillRect(0, 0, 128, 14, 1);
	dp.setCursor(10, 4);
	dp.setTextColor(0);
	dp.setTextSize(1);
	switch (cursor) {
	case 0: //wisselstraten instellen
		dp.print(F("Wisselstraat"));
		//Welke wisselstraat set instellen
		if (para == 0) {
			dp.fillRect(10, 18, 19, 18, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		dp.setCursor(14, 20);
		dp.setTextSize(2);
		dp.print(wss + 1); //wss=wisselstraat set

		//**** Is deze wisselstraat set in gebruik?
		_temp = 1; //_temp=kleur
		if (para == 1) {
			dp.fillRect(31, 19, 15, 15, 1); //teken een wit rechthoek
			_temp = 0;
		}

		if (WS[wss] & (1 << 7)) {
			//gesloten circel
			dp.fillCircle(38, 26, 5, _temp);
		}
		else {
			//open rondje
			dp.drawCircle(38, 26, 5, _temp);
		}


		//***********Welke knopset is aan deze wisselstraatset toegewezen
		if (para == 2) {	//keuze van de knopset waar adres en channel van moet komen
			dp.fillRect(48, 18, 26, 18, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		_set = WS[wss];
		_set = _set << 2;
		_set = _set >> 4; //isoleer knopset 
		//byte knop; //bit0,1 knop 1~4     bit2,3,4,5 knopset 0~15  bit7=actief 

		dp.setTextSize(2);
		if (_set > 8) {
			dp.setCursor(50, 20);
		}
		else {
			dp.setCursor(62, 20);
		}
		dp.print(_set + 1); //wss=wisselstraat set	

		dp.fillRect(75, 27, 8, 2, 1); //streepje tussen set en knop

		if (para == 3) { //Welke van de 4 knoppen
			dp.fillRect(85, 18, 17, 20, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		dp.setCursor(88, 20);
		lastbutton = WS[wss];
		lastbutton &= ~(252 << 0); //reset bits 7~2
		dp.print(lastbutton + 1);


		//**************tweede regel, bit0=stand(port) bit1,2 channel bit 3,4,5,6 knopset 0~15 bit7 actie actief
		//set is de knopset, _knop is de knop, chan
		if (para == 4) { //nummer, volgorde acties 1~8
			dp.fillRect(10, 40, 20, 18, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}

		dp.setTextSize(2);
		dp.setCursor(14, 42);
		dp.print(actiecount + 1);


		if (para == 5) { //deze actie aan of uit?
			dp.fillRect(31, 42, 15, 15, 1);
			_temp = 0;
		}
		else {
			_temp = 1;
		}
		//acties ophalen, 
		//WSactie[wisselstraat][channel, knop][actiecount]

		if (WSactie[wss][lastbutton][actiecount] & (1 << 7)) {
			dp.fillCircle(38, 49, 5, _temp);
		}
		else {
			dp.drawCircle(38, 49, 5, _temp);
		}

		if (para == 6) { //schakelaar set en channel 
			dp.fillRect(48, 40, 26, 18, 1);  //48, 18, 26, 18, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		dp.setTextSize(2);
		_temp = WSactie[wss][lastbutton][actiecount]; //bit 3,4,5,6, = knopset 1~16
		_temp = _temp << 1; _temp = _temp >> 4;
		if (_temp > 8) {
			dp.setCursor(50, 42);
		}
		else {
			dp.setCursor(62, 42);
		}
		dp.print(_temp + 1);
		dp.fillRect(75, 47, 8, 2, 1); //streepje tussen set en knop

		//knop
		if (para == 7) {
			dp.fillRect(85, 40, 17, 18, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		_temp = WSactie[wss][lastbutton][actiecount];
		_temp = _temp >> 1;
		_temp &= ~(252 << 0);

		dp.setTextSize(2); //waarschijnlijk kan dit weg straks
		dp.setCursor(88, 42);
		dp.print(_temp + 1);

		// stand van het accessory
		if (para == 8) {
			dp.fillRect(100, 40, 24, 18, 1);
			_temp = 0;
		}
		else {
			_temp = 1;
		}
		if (WSactie[wss][lastbutton][actiecount] & (1 << 0)) {
			dp.drawRect(102, 52, 18, 4, _temp);
			dp.drawLine(102, 54, 118, 45, _temp);
			dp.drawLine(102, 53, 118, 44, _temp);
			dp.drawLine(102, 52, 118, 43, _temp);
			dp.drawLine(102, 51, 118, 42, _temp);
		}
		else {
			dp.fillRect(102, 52, 18, 4, _temp);
			dp.drawLine(102, 54, 119, 45, _temp);
			dp.drawLine(102, 51, 118, 42, _temp);
		}

		break;
		//***CV**********
	case 1:
		dp.print(F("CV"));
		//vaste teksten (scheelt program space)
		dp.setTextColor(1);
		//dp.setTextSize(1); //niet nodig
		dp.setCursor(49, 25);
		dp.print("adr");
		dp.setCursor(2, 47);
		dp.print("#");
		dp.setCursor(60, 47);
		dp.print("V");
		dp.setTextSize(2);

		if (para == 0) { //keuze tussen accessory(default) of loc
			dp.fillRect(5, 18, 38, 20, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		dp.setCursor(7, 20);

		if (CVreg & (1 << 0)) {
			dp.print("Acc");
		}
		else {
			dp.print("Loc");
		}
		if (para == 1) { //adres te verzenden
			dp.fillRect(68, 18, 50, 20, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		//dp.setTextSize(2); //niet nodig
		dp.setCursor(70, 20);
		if (CVadres == 0) {
			dp.print("*BC*");
		}
		else {
			if (CVreg & (1 << 0)) { //acc
				dp.print(1 + ((CVadres - 1) * 4)); //toon het dcc adres
			}
			else { //loc
				dp.print(CVadres);
			}
		}

		if (para == 2) { //CV nummer
			dp.fillRect(8, 40, 50, 18, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		//dp.setTextSize(2); //niet nodig
		dp.setCursor(10, 42);
		dp.print(CVnum + 1);
		if (para == 3) { //CVval
			dp.fillRect(68, 40, 40, 18, 1);
			dp.setTextColor(0);
		}
		else {
			dp.setTextColor(1);
		}
		dp.setCursor(70, 42);
		//dp.setTextSize(2);  //niet nodig
		dp.print(CVval);

		if (para == 4) { //uitvoeren knop
			dp.fillCircle(110, 48, 12, 1);
			dp.setCursor(106, 41),
				dp.setTextColor(0);
			//dp.setTextSize(2); //niet nodig
			dp.print(">");
		}

		break;

	case 2:
		dp.println(F("Instellingen"));
		//cursor=instelling groep, para is de instelling in deze groep
		if (para == 0) {
			dp.fillRect(10, 18, 40, 18, 1);
			dp.setTextColor(0);
		}
		else {
			dp.drawRect(10, 18, 40, 18, 0);
			dp.setTextColor(1);
		}
		dp.setTextSize(2);
		dp.setCursor(12, 20);
		if (Reg & (1 << 0)) {
			dp.print("RGB");
		}
		else {
			dp.print("GRB");
		}
		_temp = 1;
		if (para == 1) { //helderheid smartleds
			dp.fillRect(52, 18, 22, 19, 1);
			_temp = 0;
		}
		dp.drawCircle(62, 27, 8, _temp);
		dp.fillCircle(62, 27, lh, _temp);
		
		_temp = 1;
		if (para == 2) { //pauze in wisselstraat 
			dp.fillRect(5, 40, 42, 20, 1);
			_temp = 0;
		}
		dp.drawRect(7, 44, 38, 12, _temp);
		dp.fillRect(7, 44, WSdelayfactor * 2, 12, _temp);
		
		break;
	case 3: //******************Factory reset
		//dp.fillRect(0, 0, 128, 14, 1);
		//dp.setCursor(10, 2);
		//dp.setTextColor(0);
		//dp.setTextSize(1);
		dp.println("Reset");
		if (GPIOR1 & (1 << 7)) {
			dp.fillRect(5, 20, 81, 40, 0);
			if (GPIOR1 & (1 << 6))	txt = txt_alles; else  txt = txt_adres;  //"Adressen?";
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
			dp.print(txt_adres);
			if (GPIOR1 & (1 << 6)) dp.setTextColor(0); else dp.setTextColor(1);
			dp.setCursor(7, 42);
			dp.print(txt_alles);
		}

		break;
	}
	dp.display();
}

//diverse hulp voids
void WS_knopset(byte _wss, byte _knopset) {
	WS[wss] &= ~(60 << 0); //reset bits 2,3,4,5
	_knopset = _knopset << 2;
	WS[wss] += _knopset;
}

void WS_knop(byte _wss, byte _knop) {
	WS[wss] &= ~(3 << 0); //reset bits 0,1
	WS[wss] += _knop;
}

/*
 Name:		SmartLeds.ino
 Created:	7/8/2024 11:46:26 AM
 Author:	Rob Antonisse

 Test en experimenteer sketch om meer inzicht te krijgen hoe de WS2811 chips en smartleds aan te sturen.

*/

//#include <FastLED.h>

#define AantalPix 64

#define PORT PORTB
#define PORT_PIN 8


//CRGB pix[AantalPix]; //uitgaan van alle schakelaars 1 pixel kost 192 bytes, in combi met Serial (usb) loopt de boel vast.

struct Pixel {
	byte r;
	byte g;
	byte b;
};
Pixel pix[AantalPix];



unsigned long timer = 0;
unsigned long klok;

void setup() {
	Serial.begin(9600);

	//FastLED.addLeds<WS2812, 8, RGB>(pix, AantalPix);

	//ports
	DDRB |= (1 << 0); //zet pin 8 as output

}


void loop() {
	if (millis() - timer > 500) {
		timer = millis();
		//klok = micros();
		//cli();
		Animatie();
		//sei();
		//Serial.println(micros() - klok);
	}
}

//
//void Pix_one() { //stuur een 1-bit naar de smartleds
//
//	PORTB |= (1 << 0);
//
//	asm volatile(
//		// Instruction        Clock   Description   Phase     Bit Transmitted
//		//"sbi  %0, %1\n\t"  // 2      PIN HIGH       (T =  2) 
//
//		"rjmp .+0\n\t"        // 2      nop nop         (T =  4)
//		"rjmp .+0\n\t"        // 2      nop nop         (T =  6)
//		"rjmp .+0\n\t"        // 2      nop nop         (T =  8)
//		//"rjmp .+0\n\t"        // 2      nop nop         (T = 10)
//		//"rjmp .+0\n\t"        // 2      nop nop         (T = 12)
//		"nop\n\t"               // 1      nop                (T = 13)
//		);
//		
//	PORTB &= ~(1 << 0);
//		asm volatile (
//		
//		//"cbi   %0, %1\n\t" // 2      PIN LOW       (T = 15)
//		"rjmp .+0\n\t"        // 2      nop nop         (T = 17)
//		//"rjmp .+0\n\t"        // 2      nop nop         (T = 19)
//		//"nop\n\t"               // 1      nop                (T = 20)      1
//		
//			);
//}
//void Pix_zero() { //stuur een 0-bit naar de smartleds
//	// 1xt hoog; 2xt laag
//
//	PORTB |= (1 << 0);
//
//	asm volatile(
//		// Instruction        Clock   Description   Phase     Bit Transmitted
//		"rjmp .+0\n\t"        // 2      nop nop         (T =  4)
//		//"rjmp .+0\n\t"        // 2      nop nop         (T =  6)
//
//		);
//	PORTB &= ~(1 << 0);
//		asm volatile(
//
//		"rjmp .+0\n\t"        // 2      nop nop         (T = 10)
//		"rjmp .+0\n\t"        // 2      nop nop         (T = 12)
//		"rjmp .+0\n\t"        // 2      nop nop         (T = 14)
//		"rjmp .+0\n\t"        // 2      nop nop         (T = 16)
//		//"rjmp .+0\n\t"        // 2      nop nop         (T = 18)
//		//"rjmp .+0\n\t"        // 2      nop nop         (T = 20)      0
//		);
//}

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

void Animatie() {
	GPIOR0 ^= (1 << 0);

	if (GPIOR0 & 1 << 0) {

		for (byte i = 0; i < AantalPix; i++) {
			byte _red = 0;
			byte _green = 0;
			byte _blue = 0;
			pix[i].r = _red;
			pix[i].g = _green;
			pix[i].b = _blue;
		}
	}
	else {
		for (byte i = 0; i < AantalPix; i++) {
			byte _red = 10; // random(1, 10);
			byte _green =0; // random(1, 10);
			byte _blue = 0; // random(1, 10);

			pix[i].r = _red;
			pix[i].g = _green;
			pix[i].b = _blue;
		}
	}
	//FastLED.show();
	//Serial.print(pix[1].r);
	Pix_show();
}

void Pix_show() {
	byte _byte = 0;
	
	for (byte p = 0; p < AantalPix; p++) {  //p=pixel

		for (byte c = 0; c < 3; c++) { //c=color led red, green, blue
			switch (c) {
			case 0:
				_byte = pix[p].r;
				break;
			case 1:
				_byte = pix[p].g;
				break;
			case 2:
				_byte = pix[p].b;
				break;
			}

			for (byte b = 7; b < 8; b--) { //b=bit de 8 bits van een kleur byte

				if (_byte & (1 << b))
				{
					Pixone();
					//Pixzero();
				}
				else
				{
					Pixzero();
				}
			}

		}
	}
}


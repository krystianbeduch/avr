#define ENABLE_BASIC
#define ENABLE_SERIAL
#define ENABLE_INTERRUPTS
#define ENABLE_ADC
#include "framework.h"

// globalne zmienne flagowe
volatile uint8_t ledState = 0;
volatile uint8_t ledChangeFlag = 0;

ISR(USART_RX_vect) { // ZAD 2
//    char data = UDR0;
    if (UDR0 == '1') {
        ledState = 1;
        ledChangeFlag = 1;
    } 
	else if (UDR0 == '0') {
        ledState = 0;
        ledChangeFlag = 1;
    }    
}

static void INT0Callback() { // ZAD 3
	// wyslij komunikat przez USART o przerwaniu na INT0
	serialPrintStr("INT0 - opadajace zbocze\n");
}

static void INT1Callback() { // ZAD 3
	// wyslij komunikat przez USART o przerwaniu na INT1
	serialPrintStr("INT1 - zmiana stanu logicznego\n");
}


int main(void) {
	// USART
    pinMode(13, OUTPUT); // dioda do USART (czerwona), B5
	serialInit(9600);
	interruptOn();

	// PRZERWANIA
	interruptInit(2, FALLING_EDGE, INT0Callback); // D2
	interruptInit(3, CHANGE_STATE, INT1Callback); // D3

	// ADC
	pinMode(12, OUTPUT); // dioda do ADC (zielona), B4
	adcInit();

    while (1) {	
		// ZAD 2
        if (ledChangeFlag) {
            if (ledState) {
                digitalWrite(13, HIGH);
                serialPrintStr("Dioda LED wlaczona\n");
            } 
			else {
                digitalWrite(13, LOW);
                serialPrintStr("Dioda LED wylaczona\n");
            }
            ledChangeFlag = 0;
        }

		uint16_t ADCval0 = analogRead(0); // A0
		uint16_t ADCv0 = (ADCval0 * 5000) / 1024;
		uint16_t ADCval1 = analogRead(1); // A1
		uint16_t ADCv1 = (ADCval1 * 5000) / 1024;

		serialPrintStr("FOTO - ");
		serialPrintStr("ADC: ");
		serialPrintInt(ADCval0);
		serialPrintStr(" Napiecie: ");
		serialPrintInt(ADCv0 / 100);
		serialPrintStr(".");
		serialPrintInt(ADCv0 % 100);
		serialPrintStr(" V\n");
		
		serialPrintStr("POTENC - ");
		serialPrintStr("ADC: ");
		serialPrintInt(ADCval1);
		serialPrintStr(" Napiecie: ");
		serialPrintInt(ADCv1 * 100);
		serialPrintStr(" mV\n");
		
		// zapal diode w zaleznosci od wartosci potencjometru i fotorezystora
		if (ADCval0 < ADCval1) // 12 (zielona)
			digitalWrite(12, HIGH);
		else
			digitalWrite(12, LOW);

        delay(1500);

    }
	interruptOff();
    return 0;
}

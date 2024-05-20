#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>    
#include <avr/power.h>    
#include <avr/sleep.h>  
#include <avr/pgmspace.h>   
#define delay(time)	_delay_ms(time);
#define interruptOn() sei();
#define interruptOff() cli();
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define F_CPU 16000000UL

enum m {
	OUTPUT, 
	INPUT
};
enum s {
	HIGH, 
	LOW
};

enum c {
	FALLING_EDGE,
	CHANGE_STATE
};

 // ZAD 1 - OBSLUGA PORTOW
#ifdef ENABLE_BASIC
static void pinMode(uint8_t pin, uint8_t m) {
	if (pin >= 0 && pin <= 7) {
		if (m == OUTPUT)
			DDRD |= (1 << pin);
		else if (m == INPUT)
			DDRD &= ~(1 << pin);
	}
	else if (pin >= 8 && pin <= 13) {
		pin -= 8;
		if (m == OUTPUT)
			DDRB |= (1 << pin);
		else if (m == INPUT)
			DDRB &= ~(1 << pin);
	}
	else if (pin >= 14 && pin <= 19) {
		pin -= 14;
		if (m == OUTPUT)
			DDRC |= (1 << pin);
		else if (m == INPUT)
			DDRC &= ~(1 << pin);
	}
}

static void digitalWrite(uint8_t pin, uint8_t s) { 
	if (pin >= 0 && pin <= 7) {
		if (s == HIGH)
			PORTD |= (1 << pin);
		else if (s == LOW)
			PORTD &= ~(1 << pin);
	}
	else if (pin >= 8 && pin <= 13) {
		pin -= 8;
		if (s == HIGH)
			PORTB |= (1 << pin);
		else if (s == LOW)
			PORTB &= ~(1 << pin);
	}
	else if (pin >= 14 && pin <= 19) {
		pin -= 14;
		if (s == HIGH)
			PORTC |= (1 << pin);
		else if (s == LOW)
			PORTC &= ~(1 << pin);
	}
}

static uint8_t digitalRead(uint8_t pin) {
    if (pin >= 0 && pin <= 7) {
		return ((PIND & (1 << pin)) >> pin); 
    }
    else if (pin >= 8 && pin <= 13) {
        pin -= 8;
        return ((PINB & (1 << pin)) >> pin); ;
    }
    else if (pin >= 14 && pin <= 19) {
        pin -= 14;
        return ((PINC & (1 << pin)) >> pin); ;
    }
}
#endif

// ZAD 2 - USART
#ifdef ENABLE_SERIAL
static void serialInit(uint32_t baud){
	// obliczenie wartosci UBRR na podstawie szybkosci transmisji
	uint16_t ubrr = (F_CPU / (16 * baud) ) - 1;
    UBRR0H = (unsigned char)(ubrr >> 8); 
    UBRR0L = (unsigned char) ubrr;
 
	// wlaczenie odbiornika (RXEN), nadajnika (TXEN) oraz zezwolenie na generowanie przerwania (RXCO)
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); 

	// ustawienie formatu ramki: 8 bitow danych, brak parzystosci, 1 bit stopu
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (0 << UPM00) | (0 << UPM01) | (0 << USBS0);
}

static void serialWrite(uint8_t data){
	// oczekiwanie na zwolnienie bufora nadajnika
	while (!(UCSR0A & (1 << UDRE0)));

	// umieszczenie danych w buforze nadawczym
    UDR0 = data;
}

static void serialPrintStr(char *text) {
	// przesylanie kolejnych znakow z lancucha az do napotkania znaku koncowego '\0'
    while (*text)
        serialWrite(*text++);
}

static void serialPrintInt(uint16_t value){
	// bufor do przechowywania liczby w postaci lancucha
	char buffer[6];

	// konwersja liczby int na lanucha znakow
	// char * itoa( int value, char * str, int base ); 
	itoa(value, buffer, 10);
	serialPrintStr(buffer);
}
#endif

// ZAD 3 - Przerwania
#ifdef ENABLE_INTERRUPTS
// mechanizm wskaznikow funkcyjnych
static void (*_int0_func)(); 
static void (*_int1_func)(); 

//static void interruptInit(uint8_t pin, char conf, void (*f)()){
static void interruptInit(uint8_t pin, uint8_t c, void (*f)()){
	// inicjalizacja pinu INT0 (pin 2) lub INT1 (pin 3) z wlaczonym rezystorem pull-up
	if (pin == 2 || pin == 3) {
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);  // wlacz rezystor pull-up
	}
	if (c == FALLING_EDGE){ 
		_int0_func = f;
        EICRA |= (1 << ISC01) | (0 << ISC00);  // konfiguracja przerwania INT0 (ISC01 = 1, ISC00 = 0)
        EIMSK |= (1 << INT0);   // wlacz przerwanie INT0  
	}
	else if (c == CHANGE_STATE){
		_int1_func = f;
		EICRA |= (0 << ISC11) | (1 << ISC10); // konfiguracja przerwania INT1 (ISC11 = 0, ISC10 = 1)
        EIMSK |= (1 << INT1); // wlacz przerwanie INT1
	}
}

ISR(INT0_vect) {
    _int0_func();
}
ISR(INT1_vect) {
    _int1_func();    
}
#endif

// ZAD 4 - ADC
#ifdef ENABLE_ADC
void adcInit(){
	// ustaw zrodlo Vref jako napiecie odniesienia pochodzace z linii Avcc
	ADMUX |= (1 << REFS0) | (0 << REFS1);

	// preskaler 128 i wlacz ADC
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t analogRead(uint8_t channel){
	// wybor kanalu
	ADMUX = (ADMUX & 0xF0) | channel; // 1111 0000 - MUX3
 	ADCSRA |= (1 << ADSC); // start konwersji
    while (ADCSRA & (1 << ADSC)); // czekaj na zakonczenie konwersji
	ADCSRA |= (1 << ADIF); // czyszczenie flagi ADIF
	return ADC; // zwroc wartosc ADC
}
#endif


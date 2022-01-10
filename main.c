#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "HD44780.h"


#ifndef _BV
#define _BV(bit)				(1<<(bit))
#endif

#ifndef inb
#define	inb(addr)			(addr)
#endif

#ifndef outb
#define	outb(addr, data)	addr = (data)
#endif

#ifndef sbi
#define sbi(reg,bit)		reg |= (_BV(bit))
#endif

#ifndef cbi
#define cbi(reg,bit)		reg &= ~(_BV(bit))
#endif

#ifndef tbi
#define tbi(reg,bit)		reg ^= (_BV(bit))
#endif

/*
 *  Gotowe zaimplementowane:
 #define 	bit_is_set(sfr, bit)   (_SFR_BYTE(sfr) & _BV(bit))
 #define 	bit_is_clear(sfr, bit)   (!(_SFR_BYTE(sfr) & _BV(bit)))
 #define 	loop_until_bit_is_set(sfr, bit)   do { } while (bit_is_clear(sfr, bit))
 #define 	loop_until_bit_is_clear(sfr, bit)   do { } while (bit_is_set(sfr, bit))

 */

// MIN/MAX/ABS macros
#define MIN(a,b)			((a<b)?(a):(b))
#define MAX(a,b)			((a>b)?(a):(b))
#define ABS(x)				((x>0)?(x):(-x))

// ZADANIE 1
//int ADC_10bit()
//{
//	char text[20];
//	LCD_Clear();
//	LCD_GoTo(0,0);
//
//	uint16_t tmp = uint16_tADC_10bit();
//
//	sprintf(text, "%d", tmp);
//	LCD_WriteText(text);
//	return tmp;
//}

// ZADANIE 2
//uint16_t ADC_10bit()
//{
//	char text[20];
//	LCD_Clear();
//	LCD_GoTo(0,0);
//
//	uint16_t tmp = uint16_tADC_10bit();
//
//	sprintf(text, "%d", tmp);
//	LCD_WriteText(text);
//	return tmp;
//}
//
//uint16_t ADC_measure(uint16_t adc10bit)
//{
//	double tmp=((double)adc10bit*5*100)/1022;
//	uint16_t measure = (uint16_t)tmp;
//
//	LCD_GoTo(0,1);
//	uint16_t decimal = (uint16_t)(tmp/100);
//	char digit1[2];
//	sprintf(digit1, "%d", decimal);
//	LCD_WriteText(digit1);
//
//	LCD_GoTo(1,1);
//	LCD_WriteText(".");
//
//	LCD_GoTo(2,1);
//	uint16_t fraction = measure;//(uint16_t)(tmp/10);
//	char digit2[3];
//	sprintf(digit2, "%d", fraction);
//	LCD_WriteText(digit2+1);
//
//	return measure;
//}


// ZADANIE 3

//void LED_init()
//{
//	sbi(DDRC, PC3);
//}
//
//uint16_t ADC_measure(uint16_t adc10bit)
//{
//	double measure=((double)adc10bit*5*100)/1022;
//	return (uint16_t)measure;
//	return measure;
//}
//
//void Display_ADC(uint16_t measure_bit)
//{
//
//	uint16_t measure = ADC_measure(measure_bit);
//	LCD_GoTo(0,0);
//	uint16_t decimal = (uint16_t)(measure/100);
//	char digit1[2];
//	sprintf(digit1, "%d", decimal);
//	LCD_WriteText(digit1);
//
//	LCD_GoTo(1,0);
//	LCD_WriteText(".");
//
//	LCD_GoTo(2,0);
//	uint16_t fraction = measure;
//	char digit2[4];
//	sprintf(digit2, "%d", fraction);
//	if(measure >= 100)
//	{
//		LCD_WriteText(digit2+1);
//	}
//	else if(measure >= 10)
//		LCD_WriteText(digit2);
//	else
//	{
//		sprintf(digit2, "0%d",fraction);
//		LCD_WriteText(digit2);
//	}
//}
//
//_Bool Compare_ADC()
//{
//	uint16_t Variable_Value  = uint16_tADC_10bit(); //Zwracana wartosc z PA0
//	Display_ADC(Variable_Value);
//
//	uint16_t Const_Value = 511;
//
//	if(Variable_Value>Const_Value)
//		return 1;
//	else
//		return 0;
//
//}

// ZADANIE 4

void ADC_init()
{
//Napiecie referencyjne na AVCC
	sbi(ADMUX, REFS0);
	cbi(ADMUX, REFS1);

//	Czestotliwosc sygnalu taktujacego 62,5 kHz
	sbi(ADCSRA, ADPS0);
	sbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS2);

	sbi(ADCSRA, ADEN); //Wlaczenie przetwornika

//	Konfiguracja ustawia ADC do pomiaru
	sbi(ADMUX, MUX0);
	cbi(ADMUX, MUX1);
	cbi(ADMUX, MUX2);
	cbi(ADMUX, MUX3);
	cbi(ADMUX, MUX4);
}

//	Przelaczanie portow do pomiaru
void PORTA_MUX()
{
	tbi(ADMUX, MUX0);
}

// Ustawienie Portu C3 i C2 na wyjœcia
void LED_init()
{
	sbi(DDRC, PC3);
	sbi(DDRC, PC2);
}

// Pomiar przetwornika A/C
uint16_t uint16_tADC_10bit()
{
	sbi(ADCSRA, ADSC);	//Zaczyna mierzyc napiecie
	while(bit_is_set(ADCSRA,ADSC)){}	//Czekamy az przetwornik zmierzy napiecie
	return (uint16_t)ADC;
}

// Przeliczenie bitowej wartosci na wolty i razy 100
uint16_t ADC_measure(uint16_t adc10bit)
{
	double measure=((double)adc10bit*5*100)/1022;
	return (uint16_t)measure;
}

// Wyswietlanie wartosci z pomiaru na porcie A0 i A1, Wykonywanie histerezy
void Display_ADC(uint16_t measure_bit, uint16_t X)
{
	uint16_t measure = ADC_measure(measure_bit);	//Zmiana na V calkowita
	LCD_GoTo((char)X,0);
	uint16_t decimal = (uint16_t)(measure/100);		//Czesc calkowita
	char digit1[2];
	sprintf(digit1, "%d", decimal);		//Przypisanie czesci calkowitej do tablicy char
	LCD_WriteText(digit1);				//Wypisanie czesci calkowitej

	LCD_GoTo((char)(X+1),0);
	LCD_WriteText(".");

	LCD_GoTo((char)(X+2),0);
	uint16_t fraction = measure;		//Czesc ulamkowa
	char digit2[4];
	sprintf(digit2, "%d", fraction);
	if(measure >= 100)					//Wpisanie tylko czesci ulamkowej. Przesuniecie iteratora
	{
		LCD_WriteText(digit2+1);
	}
	else if(measure >= 10)				//Gdy napiecie mniejsze od 1V
		LCD_WriteText(digit2);
	else								//Gdy napiecie mniejsze od 0,1V
	{
		sprintf(digit2, "0%d",fraction);
		LCD_WriteText(digit2);
	}
}

volatile static uint16_t Demanded_Value, Real_Value;

_Bool Compare_ADC()
{
	PORTA_MUX();
	Demanded_Value = uint16_tADC_10bit(); 	//Zwracana wartosc z PA0
	Display_ADC(Demanded_Value, 0); 		//Wyswietlana wartosc z PA0 w V

	LCD_GoTo(4,0);
	LCD_WriteText("-");

	PORTA_MUX();
	Real_Value= uint16_tADC_10bit(); 		//Zwracana wartosc z PA1
	Display_ADC(Real_Value, 5); 			//Wyswietlana wartosc z PA1 w V

//	Histereza 0,5V
	if(Demanded_Value+51<Real_Value)		//Gdy rzeczywista wartosc przekroczy wymagana wartosc plus polowa histerezy
	{
		sbi(PORTC,PC2);						//Zapal diode Reg1
	}
	else if(Demanded_Value-51>Real_Value)  	//Gdy rzeczywista jest mniejsza od wymaganej wartosc minus polowa histerezy
	{
		cbi(PORTC,PC2);						//Zgas diode Reg1
	}

	if(Demanded_Value>Real_Value)			//Jesli zadana wartosc wieksza od rzeczywistej, zwroc 1
		return 1;
	else									//Jesli nie, zwroc 0
		return 0;

}

int main()
{
	ADC_init();
	LCD_Initalize();
	LCD_Home();
	LED_init();
	while(1)
	{
//		ZADANIE 1
//		_delay_ms(10);
//		ADC_10bit();
//		_delay_ms(10);

//		ZADANIE 2
//		_delay_ms(10);
//		ADC_measure(ADC_10bit());
//		_delay_ms(10);

//		ZADANIE 3
//		_delay_ms(100);
//		LCD_Clear();
//
//		if(Compare_ADC())
//		{
//			LCD_GoTo(0,1);
//			LCD_WriteText("on");
//			sbi(PORTC,PC3);
//		}
//		else
//		{
//			LCD_GoTo(0,1);
//			LCD_WriteText("off");
//			cbi(PORTC,PC3);
//		}
//		_delay_ms(100);


//		ZADANIE 4
		_delay_ms(75);
		LCD_Clear();
		if(Compare_ADC())
		{
			sbi(PORTC,PC3);
		}
		else
		{
			cbi(PORTC,PC3);
		}
	}
}

//Co to jest komparator?
//Komparator jest uk³adem porównuj¹cym. Komparatory dzielimy na cyfrowe, które porównuj¹ dwie liczby binarne i analogowe, które porównuj¹ dwa poziomy napiêæ lub pr¹dów..

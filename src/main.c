// *******************************************************************//
// Rodriguez 17/10/2023
// Operacion de motor CC con velocidad alternada.
// Utiliza Interrupciones, Timer, ADC y PWM
// *******************************************************************//
// Definiciones
// ---------------------------------------------------------------------
#define F_CPU    16000000UL
#define BOUNCE_DELAY 8 //ms
#define DISPLAY_DELAY 5 //ms

// Entradas:
#define P1      PD1           // Enciende / apaga motor
#define P2      PD0           // Controla los modos de operacion: config y normal
#define P3      PD2           // Selecciona configuracion de T1_V1 o T2_V2

// Salidas:
// Pines usados por la libreria lcd_2560.h:
#define RS	PA0			// Pin RS = PA0 (22) (Reset).
#define EN	PA1			// Pin EN = PA1 (23) (Enable).
#define D4	PA2			// Pin D4 = PA2 (24) (Data D4).
#define D5	PA3			// Pin D5 = PA3 (25) (Data D5).
#define D6	PA4			// Pin D6 = PA4 (26) (Data D6).
#define D7	PA5			// Pin D7 = PA5 (27) (Data D7).

// Macros de usuario
// -------------------------------------------------------------------
#define TOP 125         // Con N=64 --> T = 1/2k en la senal PWM
#define	sbi(p,b)	p |= _BV(b)					//	sbi(p,b) setea el bit b de p.
#define	cbi(p,b)	p &= ~(_BV(b))				//	cbi(p,b) borra el bit b de p.
#define	tbi(p,b)	p ^= _BV(b)					//	tbi(p,b) togglea el bit b de p.
#define is_high(p,b)	(p & _BV(b)) == _BV(b)	//	is_high(p,b) p/testear si el bit b de p es 1.
#define is_low(p,b)		(p & _BV(b)) == 0		//	is_low(p,b) p/testear si el bit b de p es 0.

// Inclusion de archivos de cabecera
// -------------------------------------------------------------------
#include <stdio.h>				// Cabecera estandar de E/S.
#include <stdlib.h>				// Cabecera de la biblioteca estandar de proposito general.
#include <avr/io.h>				// Contiene definiciones estandares (puertos, memorias, etc.)
#include <util/delay.h>			// Contiene macros para generar retardos.
#include <avr/interrupt.h>		// Contiene macros para uso de interrupciones.
#include "lcd_2560.h"			// Contiene funciones para manejo del LCD.

// Variables globales
// -------------------------------------------------------------------
volatile int FlagP1 = 0; 							// FlagP1 = 0 -> Motor apagado
volatile int FlagP2 = 1; 							// FlagP2 = 1 -> Modo configuracion
volatile int FlagP3 = 1;						    // FlagP3 = 1 -> Configuracion T1V1
volatile int TIME = 0;			                    // Variable contador para el Timer 2
volatile int T=10;                                  // Lectura del potenciometro RV1 (Tiempo)
volatile int V=45;                                  // Lectura del potenciometro RV2 (Velocidad)
volatile int motor_on = 0;                          // estado del motor
int T1=10;                                          // Tiempo etapa 1
int V1=95;                                          // Velocidad etapa 1
int T2=10;                                          // Tiempo etapa 2
int V2=40;                                          // Velocidad etapa 2

// Declararacion de funciones
// -------------------------------------------------------------------
void initPorts();
void boot();
void initExternalInterrupts();
void initTimer2();               // Inicializacion del Timer2 como temporizador (10ms)
void initTimer0();               // Inicializacion del Timer0 como Fast PWM
void initADConverter();
void config();
void configTV(int op);
void normal();
void lcd();

// Programa
// -------------------------------------------------------------------
int main(void){

	initPorts();
	boot();
	initExternalInterrupts();
	initTimer2();
	initTimer0();
	initADConverter();
	Lcd4_Init();				// Inicializa el LCD (siempre debe estar antes de escribir el LCD).
	Lcd4_Clear();				// Borra el display.
	sei();						// Habilita interrupciones.

	// Inicio
	while(1){
		if(FlagP2){
			config();
		}
		else{
			normal();
		}

		lcd();
	}
}

// Interrupciones
// ----------------------------------------------------------------------------------------
ISR(INT0_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P2)){        // Comprueba si P2 sigue en BAJO
		tbi(FlagP2,0);           // Establecer la bandera para indicar la interrupcion
	}
}

ISR(INT1_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P1)){        // Comprueba si P1 sigue en BAJO
		tbi(FlagP1,0);           // Establecer la bandera para indicar la interrupcion
	}
}

ISR(INT2_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P3) && FlagP2){    // Comprueba si P3 sigue en BAJO y está en cnf()
		tbi(FlagP3,0);                 // Establecer la bandera para indicar la interrupcion
	}
}

ISR(TIMER2_COMPA_vect){
    TIME++;					       // Incrementa contador cada 10ms
}

ISR(ADC_vect){
	// Guarda la conversion dependiendo el canal
	if(!(ADMUX & (1 << MUX0))){
        T = ADC;                   // ADC0
    }
	else{
        V = ADC;                   // ADC1
    }

	// Togglea el canal
    ADMUX ^= (1 << MUX0);
}

// Funciones
// ----------------------------------------------------------------------------------------
// Configuracion e Inicializacion de puertos
void initPorts(){
	DDRA = 0x3F;       // PA0, PA1, PA2, PA3, PA4, PA5 como salida
	PORTA = 0x00;      // Inicializa el puerto A

	DDRF = 0x00;	   // Puerto F todo como entrada (para conversor AD)

	DDRG = 0x20;       // Pin PG5 como salida para OC0B (senal PWM)

	// Se configura a PD0, PD1 y PD2 como puertos de entrada con resistencia pull-up internas:
	PORTD = (1 << P1) | (1 << P2) | (1 << P3);
}

// Secuencia de arranque
void boot(){
	// Secuencia de Arranque
	}

// Inicializacion de interrupciones externas
void initExternalInterrupts(){
	// Habilitacion de interrupciones externas:
	EICRA |= (1 << ISC01) | (1 << ISC11) | (1 << ISC21);	// Configura INT0, INT1 e INT2 sensible a flanco desc.
	EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2);	    // Habilita INT0, INT1 e INT2
	EIFR = 0x00;
	//sei();							                    // Habilita las interrup. globalmente.
}

// Inicializacion del Timer2 como temporizador (10ms)
void initTimer2(){
	TCCR2A |= (1 << WGM01);	                  // Modo CTC.
	TCCR2B |= ((1 << CS02) | (1 << CS00));    // Prescaler N = 1024.
	TIMSK2 = (1 << OCIE2A);                   // 0x02; Habilita interrupcion por igualacion.
	OCR2A = 155;				              // Carga el valor de TOPE (155).

	TIFR0 = 0x00;                             // Borra flags de interrupciones del Timer 2.
}

// Inicializacion del Timer0 como Fast PWM
void initTimer0(){
	// Configura como FastPWM con TOP OCR0A
	TCCR0A |= ((1 << WGM01) | (1 << WGM00));
	TCCR0A |= (1 << WGM02);

	// Modo PWM no invertido (VERIFICAR)
	TCCR0A |= (1 << COM0B1);                  // (El pin OC0B se borra cuando TCNT0 iguala a OCR0A y se settea cuando iguala a BOTTOM)

	// Seleccion de la señal del reloj (N=64)
	TCCR0B |= ((1 << CS01) | (1 << CS00));

	OCR0A = TOP;                              // Para frecuencia de 2kHz

	// USAR TIMSK0 ACA???
	TIFR0 = 0x00;                             // Borra flags de interrupciones del Timer 0.
}

// Configuracion del conversor Analogico-Digital
void initADConverter(){
	// Desconecta la parte digital del pin ADC0/PF0 y ADC1/PF1.
	DIDR0 |= ((1 << ADC0D) | (1 << ADC1D))

	// Config. la ref. de tension tomada del pin AVCC (placa Arduino AVCC = Vcc = 5V).
	// Conversion AD de 10 bits (ADLAR = 0) y con el Multiplexor selecciona canal 0 (ADC0/PF0).
	ADMUX |= (1 << REFS0);

	// Modo Free Running, ACME=0 y MUX5=0
	ADCSRB = 0x00;

	// Habilita auto triggering (ADATE = 1), interrupcion por conversion (ADIE = 1) y prescaler en 128
	ADCSRA |= ((1 << ADATE) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2));
}

// Modo Configuracion (seleccion de tiempo y velocidad)
void config(){
	// Inicia conversion
	ADCSRA |= ((1 << ADEN) | (1 << ADSC));

	configTV(FlagP3);

    // Detiene conversion
	ADCSRA &= ~(1 << ADEN);
}

// Asignacion de T1/V1 o T2/V2
void configTV(int op){
	// Si op = 1 --> actualiza T1 y V1
	// Si op = 0 --> actualiza T2 y V2
	if(op){
		T1 = 1000 + (T * 1000/1023);      // De 10" a 20" (1000 a 2000, para operar con TIME)
		V1 = 40 + (V * 55/1023);          // De 40% a 95%
	}
	else{
		T2 = 1000 + (T * 1000/1023);      // De 10" a 20" (1000 a 2000, para operar con TIME)
		V2 = 40 + (V * 55/1023);          // De 40% a 95%
	}
}

// Modo normal (funcionamiento del motor)
void normal(){
	// Modo de operacion normal
	// Si FlagP1=1 (encender motor):
	//      1) Genera señal PWM con ciclo util V1 durante T1
	//      2) Genera señal PWM con ciclo util V2 durante T2 (actualiza OCR2B)
	// Si FlagP1=0 apagar.

	/*
	TIME = 0;

	while(TIME < T1){

	}
	*/
}

// Visualizar datos en display LCD
void lcd(){
	// codigo para mostrar en LCD
}

/*
Problemas a solucionar:
1) Conversion ADC de las tensiones en los potenciometros RV1 y RV2
2) Generacion de las señales PWM (V1 durante T1, V2 durante T2)
3) Mostrar datos en LCD

Controlar con qué unidades conviene guardar los tiempos

*/
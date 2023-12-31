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
// Pines usados por la librer�a lcd_2560.h:
#define RS	eS_PORTA0			// Pin RS = PA0 (22) (Reset).
#define EN	eS_PORTA1			// Pin EN = PA1 (23) (Enable).
#define D4	eS_PORTA2			// Pin D4 = PA2 (24) (Data D4).
#define D5	eS_PORTA3			// Pin D5 = PA3 (25) (Data D5).
#define D6	eS_PORTA4			// Pin D6 = PA4 (26) (Data D6).
#define D7	eS_PORTA5			// Pin D7 = PA5 (27) (Data D7).
#define LED_G     PA6           // Se conecta al LED verde para indicar FlagP1 = 1 (encendido del motor en modo normal)
#define LED_R     PA7           // Se conecta al LED rojo para indicar FlagP1 = 0 (apagado del motor en modo normal)
#define PORT_PWM  PG5           // Puerto PWM

// Macros de usuario
// -------------------------------------------------------------------
#define T_PWM 124                               // Con N=64 --> T = 1/2k en la senal PWM
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
volatile int FlagP1 = 0; 							// FlagP1 = 0 -> Apagar Motor
volatile int FlagP2 = 1; 							// FlagP2 = 1 -> Modo Configuracion
volatile int FlagP3 = 1;						    // FlagP3 = 1 -> Configuracion T1V1
volatile int CLK = 0;			                    // Variable contador para el Timer 2
volatile int RV1=512;                               // Lectura del potenciometro RV1 (Tiempo)       (0 a 1023)
volatile int RV2=1023;                              // Lectura del potenciometro RV2 (Velocidad)    (0 a 1023)
volatile int motorOn = 0;                           // Estado del motor
volatile int dutyCycle;                             // Ciclo util de la senal PWM
int T1=10;                                          // Tiempo etapa 1
int V1=95;                                          // Velocidad etapa 1
int T2=10;                                          // Tiempo etapa 2
int V2=40;                                          // Velocidad etapa 2
char buffer[8];                                     // Para cargar en lcd
int interface;                                      // 0 (normal), 1 (config1), 2 (config2)
int update;                                         // indica si es necesario actualizar el lcd

int lastT1=10, lastT2=10, lastV1=95, lastV2=40;

// Declararacion de funciones
// -------------------------------------------------------------------
void initPorts();
void boot();
void initExternalInterrupts();
void initTimer0();               // Inicializacion del Timer0 como Fast PWM
void initTimer1();               // Inicializacion del Timer1 como clock (100ms)
void initADConverter();
void config();
void configTV(int op);
void normal();
void turnOnPWM();
void turnOffPWM();
void lcd();
void interfaceNormal();
void interfaceConfig1();
void interfaceConfig2();
void ledIndicateP1();
int calDutyCycle(int V);

// Programa
// -------------------------------------------------------------------
int main(void){

	initPorts();
	boot();
	initExternalInterrupts();
	initTimer1();
	initTimer0();
	initADConverter();
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
		ledIndicateP1();
	}
}

// Interrupciones
// ----------------------------------------------------------------------------------------
ISR(INT0_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P2)){              // Comprueba si P2 sigue en BAJO
		tbi(FlagP2,0);                 // Establecer la bandera para indicar la interrupcion
	}
}

ISR(INT1_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P1)){              // Comprueba si P1 sigue en BAJO
		tbi(FlagP1,0);                 // Establecer la bandera para indicar la interrupcion
	}
}

ISR(INT2_vect){
	_delay_ms(BOUNCE_DELAY);
	if (is_low(PIND,P3) && FlagP2){    // Comprueba si P3 sigue en BAJO y está en cnf()
		tbi(FlagP3,0);                 // Establecer la bandera para indicar la interrupcion
	}
}

ISR(TIMER0_COMPA_vect){
	// Actualiza Ciclo util
	OCR0B = dutyCycle;
}

ISR(TIMER1_COMPA_vect) {
    CLK++;
}

ISR(ADC_vect){
	// Guarda la conversion dependiendo el canal
	if(!(ADMUX & (1 << MUX0))){
        RV1 = ADC;                   // ADC0
    }
	else{
        RV2 = ADC;                   // ADC1
    }

	// Togglea el canal
    ADMUX ^= (1 << MUX0);
}

// Funciones
// ----------------------------------------------------------------------------------------
// Configuracion e Inicializacion de puertos
void initPorts(){
	DDRA = 0xFF;       // Puerto A como salida
	PORTA = 0x00;      // Inicializa el puerto A

	DDRF = 0x00;	   // Puerto F todo como entrada (para conversor AD)

	DDRG = (1 << PORT_PWM);       // Pin PG5 como salida para OC0B (senal PWM)

	// Se configura a PD0, PD1 y PD2 como puertos de entrada con resistencia pull-up internas:
	PORTD = (1 << P1) | (1 << P2) | (1 << P3);
}

// Secuencia de arranque
void boot(){
	Lcd4_Init();
	Lcd4_Clear();

	sprintf(buffer, "Init...");
	Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
	Lcd4_Write_String(buffer);									// Escribe string

	PORTA |= (1 << LED_G) | (1 << LED_R);                       // Enciende los LEDs

	_delay_ms(3000);

	Lcd4_Clear();                                               // Limpia el display
	PORTA &= ~((1 << LED_G) | (1 << LED_R));                    // Apaga los LEDs
}

// Inicializacion de interrupciones externas
void initExternalInterrupts(){
	EICRA |= (1 << ISC01) | (1 << ISC11) | (1 << ISC21);	// Configura INT0, INT1 e INT2 sensible a flanco desc.
	EIMSK |= (1 << INT0) | (1 << INT1) | (1 << INT2);	    // Habilita INT0, INT1 e INT2
	EIFR = 0x00;
	//sei();							                    // Habilita las interrup. globalmente.
}

// Inicializacion del Timer0 como Fast PWM
void initTimer0(){
	TCCR0A = 0x00;
	TCCR0B = 0x00;

	// Configura como FastPWM con TOP OCR0A
	TCCR0A |= ((1 << WGM01) | (1 << WGM00));
	TCCR0B |= (1 << WGM02);

	// Modo PWM invertido
	TCCR0A |= (1 << COM0B1) | (1 << COM0B0);

	OCR0A = T_PWM;                              // Seleccion de tope para frecuencia de 2kHz

	TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));   // Prescaler en 0

	TIMSK0 |= (1 << OCIE0A);                    // Habilita interrupcion por comparacion con OCR0A
	TIFR0 = 0x00;                               // Borra flags de interrupciones del Timer 0.
}

// Inicializacion del Timer1 como temporizador (100ms)
void initTimer1(){
	TCCR1B |= (1 << CS11) | (1 << CS10);   // Configura prescaler de 256
	TCCR1B |= (1 << WGM12);                // Modo CTC
	OCR1A = 24999;                         // Valor de comparación para 100
	TIMSK1 |= (1 << OCIE1A);               // Habilita la interrupción por comparacion con OCR1A
}

// Configuracion del conversor Analogico-Digital
void initADConverter(){
	// Desconecta la parte digital del pin ADC0/PF0 y ADC1/PF1.
	DIDR0 |= ((1 << ADC0D) | (1 << ADC1D));

	// Config. la ref. de tension tomada del pin AVCC (placa Arduino AVCC = Vcc = 5V).
	// Conversion AD de 10 bits (ADLAR = 0) y con el Multiplexor selecciona canal 0 (ADC0/PF0).
	ADMUX |= (1 << REFS0);

	// Modo Free Running, ACME=0 y MUX5=0
	ADCSRB = 0x00;

	// Habilita interrupcion por conversion (ADIE = 1) y prescaler en 128
	ADCSRA |= ((1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2));

	// Habilita ADC (ADEN = 1)
	ADCSRA |= (1 << ADEN);
}

// Modo Configuracion (seleccion de tiempo y velocidad)
void config(){
	turnOffPWM();                               // APAGAR PWM

	ADCSRA |= (1 << ADSC);                    // Inicia la conversion

	configTV(FlagP3);                           // Configura T1/V1 o T2/V2
}

// Asignacion de T1/V1 o T2/V2
void configTV(int op){
	// Si op = 1 --> Actualiza T1 y V1
	// Si op = 0 --> Actualiza T2 y V2
	if(op){
		T1 = (int)(10 + (RV1 * 0.01));         // De 10" a 20"   /  0.01 = 10/1023
		V1 = (int)(40 + (RV2 * 0.05));         // De 40% a 95%    /  0.05 = 55/1023

		interface=1;
	}
	else{
		T2 = (int)(10 + (RV1 * 0.01));         // De 10" a 20"   /  0.01 = 10/1023
		V2 = (int)(40 + (RV2 * 0.05));         // De 40% a 95%    /  0.05 = 55/1023

		interface=2;
	}
	update=1;
}

// Modo normal (funcionamiento del motor)
void normal(){

	if(FlagP1){                                 // ENCENDER PWM
		if(motorOn){                            // Ya estaba encendido -> debe mantener el funcionamiento alternante
										        // Identifico etapa y actualizo Ciclo Util (si es necesario)
			if(CLK < 10*T1){                   // Etapa 1 y es necesario actualizar Ciclo Util
				dutyCycle = calDutyCycle(V1);   // Actualiza Ciclo Util con V1
			}
			else{                               // Etapa 2
				dutyCycle = calDutyCycle(V2);   // Actualiza Ciclo Util con V2
			}

			if(CLK > 10 * (T1 + T2)){          // Si el clock supera a 100*(T1+T2), se reinicia
				CLK = 0;
			}
		}
		else{                                   // No estaba encendido -> Asignar Ciclo Util y Encender
			CLK = 0;                            // Reinicio el clock
			dutyCycle = calDutyCycle(V1);       // Asigna Ciclo Util con V1
			turnOnPWM();                        // Enciende PWM
		}
	}
	else{                                       // APAGAR PWM
		turnOffPWM();
	}

	if(interface!=0){
		update=1;
		interface=0;
	}
}

// Enciende el temporizador Timer0 (Fast PWM)
void turnOnPWM(){
	TCCR0A |= (1 << COM0B1) | (1 << COM0B0);
	// Seleccion de la señal del reloj (N=64)
	TCCR0B |= ((1 << CS01) | (1 << CS00));
	motorOn = 1;
}

// Detiene el temporizador Timer0 (Fast PWM)
void turnOffPWM(){
	TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));
	// Prescaler = 0
	TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
	TCNT0 = 0;
	motorOn = 0;
}

// Visualizar datos en display LCD
void lcd(){
	// interface toma 3 valores durante la ejecucion del main
	//     * interface = 0 --> Modo normal
	//     * interface = 1 --> Modo configuracion 1
	//     * interface = 2 --> Modo configuracion 2
	// Con un switch, se seleccionará qué mostrar en el LCD

	if(update){
		Lcd4_Clear();

		switch(interface){
			case 0:
				interfaceNormal();
				break;

			case 1:
				interfaceConfig1();
				break;

			case 2:
				interfaceConfig2();
				break;

			default:
				break;
		}
		update=0;
	}
}

// Mostrar interface Normal en LCD
void interfaceNormal(){
	sprintf(buffer, "%is %i%%", T1, V1);
	Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
	Lcd4_Write_String(buffer);									// Escribe string

	sprintf(buffer, "%is %i%%", T2, V2);
	Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 2, columna 0
	Lcd4_Write_String(buffer);
}

// Mostrar interface configuracion etapa 1
void interfaceConfig1(){
	sprintf(buffer, "CONFIG1");
	Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
	Lcd4_Write_String(buffer);									// Escribe string

	sprintf(buffer, "%is %i%%", T1, V1);
	Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 2, columna 0
	Lcd4_Write_String(buffer);

	lastT1 = T1;
	lastV1 = V1;
	_delay_ms(300);
}

// Mostrar interface configuracion etapa 2
void interfaceConfig2(){
	sprintf(buffer, "CONFIG2");
	Lcd4_Set_Cursor(1,0);										// Posiciona cursor en fila 1, columna 0
	Lcd4_Write_String(buffer);									// Escribe string

	sprintf(buffer, "%is %i%%", T2, V2);
	Lcd4_Set_Cursor(2,0);										// Posiciona cursor en fila 2, columna 0
	Lcd4_Write_String(buffer);

	lastT2 = T2;
	lastV2 = V2;
	_delay_ms(300);
}

// Independientemente del modo, indica la seleccion del pulsador P1 (encender / apagar motor)
void ledIndicateP1(){
	if(FlagP1){
		PORTA |= (1 << LED_G);         // Enciende LED verde
		PORTA &= ~(1 << LED_R);        // Apaga LED rojo
	}
	else{
		PORTA |= (1 << LED_R);         // Enciende LED rojo
		PORTA &= ~(1 << LED_G);        // Apaga LED verde
	}
}

// Calculo del ciclo util
int calDutyCycle(int V){
	// Calculo a realizar: (100 - V) * (125/100)
	// (100 - V) -> porque V va de 40 a 95, y el PWM trabaja en modo invertido
	// (125 / 100) -> 125 ciclos del PWM representan el 100% del ciclo util
	return((int)(125 - 1.25*V));
}

/*
Cuestiones a solucionar:
    * Implementar de manera más elegante las interfaces
	* Cambiar la temporizacion del clock (10 ms genera un precision innecesaria)
*/
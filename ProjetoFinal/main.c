/*
 * Author: Eloisa Bento Sarmento
		   Pedro Henrique Fernandes Monteiro	 
 */ 

#define F_CPU 16000000UL // Frequ�ncia de opera��o do micro

// --------- Defini��es de bibliotecas ---------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// --------- Defini��es de macros ---------
#define set_bit(adress,bit) (adress|=(1<<bit))
#define tst_bit(Y,bit_x) (Y&(1<<bit_x)) 
#define TOP 39999 // Valor para a m�xima contagem
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define tam_vetor 3

// --------- Prot�tipo das fun��es ---------
void alcoolTrigger(float maxAngle, float minAngle);
void blink();
void int2string(unsigned int valor, unsigned char *disp);
void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
unsigned char USART_Receive(void);

// --------- Vari�veis globais --------- 
uint8_t i, cycles, aux = 0, counter = 0;
unsigned char level[5];
unsigned char activations[tam_vetor];

ISR(ANALOG_COMP_vect)
{ 
	if(!tst_bit(ACSR,ACO)) // Verifica qual mudan�a ocorreu na sa�da do comparador
	{
		USART_Transmit(' ');
		USART_Transmit('N');
		USART_Transmit('i');
		USART_Transmit('v');
		USART_Transmit('e');
		USART_Transmit('l');
		USART_Transmit(' ');
		USART_Transmit('b');
		USART_Transmit('a');
		USART_Transmit('i');
		USART_Transmit('x');
		USART_Transmit('o');
		USART_Transmit(' ');
	}
}

ISR(USART_RX_vect)
{	
	char received;
	received = UDR0;
	
	if(received == 'q')
	{
		USART_Transmit(' ');
		USART_Transmit(level[0]);
		USART_Transmit(level[1]);
		USART_Transmit(level[2]);
		USART_Transmit(level[3]);
		USART_Transmit(level[4]);
		USART_Transmit(' ');
	}
	else if(received == 'a')
	{
		USART_Transmit(' ');
		USART_Transmit(activations[0]);
		USART_Transmit(activations[1]);
		USART_Transmit(activations[2]);
		USART_Transmit(' ');
	}	
}

// --------- Interrup��o para piscar LED de acordo com o n�vel de �lcool selecionado ---------
ISR(INT0_vect)
{
	if(aux >= 3) 
	{
		aux = 0;
	}
	
	aux++;
	
	if(aux == 1) 
	{
		cycles = 1;
		level[0] = 'B';
		level[1] = 'a';
		level[2] = 'i';
		level[3] = 'x';
		level[4] = 'a';
	}
	else if(aux == 2)
	{
		cycles = 2;
		level[0] = 'M';
		level[1] = 'e';
		level[2] = 'd';
		level[3] = 'i';
		level[4] = 'a';
	}	
	else if(aux == 3) 
	{
		cycles = 3;
		level[0] = 'A';
		level[1] = 'l';
		level[2] = 't';
		level[3] = 'a';
		level[4] = ' ';
	}
	
	i = 0;
}

// --------- Interrup��o para piscar LED e ativar servo caso algum obst�culo seja detectado pelo sensor ---------
ISR(INT1_vect)
{
	PORTC = 0b00000001;
	_delay_ms(300);
	PORTC = 0b00000000;
	_delay_ms(150);
	
	counter++;
	
	int2string(counter, activations);
	
	if(aux == 1) // Se m�nimo est� selecionado
	{
		alcoolTrigger(110, 90);
	}
	else if(aux == 2) // Se m�dio est� selecionado
	{
		alcoolTrigger(130, 90);
	}
	else if(aux == 3) // Se m�ximo est� selecionado
	{
		alcoolTrigger(150, 90);
	}		
}

// --------- Fun��o aciona o servo motor de acordo com o �ngulo fornecido, tamb�m aciona o LED  ---------
void alcoolTrigger(float maxAngle, float minAngle)
{
		float pulseWidthMin;
		float pulseWidthMax;
		float minimumPulseWidth = 1;
		float maximumPulseWidth = 2;
		
		pulseWidthMin = (minimumPulseWidth + (minAngle*((maximumPulseWidth - minimumPulseWidth)/181)));
		pulseWidthMax = (minimumPulseWidth + (maxAngle*((maximumPulseWidth - minimumPulseWidth)/181)));

		OCR1A =	(pulseWidthMax*40000)/20; // Regra de tr�s para determinar este valor
		_delay_ms(2000);
		OCR1A =	(pulseWidthMin*40000)/20; // Regra de tr�s para determinar este valor
}

// --------- Fun��o para piscar LED de acordo com o n�vel de �lcool selecionado ---------
void blink()
{
	if(i < cycles)
	{
		PORTC = 0b00000100;
		_delay_ms(300);
		PORTC = 0b00000000;
		_delay_ms(150);
		i++;
	}
}

void int2string(unsigned int valor, unsigned char *disp)
{
	for(uint8_t n=0; n<tam_vetor; n++)
	disp[n] = 0 + 48;
	disp += (tam_vetor-1);
	do
	{
		*disp = (valor%10) + 48;
		valor /= 10;
		disp--;
	}while (valor!=0);
}

void USART_Init(unsigned int ubrr)
{
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	
	DDRC = 0xFF;
}

void USART_Transmit(unsigned char data)
{
	while(!( UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

unsigned char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

int main(void)
{
	// USART
	USART_Init(MYUBRR); // Inicia USART
	
	//GPIO
	DDRD =	0b00110011;	/* Pino da porta D2 como entrada (Bot�o confg. n�vel de �lcool)
						   Pino da porta D3 como entrada (Bot�o receptor da luz refletida pela presen�a da m�o)
						   Pino da porta D5 como sa�da (LED Sensor)
						   Portas do comparador como entrada (D6 e D7) */
	PORTD = 0b11001100;	// Pull-ups das portas D2, D3, D6 e D7 habilitados
	DDRC = 0b00000101;  // Pinos dos LEDs de n�vel (PC2) e detec��o de obst�culo (PC0) como sa�das
	DDRB = 0b00000010;  // Somente pino do servo como sa�da PB1 
	
	// LED e Sensor
	EICRA = 0b00001010; // Interrup��o externas INT0 e INT1 na borda de descida
	EIMSK = 0b00000011; // Habilita as interrup��es externas INT0 e INT1
	
	// Servo
	ICR1 = TOP;         // Configura o per�odo do PWM (20 ms), TOP = (F_CPU/(N*F_PWM))-1, com N = 8 e F_PWM = 50 Hz
	TCCR1A = (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1<<WGM12) | (1 << CS11);
	set_bit(TCCR1A,COM1A1);
	set_bit(TCCR1A,COM1B1);
	OCR1A =	3000;		
	
	// N�vel de �lcool no recipiente	
	DIDR1 = 0b00000011; // Desabilita as entradas digitais nos pinos AIN0 e AIN1
	ACSR = 1<<ACIE;     // Habilita interrup. por mudan�a de estado na sa�da do comparador
	sei();              // Habilita a chave geral de interrup��es
	
	
	while(1) 
    {
		blink();
    }
}



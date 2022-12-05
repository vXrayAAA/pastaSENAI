/*
 * File:   aula_ADC_copia.c
 * Author: 55366771870
 *
 * Created on 6 de Setembro de 2022, 08:55
 */



#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#define _XTAL_FREQ  20000000

#define saida PORTB
#define rs    PORTBbits.RB4
#define en    PORTBbits.RB5

#define C1 PORTCbits.RC2
#define C2 PORTCbits.RC6
#define C3 PORTCbits.RC1
#define L1 PORTCbits.RC5
#define L2 PORTCbits.RC4
#define L3 PORTCbits.RC0
#define L4 PORTCbits.RC3

#include <xc.h>

void lcd4bits(unsigned char valor,unsigned char pino)
{
    saida = (valor >> 4);
    rs = pino;
    en = 1, en = 0;
    __delay_ms(5);
    
    saida = valor;
    rs = pino;
    en = 1, en = 0;
    __delay_ms(5);
}

void lcdIniciar(void)
{
    lcd4bits(0X02,0);
    lcd4bits(0X28,0);   
    lcd4bits(0X0E,0);
    lcd4bits(0X01,0);
}

void lcdPos(unsigned char lin, unsigned char col)
{ 
    if(lin==1)lcd4bits(127+col,0);
    if(lin==2)lcd4bits(191+col,0);
}

void lcdTexto(unsigned char *palavra)
{
    while(*palavra)
    {
      lcd4bits(*palavra,1);
      palavra++;
    }    
}

void picIniciar(void)
{
    TRISB = 0X00;
    PORTB = 0X00;
    TRISC = 0X39;
    PORTC = 0X00;
    TRISD = 0XF0;
    PORTD = 0X00;
    ANSELH = 0X00;
}

void adcIniciarCanal(unsigned char numCanal)	// Inicializa o ADC com numero de canal(ais) (0 a 15). Opcao em '0' entradas de TRISA modo digital.
{
	/* Atencao:
	 * Se 'numCanal = 0' entradas de TRISA modo digital
	 * Não é possivel escolher somente: 2, 3, 4 canais ou 7 canais sequenciais.
	 */
    #define bit1(valor,bit) valor |= (1<<bit)
	unsigned char adTmpLac=0;
	TRISA=0;
	if(numCanal > 13)numCanal = 13;
    PIR1bits.ADIF = 0;	// Limpa Flag da interrupcao do modulo.
    PIE1bits.ADIE = 0;	// Desliga a interrupcao do ADC.
    if(numCanal > 7)numCanal = 7;
		// canal em uso:     0    1    5    5    5    5    6    8    8
	unsigned char adcTab[]={0x86,0x8E,0x82,0x82,0x82,0x82,0x89,0x80,0x80};
	do
	{
		bit1(TRISA,adTmpLac);
		adTmpLac++;
	}while(adTmpLac<numCanal);
	ADCON1 = adcTab[numCanal];
	PORTA=0x00;                                 // Limpa PORT.
    __delay_ms(35);                             // Aguarda estabilizar...
}

unsigned int adcCanal(unsigned char vlrCanal)	// Seleciona canal (0 a 15) e retorna com valor (INT) (0 a 1023).
{
    unsigned int vlrTmp;						// Variável local temporária.
	ADCON0 = (vlrCanal<<2) | ADCON0;            // Posiciona o valor do canal no registro.
    ADCON0bits.ADON = 1;						// Habilita módulo.
    __delay_ms(1);								// Aguarda...
    ADCON0bits.GO = 1;							// Inicia conversão.
    while(ADCON0bits.GO);						// Aguarda terminar a conversão.
    vlrTmp = (ADRESH<<8) | ADRESL;				// Unindo as partes para compor o valor.
    return (vlrTmp);							// Retorna com o valor bruto.
}

void contador(unsigned char lin, unsigned col, unsigned int valor)
{
    unsigned char milhar,centena,dezena,unidade;
    
    lcdPos(lin,col);
    
    milhar = (valor /1000)+0X30;
    centena= (valor/100)%10+0X30;
    dezena = (valor/10)%10+0X30;
    unidade =(valor % 10) +0X30;
    
    // lcdPos(2,1);
    
    lcd4bits(milhar,1);
    lcd4bits(centena ,1);
    lcd4bits(dezena,1);
    lcd4bits(unidade,1);
}

unsigned char teclado(void)
{
    unsigned char tecla;
    
  C1=1,C2=0,C3=0;
  if(L1) tecla='1';
  if(L2) tecla='4';
  if(L3) tecla='7';
  if(L4) tecla='*';
  
  C1=0,C2=1,C3=0;
  if(L1) tecla='2';
  if(L2) tecla='5';
  if(L3) tecla='8';
  if(L4) tecla='0';
  
  C1=0,C2=0,C3=1;
  if(L1) tecla='3';
  if(L2) tecla='6';
  if(L3) tecla='9';
  if(L4) tecla='#';
  
  return(tecla);
}

void main(void)
{
    unsigned int valorBin;
    unsigned char celsius;
    unsigned char fah;
    unsigned int kelvin;
    picIniciar();
    lcdIniciar();
    adcIniciarCanal(1);
    lcdPos(1,1);
    lcdTexto("nik");
    while(1)
    {
        valorBin = adcCanal(0);
//        contador(2,1,valorBin); 
        
        celsius = valorBin * 0.4848;
        contador(1,9,celsius);
        lcdTexto("\xDF C");
        __delay_ms(500);
        
        fah = celsius*1.8+32;
        contador(2,1,fah);
        lcdTexto("\xDF F");
        __delay_ms(500);
        
        kelvin = celsius+273.15;
        contador(2,9,kelvin);
        lcdTexto("  K");
        __delay_ms(500);
    }
}


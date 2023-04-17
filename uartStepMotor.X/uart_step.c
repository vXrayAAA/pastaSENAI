/*
 * File:   uart_step.c
 * Author: 47872216809
 *
 * Created on 17 de Abril de 2023, 15:10
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


#include <xc.h>

#define _XTAL_FREQ   20000000

#define saida   PORTB
#define rs      PORTBbits.RB4
#define en      PORTBbits.RB5

#define C1      PORTCbits.RC2
#define C2      PORTCbits.RC6
#define C3      PORTCbits.RC1
#define C4      PORTCbits.RC7
#define L1      PORTCbits.RC5
#define L2      PORTCbits.RC4
#define L3      PORTCbits.RC0
#define L4      PORTCbits.RC3

#define saida1  PORTDbits.RD0
#define saida2  PORTDbits.RD1
#define saida3  PORTDbits.RD2

void lcd4Bits(unsigned char valor, unsigned char pino)
{
    PORTB = (valor >> 4);
    PORTBbits.RB4 = pino;
    PORTBbits.RB5 = 1, PORTBbits.RB5 = 0;
    __delay_ms(5);

    PORTB = valor;
    PORTBbits.RB4 = pino;
    PORTBbits.RB5 = 1, PORTBbits.RB5 = 0;
    __delay_ms(5);
}

void lcdIniciar(void)
{
    lcd4Bits(0x02,0);
    lcd4Bits(0x28,0);
    lcd4Bits(0x0E,0);
    lcd4Bits(0x01,0);
}

void lcdPos(unsigned char lin,unsigned char col)
{
    if(lin==1)lcd4Bits(127+col,0);
    if(lin==2)lcd4Bits(191+col,0);
}

void lcdTexto(char *palavra)
{
    while(*palavra)
    {
        lcd4Bits(*palavra,1);
        palavra++;
    }
}

void picIniciar(void)
{
    TRISB = 0x00;
    PORTB = 0x00;
    TRISC = 0x39;
    PORTC = 0x00;
    TRISD = 0xF0;
    PORTD = 0x00;
    ANSELH= 0x00;
}

void adcIniciarCanal(unsigned char numCanal)
{



 unsigned char adTmpLac=0;
 TRISA=0;
 if(numCanal > 13)numCanal = 13;
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 0;
    if(numCanal > 7)numCanal = 7;

 unsigned char adcTab[]={0x86,0x8E,0x82,0x82,0x82,0x82,0x89,0x80,0x80};
 do
 {
  TRISA |= (1<<adTmpLac);
  adTmpLac++;
 }while(adTmpLac<numCanal);
 ADCON1 = adcTab[numCanal];
 PORTA=0x00;
    __delay_ms(5);
}

unsigned int adcCanal(unsigned char vlrCanal)
{
    unsigned int vlrTmp;
 ADCON0 = (vlrCanal<<2);
    ADCON0bits.ADON = 1;
    __delay_ms(35);
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO);
    vlrTmp = (ADRESH<<8) | ADRESL;
    return (vlrTmp);
}

void contador(unsigned char lin, unsigned char col, unsigned int valor)
{
    unsigned char unidade, dezena, centena, milhar;

    lcdPos(lin,col);

    milhar = (valor / 1000) + 0x30;
    centena = (valor / 100) % 10 + 0x30;
    dezena = (valor / 10) % 10 + 0x30;
    unidade = (valor % 10) + 0x30;

    if(milhar =='0') milhar=' ';
    if(centena=='0' && milhar==' ') centena=' ';
    if(dezena =='0' && centena==' ') dezena=' ';

    lcd4Bits(milhar ,1);
    lcd4Bits(centena,1);
    lcd4Bits(dezena ,1);
    lcd4Bits(unidade,1);
}

//unsigned char teclado(void)
//{
//    unsigned char tecla;
//    PORTCbits.RC2=1,PORTCbits.RC6=0,PORTCbits.RC1=0,PORTCbits.RC7=0;
//    if(PORTCbits.RC5) tecla='7';
//    if(PORTCbits.RC4) tecla='4';
//    if(PORTCbits.RC0) tecla='1';
//    if(PORTCbits.RC3) tecla='*';
//
//    PORTCbits.RC2=0,PORTCbits.RC6=1,PORTCbits.RC1=0,PORTCbits.RC7=0;
//    if(PORTCbits.RC5) tecla='8';
//    if(PORTCbits.RC4) tecla='5';
//    if(PORTCbits.RC0) tecla='2';
//    if(PORTCbits.RC3) tecla='0';
//
//    PORTCbits.RC2=0,PORTCbits.RC6=0,PORTCbits.RC1=1,PORTCbits.RC7=0;
//    if(PORTCbits.RC5) tecla='9';
//    if(PORTCbits.RC4) tecla='6';
//    if(PORTCbits.RC0) tecla='3';
//    if(PORTCbits.RC3) tecla='#';
//
//    PORTCbits.RC2=0,PORTCbits.RC6=0,PORTCbits.RC1=0,PORTCbits.RC7=1;
//    if(PORTCbits.RC5) tecla='A';
//    if(PORTCbits.RC4) tecla='B';
//    if(PORTCbits.RC0) tecla='C';
//    if(PORTCbits.RC3) tecla='D';
//
//    return (tecla);
//}

char celsius(unsigned int valorBin)
{
    unsigned char temp;
    temp = valorBin * 0.4878;
    return (temp);
}



unsigned char matEsq[]={0x01,0x02,0x04,0x08};
unsigned char matDir[]={0x08,0x04,0x02,0x01};


void motorPasso(unsigned int ang, char sentido)
{
    unsigned int contar; //var. local
    ang = ang * 5.6889;  //fator de conversão para transformar ângulos em passos do motor.
    for(contar=0;contar<ang;contar++)  //loop 
    {
        if(sentido)PORTD=matDir[contar%4];
        else PORTD=matEsq[contar%4];
        __delay_ms(10);
    }
}


void uartIniciar(void)
{
    SPBRG=31;
    TXSTAbits.SYNC=0;
    RCSTAbits.SPEN=1;
    TRISCbits.TRISC6=1;
    TRISCbits.TRISC7=1;
    RCSTAbits.CREN=1;
    TXSTAbits.TXEN=1;
}

void uartTXC(const char dado)
{
    TXREG = dado;
    while(!TXSTAbits.TRMT);
}

unsigned char uartRXC(void)
{
 unsigned char tmrOut=0, kTemp=0;
 unsigned char rxValor=0;
    if(PIR1bits.RCIF)
 {
  while(!PIR1bits.RCIF)
  {
   kTemp++;
   if (kTemp>100) break;
  }
  rxValor = RCREG;
 }

    return (rxValor);
}

void uartTXT(const char *texto)
{
    unsigned char i;
    for(i=0;texto[i]!='\0';i++)
    uartTXC(texto[i]);
}



void main(void)
{
    
    uartIniciar();
    uartTXT("hello,world!\r\n");

    unsigned int ang;
    unsigned int valorBin = 0;
    unsigned char valor, contar=0;

    uartTXT("digite o angulo:\r\n");

    while(contar<3)
    {
        valor = uartRXC();
        if(valor>='0' && valor <='9')
        {
            uartTXC(valor);
            if(contar==0)valorBin=(valor-'0')*100;
            if(contar==1)valorBin=valorBin+(valor-'0')*10;
            if(contar==2)valorBin=valorBin+(valor-'0');

            contar++;

        }

    }




    TRISD = 0,PORTD=0;
    uartTXT("\r\n");
    uartTXT("esq ou dir");
    contar = 0;

    uartTXT("\r\n");

    while(contar<1)
    {
        ang = uartRXC();

        if(ang>='0' && ang <='1')
        {
          uartTXC(ang);

          if(ang==0) valorBin=(ang - 0x30)*100;
          if(ang==1) motorPasso(valorBin,1);
          contar++;

          if(ang=='0')
          {
             uartTXT("motor esq \r\n");
             motorPasso(valorBin,0);
          }
          if(ang=='1')
          {
             uartTXT("motor dir \r\n");
             motorPasso(valorBin,1);
          }

        }

    }

}
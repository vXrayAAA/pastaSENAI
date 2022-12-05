/*
 * File:   robo_Somativa-05_Exemplo.c
 * Author: Fred_Dell
 * Somativa 05 - Exemplo de aplicacao
 * Created on 16 de Mar?o de 2022, 16:32
 */
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
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

#define _XTAL_FREQ	20000000

#include <xc.h>
#include "robo_v01.h"

/* Os valores dos passos devem ser feitos na planilha. */

void main(void)
{	
	unsigned int distancia;														// Usado somente se for medir distancia.
	
	roboIniciar();																// Obrigatorio: Inicializa o harware do robo.
	lcdIniciar();																// Obrigatorio: Inicializa o harware do LCD.
	lcdTexto("NLLRM",1,2);											// Mensagem de identificacao na 1? linha.
	
	while(1)
	{
		/* Funcao 'Start'. Serve para dar inicio. */
		lcdTexto("Standby.           ",3,1);									// Mensagem de aguardando acao.
		while(start) NOP();														// Aguarda ate acionar o botao na CPU.
		lcdTexto("Parado             ",3,1);									// Status do movimento na 3? linha.
		__delay_ms(1000);														// Aguarda 1s.
		
		/* A sequencia a seguir e para medir distancia.
		   Se nao houver necessidade de medir, apenas ignore este bloco. */
		/* O servo inicia em 0? e vai ate 330? */
		lcdTexto("Distancia: 0000mm. ",3,1);									// Mensagem padrao na 3? linha.
		distancia = roboMedirDistancia(1400);									// Posiciona o servo para frente com 1400us (165?) (consulte a planilha).
		converteASCII(distancia);												// Converter o valor em ASCII.
		lcdTexto(valorASCII,3,12);												// Posiciona e mostra o valor na 3? linha.
		__delay_ms(2000);														// Aguarda 2s.
		// Fim do bloco de medir distancia.
		
		lcdTexto("Em Frente 500mm    ",3,1);									// Status do movimento na 3? linha.
		roboPraFrente(1504);														// Robo desloca 940 passos(~500mm) para frente (consulte a planilha).
		__delay_ms(2000);														// Aguarda 2s.
		
        lcdTexto("Virar esq",3,1);
        roboPraEsquerda(-325);
        __delay_ms(2000);
        
        
        lcdTexto("Em Frente 500mm    ",3,1);									// Status do movimento na 3? linha.
		roboPraFrente(940);														// Robo desloca 940 passos(~500mm) para frente (consulte a planilha).
		__delay_ms(2000);				
        
        
        
		lcdTexto("Giro: +90\xDF      ",3,1);									// Status do movimento na 3? linha.
		roboPraDireita(325);													// Robo gira 325 passos(~173mm), ou seja, ~90? para direita.
		__delay_ms(2000);														// Aguarda 2s.
	}
}


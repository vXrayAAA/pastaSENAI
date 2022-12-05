#ifndef ROBO_V01_H
#define ROBO_V01_H

/* Motor de Passo do Robo */
#define roboPort	PORTD														// Hardware do robo.
#define roboCfg		TRISD														// Hardware do robo.
#define __tmrMp		10															// Tempo do motor de passo do robo.
#define start		PORTAbits.RA4												// Hardware do botao 'Start'.

/* ------- SERVO ------ */
/* Servo HS-485HB */
#define servoTonMin   400														// Valor minimo de TON em us.
#define servoTonMax  2400														// Valor maximo de TON em us.
#define servoCiclo  19500														// Valor do ciclo do servo em us.
#define servoTick     102														// Valor da interrupcao em us.

unsigned int servoVarTon=0;														// Tempo que vai ficar ligado.
unsigned int servoVarCiclo=0;													// O numero de vezes ate completar 20000.
unsigned int servoVarTick=0;													// Conta quantos tick's ja ocorreram.

/* ------- Parallax ------ */
#define echoCfg	    TRISAbits.TRISA0											// Configuracao do pino de sinal do modulo.
#define echoPino    PORTAbits.RA0												// Pino de entrada e saida de sinal do modulo.

unsigned int tempoEcho;															// 
unsigned int paralaxDistancia;													// 

#define servoCfg    TRISAbits.TRISA1											// 
#define servoPino   PORTAbits.RA1												// 

/* ------- LCD ------ */
#define saidaLCD	PORTB														// Alias do PORT de saida.
#define confgLCD	TRISB														// Alias de configuracao do PORT.
#define pinRS		PORTBbits.RB4												// Alias do pino 'RS'.
#define pinEN		PORTBbits.RB5												// Alias do pino 'E'.

/* ASCII */
unsigned char valorASCII[]={"0000"};											// Vari�vel Global para valor em ASCII.
void converteASCII(unsigned int valorNumero)									// Converte o valor 'int' em ASCII.
{
	unsigned char milhar;														// Vari�vel local para c�lculo.
    unsigned char centena;														// Vari�vel local para c�lculo.
    unsigned char dezena;														// Vari�vel local para c�lculo.
    unsigned char unidade;														// Vari�vel local para c�lculo.
    
	milhar = valorNumero/1000;													// Separa o milhar 
    centena = (valorNumero/100)%10;												// Separa a centena.
    dezena = (valorNumero/10)%10;												// Separa a dezena.
    unidade = valorNumero%10;													// Separa a unidade.

	valorASCII[3]=unidade+0x30;													// Salva valor na matriz em ASCII.
	valorASCII[2]=dezena+0x30;													// Salva valor na matriz em ASCII.
    valorASCII[1]=centena+0x30;													// Salva valor na matriz em ASCII.
    valorASCII[0]=milhar+0x30;													// Salva valor na matriz em ASCII.
}


/* ------- TIMERS ------ */
void timersIniciar(void)														// Rotina de inicializacao do hardware dos Timers.
{
    GIE=0;																		// Desliga todas as interrup��es (Global).
    INTCONbits.TMR0IE=0;														// Desliga a interrupcao por estouro em Timer 0.
    INTCONbits.TMR0IF=0;														// Limpa flag de interrupcao. 
    PIE1bits.TMR1IE=0;															// Desliga a interrupcao por estouro em Timer 1.
    PIR1bits.TMR1IF=0;															// Libera flag da interrupcao.
	OPTION_REG = 0x00;															// Clock interno e passa pelo prescaler 1:2.Incremento na borda de subida.
    T1CON=0x00;																	// Configuracao do Timer 1.
    TMR1L=0x00;																	// Carrega valor incial.
    TMR1H=0x00;																	// Carrega valor incial.
    GIE=1;																		// Liga interrupcao Global.
    INTCONbits.TMR0IE=1;														// Liga a interrupcao por estouro em Timer 0.
    PIE1bits.TMR1IE=1;															// Liga a interrupcao por estouro em Timer 1.
}

void __interrupt() interrupcao(void)											// Rotina da interrupcao.
{
    if(INTCONbits.TMR0IF)														// Se a interrupcao foi do Timer 0...
    {
		servoVarTick+=servoTick;												// Incrementa posicao do ciclo.
		if(servoVarTick <= servoVarTon)servoPino=1;								// Testa se esta ativo (TON)...
		else servoPino=0;														// ou inativo (TOFF).
		if(servoVarTick>=servoCiclo)servoVarTick=0;								// Testa status do ciclo.
		INTCONbits.TMR0IF=0;													// Limpa flag de interrupcao. 
    }
    if(PIR1bits.TMR1IF)															// Se a interrupcao foi do Timer 1...
    {
        TMR1L=0x00;																// Ajusta o valor inicial do Timer. 
		TMR1H=0x00;																// Ajusta o valor inicial do Timer.
		PIR1bits.TMR1IF=0;														// Libera flag da interrupcao.
    }
}

void parallaxIniciar(void)														// Rotina de inicializacao do Sensor Ultra-sonico
{
    echoCfg=0;																	// Pino como saida.
    echoPino=0;																	// Inicia o pino em '0'.
	ADCON0 = 0x00;																// Desliga as entradas analogicas.
    ADCON1 = 0x00;																// Desliga as entradas analogicas.
	ANSEL  = 0x00;																// Desliga as entradas analogicas.
	ANSELH = 0x00;																// Desliga as entradas analogicas.
}

void parallaxLer(void)															// Rotina de leitura do Sensor Ultra-sonico
{
    echoCfg = 0;																// Configura o pino como saida.
    echoPino = 0;																// Desativa o pino de sinal do modulo.
    __delay_us(5);																// Atraso, 2us min. 5us t�pico.
    echoPino = 1;																// Ativa o pino de sinal do modulo.
    __delay_us(5);																// Atraso, 2us min. 5us t�pico.
    echoPino = 0;																// Desativa o pino de sinal do modulo.
    echoCfg = 1;																// Configura o pino como entrada.
    while(!echoPino);															// Aguarda troca de nivel no pino (borda de subida).
    T1CONbits.TMR1ON=1;															// Na mudan�a de nivel, liga o Timer 1.
    while(echoPino);															// Aguarda troca de nivel no pino (borda de descida).
    T1CONbits.TMR1ON=0;															// Na mudan�a de nivel, desliga o Timer 1.
    
    /* Se:
     * distancia=tempo/5.8	-> Converte 'Tempo' em 'Distancia'. d=t(us)/5.882 (mm).
     * Distancia Ping: Max: 3000mm Min: 20mm.
     */
    tempoEcho=(TMR1H<<8)|TMR1L ;												// Combina as partes (alta e baixa) do Timer.
	paralaxDistancia=tempoEcho*0.0362068965;									// Converte 'Tempo' em 'Distancia'.

    TMR1L=0x00;																	// Ajusta o valor inicial do Timer. Parte Baixa.
    TMR1H=0x00;																	// Ajusta o valor inicial do Timer. Parte Alta.
}

void servoIniciar(void)															// Rotina de inicializacao do hardware do Servo.
{
    servoCfg=0;																	// Pino como saida.
    servoPino=0;																// Inicia o pino em '0'.
}

void __pEn(void)																// Gera pulso no pino 'E'
{
    pinEN=1;																	// Ativa o pino.
	__delay_us(500);
    pinEN=0;			// Gera borda de descida.
    __delay_ms(5);			// Aguarda estabilizar.
}

void __lcd4Bits(const char valor8Bits, const char valorRS)						// Converte 8bits em 2x 4bits
{
    saidaLCD=(valor8Bits & 0xF0)>>4;	// MSB na saida
    pinRS=valorRS;		// RS=0(Comando)|RS=1(Dado)
    __pEn();				// Gera pulso no pino 'E'
    saidaLCD=(valor8Bits & 0x0F);	// LSB na saida
    pinRS=valorRS;		// RS=0(Comando)|RS=1(Dado)
    __pEn();				// Gera pulso no pino 'E'
}

void lcdIniciar(void)															// Inicializa o hardware.
{
	__delay_ms(500);
	ANSELH=0x00;
    confgLCD=0x00;			// Configura entrada/saida dos pinos do PORT.
    saidaLCD=0x00;			// Limpa o PORT.
    __lcd4Bits(0x02,0);		// Cursor em Home.
    __lcd4Bits(0x28,0);		// Bus de 4bits e 2 linhas.
    __lcd4Bits(0x0C,0);		// Liga s� o display.
//    __lcd4Bits(0x0E,0);		// Liga display e cursor.
    __lcd4Bits(0x01,0);		// Limpa o LCD.
}

void __lcdPos(const char lcd01X, const char lcd01Y)								// Sub rotina para posicionar o cursor (linha e coluna).
{
    unsigned char lcd01Tmp;				// Variavel local tipo 'char'.
    lcd01Tmp = (unsigned)127 + lcd01Y;	// A soma define posicao da coluna e...
	/* 20x4 */
	if (lcd01X == 2) lcd01Tmp += 64;	// ...verifica se deve somar para a 2 linha.
    if (lcd01X == 3) lcd01Tmp += 20;	// ...verifica se deve somar para a 3 linha.
    if (lcd01X == 4) lcd01Tmp += 84;	// ...verifica se deve somar para a 4 linha.
    __lcd4Bits(lcd01Tmp,0);				// Chama sub rotina de comando (linha e coluna).
}

void lcdCaracter(const char lcd01Char, const char linha, const char coluna)		// Envia um caracter ao LCD.
{
	__lcdPos(linha,coluna);
	__lcd4Bits(lcd01Char,1);// Chama sub rotina para enviar dados.
}

void lcdTexto(const char *lcd01Texto, const char linha, const char coluna)		// Envia um texto(String) ao LCD.
{
	__lcdPos(linha,coluna);
    while(*lcd01Texto)			// Executa enquanto diferente de 'null'.
    {
		__lcd4Bits(*lcd01Texto,1);// Chama sub rotina para enviar dados.
		lcd01Texto++;			// Incrementa o ponteiro 'buffer'.
    }
}

/* ------- MOTOR ------ */
/* Motor de Passo
 * vlrX = Passos do motor 'X' (esquerdo)
 * sentX = Sentido de rotacao (0=horario e 1=antihorario)
 * vlrY	= Passos do motor 'Y' (direito)
 * sentY = Sentido de rotacao (0=horario e 1=antihorario)
*/
const unsigned char __biPol1E[] = {0x11,0x88,0x22,0x44};						// 4 fios
const unsigned char __biPol1D[] = {0x44,0x22,0x88,0x11};						// 4 fios
//const unsigned char __biPol2E[] = {0x11,0x99,0x88,0xAA,0x22,0x66,0x44,0x55};	// 4 fios
//const unsigned char __biPol2D[] = {0x55,0x44,0x66,0x22,0xAA,0x88,0x99,0x11};	// 4 fios

void mpXY(unsigned int vlrX, unsigned char sentX, unsigned int vlrY, unsigned char sentY)
{
	unsigned char flagX = 1, flagY = 1;											// Variavel do laco. Habilitados.
	unsigned char a, b;															// Variavel recupera valor da tabela.
	
	while (flagX | flagY)														// Laco se repete enquanto houver dado.
	{
		if (vlrX == 0)															// Se o valor do passo for zero...
		{
			a = 0;																// Desliga os pinos da saida.
			flagX = 0;															// Desabilita flag do motor.
		} else																	// ... sen�o:
		{
			if (sentX == 0) a = __biPol1E[vlrX % 4] & 0x0F;						// Se o sentido for horario: recupera e formata o dado da matriz.
			if (sentX == 1) a = __biPol1D[vlrX % 4] & 0x0F;						// Se o sentido for antihorario: recupera e formata o dado da matriz.
//			if (sentX == 0) a = __biPol2E[vlrX % 8] & 0x0F;						// Se o sentido for horario: recupera e formata o dado da matriz.
//			if (sentX == 1) a = __biPol2D[vlrX % 8] & 0x0F;						// Se o sentido for antihorario: recupera e formata o dado da matriz.
			vlrX--;																// Decrementa o valor do passo.
		}
		if (vlrY == 0)															// Se o valor do passo for zero...
		{
			b = 0;																// Desliga os pinos da saida.
			flagY = 0;															// Desabilita flag do motor.
		} else																	// ... sen�o:
		{
			if (sentY == 0) b = __biPol1E[vlrY % 4] & 0xF0;						// Se o sentido for horario: recupera e formata o dado da matriz.
			if (sentY == 1) b = __biPol1D[vlrY % 4] & 0xF0;						// Se o sentido for antihorario: recupera e formata o dado da matriz.
//			if (sentY == 0) b = __biPol2E[vlrY % 8] & 0xF0;						// Se o sentido for horario: recupera e formata o dado da matriz.
//			if (sentY == 1) b = __biPol2D[vlrY % 8] & 0xF0;						// Se o sentido for antihorario: recupera e formata o dado da matriz.
			vlrY--;																// Decrementa o valor do passo.
		}
		roboPort = a | b;														// Coloca na saida os valores interpolados.
		__delay_ms(__tmrMp);													// Aguarda...
	}
}

/* ------- ROBO ------ */
void roboIniciar(void)															// Rotina de inicializacao do robo.
{
	roboCfg=0x00;																// Configura pino como saida.
	roboPort=0x00;																// Inicia o pino em '0'.
	timersIniciar();															// Inicializa o Timer0(Servo) e Timer1(Parallax)
	parallaxIniciar();															// Inicializa o hardware do sensor.
	servoIniciar();																// Inicializa o hardware do servo.
}

void roboPraFrente(unsigned int vlrDistanciaF)									// Valor da distancia em passos.
{
	mpXY(vlrDistanciaF,0,vlrDistanciaF,0);										// 
}

void roboPraTraz(unsigned int vlrDistanciaT)									// Valor da distancia em passos.
{
	mpXY(vlrDistanciaT,1,vlrDistanciaT,1);										// 
}

void roboPraDireita(unsigned int vlrAnguloD)									// Valor do angulo em passos.
{
	mpXY(vlrAnguloD,0,vlrAnguloD,1);											// 
}

void roboPraEsquerda(unsigned int vlrAnguloE)									// Valor do angulo em passos.
{
	mpXY(vlrAnguloE,1,vlrAnguloE,0);											// 
}

unsigned int roboMedirDistancia(unsigned int tempo)								// Medir a distancia(mm) pelo Ultra som, dado o tempo(us) do servo.
{
	servoVarTon = tempo;
	parallaxLer();
	return(paralaxDistancia);
}

/* ------- INFO ------ */
/*
 * Link: https://www.learnrobotics.org/blog/raspberry-pi-servo-motor/
 * 
 Servo Motor:
 * Duty Cicle (DC):
 * Min=2%
 * Max=12%
 * 
 * Angle:
 * Min=0�
 * Max=180�
 * 
 * DC = (angle / 18) + 2
 */

#endif

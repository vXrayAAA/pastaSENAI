/*
 * File:   Kel.c
 * Author: 44846220842
 *
 * Created on 28 de Outubro de 2022, 09:23
 */


#include <xc.h>
#include "config.h"
#include "lib_v02.h"
void main(void) 
{
    
    uartIniciar();
    uartTXT("nik\r\n");
    unsigned char teclaVlr = 0;
    char motorPassoVlr[] = {"passos"};
    
    unsigned char contar,resultado;
    unsigned char dez,uni;
    unsigned char repetir;
    repetir=1;
    while(1)
    {  
        repetir = uartRXC();
        
        if(repetir>'0' && repetir<='9')
        {
    for(contar=0;contar<10;contar++)
    {
        uartTXC(repetir);    
        uartTXT(" x ");
        uartTXC(contar+'0');
        uartTXT(" = ");
        resultado = (repetir- '0') * contar;
        dez = resultado / 10;
        uni = resultado % 10;
        uartTXC(dez+'0');
        uartTXC(uni+'0');     
        uartTXT("\r\n");
    }
//        repetir++;
//        if(repetir>9)repetir=1;
        uartTXT("\r\n");
        __delay_ms(2000);
    }
        }
        
    
  
    
    
    
}

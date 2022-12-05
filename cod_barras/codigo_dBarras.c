/*
 * File:   codigo_dBarras.c
 * Author: 47872216809
 *
 * Created on 4 de Outubro de 2022, 09:50
 */


#include <xc.h>
#include "config.h"
#include "lib_v01.h"

unsigned char ean[]={0,0,0,0,0,0,0,0,0,0,0,0,0,'\0'};


void convASC(unsigned char *valor2Asc)
{
    unsigned char contar;
    for(contar = 0;contar<13;contar++)
    {
        valor2Asc[contar]=valor2Asc[contar]+'0';
    }
}


void calculaEAN(char *valor)
{
    unsigned char par=0,impar=0;
    unsigned char contar;
    unsigned int soma;
    
    for(contar = 0; contar<12;contar = contar+2)
    {
        impar = impar + ean[contar];
        
    }
    for(contar = 1; contar<12;contar = contar+2)
    {
        par = par + ean[contar];
        
        
    }
    
    par = par * 3;
        
        soma = par + impar;
        soma = soma %10;
        soma = 10 - soma;
        valor[12] = soma;
    
    
}

void main(void) 
{
   unsigned char tecla,contar=0; 
    
   picIniciar();
   lcdIniciar();
   lcdPos(1,1);
   lcdTexto("nik");
   
   
   
   lcdPos(2,1);
    while(contar<12)
    {
        tecla = teclado();
        __delay_ms(50);
        if(tecla >= '0' && tecla <='9')
        {
            lcd4bits(tecla,1);
            ean[contar] = tecla - 0x30;
            contar++;
        }
    }
   
   
   calculaEAN(ean);
   convASC(ean);
   lcdPos(2,1);
   lcdTexto(ean);
   while(1)
   {
       
   }
}

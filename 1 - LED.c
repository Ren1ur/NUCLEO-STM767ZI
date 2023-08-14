#include "stm32f7xx.h"

int main(void)
{
uint32_t i;

RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;

GPIOB->MODER = 0x50050000;
GPIOB->OTYPER = 0;
GPIOB->OSPEEDR = 0;

while(1)
{
//GPIOB->ODR = 0x8000;
for(i=0;i<500000;i++){}

GPIOB->ODR = 0x4000;
for(i=0;i<500000;i++){}

//GPIOB->ODR = 0x200;
for(i=0;i<500000;i++){}
	
//GPIOB->ODR = 0x100;
for(i=0;i<500000;i++){}
}
}

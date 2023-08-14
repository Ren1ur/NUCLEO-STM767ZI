#include "stm32f7xx.h"

// Функция переключения на работу от PLL
int ClockInit(void)
{
    // Включение HSE
    RCC->CR |= RCC_CR_HSEON;
    //  Ожидание успешного запуска или тайм-аута
    volatile int StartUpCounter;
    for(StartUpCounter = 0; ;StartUpCounter++)
    {
        if(RCC->CR & RCC_CR_HSERDY)
            break;  // Выход из цикла при успешном запуске
        if(StartUpCounter > 0x1000)
        {
            RCC->CR &= ~RCC_CR_HSEON;   // Остановка HSE
            return 1;   // Выход с ошибкой по истечению времени
        }
    }
    // Настройка PLL
    RCC->PLLCFGR |= (0x07<<RCC_PLLCFGR_PLLM_Pos)    // PLL Множитель равен 9
        | RCC_PLLCFGR_PLLSRC_Pos;                   // Тактирование от HSE
    // Запуск PLL
    RCC->CR |= RCC_CR_PLLON;
    //  Ожидание успешного запуска или тайм-аута
    for(StartUpCounter = 0; ;StartUpCounter++)
    {
        if(RCC->CR & (1<<RCC_CR_PLLRDY_Pos))
            break;  // Выход из цикла при успешном запуске
        if(StartUpCounter > 0x1000)
        {
            RCC->CR &= ~RCC_CR_HSEON;   // Остановка HSE
            RCC->CR &= ~RCC_CR_PLLON;   // Остановка PLL
            return 2;   // Выход с ошибкой по истечению времени
        }
    }
    // Устанавливается 2 цикла ожидания для FLASH
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
    RCC->CFGR |= RCC_CFGR_PPRE1_2;  // Делитель шины APB1 равен 2
    RCC->CFGR |= RCC_CFGR_PPRE2;    // Делитель шины APB2 отключен (0 по умолчанию)
    RCC->CFGR |= RCC_CFGR_HPRE_0;     // Делитель шины AHB равен 2
    // Переключение на работу от PLL
    RCC->CFGR |= RCC_CFGR_SW_1;
    // Ожидание переключения
    while((RCC->CFGR & RCC_CFGR_SWS_Msk) != (RCC_CFGR_SWS_1)) {}
    // После переключения отключается RC-генератор HSI
    RCC->CR &= ~RCC_CR_HSION;
}


int main(void)

{
uint32_t State;

State = ClockInit();

uint32_t i;

RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;

GPIOB->MODER = 0x10004001;
GPIOB->OTYPER = 0;
GPIOB->OSPEEDR = 0;

GPIOC->MODER = 0x00000000;
GPIOC->OTYPER = 0;
GPIOC->OSPEEDR = 0;

RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
TIM2->SMCR &= ~TIM_SMCR_SMS;    // Отключаем Slave mode
//TIM2->CR1 = TIM_CR1_CEN;        // Установили режим работы таймера

TIM2->SR = 0;
TIM2->PSC = 719;    // Устанавливаем в регистр предделения
TIM2->ARR = 49999;  // и перезагрузки нужные значения

//TIM2->DIER |= TIM_DIER_UIE; // Разрешаем прерывания в регистре разрешения таймера
//NVIC_EnableIRQ(TIM2_UP_IRQn);   // и в регистре контроллера прерываний

TIM2->CR1 |= TIM_CR1_CEN; // Запуск счета таймера

while(1)
{
    TIM2->SR = 0;
    GPIOB->ODR = 0x4000;
    while(!(TIM2->SR & TIM_SR_UIF)){};
    
    TIM2->SR = 0;
    GPIOB->ODR = 0x0001;
    while(!(TIM2->SR & TIM_SR_UIF)){};
/*
//TIM2->
GPIOB->ODR = 0x4000;
for(i=0;i<500000;i++){}

if((GPIOC->IDR & 0x2000) == 0x2000) {
GPIOB->ODR = 0x0080;
for(i=0;i<500000;i++){}
} else {
GPIOB->ODR = 0x0001;
for(i=0;i<500000;i++){}
}*/
}
}

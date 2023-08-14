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
    return 0;
}

void EXTI_Init(void)
{
//void NVIC_EnableIRQ(IRQn_Type IRQn)       //Разрешение прерываения с номером IRQn
//void NVIC_DisableIRQ(IRQn_Type IRQn)      //Запрет прерываения с номером IRQn
//NVIC_EnableIRQ(EXTI15_10_IRQn);
//TIM1_UP_IRQn);
// Для блока программы, который должен работать без прерываний
//__disable_irq();  // запрет прерывания
//__enable_irq();  // разрешение прерывания

    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;  		// Тактирование порта GPIOС
    RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;  		// Тактирование SYSCFG

    /*  Настройка GPIO  
    Пин: PC13
    Режим: Input Pull Up    */
    
    GPIOC->MODER = 0x00000000;
    GPIOC->OTYPER = 0;
    GPIOC->OSPEEDR = 0;
    //GPIOC->AFR[1] |= 0x00D00000;
    //GPIOC->PUPDR |= 0;
    GPIOC->ODR = 0x2000;
    
    /*  Настройка EXTI  */
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;   // Подключение  канала EXTI к порту PC13
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->RTSR |= EXTI_RTSR_TR13;                  // Прерывание по нарастанию импульса
    //EXTI->FTSR |= EXTI_FTSR_TR13;                	 // Прерывание по спаду импульса
    EXTI->PR = EXTI_PR_PR13;                       // Сброс флага прерывания
    EXTI->IMR |= EXTI_IMR_MR13;                      // Включение прерывания 13-го канала EXTI
    //EXTI->EMR |= EXTI_EMR_MR13; 
    //EXTI->SWIER |= EXTI_SWIER_SWIER13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);                // Разрешение прерывания в контроллере прерываний
}

void EXTI15_10_IRQHandler(void)
{
    uint32_t BTN_PRESSED;   BTN_PRESSED = 0;
    if((GPIOC->IDR & 0x2000) == 0x2000)
        BTN_PRESSED = 1;
    else BTN_PRESSED = 0;
        
    if((GPIOB->ODR & 0x0080) != 0x0080)
        GPIOB->ODR |= 0x0080;
    else GPIOB->ODR &= ~0x0080 ;
    //TIM2->SR = 0;
    EXTI->PR = EXTI_PR_PR13;    // Сброс флага прерывания
    //while(!(TIM2->SR & TIM_SR_UIF)){};
    while((GPIOC->IDR & 0x2000) == 0x2000){};
    
}

void TIM2_Init(void)
{
    
    RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
    TIM2->SMCR &= ~TIM_SMCR_SMS;    // Отключаем Slave mode
    //TIM2->CR1 = TIM_CR1_CEN;        // Установили режим работы таймера

    TIM2->SR = 0;       // Сбрасываем значение таймера
    TIM2->PSC = 719;    // Устанавливаем в регистр предделения
    TIM2->ARR = 49999;  // и перезагрузки нужные значения

    //TIM2->DIER |= TIM_DIER_UIE; // Разрешаем прерывания в регистре разрешения таймера
    //NVIC_EnableIRQ(TIM2_UP_IRQn);   // и в регистре контроллера прерываний

    TIM2->CR1 |= TIM_CR1_CEN; // Запуск счета таймера
}

int main(void)
{
ClockInit();
/*
// Конфигурация таймера TIM1
RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
TIM1->SMCR &= ~TIM_SMCR_SMS;
TIM1->CR1 = TIM_CR1_CEN;
TIM2->PSC = 719;
TIM2->ARR = 24999;
TIM1->DIER |= TIM_DIER_UIE;
//NVIC_EnableIRQn(TIM1_UP_IRQn);
*/

//uint32_t i;

RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;

GPIOB->MODER = 0x10004001;
GPIOB->OTYPER = 0;
GPIOB->OSPEEDR = 0;

EXTI_Init();
TIM2_Init();
__enable_irq();

while(1)
{
    TIM2->SR = 0;
    if((GPIOB->ODR & 0x4000) != 0x4000)
        GPIOB->ODR |= 0x4000;
    else GPIOB->ODR &= 0x0FFF;
    while(!(TIM2->SR & TIM_SR_UIF)){};
    
    TIM2->SR = 0;
    if((GPIOB->ODR & 0x0001) != 0x0001)
        GPIOB->ODR |= 0x0001;
    else GPIOB->ODR &= ~0x0001;
    while(!(TIM2->SR & TIM_SR_UIF)){};

    //if((GPIOC->IDR & 0x2000) == 0x2000)
    //    EXTI15_10_IRQHandler();

/*
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
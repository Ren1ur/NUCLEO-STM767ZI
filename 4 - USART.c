#include "stm32f7xx.h"

    /* Глобальные переменные */
uint32_t temp = 0, data= 1, i;
uint8_t b;
uint32_t bufRX;
uint32_t InitCount = 0, USART_InitCount = 0, USART_HandlerCount = 0;


int ClockInit(void)
{   // Переключение на работу от PLL - 72MHz
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
{   /* Инициализация прерываний EXTI для кнопки user*/
//__disable_irq();  // запрет прерывания
//__enable_irq();  // разрешение прерывания

    RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;  // Тактирование SYSCFG

    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;  // Тактирование порта GPIOС
    GPIOC->MODER = 0x00000000;
    GPIOC->OTYPER = 0;
    GPIOC->OSPEEDR = 0;
    //GPIOC->AFR[1] |= 0x00D00000;
    //GPIOC->PUPDR |= 0;
    GPIOC->ODR = 0x2000;
    
    /*  Настройка EXTI  */
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;   // Подключение  канала EXTI к порту PC13
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->RTSR |= EXTI_RTSR_TR13;                       // Прерывание по нарастанию импульса
    //EXTI->FTSR |= EXTI_FTSR_TR13;                       // Прерывание по спаду импульса
    EXTI->PR = EXTI_PR_PR13;                            // Сброс флага прерывания
    EXTI->IMR |= EXTI_IMR_MR13;                         // Включение прерывания 13-го канала EXTI
    //EXTI->EMR |= EXTI_EMR_MR13; 
    //EXTI->SWIER |= EXTI_SWIER_SWIER13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);                        // Разрешение прерывания в контроллере прерываний
}

void LED_Init()
{   /* Настройка светодиодов User LED 1,2,3 */
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER = 0x10004001;
    GPIOB->OTYPER = 0;
    GPIOB->OSPEEDR = 0;
}

void LED_Switch(uint32_t Num)
{   /* Переключает светодиоды 1,2,3 */
    switch(Num){
    case 1: {GPIOB->ODR ^= 0x0001; break;}
    case 2: {GPIOB->ODR ^= 0x0080; break;}
    case 3: {GPIOB->ODR ^= 0x4000; break;}
    default: break;
    }
}

uint16_t TIM1_I = 0;
void Delay(uint32_t ms)
{   /* Функция задержки, принимающая в качестве аргумента кол-во миллисекунд */
    if(TIM1_I == 0)
    {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        TIM1->SMCR &= ~TIM_SMCR_SMS;    // Отключаем Slave mode
        TIM1->PSC = 719;    // Устанавливаем в регистр предделения нужное значение
        TIM1_I++;
    }
    TIM1->SR = 0;       // Сбрасываем значение таймера
    TIM1->ARR = ms*100 - 1;  // и перезагрузки нужные значения
    while(!(TIM1->SR & TIM_SR_UIF)){};
}


void USART_Init(void)
{   /* Инициализация USART2 */

    /* Настройка портов PD6 и PD5 как RX (вход) и TX (выход) соответственно */    
    RCC->APB1ENR |= RCC_AHB1ENR_GPIODEN;    // Включаем тактирование портов D
    GPIOD->MODER &= ~0x00003C00;            // Чистка регитра MODER от значений пинов 5 и 6
    GPIOD->MODER |= 0x00002800;             // Установка пинов 5 и 6 в Alternate function mode
    GPIOD->OSPEEDR |= 0x00002800;           // Установка пинов 5 и 6 на низкую скорость
    GPIOD->PUPDR |= 0;
    GPIOD->ODR |= 0x0020;                   // Установка пина 5 на выход
    GPIOD->AFR[1] |= 0x07700000;            // Записываем AF7 в ячейку для пинов 5 и 6
    
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // USART2 Clock ON
    USART2->CR1 = 0;    USART2->CR2 = 0;    USART2->CR3 = 0;    // Обнуление настроек USART2
    USART2->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);              // Задаются длина слова (M[1:0] = 00: 1 Стартовый бит, 8 бит данных, n стоп-бит)
    USART2->BRR = 0x1D4C;                   // // Задается скорость обмена порта USART2 равная 9600 бод
    USART2->CR2 = ~USART_CR2_STOP;  // Значение STOP = 00 соответствует 1 стоп-биту

    // Настройка для Polling mode
    //USART2->CR2 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;   // Разрешение работы RX TX
/*
    USART2->TDR = data; // Передать байт данных
    //  Прежде чем передавать следующий байт необходимо дождаться окончания передачи предыдущего байта,
    //  анализируя состояние разряда TC регистра USART_ISR
    while((USART2->ISR USART_ISR_TC) == 0) {} // Ждать окончания передачи
    USART2->TDR = 0;    // Очистить флаг окончания очереди
        // или USART2->ICR = USART_ICR_TCCF;

    //  Прием информации от порта
    while((USART2->ISR & USART_ISR_RXNE) == 0) {}   //  Ждать приема передачи
    temp = USART2->RDR;     // Считать принятый байт
    USART2->RDR = 0;        // Сброс флага окончания приема производится после обнуления регистра данных
*/
    
    // Настройка для Interrupt Mode:    USART2 ON, TX ON, RX ON, RXNE ON
    //USART2->CR1 |= USART_CR1_TCIE;      // Разрешаем прерывание по окончанию передачи   
    //USART2->CR1 |= USART_CR1_RXNEIE;    // Разрешаем прерывание по приему данных
    //USART2->CR1 |= USART_CR1_TE;        // Transmitter enable
    //USART2->CR1 |= USART_CR1_RE;        // Receiver enable
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_TCIE; 
    NVIC_EnableIRQ(USART2_IRQn);    // Разрешаем прерывания от USART2

    USART_InitCount = 1;
}
void USART2_IRQHandler(void)
{   /* Прерывание по окнончанию принятия/отправки слова по USART2 */
    LED_Switch(1);
    //unsigned char temp, data;    // Временная байтовая переменная
    if((USART2->ISR & USART_ISR_TXE) != USART_ISR_TXE) {}
        USART2->TDR = data++;
    if((USART2->ISR & USART_ISR_RXNE) != USART_ISR_RXNE) {}   // Если прием данных завершен
        temp = USART2->RDR;  // Считать принятый байт
        if(temp == '1')
            LED_Switch(1);
        else if(temp == '2')
            LED_Switch(2);
/*
    if((USART2->ISR & USART_ISR_TC) != 0) { // Если передача завершена
        USART_HandlerCount = 2;
        USART2->ICR = USART_ICR_TCCF;       // Сбросить флаг 
    }
*/
    //USART2->ICR = ~USART_ICR_TCCF;
/*  Использование прерываний освобождает микроконтроллер от необходимости
    постоянной проверки флагов и позволяет высвободить его ресурсы для других работ. */
}

void EXTI15_10_IRQHandler(void)
{   /* Прерывание по нажанию кнопки USER */
    //uint32_t BTN_PRESSED;   BTN_PRESSED = 0;
    USART2->TDR = data++;
    LED_Switch(2);
    EXTI->PR = EXTI_PR_PR13;    // Сброс флага прерывания
    //while(!(TIM2->SR & TIM_SR_UIF)){};
    while((GPIOC->IDR & 0x2000) == 0x2000){};
    LED_Switch(2);
}

void TIM2_Init(void)
{   /* Инициализация таймера TIM2 */
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
{   /* Основная программа с телом цикла */
ClockInit();    InitCount = 1;
TIM2_Init();    InitCount = 2;
LED_Init();     InitCount = 3;
USART_Init();   InitCount = 4;
EXTI_Init();    InitCount = 5;
__enable_irq();
while(1)
{
{   /* Цикл приема/отправки данных по UART1 */
LED_Switch(3);              // Моргающий индикатор LED показывает, что цикл основной программы работает
for(i=0; i<300000; i++);    // введение задержки
/*USART2->TDR = data; // Передать байт данных
    //  Прежде чем передавать следующий байт необходимо дождаться окончания передачи предыдущего байта,
    //  анализируя состояние разряда TC регистра USART_ISR
    while((USART2->ISR & USART_ISR_TC) == 0) {} // Ждать окончания передачи
    USART2->TDR = 0;    // Очистить флаг окончания очереди
        // или USART2->ICR = USART_ICR_TCCF;
    //  Прием информации от порта
    while((USART2->ISR & USART_ISR_RXNE) == 0) {}   //  Ждать приема передачи
    temp = USART2->RDR;     // Считать принятый байт
    USART2->RDR = 0;        // Сброс флага окончания приема производится после обнуления регистра данных*/
}    
{   /* Моргание светодиодами *//*
    TIM2->SR = 0;  
    LED_Switch(1);
    while(!(TIM2->SR & TIM_SR_UIF)){};

    TIM2->SR = 0;
    LED_Switch(3); 
    while(!(TIM2->SR & TIM_SR_UIF)){};      //*/
}
}
}

#include "stm32f7xx.h"
#include "math.h"
#define ComFreq 72000000
#define SmasSize 72
#define Freq_out 72

    /* Глобальные переменные */
uint32_t i, bufRX;
uint16_t ClockError = 0;
uint32_t TIM2_Counter = 0, TIM3_Counter = 0, TIM4_Counter = 0, Main_Counter = 0;
uint16_t Rec_Counter = 0;
uint16_t ADC_Data = 0, Rec_Data = 0, Send_Data = 0;
uint8_t PauseState = 0;
uint16_t CtrlVal = 0, MaxVal = 0;
double sin_val = 0;
const float pi = 3.14;
uint16_t sinus[72] = {  // Значения синусоиды через равные промежутки времени
2225, 2402, 2577, 2747, 2912, 3070, 3221, 3363, 3494, 3615, 
3724, 3820, 3902, 3971, 4024, 4063, 4085, 4095, 4085, 4063,
4024, 3971, 3902, 3820, 3724, 3615, 3494, 3363, 3221, 3070,
2912, 2747, 2577, 2402, 2225, 2047, 1869, 1692, 1517, 1347,
1182, 1024,  873,  731,  600,  479,  370,  274,  192,  124,
  70,   31,   10,    0,   10,   31,   70,  123,  192,  274,
 370,  479,  600,  730,  873, 1023, 1182, 1347, 1517, 1691,
1868, /*2047*/ 2
};

uint16_t in_sin[SmasSize] = {};  // Массив, куда записываются данные с АЦП

uint16_t mas[72] = {  // Массив для тестов
1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72 };


    // Структуры
struct Out_sign{
    uint16_t *mas;  // Мгновенные значения сигнала заданы в массиве
    uint16_t size;  // Размер массива
    uint16_t freq;  // Частота сигнала
    uint16_t it;    // Итерация
};

struct Out_sign out_s = {.size = 72, .freq = 72, .it = 0};

/*
    ----------------------------------
          Блок объявления функций
    ----------------------------------
*/
    // Обычные функции
void Delay(uint32_t ns);        // Задержка в системных тиках * 72
void ToggleLED(uint8_t Num);    // Переключение светодиода по номерам 1-3
void SendData(uint8_t data);    // Функция отпраки данных по USART  (не используется)
uint8_t RecData(uint8_t data);  // Функция приема данных по USART   (не используется)
void ShowVal(int *val);         // Функция вывода значения как CtrlVal (для отладки)
void SignTrans(void);           // Функция преобразования сигнала
void DAC_out(struct Out_sign);

    // Функции инициализации
int SystemClock_Config(void);   // Переключение на частоту работы 72MHz
void LED_Init(void);            // Светодиоды
void ADC1_Init(void);           // АЦП
void DAC_Init(void);            // ЦАП
void DMA_Init(void);            // DMA
void EXTI_Init(void);           // Внешние прерывания
void TIM2_Init(void);           // Таймер TIM2
void TIM3_Init(void);           // Таймер TIM3
void TIM4_Init(void);           // Таймер TIM4
void USART2_Init(void);         // USART
    // Функции обработчиков прерываний
void ADC_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void TIM2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void USART2_IRQHandler(void);
/*
    ----------------------------------
           Основная программа
    ----------------------------------
*/
int main(void) { 					// Основная программа с телом цикла
	SystemClock_Config();		// Настройка тактирования
    LED_Init();     
    EXTI_Init();    			// Инициализация светодиодов и внешних прерываний
    ADC1_Init();    			// Инициализация АЦП
    DAC_Init();     			// Инициализация ЦАП
    TIM2_Init();
		TIM3_Init();
		TIM4_Init();  				// Инициализация таймеров  
    USART2_Init();  			// Инициализация USART
    //DMA_Init();     		// Инициализация DMA (с ошибками)

    __enable_irq();     	// Общее разрешение прерываний

    TIM2_Counter = 0;   TIM2->SR = 0;   // Задаем начальные значения счетчиков (для отладки)
    Main_Counter = 0;   Send_Data = 0;      i = 0;
    Rec_Counter = 0;    PauseState = 0;
    //ADC1->CR2 |= ADC_CR2_SWSTART;     // - ?
    DMA1_Stream6->CR |= DMA_SxCR_EN;

    ADC_Data = 0xFFF;
    sin_val = sin((float)ADC_Data/0xFFF);
    
while(1) {
    if(PauseState != 0)
        __disable_irq();
    while(PauseState != 0){};
    Main_Counter += 1;
    //DAC->DHR12R1 = sinus[i];
    //i = i+1 - 72*(i >= 71);
    //ToggleLED(1);
    //SendData(Rec_Data++);
    
    //ShowVal(&mas[i]);
    //i = i+1 - 71*(i>70);
    if((USART2->TDR)>MaxVal)
        MaxVal = USART2->TDR;
    
    }    
}
/*
    ----------------------------------
            Описание функций
    ----------------------------------
*/

void ShowVal(int *val) {                    // Вывод значения в отладке
    CtrlVal = *val;
}

void SignTrans(void){                       // Функция преобразования сигнала
    
    uint16_t MaxSin = 0;
    uint16_t i;
    for(i=0; i<SmasSize; i++)         // Поиск max значения сигнала в периоде
        if(in_sin[i]>MaxSin)
            MaxSin = in_sin[i];
    for(int i=SmasSize/2; i<SmasSize; i++)    // "Переворачиваем" сигнал каждый второй полупериод
        in_sin[i] = MaxSin - in_sin[i];
}

void DAC_out(struct Out_sign out){          // Функция вывода сигнала на ЦАП
    //static uint16_t DAC_i;
    //for(DAC_i = 0; DAC_i < out.freq; DAC_i++)
        DAC->DHR12R2 = out_s.mas[out.it];   out.it += 1;
}
int SystemClock_Config(void) {              // Переключение на работу от PLL - 72MHz
    RCC->CR |= RCC_CR_HSEON;                    // Включение HSE
    volatile int StartUpCounter;
    for(StartUpCounter = 0; ;StartUpCounter++)  //  Ожидание успешного запуска или тайм-аута
    {
        if(RCC->CR & RCC_CR_HSERDY)
            break;                              // Выход из цикла при успешном запуске
        if(StartUpCounter > 0x1000)
        {
            RCC->CR &= ~RCC_CR_HSEON;           // Остановка HSE и
            return 1;                           // выход с ошибкой по истечению времени
        }
    }
    // Настройка PLL
    RCC->PLLCFGR = (0x07U << RCC_PLLCFGR_PLLM_Pos)// Множитель частоты равен 7
        | (0x0FCU << RCC_PLLCFGR_PLLN_Pos)        // Делитель частоты равен 252
        | (0x00U << RCC_PLLCFGR_PLLP_Pos)         // Главный делитель системной частоты равен 2
        | (0x2U << RCC_PLLCFGR_PLLQ_Pos)        
        | RCC_PLLCFGR_PLLSRC
        | RCC_PLLCFGR_PLLSRC_Pos;               // Тактирование от HSE
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_4;
    RCC->CR |= RCC_CR_PLLON;                    // Запуск PLL
    for(StartUpCounter = 0; ;StartUpCounter++)  // Ожидание успешного запуска или тайм-аута
    {
        if(RCC->CR & (1<<RCC_CR_PLLRDY_Pos))
            break;                              // Выход из цикла при успешном запуске
        if(StartUpCounter > 0x1000)
        {
            RCC->CR &= ~RCC_CR_HSEON;           // Остановка HSE
            RCC->CR &= ~RCC_CR_PLLON;           // Остановка PLL
            return 2;                           // Выход с ошибкой по истечению времени
        }
    }
    // Устанавливается 2 цикла ожидания для FLASH
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2;    // Делитель шины AHB равен 2, частота - 72MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;   // Делитель шины APB1 равен 2, частота - 36MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;   // Делитель шины APB2 отключен (0 по умолчанию), частота - 72MHz
    RCC->CFGR |= RCC_CFGR_SW_1;         // Переключение на работу от PLL
    // Ожидание переключения
    while((RCC->CFGR & RCC_CFGR_SWS_Msk) != (RCC_CFGR_SWS_1)) {}
    // После переключения отключается RC-генератор HSI
    //RCC->CR &= ~RCC_CR_HSION;
    return 0;
}

void Delay(uint32_t ns) {                   // Функция задержки (в системных тиках * 72)
    for(volatile uint32_t del = 0; del < ns*72; del++);
}

void LED_Init(void) {                       // Настройка светодиодов User LED 1,2,3
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER = 0x10004001;
    GPIOB->OTYPER = 0;
    GPIOB->OSPEEDR = 0;
}

void ToggleLED(uint8_t Num) {               // Переключает светодиоды 1,2,3
    switch(Num){
    case 1: {GPIOB->ODR ^= 0x0001; break;}
    case 2: {GPIOB->ODR ^= 0x0080; break;}
    case 3: {GPIOB->ODR ^= 0x4000; break;}
    default: break;
    }
}

void ADC1_Init(void) {                      // Инициализация АЦП
    // Настройки тактирования
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;            // Тактирование
    GPIOA->MODER |= (0x3<<GPIO_MODER_MODER6_Pos);   // Установка пина 6 в Analog mode
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;             // Тактирование АЦП
    // Настройка параместров АЦП 
    ADC123_COMMON->CCR |= ADC_CCR_ADCPRE_1;         // Делитель частоты равен на 6
    ADC1->CR1 &= ~ADC_CR1_RES_Msk;                  // Разрешение 12 бит
    ADC1->SMPR2 |= (0x111U << ADC_SMPR2_SMP6_Pos);  // Устанавливаем количество циклов для пересчета
    ADC1->SQR3 = (0x06U << ADC_SQR3_SQ1_Pos);       // Включение 6-го канала
    // Настройка работы от таймера TIM4
    ADC1->CR2 |=  (0xCU << ADC_CR2_EXTSEL_Pos);     // Выбор тригера TIM4 TRGO
    ADC1->CR2 |=  (0x1U << ADC_CR2_EXTEN_Pos);      // Запрет непрерывного режима
    // Включение
    ADC1->CR2 &= ~ADC_CR2_CONT;                     // Запрет непрерывного режима
    //ADC1->CR2 |= ADC_CR2_EOCS;                    // Разрешение серийного преобразования
    ADC1->CR2 |= ADC_CR2_ADON;                      // Включение питания АЦП
    Delay(10);
    // Включение прерываний по концу преобразования
    ADC1->CR1 |= ADC_CR1_EOCIE;
    NVIC_EnableIRQ(ADC_IRQn);
}


void ADC_IRQHandler(void) {                 // Обработчик прерываний АЦП
    //ToggleLED(3);
    TIM4_Counter += 1;
    if((ADC1->SR & ADC_SR_EOC) != 0)
    {
        static uint16_t ADC_i = 0;
        ADC_Data = ADC1->DR;            // Чтение из регистра данных
        in_sin[ADC_i] = ADC1->DR;       // Запись очередного значения массив
        
        //DAC->DHR12R2 = ADC_Data;
        //DAC->DHR12R2 = sinus[i++];
        //USART2->TDR = ADC_Data;

        ADC_i += 1;
        if(ADC_i>=SmasSize)  ADC_i = 0;
    }
    ADC1->SR &= ~ADC_SR_EOC;            // Сброс флага прерывания
    /* Должен сниматься автоматически после чтения из регистра данных */
}

void DAC_Init(void) {                       // Инициализация ЦАП
    // Настройка выходов
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;            // Тактирование портов A
    GPIOA->MODER |= (0x3 << GPIO_MODER_MODER4_Pos); // Установка пина 4 в Analog mode
    GPIOA->MODER |= (0x3 << GPIO_MODER_MODER5_Pos); // Установка пина 5 в Analog mode
    // Настройка самого ЦАП
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;              // Тактирование портов A
    // Настройка работы от таймера TIM4
    //DAC1->CR |= (0x9U << DAC_CR_TSEL2_Pos);        // Выбор тригера TIM4 TRGO
    //DAC1->CR |= DAC_CR_TEN2;
    DAC1->CR |= DAC_CR_DMAEN2;
    DAC1->CR |= DAC_CR_DMAEN1;
    DAC1->CR |= DAC_CR_BOFF2;
    DAC1->CR |= DAC_CR_EN1;                          // Включение 1-го канала ЦАП
    DAC1->CR |= DAC_CR_EN2;                          // Включение 2-го канала ЦАП
    RCC->APB1ENR |=  RCC_APB1ENR_DACEN;             // Тактирование ЦАП
}

/*void DMA_Init(void) {                     // Инициализация DMA
    // ADC1: DMA2, Channel 0, Stream 0 | 4
    // DAC1: DMA1, Channel 7, Stream 5
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;                 // Тактирование DMA1
    //DMA1_Stream6->CR |= (0x7U << DMA_SxCR_CHSEL_Pos);   // Выбор 7-го канала для связи с ЦАП
    DMA1_Stream6->CR |= (0x4U << DMA_SxCR_CHSEL_Pos);   // Выбор 7-го канала для связи с USART2
    DMA1_Stream6->CR |= (0x0U << DMA_SxCR_MSIZE_Pos);   // Размер данных памяти
    DMA1_Stream6->CR |= (0x0U << DMA_SxCR_PSIZE_Pos);   // Размер данных периферии
    DMA1_Stream6->CR |= DMA_SxCR_CIRC;                  // Циклическая передача
    DMA1_Stream6->CR |= (0x1U << DMA_SxCR_DIR_Pos);     // Направление передачи (из памяти в периферию)
    
//    DMA1_Stream6->PAR |= (uint32_t) (&DAC->DHR12R1);    // Запись адреса периферии
    DMA1_Stream6->PAR |= (uint32_t) &(USART2->TDR);   	// Запись адреса периферии
    Rec_Data = 155;
//    DMA1_Stream6->M0AR |= (uint32_t) &Rec_Data;
    DMA1_Stream6->M0AR = &mas;            							// Запись адреса памяти
    DMA1_Stream6->NDTR = 71;                            // Количество данных
    DMA1_Stream6->CR |= DMA_SxCR_MINC;
//    DMA1_Stream6->NDTR = 1;                             // Количество данных
//    DMA1_Stream6->M1AR |= (uint32_t) &sinus;

    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

//    DMA1_Stream6->CR |= DMA_SxCR_EN;      // Включение DMA
    DMA1_Stream6->CR |= DMA_SxCR_TCIE;    // Включение прерывания после удачной передачи
    DMA1_Stream6->CR |= DMA_SxCR_TEIE;    // Включение прерывания после ошибки передачи
}

void DMA1_Stream6_IRQHandler(void)
{
    //ToggleLED(2);
    if((DMA1->HISR & DMA_HISR_TCIF6) != 0)
    {
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        Rec_Counter += 1;
        ToggleLED(3);
        //if(DMA1_Stream6->NDTR <= 0) DMA1_Stream6->NDTR = 72;
        //PauseState = (Rec_Counter >= 71);
        if(PauseState == 0) {
            DMA1_Stream6->CR |= DMA_SxCR_EN;
        }
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
    }
};
*/
void EXTI_Init(void) {                      // Инициализация прерываний EXTI для кнопки USER
    RCC->APB2ENR|=RCC_APB2ENR_SYSCFGEN;     // Тактирование SYSCFG
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN;      // Тактирование порта GPIOС
    GPIOC->MODER = 0x00000000;
    GPIOC->OTYPER = 0;
    GPIOC->OSPEEDR = 0;
    GPIOC->ODR = 0x2000;
    
    // Настройка EXTI
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;    // Подключение  канала EXTI к порту PC13
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;  
    EXTI->RTSR |= EXTI_RTSR_TR13;                   // Прерывание по нарастанию импульса
    EXTI->PR = EXTI_PR_PR13;                        // Сброс флага прерывания
    EXTI->IMR |= EXTI_IMR_MR13;                     // Включение прерывания 13-го канала EXTI
    NVIC_EnableIRQ(EXTI15_10_IRQn);                 // Разрешение прерывания в контроллере прерываний
}

void EXTI15_10_IRQHandler(void) {           // Прерывание по нажанию кнопки USER
    ToggleLED(2);                   // Переключение синего светодиода
    PauseState = 1 - 1*PauseState;  // Переключение в режим ожидания (пауза)
    EXTI->PR = EXTI_PR_PR13;        // Сброс флага прерывания
    while((GPIOC->IDR & 0x2000) == 0x2000){};   // Ждем пока отожмется кнопка
}

void TIM2_Init(void) {                      // Инициализация таймера TIM2
    RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;   // Включение тактирования
    TIM2->SMCR &= ~TIM_SMCR_SMS;        // Отключаем Slave mode
    TIM2->SR = 0;                       // Сбрасываем значение таймера
    TIM2->PSC = 71;                     // Устанавливаем в регистр предделения
    TIM2->ARR = 999999;                 // и перезагрузки нужные значения
    TIM2->DIER |= TIM_DIER_UIE;         // Разрешаем прерывания по обновлению таймера
    NVIC_EnableIRQ(TIM2_IRQn);          // Разрешаем глобальные прерывания для TIM2
    TIM2->CR1 |= TIM_CR1_CEN;           // Запуск счета таймера
}

void TIM2_IRQHandler(void) {            		// Обработчик прерываний таймера
    // Таймер отмеряет 60 секунд, после чего устанавливает PauseState = 1
    if((TIM2->SR & TIM_SR_UIF) != 0) {
        TIM2->SR = 0;           		// Обнулить значение таймера
        TIM2_Counter += 1;      		// Счетчик срабатываний таймера (для отладки)
        ToggleLED(1);           		// Переключение зеленого светодиода
        TIM2->SR = ~TIM_SR_UIF; 		// Сбросить флаг
        
    ///////////////////////////////////////
        PauseState = 1*(TIM2_Counter >= 60);
        //USART2->TDR = Send_Data;
    }
}

void TIM3_Init(void) {                  // Инициализация таймера TIM3 для ЦАП
		uint16_t freq = out_s.freq;
    RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;   // Включение тактирования
    TIM3->SMCR &= ~TIM_SMCR_SMS;        // Отключаем Slave mode
    TIM3->CR2 |= 
        (0x2U << TIM_CR2_MMS_Pos);      // Разрешаем TRGO по Update event
    TIM3->PSC = 1;

    // Расчет задержки для обновления с заданной частотой
    volatile uint32_t t;                // Значение задержки таймера
    t = int(ComFreq / freq) + 1*((2*(ComFreq % freq)) > freq);
    if(t > 0xFFFF) {
        for(uint16_t k = 10; (k>1) || (t < 0xFFFF); k--)
            while((t % k == 0) || (t < 0xFFFF))
            {	t = t/k;		TIM3->ARR *= k; }
    }
    TIM3->PSC = t - 1;
		TIM3->DIER |= TIM_DIER_UIE;         // Разрешаем прерывания по обновлению таймера
		NVIC_EnableIRQ(TIM3_IRQn);          // Разрешаем глобальные прерывания для TIM2
    TIM3->CR1 |= TIM_CR1_CEN;           // Запуск счета таймера
    TIM3->SR = 0;                       // Сбрасываем значение таймера
}

void TIM4_Init(void) {                      // Инициализация таймера TIM4 для запуска АЦП
    RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;       // Включение тактирования
    TIM4->SMCR &= ~TIM_SMCR_SMS;            // Запрещаем Slave mode
    TIM4->CR2 |= (0x2U << TIM_CR2_MMS_Pos); // Разрешаем TRGO по Update event
    TIM4->SR = 0;                           // Сбрасываем значение таймера

    /* Дальше идет настройка параметров, отвечающих за частоту срабатывания таймера.
    Для частоты 1HZ натройки следующие: PSC = 7199, ARR = 9999.
    Срегистром ARR следует быть осторожнее, так как в отличие от TIM2, TIM4 обладает
    порезаным функционалом, и регистр ARR может содержать только сильно меньшие значения.   */

    TIM4->PSC = 71;
    TIM4->ARR = 99;
    //TIM4->ARR |= 0xF0000;     // - ?
    // Запуск счета таймера
    TIM4->CR1 |= TIM_CR1_CEN;
}

void USART2_Init(void) {                    // Инициализация USART2
    // Настройка портов PD6 и PD5 как RX и TX соответственно
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;    // Включаем тактирование портов D
    GPIOD->MODER &= ~0x00003C00;            // Чистка регитра MODER от значений пинов 5 и 6
    GPIOD->MODER |= 0x00002800;             // Установка пинов 6 и 5 в Alternate function mode
    GPIOD->OSPEEDR |= 0x00003C00;           // Установка пинов 6 и 5 на Very High speed
    GPIOD->AFR[0] |= 0x07700000;            // Записываем AF7 для пинов 5 и 6
    // Настройка параметров USART
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // Тактирование от APB1 (36 MHz)
    USART2->CR1 = 0;                        // Обнуление регистров CR1,2,3 настроек USART2
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->CR1 &= ~USART_CR1_M;            // Задается длина слова (M[1:0] = 00: 1 Стартовый бит, 8 бит данных)
    /* Вычисляется значение делителя для скорости паередачи
    Установленная частота шины APB1 - 36MHz. OverSampling по умолчанию - 16.
    Требуемая скорость передачи - 115200 бод. 
    BRR = USARTDIV = 36 * 10^6 / 115200 / 16 = 19.53125     */
    USART2->BRR = (0x13<<USART_BRR_DIV_MANTISSA_Pos)    // 19 = 0x13
        | (0x9<<USART_BRR_DIV_FRACTION_Pos);            // 0.53125 * 16 = 8.5 < 0x9
    USART2->CR2 &= ~USART_CR2_STOP;         	// Значение STOP = 00 соответствует 1 стоп-биту
    //USART2->CR1 |= USART_CR3_DMAR;          // Разрешение работы DMA
    //USART2->CR1 |= USART_CR1_OVER8; 				// Oversampling устанавлиается в 8
    //USART2->CR1 |= USART_CR1_PCE;   				// Разрешена проверка на четность
    //USART2->CR1 |= USART_CR1_PS;    				// Проверка на четность - odd parity
		
		/*	Interrupt Mode:    USART2, TX, RX, RXNE
				USART_CR1_TCIE   - Прерывание по окончанию передачи   
				USART_CR1_RXNEIE - Прерывание по приему данных
				USART_CR1_TE     - Transmitter enable
				USART_CR1_RE     - Receiver enable		*/
				
    //USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; 
		//NVIC_EnableIRQ(USART2_IRQn);    								// Разрешаем прерывания от USART2
    USART2->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT; // Настройка для передачи с DMA
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE;
    // Polling mode:    USART2, TX, RX
}

void SendData(uint8_t data) {               // Функция отправки данных по USART2
    ToggleLED(3);
    //Send_Data = data;
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;     // Выключение DMA
    DMA1_Stream6->NDTR = 72;
    DMA1_Stream6->CR |= DMA_SxCR_EN;      // Включение DMA    
    while(!(USART2->ISR & USART_ISR_TC));
//    USART2->TDR = data;
    ToggleLED(3);
}

uint8_t RecData(uint8_t data) {             // Функция приема данных по USART2
    while(!(USART2->ISR & USART_ISR_RXNE));
    return USART2->RDR & 0xFF;
}

void USART2_IRQHandler(void) {              // Прерывание по окнончанию принятия/отправки слова по USART2
    //unsigned char temp, data;    						// Временная байтовая переменная
    ToggleLED(3);
    if((USART2->ISR & USART_ISR_RXNE) != 0) // Если прием данных завершен
    {
        Rec_Data = (USART2->RDR & 0x0F);  	// Считать принятый байт
        ToggleLED(Rec_Data);
    }

    if((USART2->ISR & USART_ISR_TC) != 0)   // Если передача завершена
    {
        USART2->ICR = USART_ICR_TCCF;       // Сбросить флаг
        //ToggleLED(3);
    }
//  Использование прерываний освобождает микроконтроллер от необходимости
//  постоянной проверки флагов и позволяет высвободить его ресурсы для других работ
}

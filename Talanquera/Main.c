#include <stdint.h>
#include "stm32l053xx.h"

//Mapeo Keypad
const uint8_t KEYPAD_MAP[4][4] = {
    {1, 2, 3, 'A'},
    {4, 5, 6, 'B'},
    {7, 8, 9, 'C'},
    {'*', 0, '#', 'D'}
};

const uint8_t SEGMENT_MAP[14] = {   //valores para que se  pinte el display
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,
    0x77, 0x7C, 0x39, 0x5E
};

volatile uint8_t display_buffer[4] = {0, 0, 0, 0}; // Almacena los valores de los dígitos
volatile uint8_t current_digit = 0;               // Dígito actual para multiplexación
volatile int8_t last_key = -1;                    // Última tecla presionada
volatile uint8_t debounce_timer = 0;              // Temporizador para debouncing
volatile uint16_t timer_value = 0;                // Valor del temporizador en segundos
volatile uint8_t countdown_active = 0; 			// Indica si la cuenta regresiva está activa
volatile uint8_t fast_second_counter = 0; 		// Contador para ajustar los segundos
volatile uint16_t counter = 0; // Contador de carros

// Prototipos de funciones
void configure_gpio(void); // Configura los pines GPIO
void configure_timers(void); // Configura los temporizadores
void scan_keypad(void); // Escanea las teclas presionadas
void update_display(void); // Activación del display
void handle_timer(void); // Manejo de interrupciones
void TIM2_IRQHandler(void); //TIM para servomotor
void TIM21_IRQHandler(void); //TIM del Keypad
void TIM6_IRQHandler(void); //TIM para el Display
void USART2_Write(char ch);  // Comunicación serial
void USART2_Write_String(const char *str); // PARA COMUNICACIÓN SERIAL
//Funciones para el servo 
void servo_write(uint8_t angle);//  Controla ángulo del servomotor
void stop_servo(void); // Detener el movimiento del servomotor
void start_servo(void); //Inicia el movimiento del servo 
void start_moving_servo(void); //Inicia el tiempo y el movimiento del servomotor
// USART2
void USART2_Init(void) {
    RCC->IOPENR |= (1<<0);  // Habilitar reloj GPIOA
    RCC->APB1ENR |= (1<<17); // Habilitar USART2

    // se configura PA2 para usar como USART2_TX
    GPIOA->MODER &= ~(0x3 << (2 * 2));
    GPIOA->MODER |= (0x2 << (2 * 2));
    GPIOA->AFR[0] |= (0x4 << (2 * 4));

    // se configura PA3 para usar como USART2_RX
    GPIOA->MODER &= ~(0x3 << (3 * 2));
    GPIOA->MODER |= (0x2 << (3 * 2));
    GPIOA->AFR[0] |= (0x4 << (3 * 4));

    USART2->BRR = 218;  //9600 baudios
    USART2->CR1 |= (1<<2);  // Habilita RX
    USART2->CR1 |= (1<<3);  // Habilita TX
    USART2->CR1 |= (1<<0);  // Habilita USART
}

void mostrar_menu_bienvenida(void) { //Mensaje para el monitor serial
	 USART2_Write_String("\n--- Bienvenido a parqueo de Dany---\n");
	    USART2_Write_String("Paso 1. Presioné '#' para levantar la talanquera\n");
	    USART2_Write_String("Paso 2. Presione '*' para terminar proceso de cerrado \n");
}

// Función principal
int main(void) {

    __disable_irq(); //Desactiva las interrupciones
    USART2_Init();      // Inicializar comunicación serial
    mostrar_menu_bienvenida();

    // Configurar GPIO y temporizadores
    configure_gpio();
    configure_timers();

    __enable_irq(); //Acitiva las interrupciones 

    while (1) {
    	if (last_key != -1) {
    	    if (last_key == '#') {  // Si se presiona la tecla #
    	        USART2_Write_String("Se abrió la talanquera...\n");
    	        USART2_Write_String("CUIDADO....CERRANDO TALANQUERA...\n");
    	        counter++;          // Incrementa el contador

    	        // Actualiza el display buffer para reflejar el nuevo valor
    	        display_buffer[0] = counter % 10;          // Unidad
    	        display_buffer[1] = (counter / 10) % 10;   // Decena
    	        display_buffer[2] = (counter / 100) % 10;  // Centena
    	        display_buffer[3] = (counter / 1000) % 10; // Mil

    	        char buffer[50]; //Crea arreglo de 50 caracteres 
    	        sprintf(buffer, "Número de carros que han ingresado: %d\n", counter); //Manda mensaje comn el numero que tiene almacenado Count, para los carros del parqueo
    	        USART2_Write_String(buffer);

    	        start_moving_servo(); // Realiza el movimiento del servo
    	    }
    	    else if (last_key == '*') { // Si se presiona la tecla *
    	        // Solo detiene el movimiento del servo, no afecta el conteo
    	        USART2_Write_String("Talanquera cerrada completamente\n");
    	        stop_servo(); // Detiene el servo
    	    }

    	    last_key = -1; // Resetea la tecla presionada
    	}
        }
    }

// Configuración de GPIO
void configure_gpio(void) {

    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;         // Activa el reloj para GPIOA
    GPIOA->MODER &= ~GPIO_MODER_MODE0_Msk;     // Limpia el modo del pin PA0
    GPIOA->MODER |= GPIO_MODER_MODE0_1;        // Configura PA0 como salida alternativa
    GPIOA->AFR[0] |= (0x2 << GPIO_AFRL_AFSEL0_Pos); // Selecciona AF2 para TIM21 en PA0

    // Habilitar reloj para GPIOB y GPIOC
    RCC->IOPENR |= (1 << 1) | (1 << 2);

    // Filas (PB8-PB11) como entradas con pull-up
    GPIOB->MODER &= ~(0xFF << 16);  // Input mode
    GPIOB->PUPDR |= (0x55 << 16);   // Pull-up

    // Columnas (PB12-PB15) como salidas
    GPIOB->MODER &= ~(0xFF << 24);
    GPIOB->MODER |= (0x55 << 24); // Output mode

    // Segmentos (PB0-PB7) como salidas
    GPIOB->MODER &= ~(0xFFFF);
    GPIOB->MODER |= (0x5555); // Output mode

    GPIOC->MODER &= ~((0x3 << (2 * 5)) | (0x3 << (2 * 6)) | (0x3 << (2 * 8)) | (0x3 << (2 * 4)));
    GPIOC->MODER |= ((0x1 << (2 * 5)) | (0x1 << (2 * 6)) | (0x1 << (2 * 8)) | (0x1 << (2 * 4)));

}

// Configuración de temporizadores
void configure_timers(void) {
	   
	    // TIM21: Escaneo del teclado @10ms
	    RCC->APB2ENR |= (1 << 2);   // Habilitar reloj para TIM21
	    TIM21->PSC = 15999;
	    TIM21->ARR = 1;
	    TIM21->CR1 = (1 << 0);      // Iniciar TIM21
	    TIM21->DIER |= (1 << 0);    // Habilitar interrupción de TIM21

	    // TIM6: Multiplexación del display @2ms
	    RCC->APB1ENR |= (1 << 4);
	    TIM6->PSC = 15999;
	    TIM6->ARR = 2;
	    TIM6->CR1 |= TIM_CR1_CEN; // Habilitar temporizador
	    TIM6->DIER |= (1 << 0);

	    // TIM2: Control del servomotor
	    RCC->APB1ENR |= (1 << 0);        // Habilitar reloj para TIM2
	    TIM2->PSC = 4 - 1;               // Configura el prescaler a 4 para obtener 4 MHz
	    TIM2->ARR = 19999;               // Configura el período para 20 ms (50 Hz)
	    TIM2->CCR1 = 1500;               // Pulso de 1.5 ms (posición central)
	    TIM2->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos);  // Configura el canal 1 en modo PWM1
	    TIM2->CCER |= TIM_CCER_CC1E;     // Habilita la salida del canal 1

	    // Habilitar interrupciones
	    NVIC_EnableIRQ(TIM2_IRQn);
	    NVIC_EnableIRQ(TIM6_IRQn);
	    NVIC_EnableIRQ(TIM21_IRQn);
}

// Escaneo del teclado
void scan_keypad(void) {
    static uint8_t col = 0;

    // Desactivar todas las columnas
    GPIOB->ODR |= 0xF000;

    // Activar la columna actual
    GPIOB->ODR &= ~(1 << (col + 12));

    // Leer filas
    uint16_t row_state = (GPIOB->IDR >> 8) & 0xF;

    for (uint8_t row = 0; row < 4; row++) {
        if (!(row_state & (1 << row))) { // Fila activa
            if (debounce_timer == 0) {
                last_key = KEYPAD_MAP[row][col]; // Lee el valor asignado de la matriz 
                debounce_timer = 20;
            }
            break;
        }
    }

    // Pasar a la siguiente columna
    col = (col + 1) % 4;
}

// Actualización del display  para realizar corrimiento de numeros 
void update_display(void) {
    // Apagar todos los dígitos
    GPIOC->ODR &= ~((1 << 5) | (1 << 6) | (1 << 8) | (1 << 4));

    // Actualizar segmentos
    GPIOB->ODR = (GPIOB->ODR & 0xFF00) | SEGMENT_MAP[display_buffer[current_digit]];

    // Encender el dígito actual
    GPIOC->ODR |= (1 << (5 + current_digit));

    // Pasar al siguiente dígito
    current_digit = (current_digit + 1) % 4;
}


// Uso del TIM21 para leer el teclado
void TIM21_IRQHandler(void) {
    TIM21->SR = 0; // Limpiar bandera de interrupción
    scan_keypad();
    if (debounce_timer > 0) debounce_timer--;
}

// Control del servomotor (ahora en TIM2)
void TIM2_IRQHandler(void) {
    TIM2->SR = 0; // Limpiar bandera de interrupción
    
}
//Uso del TIM6 para multiplexación de display, limpia la bandera de TIM 6 y llama a la función para limpiar los display
void TIM6_IRQHandler(void) {
    TIM6->SR = 0; // Limpiar bandera de interrupción
    update_display();
}

//Funcion para enviar un caracter por USART2, es la que nos ayuda a enviar los mensajes
void USART2_Write(char ch) {
    while (!(USART2->ISR & (1<<7)));  // Esperar a que el registro de datos esté vacío
    USART2->TDR = ch;
}

//Funcion para enviar una cadena por USART2
void USART2_Write_String(const char *str) {
    for (uint16_t i = 0; str[i] !='\0'; i++) {
        USART2_Write(str[i]);
    }
}

   //Mapeo de ángulos para el servo
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Ajuste del ángulo del servo (ahora con TIM2)
void servo_write(uint8_t angle) {
    if (angle < 0) angle = 0;       // para limitar ángulo mínimo
    if (angle > 180) angle = 180;   // Limitar ángulo máximo

    // Mapea el ángulo al rango de pulsos (1 ms a 2 ms, equivalente a 500 a 2500 en CCR1 (canal 1)
    TIM2->CCR1 = map(angle, 0, 180, 500, 2500);

}

// Función para iniciar el servo
void start_servo(void) {
    TIM2->CR1 |= TIM_CR1_CEN; // Inicia TIM2
}

// Función para detener el servo
void stop_servo(void) {
    TIM2->CR1 &= ~TIM_CR1_CEN; // Detiene TIM2
}
// Mueve el servo desde 0° hasta 180°
void servo_move(void) {
    for (uint8_t angle = 20; angle <= 400; angle++) {
        servo_write(angle); // Cambia el ángulo
        delay(10);          // Retardo para un movimiento suave
        if (last_key == '*') { // Verifica si se presionó '*'

            stop_servo();    // Detiene el servo
            last_key = -1;   // Resetea la tecla presionada
            return;          // Salir de la función
        }
    }
}


// Llama a esta función cuando quieras iniciar el movimiento
void start_moving_servo(void) {
    start_servo();    // Inicia el Timer 2 para generar PWM
    servo_move();     // Realiza el movimiento de 0° a 180°
    stop_servo();     // Detiene el Timer 2

}

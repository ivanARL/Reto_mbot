#include <Arduino_FreeRTOS.h>        // Biblioteca principal para usar FreeRTOS en Arduino
#include <avr/io.h>                  // Acceso directo a registros del microcontrolador AVR
#include <avr/interrupt.h>           // Permite usar interrupciones a bajo nivel
#include <semphr.h>                  // Biblioteca de FreeRTOS para usar semáforos
#include <MeMegaPi.h>                // Biblioteca para controlar el mBot Mega con motores

// === Configuración de los motores ===
MeMegaPiDCMotor motor_1(1);          // Motor en el puerto 1 (frontal derecho)
MeMegaPiDCMotor motor_9(9);          // Motor en el puerto 9 (trasero derecho)
MeMegaPiDCMotor motor_2(2);          // Motor en el puerto 2 (trasero izquierdo)
MeMegaPiDCMotor motor_10(10);        // Motor en el puerto 10 (frontal izquierdo)

// === Variables globales compartidas entre tareas e interrupciones ===
volatile int anguloRecibido = 90;    // Ángulo recibido por UART, inicia en 90° (recto)
volatile bool interrupcionActiva = false; // Bandera para indicar si la interrupción se activó

// === Handles de FreeRTOS ===
SemaphoreHandle_t xPinASem;          // Semáforo binario para sincronizar interrupción con tarea
TaskHandle_t xHandleMovimiento = NULL; // Handle de la tarea de movimiento, puede usarse para suspender o reanudar

// === Inicialización UART con registros AVR ===
void uart_init() {
  uint16_t ubrr = 103;               // Valor para 9600 baudios con cristal de 16 MHz
  UBRR0H = (ubrr >> 8);              // Parte alta del registro de configuración
  UBRR0L = ubrr;                     // Parte baja del registro de configuración
  UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Habilita recepción y transmisión UART
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Formato: 8 bits, sin paridad, 1 bit de parada
}

void uart_putchar(char c) {
  while (!(UCSR0A & (1 << UDRE0)));  // Espera a que el registro de envío esté vacío
  UDR0 = c;                          // Envía el carácter por UART
}

void uart_print(const char* str) {
  while (*str) uart_putchar(*str++); // Envía cadena carácter por carácter
}

char uart_getchar() {
  while (!(UCSR0A & (1 << RXC0)));   // Espera a que haya datos recibidos
  return UDR0;                       // Retorna el carácter recibido
}

bool uart_available() {
  return (UCSR0A & (1 << RXC0));     // Devuelve verdadero si hay datos disponibles para leer
}

// === Control individual de motores con dirección ===
void motor_foward_left_run(int16_t speed) { motor_10.run(speed); }
void motor_foward_right_run(int16_t speed) { motor_1.run(-speed); }
void motor_back_left_run(int16_t speed) { motor_2.run(speed); }
void motor_back_right_run(int16_t speed) { motor_9.run(-speed); }

// === Control de movimiento general usando vector de velocidad ===
void move_control(int16_t vx, int16_t vy, int16_t vw) {
  // Cálculo de velocidad para cada rueda usando combinación lineal de vectores
  int16_t fl = vy + vx + vw;  // Rueda frontal izquierda
  int16_t fr = vy - vx - vw;  // Rueda frontal derecha
  int16_t bl = vy - vx + vw;  // Rueda trasera izquierda
  int16_t br = vy + vx - vw;  // Rueda trasera derecha
  motor_foward_left_run(fl);
  motor_foward_right_run(fr);
  motor_back_left_run(bl);
  motor_back_right_run(br);
}

void stop_motors() {
  move_control(0, 0, 0); // Detiene todos los motores
}

// === Tarea que recibe ángulos enviados por UART desde la Raspberry Pi ===
void tareaRecibirAngulo(void *pvParameters) {
  static char input[10];            // Buffer para almacenar entrada serial
  static uint8_t index = 0;         // Índice del buffer

  while (1) {
    while (uart_available()) {      // Si hay datos disponibles
      char c = uart_getchar();      // Leer un carácter

      if (c == '\n') {              // Si es fin de línea
        input[index] = '\0';        // Terminar la cadena
        int ang = atoi(input);      // Convertir a entero
        if (ang < 0) ang = -ang;    // Asegurar que sea positivo
        if (ang >= 0 && ang <= 180) {
          anguloRecibido = ang;     // Guardar el ángulo si es válido
          uart_print("Ángulo recibido: ");
        } else {
          uart_print("Valor no válido: ");
        }
        uart_print(input);          // Imprimir valor leído
        uart_putchar('\n');         // Nueva línea
        index = 0;                  // Reiniciar índice
      } else if (c != '\r' && index < sizeof(input) - 1) {
        input[index++] = c;         // Guardar carácter en buffer
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // Esperar 10 ms antes de revisar otra vez
  }
}

// === Tarea que controla el movimiento en base al ángulo recibido ===
void tareaControlMovimiento(void *pvParameters) {
  const int vel_recto_lento = 80;
  const int vel_curva = 120;
  const int vel_giro_fuerte = 300;
  const int vel_giro_moderado = 200;

  int contadorCurva = 0;           // Cuenta pasos rectos antes de girar
  bool girando = false;            // Bandera para indicar si está girando

  while (1) {
    // === Si se activó la interrupción por el pin A8 ===
    if (xSemaphoreTake(xPinASem, 0) == pdPASS || interrupcionActiva) {
      stop_motors();               // Detener los motores
      while ((PINK & (1 << PK0)) == 0) {
        vTaskDelay(20 / portTICK_PERIOD_MS); // Esperar a que se libere el pin
      }
      interrupcionActiva = false;
      continue;
    }

    int ang = anguloRecibido;
    static bool giroDerecha = true;

    if (ang < 0 || ang > 180) {
      // Valor inválido: gira alternadamente izquierda/derecha
      int giro = giroDerecha ? vel_giro_moderado : -vel_giro_moderado;
      move_control(0, -vel_curva, giro);
      giroDerecha = !giroDerecha;
      vTaskDelay(60 / portTICK_PERIOD_MS);
      girando = false;
      contadorCurva = 0;
    } else if (ang >= 80 && ang <= 100) {
      // Va recto
      move_control(0, -vel_recto_lento, 0);
      girando = false;
      contadorCurva = 0;
    } else if ((ang < 80 || ang > 100) && contadorCurva < 30 && !girando) {
      // Mantiene trayectoria recta antes de girar
      move_control(0, -vel_recto_lento, 0);
      contadorCurva++;
    } else if (contadorCurva >= 30 || girando) {
      // Comienza a girar según el ángulo
      girando = true;
      if (ang < 80) {
        int giro = (ang < 50) ? vel_giro_fuerte : vel_giro_moderado;
        move_control(0, -vel_curva, giro);
      } else if (ang > 100) {
        int giro = (ang > 130) ? -vel_giro_fuerte : -vel_giro_moderado;
        move_control(0, -vel_curva, giro);
      }
      vTaskDelay(61 / portTICK_PERIOD_MS);
      girando = false;
      contadorCurva = 0;
    }

    vTaskDelay(46 / portTICK_PERIOD_MS); // Tiempo base de actualización
  }
}

// === Configuración inicial ===
void setup() {
  uart_init();                     // Inicializar comunicación UART
  uart_print("Esperando datos...\n");

  // === Configurar pin A8 como entrada con resistencia pull-up ===
  DDRK &= ~(1 << PK0);             // A8 (PK0) como entrada
  PORTK |= (1 << PK0);             // Habilitar resistencia pull-up

  // === Configurar interrupción por cambio de estado en A8 ===
  PCICR |= (1 << PCIE2);           // Habilitar interrupciones del grupo PCINT[23:16]
  PCMSK2 |= (1 << PCINT16);        // Habilitar interrupción individual para PCINT16 (A8)
  sei();                           // Habilitar interrupciones globales

  // === Crear semáforo y tareas ===
  xPinASem = xSemaphoreCreateBinary(); // Crear semáforo binario
  xTaskCreate(tareaRecibirAngulo, "LeerSerial", 128, NULL, 1, NULL); // Crear tarea de lectura UART
  xTaskCreate(tareaControlMovimiento, "Control", 128, NULL, 1, &xHandleMovimiento); // Crear tarea de movimiento

  vTaskStartScheduler();           // Iniciar el planificador de FreeRTOS
}

// === Loop no se usa en FreeRTOS ===
void loop() {
  // Nunca se ejecuta porque FreeRTOS toma el control
}

// === Interrupción cuando cambia el pin A8 ===
ISR(PCINT2_vect) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if ((PINK & (1 << PK0)) == 0) { // Si A8 está en LOW (activación)
    interrupcionActiva = true;    // Señalamos que se activó interrupción
    xSemaphoreGiveFromISR(xPinASem, &xHigherPriorityTaskWoken); // Liberamos el semáforo desde ISR
    if (xHigherPriorityTaskWoken) taskYIELD(); // Cambio de contexto si una tarea de mayor prioridad estaba esperando
  }
}

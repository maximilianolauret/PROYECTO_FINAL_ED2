/*
 * ============================================================================
 *  Soporte de soldadura "manos libres" - Proyecto final Electronica Digital 2
 *  PROGRAMA FINAL - version con pines alternados (un pin libre entre cada
 *  conexion) para la placa perforada
 * ============================================================================
 *  MCU        : PIC16F887 @ 8 MHz (oscilador interno, sin cristal)
 *  Compilador : XC8 (MPLAB X)
 *
 *  Eje Y    : NEMA17 + A4988      -> pulsadores SUBIR / BAJAR
 *  Rotacion : 28BYJ-48 + ULN2003A -> potenciometro = angulo 0..360 grados
 *
 *  Conexiones (pin del DIP-40):
 *    pin  2  RA0/AN0  Cursor del potenciometro (+ 100 nF a GND). Entrada analogica.
 *    pin  9  RE1      A4988 EN (activo en bajo, pull-up 10k a +5V). Se habilita
 *                     3 s despues de arrancar.
 *    pin 15  RC0      A4988 STEP
 *    pin 17  RC2      A4988 DIR
 *    pin 19  RD0      ULN2003A IN1
 *    pin 21  RD2      ULN2003A IN2
 *    pin 27  RD4      ULN2003A IN3
 *    pin 29  RD6      ULN2003A IN4
 *    pin 34  RB1      Pulsador BAJAR (a GND, pull-up interno de PORTB)
 *    pin 36  RB3      Pulsador SUBIR (a GND, pull-up interno de PORTB)
 *    pin 37  RB4      Final de carrera INFERIOR: sin conectar (pull-up interno
 *    pin 38  RB5      Final de carrera SUPERIOR:  lo deja siempre inactivo)
 *    pin 30  RD7      LED de vida (opcional, sin conectar en la placa)
 *    MCLR             Libre (MCLRE = OFF)
 *    RB6/RB7          Libres (PGC/PGD para grabar)
 *
 *    A4988: EN en RE1 con pull-up 10k (deshabilitado hasta que el PIC
 *           lo baje), RESET unido a SLEEP,
 *           MS1/MS2/MS3 sin conectar (pull-downs internos -> paso completo),
 *           VDD = 5 V, VMOT = 12 V con capacitor 100 uF.
 *    Alimentacion: fuente de 5 V para todo, fuente de 12 V solo para VMOT,
 *                  masas unidas.
 *
 *  Funcionamiento:
 *    - Timer0 interrumpe cada 1 ms. La ISR genera los pasos de AMBOS motores
 *      (sin delays), por eso se pueden mover los dos a la vez.
 *    - El main, cada 10 ms, lee pulsadores (con antirrebote) y el ADC, y
 *      solo actualiza las variables de mando (y_cmd, rot_objetivo).
 *    - Al encender, el A4988 queda deshabilitado (EN = 1) durante 3 s para
 *      que la alimentacion del motor se estabilice; despues queda habilitado.
 *    - El eje Y se mueve mientras se mantiene un pulsador; ninguno o ambos:
 *      quieto, con torque de retencion.
 *    - Finales de carrera (si algun dia se conectan, a GND): con el superior
 *      activo no sube, con el inferior activo no baja.
 * ============================================================================
 */

/* ----------------------------- Bits de configuracion ---------------------- */
#pragma config FOSC  = INTRC_NOCLKOUT   // Oscilador interno, RA6/RA7 como I/O
#pragma config WDTE  = OFF              // Watchdog apagado
#pragma config PWRTE = ON               // Power-up timer
#pragma config MCLRE = OFF              // RE3 como entrada (sin pull-up en MCLR)
#pragma config CP    = OFF
#pragma config CPD   = OFF
#pragma config BOREN = ON
#pragma config IESO  = OFF
#pragma config FCMEN = OFF
#pragma config LVP   = OFF              // Necesario para usar RB3 como I/O normal
#pragma config BOR4V = BOR21V           // Brown-out a 2.1 V
#pragma config WRT   = OFF

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 8000000UL

/* ----------------------------- Parametros ajustables ---------------------- */
#define Y_DIR_SUBE            1   // Nivel de DIR que hace SUBIR (invertir si va al reves)
#define Y_DIV                 1   // ms entre flancos de STEP -> f_paso = 1000/(2*Y_DIV) Hz

/* 28BYJ-48 en PASO COMPLETO de 2 fases (2 bobinas a la vez): maximo torque.
   2048 pasos por vuelta del eje de salida. */
#define ROT_PASOS_POR_CUENTA  2   // 1023 cuentas ADC * 2 = 2046 pasos ~ 360 grados
#define ROT_HISTERESIS        4   // cuentas ADC de zona muerta (evita temblor por ruido)

/* Rampa de velocidad (ms por paso): arranca lento, acelera y frena al llegar.
   Ultimo valor = velocidad maxima. 3 ms -> 333 pasos/s -> ~6 s por vuelta. */
static const uint8_t RAMPA[] = { 10, 7, 5, 4, 3 };
#define RAMPA_N      (sizeof(RAMPA) / sizeof(RAMPA[0]))
#define RAMPA_PASOS  16           // pasos que dura cada escalon de la rampa

#define LED_VIDA_MS         500   // Periodo de parpadeo del LED de vida (RD7)

/* ----------------------------- Pines -------------------------------------- */
#define BTN_SUBIR     PORTBbits.RB3
#define BTN_BAJAR     PORTBbits.RB1
#define FC_SUPERIOR   PORTBbits.RB5
#define FC_INFERIOR   PORTBbits.RB4
#define PRESIONADO    0           // Pulsadores y finales de carrera a GND -> activos en bajo

#define MASK_STEP     0x01        // RC0
#define MASK_DIR      0x04        // RC2

#define MASK_LED      0x80        // RD7

#define A4988_EN      PORTEbits.RE1   // Activo en bajo
#define EN_DEMORA_MS  3000            // Espera antes de habilitar el driver

/* Timer0: Fosc/4 = 2 MHz, prescaler 1:8 -> 4 us/cuenta; 250 cuentas = 1 ms */
#define TMR0_RECARGA  6

/* Comandos del eje Y */
#define Y_STOP        0
#define Y_SUBIR       1
#define Y_BAJAR       2

/* Secuencia de paso completo, 2 fases, ya escrita en los bits de PORTD:
   IN1 = RD0 (0x01), IN2 = RD2 (0x04), IN3 = RD4 (0x10), IN4 = RD6 (0x40)
   IN1+IN2, IN2+IN3, IN3+IN4, IN4+IN1 */
static const uint8_t SECUENCIA[4] = { 0x05, 0x14, 0x50, 0x41 };

/* ----------------------------- Variables compartidas ---------------------- */
volatile uint8_t  y_cmd        = Y_STOP;  // Escrita por main, leida por ISR
volatile int16_t  rot_objetivo = 0;       // Escrita por main (con GIE=0), leida por ISR
volatile int16_t  rot_actual   = 0;       // Solo la modifica la ISR
volatile uint8_t  flag_10ms    = 0;       // Base de tiempo para el main

/* Variables propias de la ISR */
static uint8_t  sh_portc     = 0x00;      // Sombra de PORTC
static uint8_t  bobinas      = 0x00;      // Bits de PORTD de IN1..IN4
static uint8_t  led          = 0x00;      // Estado de RD7
static uint8_t  y_dir_actual = 0xFF;      // 0xFF = DIR todavia no fijado
static uint8_t  cont_y       = 0;
static uint8_t  cont_rot     = 0;
static uint8_t  cont_10ms    = 0;
static uint16_t cont_led     = 0;
static uint8_t  fase         = 0;         // Indice en SECUENCIA[]
static uint8_t  periodo_rot  = 10;        // ms hasta el proximo paso (segun rampa)
static uint16_t pasos_rampa  = 0;         // pasos dados desde el arranque
static int8_t   ultima_dir   = 0;         // +1, -1 o 0 (quieto)

/* ============================================================================
 *  Interrupcion: Timer0 cada 1 ms
 * ============================================================================ */
void __interrupt() isr(void)
{
    if (INTCONbits.T0IE && INTCONbits.T0IF)
    {
        TMR0 = TMR0_RECARGA;
        INTCONbits.T0IF = 0;

        /* ---- Base de tiempo de 10 ms para el programa principal ---- */
        if (++cont_10ms >= 10) {
            cont_10ms = 0;
            flag_10ms = 1;
        }

        /* ---- LED de vida ---- */
        if (++cont_led >= LED_VIDA_MS) {
            cont_led = 0;
            led ^= MASK_LED;
        }

        /* ---- Eje Y (A4988) ---- */
        uint8_t mover_y = 0;
        if      (y_cmd == Y_SUBIR) mover_y = (FC_SUPERIOR != PRESIONADO);
        else if (y_cmd == Y_BAJAR) mover_y = (FC_INFERIOR != PRESIONADO);

        if (mover_y) {
            uint8_t dir = (y_cmd == Y_SUBIR) ? Y_DIR_SUBE : !Y_DIR_SUBE;

            if (dir != y_dir_actual) {
                /* Cambio de sentido: se fija DIR y se espera 1 ms antes de
                   dar pasos (el A4988 pide DIR estable antes del flanco). */
                y_dir_actual = dir;
                if (dir) sh_portc |=  MASK_DIR;
                else     sh_portc &= ~MASK_DIR;
                sh_portc &= ~MASK_STEP;
                cont_y = 0;
            } else if (++cont_y >= Y_DIV) {
                cont_y = 0;
                sh_portc ^= MASK_STEP;           // Cada flanco de subida = 1 paso
            }
        } else {
            sh_portc &= ~MASK_STEP;              // STEP en reposo en bajo
            cont_y = 0;
        }
        PORTC = sh_portc;

        /* ---- Rotacion (28BYJ-48 + ULN2003A) ---- */
        if (++cont_rot >= periodo_rot) {
            cont_rot = 0;
            int16_t dif = rot_objetivo - rot_actual;

            if (dif == 0) {
                /* Llego: bobinas apagadas (la reduccion 1:64 sostiene) */
                bobinas     = 0x00;
                pasos_rampa = 0;
                ultima_dir  = 0;
                periodo_rot = RAMPA[0];
            } else {
                int8_t   dir  = (dif > 0) ? 1 : -1;
                uint16_t dist = (uint16_t)((dif > 0) ? dif : -dif);

                if (dir != ultima_dir) {        // Arranque o cambio de sentido:
                    ultima_dir  = dir;          // vuelve a empezar lento
                    pasos_rampa = 0;
                }

                rot_actual += dir;
                fase = (uint8_t)((fase + dir) & 0x03);
                bobinas = SECUENCIA[fase];

                if (pasos_rampa < 0xFFFF) pasos_rampa++;

                uint16_t n_acel  = pasos_rampa / RAMPA_PASOS;
                uint16_t n_freno = (dist - 1) / RAMPA_PASOS;
                uint16_t n = (n_acel < n_freno) ? n_acel : n_freno;
                if (n > RAMPA_N - 1) n = RAMPA_N - 1;
                periodo_rot = RAMPA[n];
            }
        }
        PORTD = bobinas | led;                   // Bobinas (RD0/2/4/6) + LED (RD7)
    }
}

/* ============================================================================
 *  Inicializacion
 * ============================================================================ */
static void init_oscilador(void)
{
    OSCCON = 0x71;                  // IRCF=111 (8 MHz), SCS=1 (oscilador interno)
}

static void init_puertos(void)
{
    ANSEL  = 0x01;                  // Solo AN0 analogico
    ANSELH = 0x00;                  // PORTB digital (en el 887 arranca analogico)

    TRISA = 0x01;                   // RA0 entrada (potenciometro)
    TRISB = 0x3A;                   // RB1, RB3 (pulsadores) y RB4, RB5 (fin. carrera) entradas
    TRISC = 0x00;                   // STEP, DIR salidas
    TRISD = 0x00;                   // Bobinas y LED salidas
    PORTE = 0x02;                   // RE1 = 1 -> A4988 deshabilitado (antes del TRIS)
    TRISE = 0x00;                   // RE1 salida (AN6 ya es digital: ANSEL = 0x01)

    PORTA = 0x00;
    PORTB = 0x00;
    PORTD = 0x00;
    sh_portc = 0x00;                // STEP=0, DIR=0
    PORTC = sh_portc;

    WPUB = 0x3A;                    // Pull-ups internos en RB1, RB3, RB4, RB5
}

static void init_adc(void)
{
    ADCON1 = 0x80;                  // Justificado a derecha, Vref = VDD/VSS
    ADCON0 = 0x81;                  // Fosc/32 (Tad = 4 us), canal AN0, ADC encendido
    __delay_us(20);
}

static void init_timer0(void)
{
    OPTION_REG = 0x02;              // nRBPU=0 (pull-ups ON), T0CS=0, PSA=0, PS=1:8
    TMR0 = TMR0_RECARGA;
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;
    INTCONbits.GIE  = 1;
}

/* ============================================================================
 *  ADC
 * ============================================================================ */
static uint16_t leer_adc(void)
{
    __delay_us(10);                 // Tiempo de adquisicion
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE);
    return (uint16_t)((ADRESH << 8) | ADRESL);
}

static uint16_t leer_adc_promedio(void)
{
    uint16_t suma = 0;
    for (uint8_t i = 0; i < 8; i++) {
        suma += leer_adc();
    }
    return suma >> 3;               // Promedio de 8 muestras (0..1023)
}

static void fijar_objetivo_rot(uint16_t adc)
{
    int16_t obj = (int16_t)(adc * ROT_PASOS_POR_CUENTA);
    INTCONbits.GIE = 0;             // Escritura atomica de 16 bits
    rot_objetivo = obj;
    INTCONbits.GIE = 1;
}

/* ============================================================================
 *  Programa principal
 * ============================================================================ */
void main(void)
{
    uint8_t  lectura, lectura_ant = 0, estable = 0;
    uint16_t adc, adc_ref;

    init_oscilador();
    init_puertos();
    init_adc();
    __delay_ms(50);                 // Estabilizacion de alimentacion y drivers

    /* Driver deshabilitado 3 s: se espera a que VMOT este estable */
    for (uint8_t i = 0; i < EN_DEMORA_MS / 100; i++) {
        __delay_ms(100);
    }
    A4988_EN = 0;                   // Habilitar A4988

    /* La posicion del potenciometro al encender se toma como posicion actual
       de la PCB (no hay sensor de origen en el eje de rotacion). */
    adc_ref      = leer_adc_promedio();
    rot_objetivo = (int16_t)(adc_ref * ROT_PASOS_POR_CUENTA);
    rot_actual   = rot_objetivo;

    init_timer0();                  // A partir de aca corre la ISR

    while (1)
    {
        if (!flag_10ms) continue;   // Ejecutar cada 10 ms
        flag_10ms = 0;

        /* ---- Pulsadores con antirrebote (2 lecturas iguales = 20 ms) ---- */
        lectura = 0;
        if (BTN_SUBIR == PRESIONADO) lectura |= 0x01;
        if (BTN_BAJAR == PRESIONADO) lectura |= 0x02;

        if (lectura == lectura_ant) estable = lectura;
        lectura_ant = lectura;

        if      (estable == 0x01) y_cmd = Y_SUBIR;
        else if (estable == 0x02) y_cmd = Y_BAJAR;
        else                      y_cmd = Y_STOP;   // Ninguno o ambos: quieto

        /* ---- Potenciometro -> angulo de la PCB ---- */
        adc = leer_adc_promedio();

        if ((adc > adc_ref + ROT_HISTERESIS) ||
            (adc + ROT_HISTERESIS < adc_ref) ||
            ((adc == 0 || adc == 1023) && adc != adc_ref))   // Permite llegar a los extremos
        {
            adc_ref = adc;
            fijar_objetivo_rot(adc);
        }
    }
}
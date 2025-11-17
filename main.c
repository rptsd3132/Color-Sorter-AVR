#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>      // for sprintf
#include <math.h>       // for fabsf

// --- Pin Definitions (PORTx, PINx, DDRx) ---
#define LIFT_SERVO_PIN    PB1  // OC1A
#define SORT_SERVO_PIN    PB2  // OC1B

#define RED_LED    PB4
#define YELLOW_LED PB5
#define GREEN_LED  PB0

// Sensor 1
#define S0_1 PD4
#define S1_1 PD5
#define S2_1 PD6
#define S3_1 PD7
#define OUT_1 PD2
#define OE_1  PC5

// Sensor 2
#define S0_2 PC0
#define S1_2 PC1
#define S2_2 PC2
#define S3_2 PC3
#define OUT_2 PD3
#define OE_2  PC4

// --- Constants ---
#define NUM_SAMPLES 10
#define GATE_US 20000UL        // 20ms gate time
#define CLOSE_DELTA 0.1f       // tolerance
#define CYCLE_DELAY_MS 20000UL // 20 seconds

// Thresholds
typedef struct {
    float rMin, rMax, bMin, bMax;
} Thresholds;

Thresholds RED_THRESH    = {1.9f, 2.6f, 1.28f, 1.69f};
Thresholds YELLOW_THRESH = {1.3f, 1.5f, 0.65f, 0.8f};
Thresholds GREEN_THRESH  = {0.82f, 1.1f, 0.9f, 1.12f};

// --- Global Variables ---
volatile uint32_t timer0_overflow_count = 0;
uint32_t lastCycle = 0;
int lastColorAngle = -1;

// --- Function Prototypes ---
void uart_init(void);
void uart_transmit(char data);
void uart_print(const char* str);
void uart_println(const char* str);
void uart_print_int(int val);
void uart_print_float(float val);

void timer0_init(void);
uint32_t get_micros(void);
uint32_t millis(void);

void servo_init(void);
void set_servo_angle(uint8_t servo, uint8_t angle);

void sensor_init(void);
void set_sensor_filter(uint8_t sensor, uint8_t s2_state, uint8_t s3_state);
float measureHz(uint8_t sensor, uint8_t s2_state, uint8_t s3_state);
float measureAvg(uint8_t sensor, uint8_t s2_state, uint8_t s3_state);
float distanceFromThreshold(float r, float b, Thresholds t);
const char* detectColorClose(float r, float b);
int colorToAngle(const char* color);
void setLEDs(const char* color);

// --- UART Functions (9600 baud) ---
void uart_init(void) {
    UBRR0H = 0;
    UBRR0L = (F_CPU / 16 / 9600 - 1); // 16MHz, 9600 baud → UBRR0 = 103
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8N1
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void uart_print(const char* str) {
    while (*str) uart_transmit(*str++);
}

void uart_println(const char* str) {
    uart_print(str);
    uart_print("\r\n");
}

void uart_print_int(int val) {
    char buf[12];
    sprintf(buf, "%d", val);
    uart_print(buf);
}

void uart_print_float(float val) {
    int i = (int)val;
    int frac = (int)((val - i) * 100);
    if (frac < 0) frac = -frac;
    uart_print_int(i);
    uart_transmit('.');
    if (frac < 10) uart_transmit('0');
    uart_print_int(frac);
}

// --- Microsecond Timer (Timer0) ---
void timer0_init(void) {
    TCCR0A = 0;
    TCCR0B = (1<<CS01) | (1<<CS00); // prescaler 64 => 1 tick = 4us @16MHz
    TIMSK0 = (1<<TOIE0);            // enable overflow interrupt
    sei();                          // enable global interrupts
}

ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
}

uint32_t get_micros(void) {
    uint8_t oldSREG = SREG;
    cli();
    uint32_t m = timer0_overflow_count;
    uint8_t t = TCNT0;
    SREG = oldSREG;
    return (m * 256UL + t) * 4UL; // each tick = 4us
}

uint32_t millis(void) {
    uint8_t oldSREG = SREG;
    cli();
    uint32_t m = timer0_overflow_count;
    SREG = oldSREG;
    return m * (256UL * 4UL / 1000); // 256 * 4us = 1.024ms per overflow
}

// --- Servo PWM (Timer1 - 50Hz, 20ms period) ---
void servo_init(void) {
    DDRB |= (1<<LIFT_SERVO_PIN) | (1<<SORT_SERVO_PIN); // Set OC1A/OC1B as output

    // Fast PWM, TOP=ICR1, non-inverting
    TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
    TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11); // prescaler 8

    ICR1 = 40000; // 20ms period: 16MHz/8 = 2MHz, 2MHz * 0.02s = 40000

    // Initial positions
    OCR1A = 1000; // 0° ~ 1ms pulse (1000 ticks @ 0.5us/tick)
    OCR1B = 1000;
}

void set_servo_angle(uint8_t servo, uint8_t angle) {
    // Angle 0-180 maps to 1000-5000 (1ms to 2.5ms pulse width)
    uint16_t pulse = 1000 + (uint32_t)angle * 4000 / 180;
    if (servo == 0) {
        OCR1A = pulse;
    } else if (servo == 1) {
        OCR1B = pulse;
    }
}

// --- GPIO Helper Macros ---
#define SET_BIT(reg, bit)   ((reg) |= (1<<(bit)))
#define CLEAR_BIT(reg, bit) ((reg) &= ~(1<<(bit)))
#define TOGGLE_BIT(reg, bit) ((reg) ^= (1<<(bit)))
#define READ_BIT(reg, bit)  (((reg) >> (bit)) & 1)

// --- Sensor Control Functions ---
void sensor_init(void) {
    // Sensor 1
    DDRD |= (1<<S0_1) | (1<<S1_1) | (1<<S2_1) | (1<<S3_1);
    DDRC |= (1<<OE_1);
    // Sensor 2
    DDRC |= (1<<S0_2) | (1<<S1_2) | (1<<S2_2) | (1<<S3_2) | (1<<OE_2);

    // Output Enable = LOW (active)
    CLEAR_BIT(PORTC, OE_1);
    CLEAR_BIT(PORTC, OE_2);

    // Frequency scaling: S0=S1=HIGH (20% scaling)
    SET_BIT(PORTD, S0_1); SET_BIT(PORTD, S1_1);
    SET_BIT(PORTC, S0_2); SET_BIT(PORTC, S1_2);
}

void set_sensor_filter(uint8_t sensor, uint8_t s2_state, uint8_t s3_state) {
    if (sensor == 0) {
        if (s2_state) SET_BIT(PORTD, S2_1); else CLEAR_BIT(PORTD, S2_1);
        if (s3_state) SET_BIT(PORTD, S3_1); else CLEAR_BIT(PORTD, S3_1);
    } else {
        if (s2_state) SET_BIT(PORTC, S2_2); else CLEAR_BIT(PORTC, S2_2);
        if (s3_state) SET_BIT(PORTC, S3_2); else CLEAR_BIT(PORTC, S3_2);
    }
    _delay_us(200);
}

float measureHz(uint8_t sensor, uint8_t s2_state, uint8_t s3_state) {
    uint8_t out_pin = (sensor == 0) ? OUT_1 : OUT_2;
    volatile uint8_t* pin_reg = (sensor == 0) ? &PIND : &PIND; // OUT_1/OUT_2 both on PORTD

    set_sensor_filter(sensor, s2_state, s3_state);

    uint32_t tstart = get_micros();
    uint32_t cycles = 0;

    while ((get_micros() - tstart) < GATE_US) {
        uint32_t highT = 0, lowT = 0;
        uint32_t timeout = tstart + GATE_US;

        // Measure HIGH pulse
        while (READ_BIT(*pin_reg, out_pin) == 0 && get_micros() < timeout);
        uint32_t start_high = get_micros();
        while (READ_BIT(*pin_reg, out_pin) == 1 && get_micros() < timeout);
        highT = get_micros() - start_high;

        // Measure LOW pulse
        while (READ_BIT(*pin_reg, out_pin) == 1 && get_micros() < timeout);
        uint32_t start_low = get_micros();
        while (READ_BIT(*pin_reg, out_pin) == 0 && get_micros() < timeout);
        lowT = get_micros() - start_low;

        if (highT == 0 || lowT == 0) break;
        cycles++;
    }

    float seconds = (get_micros() - tstart) / 1000000.0f;
    if (seconds <= 0 || cycles == 0) return 0.0f;
    return cycles / seconds;
}

float measureAvg(uint8_t sensor, uint8_t s2_state, uint8_t s3_state) {
    float sum = 0.0f;
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        sum += measureHz(sensor, s2_state, s3_state);
        _delay_ms(10); // small delay between samples
    }
    return sum / NUM_SAMPLES;
}

float distanceFromThreshold(float r, float b, Thresholds t) {
    float dr = (r < t.rMin) ? (t.rMin - r) : (r > t.rMax ? r - t.rMax : 0.0f);
    float db = (b < t.bMin) ? (t.bMin - b) : (b > t.bMax ? b - t.bMax : 0.0f);
    return dr + db;
}

const char* detectColorClose(float r, float b) {
    float dR = distanceFromThreshold(r, b, RED_THRESH);
    float dY = distanceFromThreshold(r, b, YELLOW_THRESH);
    float dG = distanceFromThreshold(r, b, GREEN_THRESH);

    if (dR <= dY && dR <= dG) return "Red";
    if (dY <= dR && dY <= dG) return "Yellow";
    return "Green";
}

int colorToAngle(const char* color) {
    if (color[0] == 'G') return 10;
    if (color[0] == 'R') return 65;
    if (color[0] == 'Y') return 120;
    return 175; // Unknown
}

void setLEDs(const char* color) {
    if (color[0] == 'R') {
        SET_BIT(PORTB, RED_LED);
        CLEAR_BIT(PORTB, YELLOW_LED);
        CLEAR_BIT(PORTB, GREEN_LED);
    } else if (color[0] == 'Y') {
        CLEAR_BIT(PORTB, RED_LED);
        SET_BIT(PORTB, YELLOW_LED);
        CLEAR_BIT(PORTB, GREEN_LED);
    } else if (color[0] == 'G') {
        CLEAR_BIT(PORTB, RED_LED);
        CLEAR_BIT(PORTB, YELLOW_LED);
        SET_BIT(PORTB, GREEN_LED);
    } else {
        CLEAR_BIT(PORTB, RED_LED);
        CLEAR_BIT(PORTB, YELLOW_LED);
        CLEAR_BIT(PORTB, GREEN_LED);
    }
}

// --- Main ---
int main(void) {
    // Initialize
    uart_init();
    timer0_init();
    servo_init();

    // LED pins
    DDRB |= (1<<RED_LED) | (1<<YELLOW_LED) | (1<<GREEN_LED);
    CLEAR_BIT(PORTB, RED_LED);
    CLEAR_BIT(PORTB, YELLOW_LED);
    CLEAR_BIT(PORTB, GREEN_LED);

    sensor_init();

    // Start servos
    set_servo_angle(0, 0); // lift
    set_servo_angle(1, 0); // sort
    _delay_ms(500);

    uart_println("Starting 2-servo color detection with lift cycle 0-90-180-0 (UART @ 9600)");

    while (1) {
        uint32_t now = millis();
        if ((now - lastCycle) < CYCLE_DELAY_MS) continue;
        lastCycle = now;

        uart_println("=== New cycle ===");

        // 1) Lift servo: 0 -> 90
        set_servo_angle(0, 90);
        _delay_ms(1000);

        // 2) Measure sensors
        float R_1 = measureAvg(0, 0, 0); // S2=LOW, S3=LOW (Red)
        float B_1 = measureAvg(0, 0, 1); // S2=LOW, S3=HIGH (Blue)
        float G_1 = measureAvg(0, 1, 1); // S2=HIGH, S3=HIGH (Green)
        float rOverG_1 = (G_1 > 0) ? R_1 / G_1 : 0.0f;
        float bOverG_1 = (G_1 > 0) ? B_1 / G_1 : 0.0f;

        float R_2 = measureAvg(1, 0, 0);
        float B_2 = measureAvg(1, 0, 1);
        float G_2 = measureAvg(1, 1, 1);
        float rOverG_2 = (G_2 > 0) ? R_2 / G_2 : 0.0f;
        float bOverG_2 = (G_2 > 0) ? B_2 / G_2 : 0.0f;

        // Compare sensors
        const char* finalColor;
        if (fabsf(rOverG_1 - rOverG_2) < CLOSE_DELTA && fabsf(bOverG_1 - bOverG_2) < CLOSE_DELTA) {
            finalColor = detectColorClose(rOverG_1, bOverG_1);
        } else {
            const char* c1 = detectColorClose(rOverG_1, bOverG_1);
            const char* c2 = detectColorClose(rOverG_2, bOverG_2);
            Thresholds t1 = (c1[0]=='R') ? RED_THRESH : (c1[0]=='Y' ? YELLOW_THRESH : GREEN_THRESH);
            Thresholds t2 = (c2[0]=='R') ? RED_THRESH : (c2[0]=='Y' ? YELLOW_THRESH : GREEN_THRESH);
            float d1 = distanceFromThreshold(rOverG_1, bOverG_1, t1);
            float d2 = distanceFromThreshold(rOverG_2, bOverG_2, t2);
            finalColor = (d1 <= d2) ? c1 : c2;
        }

        setLEDs(finalColor);

        // 3) Lift: 90 -> 180
        set_servo_angle(0, 180);
        _delay_ms(1000);

        // 4) Lift: 180 -> 0
        set_servo_angle(0, 0);
        _delay_ms(500);

        // 5) Sort servo if color changed
        int angle = colorToAngle(finalColor);
        if (angle != lastColorAngle) {
            set_servo_angle(1, angle);
            lastColorAngle = angle;
            _delay_ms(1000);
        }

        uart_print("Detected Color: ");
        uart_println(finalColor);
        uart_println("------------------------");
    }
}

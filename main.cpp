#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define IN1 PD2
#define IN2 PD3
#define IN3 PD4
#define IN4 PD5

#define ENA PB1 // Arduino D9 / OC1A / linker Motor
#define ENB PB2 // Arduino D10 / OC1B / rechter Motor

#define BAUD 115200UL

#define PWM_TOP    400
#define SPEED_MIN  240   // darunter bewegt sich der Motor nicht
#define SPEED_MAX  400

typedef enum { MOTOR_LEFT, MOTOR_RIGHT } Motor;

/* ---------------- UART0 ---------------- */

void UART_Init(void)
{
  UCSR0A = (1 << U2X0);
  UBRR0  = (uint16_t)(F_CPU / (8UL * BAUD) - 1);
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_PutChar(char c)
{
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void UART_PrintStr(const char *s)
{
  while (*s)
  {
    UART_PutChar(*s++);
  }
}

void UART_PrintUInt(uint16_t value)
{
  char buf[5];
  uint8_t i = 0;

  if (value == 0)
  {
    UART_PutChar('0');
    return;
  }
  while (value > 0)
  {
    buf[i++] = '0' + (value % 10);
    value /= 10;
  }
  while (i > 0)
  {
    UART_PutChar(buf[--i]);
  }
}

/* ---------------- ADC ---------------- */

void ADC_Init(void)
{
  ADMUX  = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_Read(uint8_t channel)
{
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

/* ---------------- Motoren ---------------- */

void motorPins_Init(void)
{
  DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
}

void PWM_Init(void)
{
  DDRB |= (1 << ENA) | (1 << ENB);
  TCCR1A = 0;
  TCCR1B = 0;

  // Timer1 Mode 14: Fast PWM, ICR1 als TOP
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

  ICR1 = PWM_TOP;

  OCR1A = 0;
  OCR1B = 0;
}

// speed: -400..+400 (negativ = rueckwaerts), 0 = Stopp
// Betrag wird auf SPEED_MIN..SPEED_MAX begrenzt
void set_motor(Motor m, int16_t speed)
{
  uint8_t  fwd = (speed >= 0);
  uint16_t pwm = fwd ? (uint16_t)speed : (uint16_t)(-speed);

  if (pwm != 0)
  {
    if (pwm < SPEED_MIN) pwm = SPEED_MIN;
    if (pwm > SPEED_MAX) pwm = SPEED_MAX;
  }

  if (m == MOTOR_LEFT)
  {
    if (fwd) { PORTD |=  (1 << IN1); PORTD &= ~(1 << IN2); }
    else     { PORTD |=  (1 << IN2); PORTD &= ~(1 << IN1); }
    OCR1A = pwm;
  }
  else
  {
    if (fwd) { PORTD |=  (1 << IN4); PORTD &= ~(1 << IN3); }
    else     { PORTD |=  (1 << IN3); PORTD &= ~(1 << IN4); }
    OCR1B = pwm;
  }
}

/* ---------------- Sensoren ---------------- */

typedef struct
{
  uint8_t ch;            // ADC-Kanal
  float   m, d, k;       // Kennlinie: cm = m / (raw + d) - k
  uint8_t minD, maxD;    // gueltiger Bereich in cm
  uint8_t dist;          // gefilterte Distanz in cm
} IRSensor;

IRSensor irFront = {0, 19679.42308f, 43.96401985f, 15, 20, 150, 150};
IRSensor irRight = {1,  8143.2f,     21.25957447f,  7, 10,  80,  80};
IRSensor irLeft  = {2,  7331.25f,    15.91666667f,  5, 10,  80,  80};

void IR_Update(IRSensor *s)
{
  ADC_Read(s->ch);                       // Dummy: Multiplexer einschwingen lassen
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 4; i++)
    sum += ADC_Read(s->ch);

  float v = s->m / ((sum >> 2) + s->d) - s->k;
  if (v < s->minD) v = s->minD;
  if (v > s->maxD) v = s->maxD;

  s->dist = (uint8_t)((s->dist + (uint8_t)v) / 2);
}

/* ---------------- Fahrparameter ---------------- */

#define LOOP_MS            40
#define DRIVE_SPEED        400   // volle Fahrt
#define MIN_SPEED          240   // Tempo kurz vor dem Hindernis
#define TURN_SPEED         300   // Drehen auf der Stelle
#define BACK_SPEED         260

#define SLOW_DISTANCE      80    // ab hier langsamer werden
#define SAFE_DISTANCE      40    // Hindernis -> ausweichen
#define CLEAR_DISTANCE     50    // Hysterese: erst wenn vorne > 50 frei, Drehen beenden
#define STOP_DISTANCE      25    // zu nah -> erst zurueck
#define SIDE_BLOCKED       20
#define WALL_CAP           40    // Seitenabstand fuer Regelung begrenzen
#define STEER_GAIN         3     // Lenkstaerke je cm Differenz

#define BACKUP_TICKS       10    // ~0.4 s
#define TURN_TIMEOUT_TICKS 40    // ~1.6 s, dann Richtung wechseln

typedef enum { ST_DRIVE, ST_BACKUP, ST_TURN } State;

int main(void)
{
  UART_Init();
  ADC_Init();
  motorPins_Init();
  PWM_Init();

  State   state    = ST_DRIVE;
  int8_t  turnDir  = 1;          // +1 = links drehen, -1 = rechts drehen
  uint8_t ticks    = 0;
  uint8_t printDiv = 0;

  UART_PrintStr("Front   Right   Left\n");

  while (1)
  {
    IR_Update(&irFront);
    IR_Update(&irLeft);
    IR_Update(&irRight);

    uint8_t f = irFront.dist;
    uint8_t l = irLeft.dist;
    uint8_t r = irRight.dist;

    switch (state)
    {
      case ST_DRIVE:
        if (f < STOP_DISTANCE ||
            (f < SAFE_DISTANCE && l < SIDE_BLOCKED && r < SIDE_BLOCKED))
        {
          // zu nah oder Sackgasse -> erst zurueck
          state = ST_BACKUP;
          ticks = BACKUP_TICKS;
        }
        else if (f < SAFE_DISTANCE)
        {
          // zur freieren Seite drehen
          if (l > r)      turnDir = 1;
          else if (r > l) turnDir = -1;
          else            turnDir = -turnDir;
          state = ST_TURN;
          ticks = TURN_TIMEOUT_TICKS;
        }
        else
        {
          // Tempo abhaengig vom Frontabstand
          int16_t speed = DRIVE_SPEED;
          if (f < SLOW_DISTANCE)
            speed = MIN_SPEED + (int32_t)(DRIVE_SPEED - MIN_SPEED) * (f - SAFE_DISTANCE)
                                 / (SLOW_DISTANCE - SAFE_DISTANCE);

          // Seitenregelung: zur freieren Seite lenken
          uint8_t lc = (l > WALL_CAP) ? WALL_CAP : l;
          uint8_t rc = (r > WALL_CAP) ? WALL_CAP : r;
          int16_t corr = ((int16_t)lc - (int16_t)rc) * STEER_GAIN;

          set_motor(MOTOR_LEFT,  speed - corr);
          set_motor(MOTOR_RIGHT, speed + corr);
        }
        break;

      case ST_BACKUP:
        set_motor(MOTOR_LEFT,  -BACK_SPEED);
        set_motor(MOTOR_RIGHT, -BACK_SPEED);
        if (--ticks == 0)
        {
          if (l > r)      turnDir = 1;
          else if (r > l) turnDir = -1;
          else            turnDir = -turnDir;
          state = ST_TURN;
          ticks = TURN_TIMEOUT_TICKS;
        }
        break;

      case ST_TURN:
        set_motor(MOTOR_LEFT,  -turnDir * TURN_SPEED);
        set_motor(MOTOR_RIGHT,  turnDir * TURN_SPEED);
        if (f > CLEAR_DISTANCE)
        {
          state = ST_DRIVE;
        }
        else if (--ticks == 0)
        {
          // festgefahren -> zurueck und andere Richtung probieren
          turnDir = -turnDir;
          state = ST_BACKUP;
          ticks = BACKUP_TICKS;
        }
        break;
    }

    // Debug-Ausgabe nur alle 5 Durchlaeufe
    if (++printDiv >= 5)
    {
      printDiv = 0;
      UART_PrintUInt(f); UART_PrintStr("      ");
      UART_PrintUInt(r); UART_PrintStr("      ");
      UART_PrintUInt(l); UART_PutChar('\n');
    }

    _delay_ms(LOOP_MS);
  }
}
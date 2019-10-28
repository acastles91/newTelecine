#include <inttypes.h>

/*
 * 101 010
 * 100 011
 * 110 001
 * 010 101
 * 011 100
 * 001 110
 */
static const uint8_t inverter[6] = { 052, 043, 061, 025, 034, 016 };
static int8_t inverter_idx = 0;
static int8_t inc = 0;

/* almost 50ms */
#define DEBOUNCE_COUNTER_START 15
static uint8_t sensor_debounce = 0;
static uint8_t sensor_state = 0;

/* almost 200ms */
#define BRAKE_COUNTER_START 60

/*
 * 100 010
 * 010 001
 * 001 100
 */
static const uint8_t brake_combination[3] = { 042, 021, 014 };
static int8_t brake_idx = 0;
static uint8_t brake_counter = 0;

/* automatic function state machine */
#define auto_reset              0
#define start_moving_forward    1
#define start_moving_backward   2
#define keep_moving             3
#define start_braking           4
#define keep_braking            5
#define auto_end                6

static uint8_t state = auto_reset;
static uint8_t is_manual = 0;

/* *almost* 300 times per second, but not quite */
ISR(TIMER0_OVF_vect)
{
  /* state machine */
  switch ( state )
  {
    case auto_reset:
      state = auto_end;
      break;

    case start_moving_forward:
      inc = +1;
      goto START_MOVING;
    case start_moving_backward:
      inc = -1;
      goto START_MOVING;

START_MOVING:
      if ( !digitalRead(9) )
        inc *= -1;
      digitalWrite(4, HIGH);
      state = keep_moving;
    case keep_moving:
      /* update inverter output */
      PORTC = inverter[inverter_idx];

      /* set next inverter output */
      inverter_idx += inc;
      if (inverter_idx == 6)
        inverter_idx = 0;
      else if (inverter_idx == -1)
        inverter_idx = 5;

      break;

    case start_braking:
      /* set inverter output to brake */
      PORTC = brake_combination[brake_idx];

      /* change brake phase combination.
      /* this avoids putting stress always on the same phases. */
      brake_idx++;
      if ( brake_idx == 3 )
        brake_idx = 0;

      /* start brake counter */
      brake_counter = BRAKE_COUNTER_START;

      /* change state */
      state = keep_braking;
      break;
    case keep_braking:
      brake_counter--;

      /* change state */
      if ( brake_counter == 0 )
        state = auto_end;

      break;
    case auto_end:
      /* disable inverter */
      digitalWrite(4, LOW);

      /* the end. */
      break;
  }

  /* buttons */
  if ( !digitalRead(7) ) /* button 1 */
  {
    if ( is_manual == 0 )
    {
      is_manual = 1;
      state = start_moving_forward;
    }
  }
  else if ( !digitalRead(8) ) /* button 2 */
  {
    if ( is_manual == 0 )
    {
      is_manual = 1;
      state = start_moving_backward;
    }
  }
  else if ( is_manual )
  {
    /* was manual, start brake */
    is_manual = 0;
    state = start_braking;
  }

  /* sensor */
  if ( sensor_debounce != 0 )
  {
    /* wait for the debounce period to end */
    sensor_debounce--;
  }
  else
  {
    uint8_t sensor_code = !digitalRead(6);
    if ( sensor_code != sensor_state )
    {
      sensor_state = sensor_code;
      Serial.write('0' + sensor_state);
      sensor_debounce = DEBOUNCE_COUNTER_START;
    }
  }
}

int main(void)
{
  /* start serial */
  Serial.begin(115200);

  /* configure pins */
  pinMode(4, OUTPUT); /* en */
  digitalWrite(4, LOW); /* disable inverter as soon as possible */

  pinMode(A0, OUTPUT); /* inverter - high 3 */
  pinMode(A1, OUTPUT); /* inverter - high 2 */
  pinMode(A2, OUTPUT); /* inverter - high 1 */
  pinMode(A3, OUTPUT); /* inverter - low 3 */
  pinMode(A4, OUTPUT); /* inverter - low 2 */
  pinMode(A5, OUTPUT); /* inverter - low 1 */

  pinMode(5, INPUT); /* sensor 1 - shutter open */
  pinMode(6, INPUT); /* sensor 2 - shutter close */
  pinMode(7, INPUT); /* button 1 - forward */
  pinMode(8, INPUT); /* button 2 - rewind */
  pinMode(9, INPUT); /* external button */

  /* configure Timer 0 to interrupt every 256 * 208 = 53248 ticks.
   * this leads to the interrupt function being called at 300,480769231 Hz
   * which is *almost* the 300 Hz we need to update the inverter with 6
   * steps leading to the 50 Hz the motor expects. */
  /* reset timers */
  GTCCR = (1 << TSM) | (1 << PSRASY) | (1 << PSRSYNC);

  TCCR0A = (0 << COM0A1) | (0 << COM0A0) /* disconnected */
         | (0 << COM0B1) | (0 << COM0B0) /* disconnected */
         | (0 << WGM01 ) | (1 << WGM00 );
  TCCR0B = (0 << FOC0A ) | (0 << FOC0B )
         | (1 << WGM02 )
         | (1 << CS02  ) | (0 << CS01  ) | (0 << CS00 ); /* 256 prescaler */
  TCNT0  = 0;
  OCR0A  = 104;
  OCR0B  = 0;
  TIMSK0 = (0 << OCIE0B) | (0 << OCIE0A) | (1 << TOIE0); /* interrupt on overflow */
  TIFR0  = (1 << OCF0B ) | (1 << OCF0A ) | (1 << TOV0 );

  GTCCR = 0;

  sei();

  while (1)
  {
    /* listen on serial */
    if ( Serial.available() )
    {
      uint8_t code = Serial.read();
      switch ( code )
      {
        case '0':
          state = start_braking;
          break;
        case '1':
          state = start_moving_forward;
          break;
        case '2':
          state = start_moving_backward;
          break;
      }
    }
  }
}

/*
 * .--------------------------.
 * |                         _|
 * |                        |_|-- power plug --------| battery
 * |                         _|
 * | ARDUINO                |_|-- brown ---- blue ---| camera - b8
 * |                        |_|-- red ------ yellow -| camera - b1
 * |                        |_|-- black ---- white --| camera - a1
 * |            FWD  REW   ___|
 * |                      | | |
 * '--------------------------'
 *                         | |
 *                         | '--- black -------------| external button
 *                         '----- red ---------------| external button
 *
 * .--------------------------.
 * |                         _|
 * |                        |_|-- red ---------------| battery
 * |                        |_|-- black -------------| battery
 * |                         _|
 * |                        |_|-- black -------------| camera - a4
 * | INVERTER               |_|-- red ---------------| camera - a3
 * |                        |_|-- brown -------------| camera - a2
 * |                         _|
 * |                        |_|-- red ---------------| shocked face
 * |                        |_|-- black -------------| shocked face
 * |                          |
 * '--------------------------'
 *
 * .--------------------------.
 * |                         _|
 * |                        |_|-- red ---------------| inverter
 * |                        |_|-- black -------------| inverter
 * | SHOCKED FACE            _|
 * |                        |_|-- black -------------| battery <-- WATCH OUT FOR THE BLACK/RED ORDER
 * |                        |_|-- red ---------------| battery <-- WATCH OUT FOR THE BLACK/RED ORDER
 * |                          |
 * '--------------------------'
 */

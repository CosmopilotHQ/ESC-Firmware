#include <Arduino.h>

#define DEADTIME 2 // us

#ifndef INVERTED_OUTPUTS
#define SETHIGH(PORT, PIN) (PORT &= ~(1 << PIN))
#define SETLOW(PORT, PIN) (PORT |= (1 << PIN))
#else
#define SETLOW(PORT, PIN) (PORT &= ~(1 << PIN))
#define SETHIGH(PORT, PIN) (PORT |= (1 << PIN))
#endif

#define MOTOR1_PMOS PB2 // Pin 6 of ardunio nano
#define MOTOR1_NMOS PB1 // Pin 5 of arduino nano

enum outputState { OUTPUT_HIGH = 1, OUTPUT_LOW = 2, OUTPUT_NEUTRAL = 0, IDEL };

outputState motor1 = IDEL;

void outputFsm(outputState &currentState, outputState desiredState);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  DDRB |=
      (1 << MOTOR1_PMOS) | (1 << MOTOR1_NMOS); // define Pin 5 and 6 as output
  outputFsm(motor1, OUTPUT_NEUTRAL); // initialise fsm with neutral state
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {

    char c = Serial.read();

    switch (c) {
    case '1':
      outputFsm(motor1, OUTPUT_HIGH);
      break;

    case '0':
      outputFsm(motor1, OUTPUT_LOW);
      break;

    default:
    case 'z':
      outputFsm(motor1, OUTPUT_NEUTRAL);
      break;
    }
  }
}

void outputFsm(outputState &currentState, outputState desiredState) {

  if (currentState == desiredState) {
    Serial.println("Already in Desired State");
    return;
  }

  switch (currentState) {
  case OUTPUT_NEUTRAL:
    switch (desiredState) {

    case OUTPUT_HIGH:
      // Turn PMOS ON
      SETHIGH(PORTB, MOTOR1_NMOS);
      SETHIGH(PORTB, MOTOR1_PMOS); // Pin5 and Pin6 HIGH
      currentState = OUTPUT_HIGH;
      break;

    case OUTPUT_LOW:
      // Turn NMOS ON
      SETLOW(PORTB, MOTOR1_NMOS);
      SETLOW(PORTB, MOTOR1_PMOS); // Pin5 and Pin6 LOW
      currentState = OUTPUT_LOW;
      break;

    default:
      // Default to OUTPUT_NEUTRAL
      SETLOW(PORTB, MOTOR1_PMOS);  // Pin6 LOW
      SETHIGH(PORTB, MOTOR1_NMOS); // Pin5 HIGH
      currentState = OUTPUT_NEUTRAL;
      break;
    }
    break;

  case OUTPUT_HIGH:
    switch (desiredState) {
    case OUTPUT_NEUTRAL:
      // Turn both MOS OFF
      SETLOW(PORTB, MOTOR1_PMOS);  // Pin6 LOW
      SETHIGH(PORTB, MOTOR1_NMOS); // Pin5 HIGH
      delayMicroseconds(DEADTIME);
      currentState = OUTPUT_NEUTRAL;
      break;

    case OUTPUT_LOW:
      // Turn PMOS OFF
      SETLOW(PORTB, MOTOR1_PMOS);  // Pin6 LOW
      SETHIGH(PORTB, MOTOR1_NMOS); // Pin5 HIGH
      delayMicroseconds(DEADTIME);
      currentState = OUTPUT_NEUTRAL;

      // Now turn on the MOS
      outputFsm(currentState, OUTPUT_LOW);
      break;

    default:
      // Default to OUTPUT_NEUTRAL
      SETLOW(PORTB, MOTOR1_PMOS);  // Pin6 LOW
      SETHIGH(PORTB, MOTOR1_NMOS); // Pin5 HIGH
      currentState = OUTPUT_NEUTRAL;
      break;
    }

    break;

  case OUTPUT_LOW:
    switch (desiredState) {
    case OUTPUT_NEUTRAL:
      // Turn both MOS OFF
      SETLOW(PORTB, MOTOR1_PMOS);  // Pin6 LOW
      SETHIGH(PORTB, MOTOR1_NMOS); // Pin5 HIGH
      delayMicroseconds(DEADTIME);
      currentState = OUTPUT_NEUTRAL;
      break;

    case OUTPUT_HIGH:
      // Turn NMOS OFF
      SETLOW(PORTB, MOTOR1_PMOS);  // Pin6 LOW
      SETHIGH(PORTB, MOTOR1_NMOS); // Pin5 HIGH
      delayMicroseconds(DEADTIME);
      currentState = OUTPUT_NEUTRAL;

      outputFsm(currentState, OUTPUT_HIGH);
      break;

    default:
      // Default to OUTPUT_NEUTRAL
      SETLOW(PORTB, MOTOR1_PMOS);  // Pin6 LOW
      SETHIGH(PORTB, MOTOR1_NMOS); // Pin5 HIGH
      currentState = OUTPUT_NEUTRAL;
      break;
    }
    break;

  default:
    SETLOW(PORTB, MOTOR1_PMOS);  // Pin6 LOW
    SETHIGH(PORTB, MOTOR1_NMOS); // Pin5 HIGH
    currentState = OUTPUT_NEUTRAL;
    break;
  }
}

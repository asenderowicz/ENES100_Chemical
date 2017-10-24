
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* 
 * For atan2(), if y is 0, it gives PI, if x is 0, it gives PI / 2, if both are 0, it gives 0.
 * atan2 returns a value (-PI, PI) in radians
 * For third quadrant, it gives negative angle referenced from 0, so atan2(-10,-10) gives -135.00000 (but in radians).
 * 
 * 
 * Navigation:
 * Sonar sensor returns an analog value regarding distance to object, no direction info given.
 * If sensor returns value above threshhold, turn until it drops below threshold. Then keep going forward,
 * compute new heading.
 * 
 * With two sonar sensors in front, go forward till we detect an obstacle in one or both of them. Turn until obstacle disappears
 * from view.
 * 
 * Servos generally want a PWM signal with a duty cycle of 1-2 milliseconds with those being the travel limits. Period should be 20 ms or 50 Hz.
 * 
 * Sonar sensor interface. Pulse trigger pin high for at least 10 us. Then measure duration of echo pin high pulse
 * and multiply by scaling factor to get mm.
 * Input capture pin (B0 on 328, L1, L0, D4 on 2560) can be used to measure pulse width.
 * TCNT1L 0x84
 * TCNT1H 0x85
 * ICR1L 0x86
 * ICR1H 0x87
 * However, use TCNT1, ICR1, let GCC take care of access. 2 cycle read, 5 cycle atomic read, 4 cycle write, 7 cycle atomic write
 * These include two ldi, less if value to write is already in register
 * Millis, delay, etc. use timer0, servo library uses timer1. Timer 1 PWM are B1, B2 (9, 10)
 * Timers 0 and 2 are 8 bit. Timer 1 is 16 bit.
 * See these: http://www.electronicwings.com/avr-atmega/ultrasonic-module-hc-sr04-interfacing-with-atmega1632
 * http://www.electronicwings.com/avr-atmega/atmega1632-timer-input-capture-mode
 */
#include "Enes100.h"
#define MARKER_ID 12
#define RX_PIN 7
#define TX_PIN 8

// These macros give values in milliradians
#define FULL_CIRCLE ((uint16_t)(M_PI * 2000.0)) // Two pi
#define THREE_QUARTER ((uint16_t)(M_PI * 1500.0)) // Three pi over two
#define HALF_CIRCLE ((uint16_t)(M_PI * 1000.0)) // Pi
#define ONE_QUARTER ((uint16_t)(M_PI * 500.0)) // Pi over two

#define degToRad(degree) ((degree * M_PI) / 180.0)
#define radToDeg(radian) ((radian * 180.0) / M_PI)

Enes100 rf("The Swiss Army Bot", CHEMICAL, MARKER_ID, RX_PIN, TX_PIN);
struct coord // Use this instead of provided coordinate class because theirs uses floats
{
  uint16_t x;
  uint16_t y;
  uint16_t theta;
};

struct coord robot; // Updated with coordinates whenevey they are received (the current location of robot).
struct coord pool;
int16_t clearance = 500; // Clearance in mm between robot and obstacle

void getLocation(void);

void setup() 
{
  
  rf.retrieveDestination();
  pool.x = (uint16_t)(1000.0 * rf.destination.x); // Convert from meters to millimeters
  pool.y = (uint16_t)(1000.0 * rf.destination.y);
  getLocation();
}

void loop() 
{
  getLocation();
}

uint16_t headingToPool(void)
{
  int16_t dy = (int16_t)pool.y - (int16_t)robot.y;
  int16_t dx = (int16_t)pool.x - (int16_t)robot.x;
  if(dx == 0) // Is destination straight up or down?
  {
    return dy > 0 ? ONE_QUARTER : THREE_QUARTER;
  }
  
  if(dy == 0) // Is destination to right or left?
  {
    return dx > 0 ? 0 : HALF_CIRCLE;
  }
  int16_t heading = 1000.0 * atan2(dy, dx); // Warning: Expensive calculation!
  if(heading < 0)
  {
    heading = FULL_CIRCLE + heading; // (0, 360) instead of (-180, 180)
  }
  return (uint16_t)heading;
}
void getLocation() // This function updates the robots's coordinates
{
  rf.updateLocation();
  robot.x = (uint16_t)(1000.0 * rf.location.x);
  robot.y = (uint16_t)(1000.0 * rf.location.y);
  robot.theta = (uint16_t)(1000.0 * rf.location.theta); // Radians to milliradians
  // Theta is referenced from north = 0, but we want it from east = 0.
  robot.theta = robot.theta > THREE_QUARTER ? robot.theta - THREE_QUARTER : robot.theta + ONE_QUARTER;
  return;
}


/*
float computeHeading(int px, int py)
{
  float dy = py - robot.y; // Offset from current location
  float dx = px - robot.x;
  if(dx == 0.0) // Is destination straight up or down?
  {
    return dy > 0.0 ? HALF_PI : (PI + HALF_PI);
  }
  
  if(dy == 0.0) // Is destination to right or left?
  {
    return dx > 0.0 ? 0.0 : PI;
  }
  
  float heading = atan2(dy, dx); // Compute angle to destination (Expensive calculation!)
  if(dy < 0.0) // Are we in lower half (since atan2 returned a negative angle)?
  {
    heading = TWO_PI - heading; // Make 0-360 instead of -180 - 180
  }
  return heading;
}

float computeSlope(int px, int py) // This function could overflow for a very steep slope
{
  return (py - robot.y) / (px - robot.x);
}*/

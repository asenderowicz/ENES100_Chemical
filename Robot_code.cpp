
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
#include "Enes100.h" //including their library
#define MARKER_ID 12 //The weird designs on the marker correspond to a number- here, 12. 
//Every time the code sees "MARKER_ID" it now replaces it with the "12"
#define RX_PIN 7 //This is the pin we use to recieve communication from the APC
#define TX_PIN 8 //This is the pin we use to transmit to the APC

// These macros give values in milliradians
//Here, we create constants for the radians corresponding to the four quadrants
//We're converting the regular float radian values to integers to convert to miliradians because we want integer values not floats
#define FULL_CIRCLE ((uint16_t)(M_PI * 2000.0)) // Two pi
#define THREE_QUARTER ((uint16_t)(M_PI * 1500.0)) // Three pi over two
#define HALF_CIRCLE ((uint16_t)(M_PI * 1000.0)) // Pi
#define ONE_QUARTER ((uint16_t)(M_PI * 500.0)) // Pi over two

#define degToRad(degree) ((degree * M_PI) / 180.0)
#define radToDeg(radian) ((radian * 180.0) / M_PI)

Enes100 rf("The Swiss Army Bot", CHEMICAL, MARKER_ID, RX_PIN, TX_PIN);
struct coord // Use this instead of provided coordinate class because theirs uses floats
{
  //"uint16"= unassigned data type. Smallest datat type that holds the values we need
  uint16_t x;
  uint16_t y;
  uint16_t theta;
};

struct coord robot; // Updated with coordinates whenevey they are received (the current location of robot).
struct coord pool; //Creating a pool object with an x, y and theta
int16_t clearance = 500; // Clearance in mm between robot and obstacle

void getLocation(void); //declare function's existence- define it properly later

void setup() //using code from their library, but adding conversions to convert to integers 
{
  
  rf.retrieveDestination(); //telling us mission site
  pool.x = (uint16_t)(1000.0 * rf.destination.x); // Convert from meters to millimeters
  pool.y = (uint16_t)(1000.0 * rf.destination.y);
  getLocation();
}

void loop() //Need to fill in
{
  getLocation();
}

uint16_t headingToPool(void)
{
  int16_t dy = (int16_t)pool.y - (int16_t)robot.y; //difference between pool's y and our y
  int16_t dx = (int16_t)pool.x - (int16_t)robot.x; //difference between pool's x and our x
  if(dx == 0) // Is destination straight up or down (we have correct x-coordinate)?
  {
    return dy > 0 ? ONE_QUARTER : THREE_QUARTER; 
    //We are basically writing an "if" statement here:
    //If we are directly above the pool, the theta we want is 3PI/2 ==> "THREE_QUARTER"
    //If we're directly below the pool, the theta we want is PI/2 ==> "ONE_QUARTER"
  }
  
  if(dy == 0) // Is destination to right or left (we have correct y-coordinate)?
  {
    return dx > 0 ? 0 : HALF_CIRCLE; //Similar "if" statement
  }
  int16_t heading = 1000.0 * atan2(dy, dx); // Warning: Expensive calculation!
  if(heading < 0)
  {
    heading = FULL_CIRCLE + heading; // converts our negative value into a value between 0 and 360
    //The inverse tangent function returns -PI/2 to PI/2, but we want it to be within 0 to 2PI
  }
  return (uint16_t)heading;
}
void getLocation() // This function updates the robots's coordinates
{
  rf.updateLocation(); //tells us our coordinates
  robot.x = (uint16_t)(1000.0 * rf.location.x); // Meters as a float to millimeters as a uint16_t
  robot.y = (uint16_t)(1000.0 * rf.location.y);
  robot.theta = (uint16_t)(1000.0 * rf.location.theta); // Radians to milliradians
  // The following line references theta from east instead of north, if necessary
  //robot.theta = robot.theta > THREE_QUARTER ? robot.theta - THREE_QUARTER : robot.theta + ONE_QUARTER;
  return;
}




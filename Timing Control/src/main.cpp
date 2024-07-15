/**
* @brief Timing Control
*/

#include "Arduino.h"

/*
  Ports definition, consult pinout for selection.
  D - digital 
  A - analog
*/
#define D_TRIGGER_CONTROL PB11 
#define A_BEAM_READ       PB1

/**
 * 
 * @brief hits the solenoid trigger for press_delay of milliseconds
 * @param press_delay
 * @retval none
 */
void press_trigger(int press_delay) 
{
  digitalWrite(D_TRIGGER_CONTROL, HIGH);
  delay(press_delay);
  digitalWrite(D_TRIGGER_CONTROL, LOW);
}

/**
 * @brief 
 * @param none
 * @retval none
 */
void setup()
{
  pinMode(D_TRIGGER_CONTROL, OUTPUT);
  pinMode(A_BEAM_READ, INPUT);
}

/**
 * @brief 
 * @param none
 * @retval none
 */
void loop()
{

}

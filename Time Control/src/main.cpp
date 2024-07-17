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
#define D_USER_LED        PC13
#define D_USER_BUTTON     PA0


bool startProcedure;

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
 * 
 * @brief This inturrept will be used to start the testing procedure
 * @param 
 * @retval none
 */
void irq_handler()
{
  //digitalWrite(D_USER_LED, HIGH);
 // press_trigger(1000);
  //digitalWrite(D_USER_LED, LOW);
  int value = analogRead(A_BEAM_READ);
  Serial.println(value);
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

  pinMode(D_USER_BUTTON, INPUT_PULLUP);
  pinMode(D_USER_LED, OUTPUT);  

  attachInterrupt(digitalPinToInterrupt(D_USER_BUTTON), irq_handler, RISING);

  Serial.begin(9600);

  Serial.write(("Hello World"));

}

/**
 * @brief 
 * @param none
 * @retval none
 */
void loop()
{
  digitalWrite(D_TRIGGER_CONTROL, LOW);

  if(true)  // take readings only from the moment 
  {
    //Serial.write((A_BEAM_READ));
  }
}
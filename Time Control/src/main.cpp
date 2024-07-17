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

#define SOLENOID_DELAY    100   // This value must be determinds

#define BEFORE_SOLENOID   0b01
#define AFTER_SOLENOID    0b10
#define BEAM_READ         0b11       


bool startProcedure = false;

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
  startProcedure = true;

  //digitalWrite(D_USER_LED, HIGH);
  // press_trigger(SOLENOID_DELAY);            
  //digitalWrite(D_USER_LED, LOW);  
  uint16_t analogValue = analogRead(A0); // Read analog value (0-1023)
  uint32_t timeMicros = micros(); // Get the current time in microseconds
    // Create a 7-byte packet
  uint8_t packet[7];

  // Pack the identifier and the analog value
  packet[0] = (BEAM_READ << 6) | (analogValue >> 4); // First byte: 2 bits for identifier + 6 MSB of analogValue
  packet[1] = (analogValue << 4) & 0xF0; // Second byte: 4 LSB of analogValue

  // Pack the time
  packet[2] = (timeMicros >> 24) & 0xFF; // Time MSB
  packet[3] = (timeMicros >> 16) & 0xFF;
  packet[4] = (timeMicros >> 8) & 0xFF;
  packet[5] = timeMicros & 0xFF; // Time LSB

  // Pack a checksum or delimiter (optional)
  packet[6] = 0xFF; // Example delimiter

  // Send the packet
  Serial.write(packet, 7);
  delay(1000);
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
  uint16_t analogValue; 
  uint32_t timeMicros; 

  /*if(startProcedure)  // Start sending the beam values once test procedure started.
  {
    int value = analogRead(A_BEAM_READ);
    Serial.println(value);
  }*/
}
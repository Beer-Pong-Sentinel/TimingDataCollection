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

#define PACKET_SIZE       7


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
}

/**
 * @brief 
 * @param none
 * @retval none
 */
void createPacket(uint8_t identifier, uint16_t analogValue, uint8_t* packet) {
    // Get the current time in microseconds (stub, replace with actual implementation)
    uint32_t timeMicros = micros();

    // Pack the identifier and the analog value (only if identifier is not 1 or 2)
    if (identifier == 1 || identifier == 2) {
        // Create a 5-byte packet with only identifier and time
        packet[0] = identifier << 6; // First byte: 2 bits for identifier, rest are 0
        packet[1] = 0; // No analog value, second byte is 0
    } else {
        // Create a 7-byte packet with identifier, analog value, and time
        packet[0] = (identifier << 6) | (analogValue >> 4); // First byte: 2 bits for identifier + 6 MSB of analogValue
        packet[1] = (analogValue << 4) & 0xF0; // Second byte: 4 LSB of analogValue
    }
    packet[2] = (timeMicros >> 24) & 0xFF; // Time MSB
    packet[3] = (timeMicros >> 16) & 0xFF;
    packet[4] = (timeMicros >> 8) & 0xFF;
    packet[5] = timeMicros & 0xFF; // Time LSB
    packet[6] = 0xFF; // Example delimiter
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
  
  uint8_t packet[PACKET_SIZE];

  if(startProcedure)  // Start sending the beam values once test procedure started.
  {
    createPacket(BEAM_READ,  analogRead(A_BEAM_READ), packet);
    Serial.write(packet, PACKET_SIZE);
  }
 
  startProcedure = false;
}
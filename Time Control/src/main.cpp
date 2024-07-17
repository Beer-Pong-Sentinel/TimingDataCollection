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

#define START_TEST        0b100
#define BEFORE_SOLENOID   0b001
#define AFTER_SOLENOID    0b010
#define BEAM_READ         0b011 
#define END_TEST          0b110   

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
 * @brief 
 * @param none
 * @retval none
 */
void createPacket(uint8_t identifier, uint16_t analogValue, uint8_t* packet) {
    // Get the current time in microseconds (stub, replace with actual implementation)
    uint32_t timeMicros = micros();

    // Pack the identifier and the analog value (only if identifier is not 1 or 2)
    if (identifier == BEAM_READ) {
 
        // Create a 7-byte packet with identifier, analog value, and time
        packet[0] = (identifier << 5) | (analogValue >> 5); // First byte: 3 bits for identifier + 5 MSB of analogValue
        packet[1] = (analogValue << 3) & 0xF8; // Second byte: 5 LSB of analogValue
    } else {
       // Create a 5-byte packet with only identifier and time
        packet[0] = identifier << 5; // First byte: 2 bits for identifier, rest are 0
        packet[1] = 0; // No analog value, second byte is 0
    }
    packet[2] = (timeMicros >> 24) & 0xFF; // Time MSB
    packet[3] = (timeMicros >> 16) & 0xFF;
    packet[4] = (timeMicros >> 8) & 0xFF;
    packet[5] = timeMicros & 0xFF; // Time LSB
    packet[6] = 0xFF; // Example delimiter
}

/**
 * 
 * @brief 
 * @param 
 * @retval none
 */
void timing_test_start()
{
  startProcedure = true;

  uint8_t packet[PACKET_SIZE];

  // Send a signal to start the test
  createPacket(START_TEST,0, packet);
  Serial.write(packet, PACKET_SIZE);


  // Send a signal at the moment before the solenoid is used
  createPacket(BEFORE_SOLENOID,0, packet);
  Serial.write(packet, PACKET_SIZE);

  press_trigger(SOLENOID_DELAY);

  // Send a signal at the moment after the solenoid is used
  createPacket(AFTER_SOLENOID,0, packet);
  Serial.write(packet, PACKET_SIZE);

  
}
/**
 * 
 * @brief This inturrept will be used to start the testing procedure
 * @param 
 * @retval none
 */
void irq_handler()
{
  timing_test_start();
  //digitalWrite(D_USER_LED, HIGH);
  // press_trigger(SOLENOID_DELAY);            
  //digitalWrite(D_USER_LED, LOW);  
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

  // When the procedure starts start taking data from the beam breaker
  if(startProcedure)  
  {
    int beam_value = analogRead(A_BEAM_READ);
    createPacket(BEAM_READ,  beam_value, packet);
    Serial.write(packet, PACKET_SIZE);

    if (beam_value == 1023) {
        startProcedure = false;
        createPacket(END_TEST, 0, packet);
        Serial.write(packet, PACKET_SIZE);
    }
  }
 
}
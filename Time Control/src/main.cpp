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
//#define A_BEAM_READ       PB1
#define D_BEAM_READ       PB1
#define D_USER_LED        PC13
#define D_USER_BUTTON     PA0

#define SOLENOID_DELAY    75   // This value must be determinds
#define BEAM_THRESHOLD    800

#define START_TEST        0b100
#define BEFORE_SOLENOID   0b001
#define AFTER_SOLENOID    0b010
#define BEAM_READ         0b011 
#define END_TEST          0b110   

#define PACKET_SIZE       5


bool startProcedure = false;
uint8_t packet[PACKET_SIZE];

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
 * @param identifier of the state/stage
 * @param packet a pointer to the packet array - 7 bytes
 * @retval none
 */
void createPacket(uint8_t identifier, uint8_t* packet) {
    uint32_t timeMicros = micros();

    packet[0] = (timeMicros >> 24) & 0xFF; // Time MSB
    packet[1] = (timeMicros >> 16) & 0xFF;
    packet[2] = (timeMicros >> 8) & 0xFF;
    packet[3] = timeMicros & 0xFF; // Time LSB
    packet[4] = (identifier << 5) | 0x1F; 
}

/**
 * 
 * @brief a simple timing procedure with one cycle, trigger, capture,end
 * @param 
 * @retval none
 */
void timing_test_start()
{
  startProcedure = true;

  // Send a signal to start the test
  createPacket(START_TEST, packet);
  Serial.write(packet, PACKET_SIZE);


  // Send a signal at the moment before the solenoid is used
  createPacket(BEFORE_SOLENOID,  packet);
  Serial.write(packet, PACKET_SIZE);

  press_trigger(SOLENOID_DELAY);

  // Send a signal at the moment after the solenoid is used
  createPacket(AFTER_SOLENOID, packet);
  Serial.write(packet, PACKET_SIZE);

  
}
/**
 * 
 * @brief This inturrept will be used to capture a change in the beam breaker circuit
 * @param 
 * @retval none
 */
void irq_handler()
{
  //timing_test_start();
  // If the beam breaks when a test procedure is running, stop the procedure
  // NOTE: you might want to change this for more complex procedures to not stop, but rather continue to next step of test
  if (startProcedure) { 
    createPacket(BEAM_READ, packet);
    Serial.write(packet, PACKET_SIZE);

    createPacket(END_TEST, packet);
    Serial.write(packet, PACKET_SIZE);

    startProcedure = false;
  }
  //createPacket(BEAM_READ, packet);
  //Serial.write(packet, PACKET_SIZE);


  
}


/**
 * @brief 
 * @param none
 * @retval none
 */
void setup()
{
  pinMode(D_TRIGGER_CONTROL, OUTPUT);
  //pinMode(A_BEAM_READ, INPUT);
  pinMode(D_BEAM_READ, INPUT);

  pinMode(D_USER_BUTTON, INPUT_PULLUP);
  pinMode(D_USER_LED, OUTPUT);  

  attachInterrupt(digitalPinToInterrupt(D_BEAM_READ), irq_handler, RISING);

  Serial.begin(9600);
}

/**
 * @brief 
 * @param none
 * @retval none
 */
void loop()
{
  digitalWrite(D_TRIGGER_CONTROL, LOW);
  

  if(digitalRead(D_USER_BUTTON)) // Low priority to capture click -> start testing procedure
  {
    timing_test_start();
  }

  // When the procedure starts start taking data from the beam breaker
  /*
  if(startProcedure)  
  {
    int beam_value = analogRead(A_BEAM_READ);
    createPacket(BEAM_READ,  beam_value, packet);
    Serial.write(packet, PACKET_SIZE);

    if (beam_value >= BEAM_THRESHOLD) {
        startProcedure = false;
        createPacket(END_TEST, 0, packet);
        Serial.write(packet, PACKET_SIZE);
    }
  }
 */
}
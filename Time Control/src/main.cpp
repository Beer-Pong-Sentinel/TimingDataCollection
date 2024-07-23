/**
* @brief Timing Control
*/

#include "Arduino.h"

/*
  Ports definition, consult pinout for selection.
  D - digital 
  A - analog
*/

#define USE_BUFFER        false

#define D_TRIGGER_CONTROL PB12      // tolerace for 5V
//#define A_BEAM_READ       PB1
#define D_BEAM_READ       PB11
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

#if USE_BUFFER == true
  uint8_t buffer[PACKET_SIZE][PACKET_SIZE]; // The size of this array should be changed depending on the procedure
  byte buffer_index;
#endif

bool startProcedure = false;
uint8_t packet[PACKET_SIZE];

/**
 * 
 * @brief hits the solenoid trigger for press_delay of milliseconds
 * @param press_delay
 * @retval none
 */
void pressTrigger(int press_delay) 
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
void createAndSendPacket(uint8_t identifier, uint8_t* packet) {
    uint32_t timeMicros = micros();

    packet[0] = (timeMicros >> 24) & 0xFF; // Time MSB
    packet[1] = (timeMicros >> 16) & 0xFF;
    packet[2] = (timeMicros >> 8) & 0xFF;
    packet[3] = timeMicros & 0xFF; // Time LSB
    packet[4] = (identifier << 5) | 0x1F; 

  #if USE_BUFFER == true
    memcpy(buffer[buffer_index], packet, PACKET_SIZE * sizeof(uint8_t)); 
    ++buffer_index;
    if (buffer_index == 5) buffer_index = 0; // Can be done with modulu but this is faster

  #else
    Serial.write(packet, PACKET_SIZE);
  #endif
}

void sendBuffer(uint8_t buffer[PACKET_SIZE][PACKET_SIZE])
{
    for (int i = 0; i < PACKET_SIZE; ++i) {
        for (int j = 0; j < PACKET_SIZE; ++j) {
            Serial.write(packet, PACKET_SIZE);
        }
    }
}
/**
 * 
 * @brief a simple timing procedure with one cycle, trigger, capture,end
 * @param 
 * @retval none
 */
void startTimingTest()
{
  startProcedure = true;

  // Send a signal to start the test
  createAndSendPacket(START_TEST, packet);

  // Send a signal at the moment before the solenoid is used
  createAndSendPacket(BEFORE_SOLENOID,  packet);

  pressTrigger(SOLENOID_DELAY);

  // Send a signal at the moment after the solenoid is used
  createAndSendPacket(AFTER_SOLENOID, packet);
}
/**
 * 
 * @brief This inturrept will be used to capture a change in the beam breaker circuit
 * @param 
 * @retval none
 */
void handleIRQ()
{
  //timing_test_start();
  // If the beam breaks when a test procedure is running, stop the procedure
  // NOTE: you might want to change this for more complex procedures to not stop, but rather continue to next step of test
  if (startProcedure) { 
    createAndSendPacket(BEAM_READ, packet);

    createAndSendPacket(END_TEST, packet);

    startProcedure = false;
    
    #if USE_BUFFER == true
      sendBuffer(buffer);
    #endif
  } 
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

  pinMode(D_USER_BUTTON, INPUT_PULLDOWN);
  pinMode(D_USER_LED, OUTPUT);  

  attachInterrupt(digitalPinToInterrupt(D_BEAM_READ), handleIRQ, RISING);

  digitalWrite(D_TRIGGER_CONTROL, LOW);

  Serial.begin(9600);
}

/**
 * @brief 
 * @param none
 * @retval none
 */
void loop()
{

  if(!startProcedure & digitalRead(D_USER_BUTTON)) // Low priority to capture click -> start testing procedure
  {
    startTimingTest();
  }

}
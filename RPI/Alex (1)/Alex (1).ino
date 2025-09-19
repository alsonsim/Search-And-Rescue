#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"

volatile TDirection dir;

#define SPEED_OF_SOUND      0.0345
#define PI                  3.141592654
// Alex's length and breadth in cm
#define ALEX_LENGTH         26
#define ALEX_BREADTH        16

float alexDiagonal = 0.0; // Alex's diagonal
float alexCirc = 0.0;     // Alex's turning circumference


/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the wheel encoder.
#define COUNTS_PER_REV_TURN       4
#define COUNTS_PER_REV            4 

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled by taking revs * WHEEL_CIRC
#define WHEEL_CIRC          16   // 22 with black tires 

// Colour Sensor Pins
#define S0 3 //PL3 (46)
#define S1 7 //PC7 (30)
#define S2 1 //PL1 (48)
#define S3 5 //PC5 (32)
#define sensorOut 1 //PG1 (40)
#define sensorPin 40
volatile char color = 'X';

// LED Pins
#define redLED 1 //PA1 (23)
#define greenLED 2 //PA2 (24)
volatile int toggleLED = 0;

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

//int pred[3] = {195, 420, 360}; // COM2
//int pgreen[3] = {330, 270, 320};
//int pwhite[3] = {210, 230, 200};

int pred[3] = {140, 360, 280}; // COM2 v2
int pgreen[3] = {350, 240, 300};
int pwhite[3] = {104, 102, 84};

//int pred[3] = {60, 197, 159}; // inside DSA
//int pgreen[3] = {124, 98, 117};
//int pwhite[3] = {44, 45, 39};

//int pred[3] = {77, 220, 175}; // outside DSA
//int pgreen[3] = {150, 110, 125};
//int pwhite[3] = {60, 60, 50};

// Ultrsonic Pins
#define trigF 1 //PC1 (36)
#define echoF 7 //PD7 (38)
#define echoFPin 38
#define trigL 2 //PB2 (51)
#define echoL 3 //PB3 (50)
#define echoLPin 50
#define trigR 5 //PA5 (27)
#define echoR 4 //PA4 (26)
#define echoRPin 26
volatile unsigned long frontD;
volatile unsigned long leftD;
volatile unsigned long rightD;

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variables to keep track of whther we've moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

// function to estimate number of wheel ticks needed to turn an angle
unsigned long computeDeltaTicks(float ang) {
  // We will assume that angular distance moved = linear distance moved in one wheel revolution. (Probably incorrect but simplifies calculation)
  // # of wheel revs to make one full 360 degree turn is alexCirc/WHEEL_CIRC
  // For ang degrees, it will be (ang * alexCirc) / (360.0 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV
  // unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV_TURN) / (360.0 * WHEEL_CIRC));
  // return ticks;
  if (ang <= 17) {
    return 1;
  }
  
  double ticks = (ang * alexCirc * COUNTS_PER_REV_TURN) / (360.0 * WHEEL_CIRC);
  ticks = round(ticks);
  unsigned long delta = (unsigned long)ticks;
  return delta;
}

void left(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  ccw(ang, speed);
}

void right(float ang, float speed) {
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  
  cw(ang, speed);
}

/*
 * Alex Communication Routines.
 */
 
TResult readPacket(TPacket *packet) {
    // Reads in data from the serial port and deserializes it. Returns deserialized data in "packet".
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
}

void sendStatus() {
  // code to send back a packet containing key information
  // Use the params array to store this information, and set the packetType and command files accordingly, then use sendResponse to send out the packet
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = color;
  statusPacket.params[1] = frontD;
  statusPacket.params[2] = leftD;
  statusPacket.params[3] = rightD;
  //statusPacket.params[4] = 0;
  //statusPacket.params[5] = 0;
  //statusPacket.params[6] = 0;
  //statusPacket.params[7] = 0;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful for debugging.
  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}
void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad magic number.
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}
void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad checksum.
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}
void sendBadCommand() {
  // Tell the Pi that we don't understand its command sent to us.
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}
void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}
void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}
void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and pullup resistors.
 */
 
// Enable pull up resistors on pins 18 and 19
void enablePullups() {
  // Use bare-metal to enable the pull-up resistors on pins 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR() {
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;  
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;  
  }
}

void rightISR() {
  if (dir == FORWARD) {
    rightForwardTicks++;
  } else if (dir == BACKWARD) {
    rightReverseTicks++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;  
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++; 
  }
  if (toggleLED) {
    PORTA &= ~(1 << greenLED);
    PORTA |= (1 << redLED);
  } else {
    PORTA &= ~(1 << redLED);
    PORTA |= (1 << greenLED);
  }
  toggleLED = 1 - toggleLED;
}

// Set up the external interrupt pins INT2 and INT3 for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 18 and 19 to be falling edge triggered. Remember to enable the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EICRA |= 0b10100000;
  EIMSK |= 0b00001100;
  SREG |= (1 << 7); // sei()
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR should call rightISR.
ISR(INT2_vect) {
  rightISR();
}
ISR(INT3_vect) { 
  leftISR();
}

/*
 * Setup and start codes for serial communications
 */
 
// Set up the serial connections
void setupSerial() {
  // 9600 baudrate
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0C = 0b00000110;  // 8N1??
  UCSR0A = 0;
}

// Start the serial connection
void startSerial() {
  UCSR0B = 0b00011000;
}

// Read the serial port. Returns the read character in ch if available. Also returns TRUE if ch is valid. 
int readSerial(char *buffer) {
  int count = 0;
  //while ((UCSR0A & 0b10000000) == 0);
  while ((UCSR0A >> 7)) {
    buffer[count++] = UDR0;
  }
  return count;
}

// Write to the serial port
void writeSerial(const char *buffer, int len) {
  for (int i = 0; i < len; i++) {
    while ((UCSR0A & 0b00100000) == 0);
    UDR0 = buffer[i];
  }
}

/*
 * Alex's setup and run codes
 */

// Clears all our counters
void clearCounters() {
  leftForwardTicks = 0; 
  rightForwardTicks = 0;
  leftReverseTicks = 0; 
  rightReverseTicks = 0;

  leftForwardTicksTurns = 0; 
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0; 
  rightReverseTicksTurns = 0;

  forwardDist = 0;
  reverseDist = 0;

  color = 'X';
  frontD = 0;
  leftD = 0;
  rightD = 0;
}

// Clears one particular counter
void clearOneCounter(int which) {
  clearCounters();
}
// Intialize Alex's internal states
void initializeState() {
  clearCounters();
}

void handleCommand(TPacket *command) {
  switch(command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
        sendOK();
        backward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
        sendOK();
        left((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        right((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
        sendOK();
        stop();
      break;

    case COMMAND_GET_STATS:
        sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
      break;
    
    case COMMAND_GET_COLOR:
        sendOK();
        measure_colour();
        match_colour();
        sendStatus();
      break;

    case COMMAND_GET_DIST:
        sendOK();
        read_US();
        sendStatus();
      break;
 
    case COMMAND_KEEP_FORWARD:
        sendOK();
        forward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_KEEP_REVERSE:
        sendOK();
        backward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_KEEP_LEFT:
        sendOK();
        left((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_KEEP_RIGHT:
        sendOK();
        right((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_UTURN_LEFT:
        sendOK();
        left((double) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_UTURN_RIGHT:
        sendOK();
        right((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_SMALL_F:
        sendOK();
        forward((double) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_SMALL_R:
        sendOK();
        backward((double) command->params[0], (float) command->params[1]);
      break;
         
    default:
      sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while(!exit) {
    TPacket hello;
    TResult result;
    
    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK) {
      if(hello.packetType == PACKET_TYPE_HELLO) {
        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD) {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

unsigned long f_dist() {
  PORTC |= (1 << trigF);
  delayMicroseconds(10);
  PORTC &= ~(1 << trigF);
  int microsecs = pulseIn(echoFPin, HIGH);
  frontD = (microsecs * SPEED_OF_SOUND)/2;
}

void read_US() {
  PORTC |= (1 << trigF);
  delayMicroseconds(10);
  PORTC &= ~(1 << trigF);
  int microsecs = pulseIn(echoFPin, HIGH);
  frontD = (microsecs * SPEED_OF_SOUND)/2;

  PORTB |= (1 << trigL);
  delayMicroseconds(10);
  PORTB &= ~(1 << trigL);
  microsecs = pulseIn(echoLPin, HIGH);
  leftD = (microsecs * SPEED_OF_SOUND)/2;

  PORTA |= (1 << trigR);
  delayMicroseconds(10);
  PORTA &= ~(1 << trigR);
  microsecs = pulseIn(echoRPin, HIGH);
  rightD = (microsecs * SPEED_OF_SOUND)/2;
}

void measure_colour() {
  // Setting RED (R) filtered photodiodes to be read
  PORTL &= ~(1 << S2);
  PORTC &= ~(1 << S3); 
  // Reading the output frequency
  redFrequency = pulseIn(sensorPin, LOW);
  delay(20);
  
  // Setting GREEN (G) filtered photodiodes to be read
  PORTL |= (1 << S2);
  PORTC |= (1 << S3);  
  // Reading the output frequency
  greenFrequency = pulseIn(sensorPin, LOW);
  delay(20);
 
  // Setting BLUE (B) filtered photodiodes to be read
  PORTL &= ~(1 << S2);
  PORTC |= (1 << S3);
  // Reading the output frequency
  blueFrequency = pulseIn(sensorPin, LOW); 
  delay(20);
}

void match_colour() {
  int err0 = abs(redFrequency - pred[0]);
  int err1 = abs(greenFrequency - pred[1]);
  int err2 = abs(blueFrequency - pred[2]);
  int red_error = (err0 + err1 + err2) / 3;

  err0 = abs(redFrequency - pgreen[0]);
  err1 = abs(greenFrequency - pgreen[1]);
  err2 = abs(blueFrequency - pgreen[2]);
  int green_error = (err0 + err1 + err2) / 3;

  err0 = abs(redFrequency - pwhite[0]);
  err1 = abs(greenFrequency - pwhite[1]);
  err2 = abs(blueFrequency - pwhite[2]);
  int white_error = (err0 + err1 + err2) / 3;

  if (red_error <= green_error && red_error <= white_error) {
    color = 82; //'R'
  } else if (green_error <= red_error && green_error <= white_error) {
    color = 71; // 'G'
  } else if (white_error <= green_error && white_error <= red_error) {
    color = 87; // 'W;
  } else {
    color = 88; // 'X'
  }
}

void setup() {
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal; 

  DDRA |= (1 << redLED) | (1 << greenLED) | (1 << trigR);
  DDRA &= ~(1 << echoR);
  DDRB |= (1 << trigL);
  DDRB &= ~(1 << echoL);
  DDRC |= (1 << trigF) | (1 << S3) | (1 << S1);
  DDRD &= ~(1 << echoF);
  DDRG &= ~(1 << sensorOut);
  DDRL |= (1 << S2) | (1 << S0);

  // Setting colour frequency scaling to 20%
  PORTL |= (1 << S0);
  PORTC |= (1 << S1);

  // Turn off LEDs
  PORTA &= (~(1 << redLED)) & (~(1 << greenLED));

  SREG &= ~(1 << 7); // cli()
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  SREG |= (1 << 7); // sei()
}

void handlePacket(TPacket *packet) {
  switch(packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
 
  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);

  if(result == PACKET_OK) {
    handlePacket(&recvPacket);
  } else if (result == PACKET_BAD) {  
    sendBadPacket();
  } else if(result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }

  /*
  if (dir == FORWARD) {
    f_dist();
    if (frontD < 7) {
      stop();
      deltaDist = 0;
      newDist = 0;
      sendStatus();
    }
  }*/

  // to improve accuracy, maybe change it to  (forwardDist > newDist || newDist - forwardDist < 5)
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == STOP) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
    }
  }
      
}

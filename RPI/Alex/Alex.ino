#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
#include <Servo.h>

#define ALEX_LENGTH 26
#define ALEX_BREADTH 16
#define SPEED_OF_SOUND 0.0345
#define SERVO_LEFT_PIN 44
#define SERVO_RIGHT_PIN 45
#define SERVO_BACK_PIN 46

Servo servoLeft;
Servo servoRight;
Servo servoBack;
// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      4 //8
#define COUNTS_PER_REV_TURN 4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          16 //20.1

float alexDiagonal = 0.0;
float alexCirc = 0.0;



volatile TDirection dir;
// Colour Sensor Pins
#define S0 22 //3
#define S1 23 //7
#define S2 24 //1
#define S3 25 //5
#define sensorOut A8 
#define sensorPin 40
uint32_t color = 0;

//uint32_t detectedColor;
const int colorSampleCount = 5;
const int colorDelay = 40;

const uint32_t RED = 82;
const uint32_t GREEN = 71;
  
//int redFrequency = 0;
//int greenFrequency = 0;
//int blueFrequency = 0;

//int pred[3] = {140, 360, 280}; // COM2 v2
//int pgreen[3] = {350, 240, 300};
//int pwhite[3] = {104, 102, 84};

/*
 * Alex's configuration constants
 */


//dbprintf(“PI is %3.2f\n”, PI)


/*
 *    Alex's State Variables
 *    
 */

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
// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;

unsigned long computeDeltaTicks(float ang)
{

    /*When Alex is moving in a straight line, he will move WHEEL_CIRC cm forward (or backward) in one
wheel revolution. We assume that when Alex is moving in circles “on a dime”, the wheels make
the same WHEEL_CIRC cm angular distance in one revolution.
- The total number of wheel turns required to turn 360 degrees is therefore alexCirc/WHEEL_CIRC,
where alexCirc is the circumference of the circle made by Alex tunring on a dime. (Once again, to
“turn on a dime” means that Alex rotates about its center axis, without moving forward or
backward).
- To turn ang degrees, the number of wheel turns is ang/360.0 * alexCirc/WHEEL_CIRC.
- The number of ticks is ang/360.0 * alexCirc/WHEEL_CIRC * COUNTS_PER_REV.
*/
    //unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

    //return ticks;
   /* if (ang <= 17) {
    return 1;
  } */
  
  /*double ticks = (ang * alexCirc * COUNTS_PER_REV_TURN) / (360.0 * WHEEL_CIRC);
  ticks = round(ticks);
  unsigned long delta = (unsigned long)ticks;
  return delta; */
  long ticks = ang / 90;
  return ticks;
}

void left(float ang, float speed) {
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
    
  targetTicks = leftReverseTicksTurns + deltaTicks;
  ccw(ang,speed);
}

void right(float ang, float speed) {
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
    
  targetTicks = rightReverseTicksTurns + deltaTicks;
  
  cw(ang,speed);
}
/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;

    // Populate the params array with the specified values
  //  statusPacket.params[0] = leftForwardTicks;
  //  statusPacket.params[1] = rightForwardTicks;
  //  statusPacket.params[2] = leftReverseTicks;
  //  statusPacket.params[3] = rightReverseTicks;
  //  statusPacket.params[4] = leftForwardTicksTurns;
 //   statusPacket.params[5] = rightForwardTicksTurns;
   // statusPacket.params[6] = leftReverseTicksTurns;
 //   statusPacket.params[7] = rightReverseTicksTurns;
    statusPacket.params[0] = color;
  statusPacket.params[1] = frontD;
  statusPacket.params[2] = leftD;
  statusPacket.params[3] = rightD;
    statusPacket.params[12] = forwardDist;
    statusPacket.params[13] = reverseDist;

    // Send the packet (assuming sendResponse is the function to send it)
    sendResponse(&statusPacket);

}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
  
}

void dbprintf(char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 

    // 1. Set PD2 (INT2) and PD3 (INT3) as INPUTS
    DDRD &= 0b11110011;

    // 2. Enable pull-up resistors on PD2 and PD3
    PORTD |= 0b00001100;
  
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{

    if (dir == FORWARD){
        leftForwardTicks++;
        forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
    }
    else if (dir == BACKWARD){
        leftReverseTicks++;
        reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
    }
    else if (dir == RIGHT){
        leftForwardTicksTurns++;
    }
    else if (dir == LEFT){
        leftReverseTicksTurns++;
    }

}

void rightISR()
{

    if (dir == FORWARD){
        rightForwardTicks++;
    }
    else if (dir == BACKWARD){
        rightReverseTicks++;
    }
    else if (dir == LEFT){
        rightForwardTicksTurns++;
    }
    else if (dir == RIGHT){
        rightReverseTicksTurns++;
    }
}


// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.

    // Configure External Interrupt Control Register (EICRA)
    // ISC21 = 1, ISC20 = 0 → Falling edge for INT2
    // ISC31 = 1, ISC30 = 0 → Falling edge for INT3
    EICRA |= (1 << ISC21) | (1 << ISC31);  // Set falling edge trigger
    EICRA &= ~((1 << ISC20) | (1 << ISC30)); // Ensure ISC20 and ISC30 are cleared

    // Enable INT2 and INT3 in the External Interrupt Mask Register (EIMSK)
    EIMSK |= (1 << INT2) | (1 << INT3);

    // Enable Global Interrupts
    sei();

}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.

// ISR for INT2 (Right Encoder)
ISR(INT2_vect) {
    rightISR();  // Increment right wheel tick count
}

// ISR for INT3 (Left Encoder)
ISR(INT3_vect) {
    leftISR();   // Increment left wheel tick count
}


// Implement INT2 and INT3 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  //Serial.begin(9600);
  UBRR0L = 103;
  UBRR0H = 0;
  UCSR0C = 0b00000110;  
  UCSR0A = 0;
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  UCSR0B = 0b00011000;
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{
  
  int count=0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while((UCSR0A >> 7))
    buffer[count++] = UDR0;

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
  for (int i = 0; i < len; i++) {
    while ((UCSR0A & 0b00100000) == 0);
    UDR0 = buffer[i];// Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
  }
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  forwardDist=0;
  reverseDist=0; 
  color = 0;
  frontD = 0;
  rightD = 0;
  leftD = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  //dbprintf("%d", command->command);
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left((double)command->params[0],(float)command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        backward((double) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((double)command->params[0], (float) command->params[1]);
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
    case COMMAND_GET_COLOR:
        sendOK();
        match_color();
        sendColor();
        sendStatus();
      break;
    case COMMAND_GET_DIST:
        sendOK();
        read_US();
        sendStatus();
      break;
    case COMMAND_OPEN:
        sendOK();
        servoLeft.write(180);    // Open position (adjust angle as needed)
        servoRight.write(0);   // Open position (adjust angle as needed)
      break;
    case COMMAND_CLOSE:
        sendOK();
        servoLeft.write(90);    // Open position (adjust angle as needed)
        servoRight.write(90);   // Open position (adjust angle as needed)
      break;
      
    case COMMAND_SLIDE:
        sendOK();
        servoBack.write(90);    // open position (adjust angle as needed
      break;

     case COMMAND_PLANE:
        sendOK();
        servoBack.write(170);    // close position (adjust angle as needed
      break; 
      
    case COMMAND_GET_STATS:
        sendStatus();
        break;
            
    case COMMAND_CLEAR_STATS:
        clearOneCounter(command->params[0]);
        sendOK();
        break;
        
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
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

void measure_color(int &red, int &green, int &blue) {
 /* // Setting RED (R) filtered photodiodes to be read
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
  delay(20); */
  red = readFrequency(LOW, LOW);
  green = readFrequency(HIGH, HIGH);
  blue = readFrequency(LOW, HIGH);

}

int readFrequency(int s2Val, int s3Val) {
  digitalWrite(S2, s2Val);
  digitalWrite(S3, s3Val);
  delay(colorDelay);
  int total = 0;
  for (int i = 0; i < colorSampleCount; i++) {
    total += pulseIn(sensorOut, LOW, 50000);
    delay(colorDelay);
  }
  return total / colorSampleCount;
}

uint32_t identifyColor(int red, int green, int blue) {
  // Simple ratio-based detection
  int redThreshold = 30;  // Adjust these values based on your testing //50
  int greenThreshold = 30; //50
  
  if (red < green - redThreshold && red < blue - redThreshold) {
    return RED;
  } else if (green < red - greenThreshold) {
    return GREEN;
  }
  return -1;
}

void match_color() {
  /*int err0 = abs(redFrequency - pred[0]);
  int err1 = abs(greenFrequency - pred[1]);
  int err2 = abs(blueFrequency - pred[2]);
  //int red_error = (err0 + err1 + err2) / 3;
  int red_error = (pred[0] + pred[1] + pred[2]) / 3;

  err0 = abs(redFrequency - pgreen[0]);
  err1 = abs(greenFrequency - pgreen[1]);
  err2 = abs(blueFrequency - pgreen[2]);
  //int green_error = (err0 + err1 + err2) / 3;
  int green_error = (pgreen[0] + pgreen[1] + pgreen[2]) / 3;

  err0 = abs(redFrequency - pwhite[0]);
  err1 = abs(greenFrequency - pwhite[1]);
  err2 = abs(blueFrequency - pwhite[2]);
  //int white_error = (err0 + err1 + err2) / 3;
  int white_error = (pwhite[0] + pwhite[1] + pwhite[2]) / 3;

  if (red_error <= green_error && red_error <= white_error) {
    color = 82; //'R'
  } else if (green_error <= red_error && green_error <= white_error) {
    color = 71; // 'G'
  } else if (white_error <= green_error && white_error <= red_error) {
    color = 87; // 'W;
  } else {
    color = 88; // 'X'
  }*/
  int r, g, b;
  measure_color(r, g, b);
  color = identifyColor(r, g, b);
}

void sendColor() {
  TPacket colorPacket;
  colorPacket.packetType = PACKET_TYPE_RESPONSE;
  colorPacket.command = RESP_COLOUR;
  colorPacket.params[0] = color;
  //colorPacket.params[1] = returnDist();
  sendResponse(&colorPacket);
}

void setup() {
  // put your setup code here, to run once:

  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  servoBack.attach(SERVO_BACK_PIN);

// Set to default (closed)
  servoLeft.write(180);   // Closed position
  servoRight.write(0); // Closed position
  servoBack.write(170); // Closed position

  DDRA |= (1 << trigR);
  DDRA &= ~(1 << echoR);
  DDRB |= (1 << trigL);
  DDRB &= ~(1 << echoL);
  DDRC |= (1 << trigF); // | (1 << S3) | (1 << S1);
  DDRD &= ~(1 << echoF);
  //DDRG &= ~(1 << sensorOut);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW); 

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  //setupColour();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
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
// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

 //forward(0, 100);

// Uncomment the code below for Week 9 Studio 2


 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 

      
    if(deltaDist > 0)
    {
      if(dir==FORWARD)
        {
      if(forwardDist > newDist)
        {
        deltaDist=0;
        newDist=0;
        stop();
        }
        }
      else
        if(dir == BACKWARD)
        {
        if(reverseDist > newDist)
        {
        deltaDist=0;
        newDist=0;
        stop();
        }
      }
      else
      if(dir == (TDirection)STOP)
        {
        deltaDist=0;
        newDist=0;
        stop();
        }
    } 

  if (deltaTicks > 0)
  {
      if (dir == LEFT)
      {
          if (leftReverseTicksTurns >= targetTicks)
          {
              deltaTicks = 0;
              targetTicks = 0;
              stop();
          }
      }
      else
          if (dir == RIGHT)
      {
          if (rightReverseTicksTurns >= targetTicks)
          {
              deltaTicks = 0;
              targetTicks = 0;
              stop();
          }
      }
      else
      if (dir == (TDirection)STOP)
      {
              deltaTicks = 0;
              targetTicks = 0;
              stop();
      }
      }
             
}

#include <math.h>

#define LIMIT_LEFT 12
#define LIMIT_RIGHT 8
#define MOTOR_MIN -3000
#define MOTOR_MAX 3000

// Structure for defining parameters
struct parmDef_s
{
  char *parm; // Parm name as ASCIIZ string
  int32_t *parmVal;
  uint8_t cmdEnum;
};

typedef struct parmDef_s parmDef_t;

volatile int32_t motorEnc;
volatile int32_t pendulumEnc;

// Track current motor speed
int16_t motorSpeed = 0;

// Max motor voltage
float motorVMax = 12;

// For carriage position control, 2000, 200, 100
int32_t pkp = 1000;
int32_t pki = 50;
int32_t pkd = 1000;
int32_t ckp = 1000;
int32_t cki = 200;
int32_t ckd = 100;
int32_t lt = 10;
int32_t pt = 1200;
int32_t pa = 1;
int32_t pd = 10;
int32_t cp = 0;
int32_t mm = 255;
int32_t aw = 0;

// LQR gains
float K_lqr[4] = {-2.9229, -35.0943, -63.1803, -8.4105}; // from voltage input model
//float K_lqr[4]= {-0.8872, -2.2137, -7.7575, -1.5627}; // from force input model

// Conversion Factors
const float angle_conv_factor_deg = 0.15;
const float angle_conv_factor_rad = 0.002618;
const float cart_conv_factor_m = 0.0001240625;

// Queues for finding derivatives
const int derivative_que_size = 5;
float cart_pos_que[derivative_que_size];
float pend_angle_que[derivative_que_size];

// Enumerations for the various commands
enum cmd_e
{
  NO_COMMAND = 0,
  PKP,
  PKI,
  PKD,
  PT,
  PA,
  PD,
  CKP,
  CKI,
  CKD,
  CP,
  LT,
  MM,
  AW,
  BAL,
  SHOW,
  ENC,
  HOME,
  HELP,
  PRNT,
  STEP,
};

parmDef_t parmDefArray[] =
{
  {"PKP", &pkp, PKP},   // Pendulum KP
  {"PKI", &pki, PKI},   // Pendulum KP
  {"PKD", &pkd, PKD},   // Pendulum KP
  {"PT",  &pt,  PT},    // Pendulum Top
  {"PA",  &pa,  PA},    // Pendulum Adjust
  {"PD",  &pd,  PD},    // Pendulum Delta
  {"CKP", &ckp, CKP},   // Cart KP
  {"CKI", &cki, CKI},   // Cart KI
  {"CKD", &ckd, CKD},   // Cart KD
  {"CP",  &cp,  CP},    // Cart Position
  {"LT",  &lt,  LT},    // Loop Time (ms)
  {"MM",  &mm,  MM},    // Motor Max
  {"AW",  &aw,  AW},    // Anti-Windup enable
  {"BAL", NULL, BAL},   // Balance
  {"SHOW",NULL, SHOW},  // Show Parameter Values
  {"ENC", NULL, ENC},   // Show Encoder Values
  {"HOME",NULL, HOME},  // Home Cart
  {"?",   NULL, HELP},  // Help
  {"HELP",NULL, HELP},  // Help
  {"PRNT",NULL, PRNT}, 
  {"STEP",NULL, STEP},
  {NULL,  NULL, 0}      // Sentinel
};

//uint16_t minTime = 65535, maxTime = 0;

// Should be called frequently while running the motor.
// If carriage out of bounds, stop motor and return true.
bool motorCheck(void)
{
  int32_t m = getMotorEncoder();
  if (((motorSpeed < 0) && (m <= MOTOR_MIN))
    || (motorSpeed > 0) && (m >= MOTOR_MAX))
  {
    motor(0);
    return true;
  }
  else
  {
    return false;
  }
}

ISR(PCINT1_vect)
{
  // This should interrupt on motor A changes only
//uint32_t startTime = TCNT1;
  // Motor encoder inputs
  //   A = A2 = PC2 = PCINT10
  //   B = A3 = PC2 = PCINT11
  uint8_t encAB = PINC & 0b00001100;     // Bit map:  0 0 0 0 A B 0 0

  // After an edge on A, if A and B are the same, increment; else decrement
  if ((encAB == 0) || (encAB == 0b00001100)) ++motorEnc;
  else --motorEnc;
  
//uint16_t endTime = TCNT1;
//uint16_t delta = endTime - startTime;
//if (delta < minTime) minTime = delta;
//else if (delta > maxTime) maxTime = delta;
}

ISR(PCINT2_vect)
{
  // This should interrupt on pendulum A and B changes
  // Pendulum encoder inputs
  //   A = D2 = PD2 = PCINT18
  //   B = D3 = PD3 = PCINT19
  static uint8_t prevEncAB; // Bit map:  0 0 0 0 A B 0 0
  uint8_t encAB = PIND;// & 0b00001100;     // Bit map:  0 0 0 0 A B 0 0

  // Optional: if no change or invalid change, do not inc/dec count
  //  if ((encAB == prevEncAB) || ((encAB ^ prevEncAB) == 0b00001100)) return;
  
  // If previous A state is the same as current B state, increment; else decrement
  if ((((encAB >> 1) ^ prevEncAB) & 0b00000100) == 0) ++pendulumEnc;
  else --pendulumEnc;
  prevEncAB = encAB;  
}

void setup() {
  // put your setup code here, to run once:

  PCMSK1 = 0b00001100;  // PCINT 10/11
  PCMSK1 = 0b00001000;  // PCINT 10
  PCMSK2 = 0b00001100;  // PCINT 18/19
  PCIFR  = 0b00000111;  // Clear any pending interrupts
  PCICR  = 0b00000110;  // Enable PCINT1, PCINT1

  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);

  pinMode(12, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11,OUTPUT);

  Serial.begin(115200);
//TCCR1A = 0;
//TCCR1B = 1;

  printHelp();
}

void printHelp(void)
{
  uint8_t parmIdx = 0;
  Serial.println("Parameters & commands:");
  while (1)
  {
    if (parmDefArray[parmIdx].parm == NULL)
    {
      // End of list
      break;
    }
    Serial.print("  - "); Serial.print(parmDefArray[parmIdx].parm);
    if (parmDefArray[parmIdx].parmVal != NULL) Serial.print(" <value>");
    Serial.println();
    ++parmIdx;
  }
}

void homeCarriage(void)
{
  Serial.println(">>>Homing carriage");
  // Move the motor left until the switch is triggered
  motor(-48);
  while (digitalRead(LIMIT_LEFT) == LOW)
  {
    delay(1);
  }
  motor(0);
  delay(500);
  // Set the motor position to the known left switch value
  motorEnc = -3100;
  // Move to position 0
  motor(48);
  while (getMotorEncoder() < 0){}
  motor(0);
}  

// Functions to access encoder values with interrupts disabled
int32_t getMotorEncoder(void)
{
  int32_t result;
  noInterrupts();
  result = motorEnc;
  interrupts();
  return result;
}
int32_t getPendulumEncoder(void)
{
  int32_t result;
  noInterrupts();
  result = pendulumEnc;
  interrupts();
  return result;
}

void loop() {
  uint8_t cmdEnum;
  
  cmdEnum = getCmd();

  switch (cmdEnum)
  {
    case HELP:
      printHelp();
      break;
    case BAL:
      balance();
      break;
    case SHOW:
      uint8_t parmIdx;
      parmIdx = 0;
      while (1)
      {
        if (parmDefArray[parmIdx].parm == NULL)
        {
          // End of list
          break;
        }
        if (parmDefArray[parmIdx].parmVal != NULL)
        {
          Serial.print(parmDefArray[parmIdx].parm); Serial.print(" = ");
          Serial.println(*parmDefArray[parmIdx].parmVal);
        }
        ++parmIdx;
      }
      break;
    case ENC:
      Serial.print("Pendulum = "); Serial.println(getPendulumEncoder());
      Serial.print("Motor    = "); Serial.println(getMotorEncoder());
      break;
    case HOME:
      homeCarriage();
      break;
    case CP:
      moveCarriage();
      break;
    case PRNT:
      //while(1){
	  //	Serial.println(getPendulumEncoder()*0.15);
      //}
      // Motor encoder: 320 PPR
      // cart travels 0.0397m per revolution                    
      while(1) { 
        Serial.println(getMotorEncoder());
      }
	case STEP:
      int startMillis = millis();
      int currentMillis = startMillis;
	  motor(40);
      while(currentMillis - startMillis <= 5000){
        Serial.println(getMotorEncoder());
        currentMillis = millis();
      }
      motor(0);
      break;
  }
}

void balance(void)
{
  unsigned long startTime = millis();
  unsigned long loopTime = startTime;
  int32_t ptSave = pt;
  int32_t motError;
  int32_t prevMotError = 0;
  float pendAngle;
  float prevPendAngle = 0;
  float pendVel;
  float cartPos;
  float prevCartPos = 0;
  float cartVel;
  float voltageCmd;
  float pwmCmd;

  // Bring the pendulum encoder into the range 0 - 2400
  while (getPendulumEncoder() < 0) pendulumEnc += 2400;
  while (getPendulumEncoder() > 2400) pendulumEnc -= 2400;
  
  Serial.println("Balancing");
  Serial.print("Pendulum position = "); Serial.println(getPendulumEncoder());

  while (1)
  {
    // PID every 10ms
    if ((millis() - loopTime) >= lt)
    {
      loopTime += lt;
      
      // Get position and angle measurements, estimate velocities
      pendAngle = (getPendulumEncoder()-pt)*angle_conv_factor_rad;
      pendVel = (pendAngle - prevPendAngle)/(lt/1000);
      cartPos = getMotorEncoder()*cart_conv_factor_m;
      cartVel = (cartPos - prevCartPos)/(lt/1000);

      // Quit if error too large (+/-45 degrees)
      if ((pendAngle > 0.802851) || (pendAngle < -0.8020851))
      {
        motor(0);
        Serial.println("Pendulum out of bounds");
        break;
      }

      // Calculate voltage cmd then PWM cmd
      voltageCmd = -(K_lqr[0]*cartPos + K_lqr[1]*cartVel + K_lqr[2]*pendAngle + K_lqr[3]*pendVel);
      pwmCmd = round(voltageCmd*(255/motorVMax));

      motError = getMotorEncoder();
      if ((motError > pd) && (prevMotError <= pd)) pt = ptSave - pa;
      else if ((motError < -pd) && (prevMotError >= pd)) pt = ptSave + pa;
      else if ((motError > 0) && (motError < prevMotError)) pt = ptSave;
      else if ((motError > -(pd-1)) && (motError < (pd-1))) pt = ptSave;
      prevMotError = motError;

      // Clip motor drive
      if (pwmCmd > 255) pwmCmd = 255;
      else if (pwmCmd < -255) pwmCmd = -255;
      motor(pwmCmd);
    }
    // Quit if carriage out of bounds
    if (motorCheck())
    {
      Serial.println("Motor out of bounds");
      break;
    }
    // Set current measurements as previous ones before starting next loop
    prevCartPos = cartPos;
    prevPendAngle = pendAngle;
  }
  motor(0);
  // Restore programmed Pendulum Top value
  pt= ptSave;
}

void moveCarriage(void)
{
  unsigned long startTime = millis();
  unsigned long loopTime = startTime;
  int32_t error;
  int32_t intMotError = 0, prevMotError = 0;
  int16_t m;
  
  Serial.println("Moving carriage");
  Serial.print("Carriage start position = "); Serial.println(getMotorEncoder());
  
  while (1)
  {
    // PID every 10ms
    if ((millis() - loopTime) >= lt)
    {
      loopTime += lt;
      
      // Motor feedback loop.  Target position is 0.
      error = cp - getMotorEncoder();
//      if ((error > 20) || (error < -20)) // Deadband
      if ((millis() - startTime) < 5000)  // Try for 5 seconds
      {
        intMotError += error;
//        if (aw != 0)
//        {
//          if (intMotError > 2000) intMotError = 2000;
//          else if (intMotError < -2000) intMotError = -2000;
//        }
        m = (ckp * error / 2048) + (cki * intMotError / 4096) + (ckd * (error - prevMotError) / 20);
        if ((aw != 0) && (m > 255) || (m < -255))
        {
          intMotError -= error;
        }
      }
      else
      {
        motor(0);
        break;
      }
      prevMotError = error;

      // Clip motor drive
      if (m > 255) m = 255;
      else if (m < -255) m = -255;
      motor(m);
    }
    // Quit if carriage out of bounds
    if (motorCheck())
    {
      Serial.println("Motor out of bounds");
      break;
    }
  }
  motor(0);
  Serial.print("Carriage end position = "); Serial.println(getMotorEncoder());
}

// Get a command from the serial port.  If the command is a parameter change, then
// change the value.  Return the enumeration for the command, or 0 if none.
uint8_t getCmd(void)
{
  uint8_t idx = 0;
  char buf[10];
  uint8_t parmIdx;

  Serial.println("Enter a command...");
  Serial.flush();
  
  // All characters up to ' ' or '=' are command
  while(1)
  {
    if (Serial.available())
    {
      char ch;
      ch = toupper(Serial.read());
      // Else if character is white space or '=', store terminator and break
      if ((ch == '=') || isspace(ch))
      {
        buf[idx] = 0;
        // Look up the string
        parmIdx = 0;
        while (1)
        {
          if (parmDefArray[parmIdx].parm == NULL)
          {
            // Reached end of array, parm/cmd not found
            Serial.println(">>>Command not found");
            return NO_COMMAND;
          }
          if (strcmp(buf, parmDefArray[parmIdx].parm) == 0)
          {
            // Found the command, break
            break;
          }
          ++parmIdx;
        }
        // Special case, on carriage return, exit immediately
        if (ch == 13) return parmDefArray[parmIdx].cmdEnum;
        break;
      }
      buf[idx] = ch;
      ++idx;
      if (idx >= (sizeof(buf) - 1))
      {
        Serial.println("Too many characters");
        return 0;
      }
    }
  }

  // Remaining characters are value (if any)
  idx=0;
  while(1)
  {
    if (Serial.available())
    {
      buf[idx] = Serial.read();
      // If character is carriage return, store terminator and break
      if (buf[idx] == 13)
      {
          buf[idx] = 0;
          break;
      }
      ++idx;
      if (idx >= (sizeof(buf) - 1))
      {
        Serial.println("Too many characters");
        return 0;
      }
    }
  }
  // Display and store value
  if (parmDefArray[parmIdx].parmVal != NULL)
  {
    *(parmDefArray[parmIdx].parmVal) = atoi(buf);
    Serial.print(parmDefArray[parmIdx].parm); Serial.print(" = ");
    Serial.println(*(parmDefArray[parmIdx].parmVal));
  }
  return parmDefArray[parmIdx].cmdEnum;
}

//inline float estimateDerivative(que[que_size]){
//	return (que[que_size] - q[0])/(float(que_size)*lt*1000);
//}

void getCmdSave(char *cmd, int32_t *cmdVal)
{
  uint8_t idx = 0;
  char buf[10];
  Serial.flush();
  
  // First character is command
  while (!Serial.available()) {}
  *cmd = Serial.read();

  // Remaining characters are value (if any)
  while(1)
  {
    if (Serial.available())
    {
      buf[idx] = Serial.read();
      // If character is carriage return, store terminator and break
      if (buf[idx] == 13)
      {
          buf[idx] = 0;
          break;
      }
      ++idx;
      if (idx >= (sizeof(buf) - 1))
      {
        Serial.println("Too many characters");
        return 0;
      }
    }
  }
  *cmdVal = atoi(buf);
}

// Speed is -255 (left) to 255 (right)
void motor(int16_t speed)
{
  if (speed > 255) speed = 255;
  else if (speed < -255) speed = -255;
  motorSpeed =speed;
  
  analogWrite(11, 0);
  if (speed == 0)
  {
    digitalWrite(9, 1);
    digitalWrite(10, 1);
  }
  else if (speed >  0)
  {
    digitalWrite(9,0);
    digitalWrite(10,1);
  }
  else
  {
    digitalWrite(9,1);
    digitalWrite(10,0);
    speed = -speed;
  }
  analogWrite(11, speed);
}

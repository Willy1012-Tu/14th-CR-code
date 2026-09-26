#include <IBusBM.h>
#include <SPI.h>
#include <mcp2515.h>

IBusBM IBus;

// System 1: 4-Wheel Mecanum Chassis
const int pwmPin[4] = {7, 6, 5, 4};   
const int in1Pin[4] = {46, 38, 36, 34};
const int in2Pin[4] = {47, 39, 37, 35};

const int basePWM[4]  = {45, 45, 45, 45};      
const int maxPWM[4]   = {255, 255, 255, 255}; 
const bool dirInvert[4] = {false, false, true, true}; 

// System 2: 5th Axis Mechanism (PID)
const int mechEncA = 2; 
const int mechEncB = 3; 
const int mechPwm  = 12; 
const int mechIn1  = 28; 
const int mechIn2  = 29; 

const float Kp = 1.5;    
const float Ki = 0.0; 
const float Kd = 0.0; 
const int mechBasePWM = 20; 
const int deadband = 5;  

volatile long mechPulses = 0; 
long mechTarget = 0;        
long lastError = 0;
float integral = 0;

unsigned long lastTime = 0;

// System 3: Auxiliary Mechanism
const int auxPwm = 11;
const int auxIn1 = 26;
const int auxIn2 = 27;
const int auxSpeed = 100; 

// System 4: CAN Bus (MKS Servo)
MCP2515 mcp2515(53); 
int lastCh8State = -1; // -1: undefined, 0: stop, 1: up, 2: down

void isrMech() { 
  if (digitalRead(mechEncB) == HIGH) {
    mechPulses++; 
  } else {
    mechPulses--; 
  }
}

void setChassisMotor(int i, int speed) {
  if (speed == 0) {
    digitalWrite(in1Pin[i], HIGH);
    digitalWrite(in2Pin[i], HIGH);
    analogWrite(pwmPin[i], 0);
  } else {
    bool wantForward = (speed > 0);
    if (dirInvert[i]) wantForward = !wantForward;

    if (wantForward) { 
      digitalWrite(in1Pin[i], LOW);  
      digitalWrite(in2Pin[i], HIGH); 
    } else { 
      digitalWrite(in1Pin[i], HIGH); 
      digitalWrite(in2Pin[i], LOW); 
    }

    int outPWM = abs(speed);
    if (outPWM < basePWM[i]) outPWM = basePWM[i];
    if (outPWM > maxPWM[i]) outPWM = maxPWM[i];
    analogWrite(pwmPin[i], outPWM);
  }
}

int getJoyValue(int channel) {
  int val = IBus.readChannel(channel);
  if (val == 0) return 0; 
  int mapped = map(val, 1000, 2000, -255, 255);
  if (abs(mapped) < 20) return 0; 
  return mapped;
}

void calculateAndSendCAN(uint16_t id, uint8_t mode, uint16_t speed, uint8_t acc, int32_t targetVal) {
  struct can_frame canMsg;
  canMsg.can_id = id;
  canMsg.can_dlc = 8; 
  uint8_t funcCode;
  
  switch (mode) {
    case 1: 
      funcCode = 0xFD;
      canMsg.data[0] = funcCode;
      {
        uint8_t dir = (targetVal >= 0) ? 0 : 1;
        uint32_t absPulses = abs(targetVal); 
        canMsg.data[1] = (dir << 7) | ((speed >> 8) & 0x0F);
        canMsg.data[2] = speed & 0xFF;
        canMsg.data[3] = acc;
        canMsg.data[4] = (absPulses >> 16) & 0xFF;
        canMsg.data[5] = (absPulses >> 8) & 0xFF;
        canMsg.data[6] = absPulses & 0xFF;
      }
      break;
    case 2: funcCode = 0xFE; goto COMMON_STRUCT;
    case 3: funcCode = 0xF4; goto COMMON_STRUCT;
    case 4: 
      funcCode = 0xF5;
      COMMON_STRUCT:
      canMsg.data[0] = funcCode;
      canMsg.data[1] = (speed >> 8) & 0xFF;
      canMsg.data[2] = speed & 0xFF;
      canMsg.data[3] = acc;
      canMsg.data[4] = (targetVal >> 16) & 0xFF;
      canMsg.data[5] = (targetVal >> 8) & 0xFF;
      canMsg.data[6] = targetVal & 0xFF;
      break;
      
    case 5: 
      funcCode = 0xF6;
      canMsg.data[0] = funcCode;
      canMsg.data[1] = (targetVal >= 0) ? 0 : 1; 
      canMsg.data[2] = (speed >> 8) & 0xFF;
      canMsg.data[3] = speed & 0xFF;
      canMsg.data[4] = acc;
      canMsg.data[5] = 0; 
      canMsg.data[6] = 0;
      break;
  }

  uint16_t sum = canMsg.can_id;
  for (int i = 0; i < 7; i++) sum += canMsg.data[i];
  canMsg.data[7] = sum & 0xFF;

  mcp2515.sendMessage(&canMsg);
}

void setup() {
  IBus.begin(Serial1); 

  for (int i = 0; i < 4; i++) {
    pinMode(pwmPin[i], OUTPUT);
    pinMode(in1Pin[i], OUTPUT);
    pinMode(in2Pin[i], OUTPUT);
    digitalWrite(in1Pin[i], HIGH);
    digitalWrite(in2Pin[i], HIGH);
    analogWrite(pwmPin[i], 0);
  }

  pinMode(mechPwm, OUTPUT);
  pinMode(mechIn1, OUTPUT);
  pinMode(mechIn2, OUTPUT);
  pinMode(mechEncA, INPUT_PULLUP);
  pinMode(mechEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(mechEncA), isrMech, RISING);

  pinMode(auxPwm, OUTPUT);
  pinMode(auxIn1, OUTPUT);
  pinMode(auxIn2, OUTPUT);
  digitalWrite(auxIn1, LOW); 
  digitalWrite(auxIn2, LOW);
  analogWrite(auxPwm, 0);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); 
  mcp2515.setNormalMode();
}

void loop() {
  unsigned long now = millis();
  unsigned long dt = now - lastTime;

  if (dt >= 10) { 
    
    // System 1: Chassis Kinematics
    float lx = getJoyValue(3); 
    float ly = getJoyValue(1); 
    float rx = getJoyValue(0); 

    if (lx == 0 && ly == 0 && rx == 0) {
      for (int i = 0; i < 4; i++) setChassisMotor(i, 0);
    } else {
      float vLF = (ly + lx + rx); 
      float vLB = (ly - lx + rx); 
      float vRB = -(ly + lx - rx); 
      float vRF = -(ly - lx - rx); 

      float maxV = abs(vLF);
      if (abs(vLB) > maxV) maxV = abs(vLB);
      if (abs(vRB) > maxV) maxV = abs(vRB);
      if (abs(vRF) > maxV) maxV = abs(vRF);

      if (maxV > 255) { 
        vLF = (vLF / maxV) * 255; vLB = (vLB / maxV) * 255; 
        vRB = (vRB / maxV) * 255; vRF = (vRF / maxV) * 255; 
      }

      float v[4] = {vLF, vLB, vRB, vRF};
      for (int i = 0; i < 4; i++) {
        if (abs(v[i]) > 20) setChassisMotor(i, v[i]);
        else setChassisMotor(i, 0); 
      }
    }

    // System 2: 5th Axis PID Control
    int ch6 = IBus.readChannel(6); 
    if (ch6 > 1500) mechTarget = -2700; 
    else mechTarget = 0;   

    long currentP;
    noInterrupts(); currentP = mechPulses; interrupts();
    
    long error = mechTarget - currentP;
    float derivative = (error - lastError) / (float)dt;

    if (abs(error) > deadband && abs(error) < 100) {
      integral += (error * dt);
      integral = constrain(integral, -1500, 1500);
    } else {
      integral = 0; 
    }

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    int finalPWM = abs(output);

    if (finalPWM > 0 && abs(error) > deadband) finalPWM += mechBasePWM; 
    if (finalPWM > 255) finalPWM = 255;
    if (abs(error) <= deadband) finalPWM = 0;

    lastError = error; 
    bool wantForward = (output > 0);

    if (abs(error) <= deadband) { 
      digitalWrite(mechIn1, LOW);  
      digitalWrite(mechIn2, LOW); 
      analogWrite(mechPwm, 255);
    } else {
      if (wantForward) {
        digitalWrite(mechIn1, LOW);  
        digitalWrite(mechIn2, HIGH);
      } else {
        digitalWrite(mechIn1, HIGH); 
        digitalWrite(mechIn2, LOW);
      }
      analogWrite(mechPwm, finalPWM);
    }

    // System 3: Auxiliary Mechanism
    int ch9  = IBus.readChannel(8); 
    int ch10 = IBus.readChannel(9); 

    if (ch9 > 1500) {
      digitalWrite(auxIn1, HIGH);
      digitalWrite(auxIn2, LOW);
      analogWrite(auxPwm, auxSpeed);
    } else if (ch10 > 1500) {
      digitalWrite(auxIn1, LOW);
      digitalWrite(auxIn2, HIGH);
      analogWrite(auxPwm, auxSpeed);
    } else {
      digitalWrite(auxIn1, LOW);
      digitalWrite(auxIn2, LOW);
      analogWrite(auxPwm, 0);
    }

    // System 4: CAN Bus MKS Control
    int ch8 = IBus.readChannel(7); 
    int currentCh8State = -1; 

    if (ch8 > 1800) {
      currentCh8State = 2; // Down
    } else if (ch8 > 1300 && ch8 < 1700) {
      currentCh8State = 1; // Up
    } else if (ch8 > 800 && ch8 <= 1300) { 
      currentCh8State = 0; // Stop
    }

    if (currentCh8State != -1 && currentCh8State != lastCh8State) {
      if (currentCh8State == 1) {
        calculateAndSendCAN(1, 5, 500, 2, 1);
      } else if (currentCh8State == 2) {
        calculateAndSendCAN(1, 5, 1500, 2, -1);
      } else if (currentCh8State == 0) {
        calculateAndSendCAN(1, 5, 500, 2, 1);
      }
      lastCh8State = currentCh8State;
    }

    lastTime = now;
  }
}

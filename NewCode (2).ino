#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <AFMotor.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

#include <avr/pgmspace.h>
typedef const __FlashStringHelper* FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper*>(s)
#define speed 200

Adafruit_PWMServoDriver robotarm = Adafruit_PWMServoDriver();
#define serv1 10
#define serv2 9

Servo servo2;
Servo servo3;

Adafruit_PWMServoDriver servo1(0);
Adafruit_PWMServoDriver servo4(2);

AF_DCMotor motor1(2);
AF_DCMotor motor2(4);

// These can be changed freely when using the bitbanged protocol
const byte PIN_PS2_ATT = A1;
const byte PIN_PS2_CMD = A2;
const byte PIN_PS2_DAT = A0;
const byte PIN_PS2_CLK = A3;

const byte PIN_BUTTONPRESS = A0;
const byte PIN_HAVECONTROLLER = A1;

const char buttonSelectName[] PROGMEM = "Select";
const char buttonL3Name[] PROGMEM = "L3";
const char buttonR3Name[] PROGMEM = "R3";
const char buttonStartName[] PROGMEM = "Start";
const char buttonUpName[] PROGMEM = "Up";
const char buttonRightName[] PROGMEM = "Right";
const char buttonDownName[] PROGMEM = "Down";
const char buttonLeftName[] PROGMEM = "Left";
const char buttonL2Name[] PROGMEM = "L2";
const char buttonR2Name[] PROGMEM = "R2";
const char buttonL1Name[] PROGMEM = "L1";
const char buttonR1Name[] PROGMEM = "R1";
const char buttonTriangleName[] PROGMEM = "Triangle";
const char buttonCircleName[] PROGMEM = "Circle";
const char buttonCrossName[] PROGMEM = "Cross";
const char buttonSquareName[] PROGMEM = "Square";
const char buttonNoneName[] PROGMEM = "None";

const char* const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {
  buttonSelectName,
  buttonL3Name,
  buttonR3Name,
  buttonStartName,
  buttonUpName,
  buttonRightName,
  buttonDownName,
  buttonLeftName,
  buttonL2Name,
  buttonR2Name,
  buttonL1Name,
  buttonR1Name,
  buttonTriangleName,
  buttonCircleName,
  buttonCrossName,
  buttonSquareName
};

byte psxButtonToIndex(PsxButtons psxButtons) {
  byte i;

  for (i = 0; i < PSX_BUTTONS_NO; ++i) {
    if (psxButtons & 0x01) {
      break;
    }

    psxButtons >>= 1U;
  }

  return i;
}

FlashStr psxgetButtonName(PsxButtons psxButton) {
  FlashStr ret = F("");

  byte b = psxButtonToIndex(psxButton);
  if (b < PSX_BUTTONS_NO) {
    PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
    ret = PSTR_TO_F(bName);
  }

  return ret;
}

void dumpButtons(PsxButtons psxButtons) {
  static PsxButtons lastB = 0;

  if (psxButtons != lastB) {
    lastB = psxButtons;  // Save it before we alter it

    Serial.print(F("Pressed: "));

    for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
      byte b = psxButtonToIndex(psxButtons);


      if (b < PSX_BUTTONS_NO) {
        PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
        Serial.print(PSTR_TO_F(bName));
        Serial.print(b);
        switch(b){
          case 4:{
            forward();
            delay(500);
            break;
          }
          case 6:{
            backward();
            delay(500);
            break;
          }
          case 5:{
            right();
            delay(500);
            stop();
            break;
          }
          case 7:{
            left();
            delay(500);
            stop();
            break;
          }
          case 10:{
            stop();
            break;
          }
          case 13:{
            break;
          }
        }
      }
      
      psxButtons &= ~(1 << b);

      if (psxButtons != 0) {
        Serial.print(F(", "));
      }
    }
    Serial.println();
  }
}

void dumpAnalog(const char* str, const byte x, const byte y) {
  Serial.print(str);
  Serial.print(F(" analog: x = "));
  Serial.print(x);
  Serial.print(F(", y = "));
  Serial.println(y);
}



const char ctrlTypeUnknown[] PROGMEM = "Unknown";
const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

const char* const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
  ctrlTypeUnknown,
  ctrlTypeDualShock,
  ctrlTypeDsWireless,
  ctrlTypeGuitHero,
  ctrlTypeOutOfBounds
};







PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;
boolean haveController = false;

void setup() {
  fastPinMode(PIN_BUTTONPRESS, OUTPUT);
  fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

  delay(300);

  Serial.begin(115200);
  Serial.println(F("Ready!"));

  servo2.attach(serv1);
  servo3.attach(serv2);

  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
}

void loop() {
  static byte slx, sly, srx, sry;

  fastDigitalWrite(PIN_HAVECONTROLLER, haveController);

  if (!haveController) {
    if (psx.begin()) {
      Serial.println(F("Controller found!"));
      delay(300);
      if (!psx.enterConfigMode()) {
        Serial.println(F("Cannot enter config mode"));
      } else {
        PsxControllerType ctype = psx.getControllerType();
        PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte>(ctype) : PSCTRL_MAX])));
        Serial.print(F("Controller Type is: "));
        Serial.println(PSTR_TO_F(cname));

        if (!psx.enableAnalogSticks()) {
          Serial.println(F("Cannot enable analog sticks"));
        }

        //~ if (!psx.setAnalogMode (false)) {
        //~ Serial.println (F("Cannot disable analog mode"));
        //~ }
        //~ delay (10);

        if (!psx.enableAnalogButtons()) {
          Serial.println(F("Cannot enable analog buttons"));
        }

        if (!psx.exitConfigMode()) {
          Serial.println(F("Cannot exit config mode"));
        }
      }

      haveController = true;
    }
  } 
  else {
    if (!psx.read()) {
      Serial.println(F("Controller lost :("));
      haveController = false;
    }

    byte lx, ly,rx, ry;
    psx.getLeftAnalog(lx, ly);
    psx.getRightAnalog(rx, ry);

    if (lx != slx || ly != sly) {
      dumpAnalog("Left", lx, ly);
      slx = lx;
      sly = ly;
      if(sly>=128){
        for(int i=128;i<=sly;i++){
          servo2.write(i);
          delay(15);
        }
      }
      else if(sly<=128){
        for(int i=sly;i>=128;i--){
          delay(100);
          servo2.write(i);
        }
      }
    }
    
    if (rx != srx || ry != sry) {
      dumpAnalog("Right", rx, ry);
      srx = rx;
      sry = ry;
      if(sry>=128){
        for(int i=128;i<=sry;i++){
          servo3.write(i);
          delay(15);
        }
        delay(15);
      }
      else if(sry<=128){
        for(int i=sry;i>=128;i--){
          delay(100);
          servo3.write(i);
        }
      }
    }
    else {
      fastDigitalWrite(PIN_BUTTONPRESS, !!psx.getButtonWord());
      dumpButtons(psx.getButtonWord());
    }
  }


  delay(1000 / 60);
}

void forward(){
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}
void backward(){
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
}
void right(){
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
}
void left(){
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
}
void stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}
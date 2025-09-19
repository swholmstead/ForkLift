#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>   // by Richard Quesada

ControllerPtr myController;

#define steeringServoPin 23
#define mastTiltServoPin 22

#define mastMotor0 32     // Used for controlling mast movement
#define mastMotor1 33     // Used for controlling mast movement
#define lightsAttach0 21  // Used for controlling headlight control
#define lightsAttach1 19  // Used for controlling headlight control

#define leftMotor0 17     // Used for controlling the left motor movement
#define leftMotor1 18     // Used for controlling the left motor movement
#define rightMotor0 26    // Used for controlling the right motor movement
#define rightMotor1 25    // Used for controlling the right motor movement

#define throttleDeadZone 15
#define steeringDeadZone 30
#define mastDeadZone 20
#define mastMoveSpeed 1
#define mastTiltMin 75
#define mastTiltMax 140
#define mastTiltDeadZone 212

#define steeringMaxSpeed 3

#define wiggleCountMax 6

Servo steeringServo;
Servo mastTiltServo;

int lightSwitchTime = 0;
float steeringValue = 90;
float steeringAdjustment = 1;
int steeringTrim = 0;
int mastTiltValue = 120;
bool lightsOn = false;
unsigned long lastWiggleTime = 0;
int wiggleCount = 0;
int wiggleDirection = 1;
long wiggleDelay = 100;
bool shouldWiggle = false;

void onConnectedController(ControllerPtr ctl) {
  ControllerProperties properties = ctl->getProperties();
  auto btAddress = properties.btaddr;
  if (myController == nullptr) {
    Serial.printf("CALLBACK: Controller is connected, ADDR=%2X:%2X:%2X:%2X:%2X:%2X\n",
                  btAddress[0], btAddress[1], btAddress[2],btAddress[3], btAddress[4], btAddress[5]);
    // Additionally, you can get certain gamepad properties like:
    // Model, VID, PID, BTAddr, flags, etc.
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                  properties.product_id);
    myController = ctl;
    BP32.enableNewBluetoothConnections(false);
    ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */);
    shouldWiggle = true;
    processLights(true);
  }
  else {
    Serial.printf("CALLBACK: Controller connected, ADDR=%2X:%2X:%2X:%2X:%2X:%2X, but another controller already connected.\n",
                  btAddress[0], btAddress[1], btAddress[2],btAddress[3], btAddress[4], btAddress[5]);
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  ControllerProperties properties = ctl->getProperties();
  auto btAddress = properties.btaddr;
  if (myController == ctl) {
    Serial.printf("CALLBACK: Controller disconnected ADDR=%2X:%2X:%2X:%2X:%2X:%2X\n",
                  btAddress[0], btAddress[1], btAddress[2],btAddress[3], btAddress[4], btAddress[5]);
    myController = nullptr;
    BP32.enableNewBluetoothConnections(true);
  }
  else {
    Serial.printf("CALLBACK: Controller disconnected, ADDR=%2X:%2X:%2X:%2X:%2X:%2X, but not active controller\n",
                  btAddress[0], btAddress[1], btAddress[2],btAddress[3], btAddress[4], btAddress[5]);
  }
}

void processGamepad(ControllerPtr ctl) {
  //Steering
  processSteering(ctl->axisX());
  //Throttle
  processThrottle(ctl->axisY());
  //Rasing and lowering of mast
  processMast(ctl->axisRY());
  //MastTilt
  processMastTilt(ctl->axisRX());
  //Aux
  processLights(ctl->thumbR() | ctl->a());

  processWiggle(ctl->b());

  processTrimRight(ctl->r1());
  processTrimLeft(ctl->l1());
}

void processWiggle(bool value) {
  if (value) {
    shouldWiggle = true;
    processLights(true);
  }
}

void wiggle() {                  
  unsigned long currentTime = millis();
  if (abs((int)(currentTime - lastWiggleTime)) >= wiggleDelay) {
    lastWiggleTime = currentTime;
    wiggleDirection = -wiggleDirection;
    wiggleCount++;
    moveMotor(rightMotor0, rightMotor1, wiggleDirection * 100);
    moveMotor(leftMotor0, leftMotor1, -1 * wiggleDirection * 100);
    if (wiggleCount >= wiggleCountMax) {
      moveMotor(leftMotor0, leftMotor1, 0);
      moveMotor(rightMotor0, rightMotor1, 0);
      wiggleCount = 0;
      shouldWiggle = false;
      processLights(true);
    }
  }
}

void processThrottle(int newValue) {
  if (abs(newValue) <= throttleDeadZone) {
    moveMotor(leftMotor0, leftMotor1, 0);
    moveMotor(rightMotor0, rightMotor1, 0);
  } else {
    float throttleValue = newValue / 2;
    if (steeringValue > 100) {
      // turning left, apply steeringAdjustment to left wheel
      moveMotor(leftMotor0, leftMotor1, throttleValue * steeringAdjustment);
      moveMotor(rightMotor0, rightMotor1, throttleValue);
    } else if (steeringValue < 80) {
      // turning right, apply steeringAdjustment to right wheel
      moveMotor(leftMotor0, leftMotor1, throttleValue);
      moveMotor(rightMotor0, rightMotor1, throttleValue * steeringAdjustment);
    } else {
      // going straight
      moveMotor(leftMotor0, leftMotor1, throttleValue);
      moveMotor(rightMotor0, rightMotor1, throttleValue);
    }
  }
}

void processMast(int newValue) {
  int mastValue = newValue / 2;
  if (mastValue > mastDeadZone) {
    mastValue -= mastDeadZone;
    moveMotor(mastMotor0, mastMotor1, mastValue);
  } else if (mastValue < -1 * mastDeadZone) {
    mastValue += mastDeadZone;
    moveMotor(mastMotor0, mastMotor1, mastValue);
  } else {
    moveMotor(mastMotor0, mastMotor1, 0);
  }
}

void processTrimRight(bool trimValue) {
  if (trimValue && steeringTrim < 20) {
    steeringTrim++;
    delay(100);
  }
}

void processTrimLeft(bool trimValue) {
  if (trimValue && steeringTrim > -20) {
    steeringTrim--;
    delay(100);
  }
}

void processSteering(int newValue) {
  // remove dead zone
  if (abs(newValue) < steeringDeadZone) {
    newValue = 0;
  }
  else if (newValue > 0) {
    newValue -= steeringDeadZone;
  }
  else {
    newValue += steeringDeadZone;
  }

  // calculate target steering angle
  int targetValue = (90 - (newValue / 10));
  // calculate and limit change rate 
  int delta = steeringValue - targetValue;
  if (abs(delta) > steeringMaxSpeed) {
    delta = steeringMaxSpeed * (delta > 0 ? 1 : -1);
  }
  steeringValue -= delta;
  steeringServo.write(steeringValue - steeringTrim);

  // calculate speed of opposite wheel based upon steering (1.0 .. 0.25)
  steeringAdjustment = 1 - (abs(newValue) / (768.0 - steeringDeadZone));
  //Serial.printf("steeringAdjustment: %f\n", steeringAdjustment);
}

void processMastTilt(int newValue) {
  int mastSpeed = 0;
  if (newValue > mastTiltDeadZone) {
    mastSpeed = (newValue - mastTiltDeadZone) / 150 + 1;
  } else if (abs(newValue) > mastTiltDeadZone) {
    mastSpeed = (newValue + mastTiltDeadZone) / 150 - 1;
  }
  mastTiltValue -= mastSpeed;
  if (mastTiltValue > mastTiltMax) {
    mastTiltValue = mastTiltMax;
  } else if (mastTiltValue < mastTiltMin) {
    mastTiltValue = mastTiltMin;
  }
  mastTiltServo.write(mastTiltValue);
  // Serial.printf("mast tilt: %d\n", mastTiltValue);
}

void processLights(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchTime) > 200) {
    if (lightsOn) {
      digitalWrite(lightsAttach0, LOW);
      digitalWrite(lightsAttach1, LOW);
      lightsOn = false;
    } else {
      digitalWrite(lightsAttach0, LOW);
      digitalWrite(lightsAttach1, HIGH);
      lightsOn = true;
    }

    lightSwitchTime = millis();
  }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 1) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -1) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}

void processController() {
  if (myController && myController->isConnected() && myController->hasData()) {
    if (myController->isGamepad()) {
      processGamepad(myController);
    } else {
      Serial.println("Unsupported controller");
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(mastMotor0, OUTPUT);
  pinMode(mastMotor1, OUTPUT);
  pinMode(lightsAttach0, OUTPUT);
  pinMode(lightsAttach1, OUTPUT);
  digitalWrite(lightsAttach0, LOW);
  digitalWrite(lightsAttach1, LOW);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);


  steeringServo.attach(steeringServoPin);
  steeringServo.write(steeringValue);
  mastTiltServo.attach(mastTiltServoPin);
  mastTiltServo.write(mastTiltValue);

  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  // You could add additional error handling here,
  // such as logging the error or attempting to recover.
  // For example, you might attempt to reset the MCP23X17
  // and retry initialization before giving up completely.
  // Then, you could gracefully exit the program or continue
  // running with limited functionality.
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processController();
  }
  if (shouldWiggle) {
    wiggle();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
}

#include <Bluepad32.h>

int motor1Pin1 = 25; 
int motor1Pin2 = 26; 
int motor2Pin1 = 32; 
int motor2Pin2 = 33; 

// ====== PWM Channels ====== //
const int pwmChannelA1 = 0;
const int pwmChannelA2 = 1;
const int pwmChannelB1 = 2;
const int pwmChannelB2 = 3;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(), ctl->axisY(),   // Left joystick X,Y
  ctl->axisRX(), ctl->axisRY()  // Right joystick X,Y
  );
}

void forward(int speed) {
  Serial.println("Moving Forward");
  ledcWrite(pwmChannelA1, speed);
  ledcWrite(pwmChannelA2, 0);
  ledcWrite(pwmChannelB1, speed);
  ledcWrite(pwmChannelB2, 0);
}

void backward(int speed) {
  Serial.println("Moving Backward");
  ledcWrite(pwmChannelA1, 0);
  ledcWrite(pwmChannelA2, speed);
  ledcWrite(pwmChannelB1, 0);
  ledcWrite(pwmChannelB2, speed);
}

void left(int speed) {
  Serial.println("Turning Left");
  ledcWrite(pwmChannelA1, 0);
  ledcWrite(pwmChannelA2, speed);
  ledcWrite(pwmChannelB1, speed);
  ledcWrite(pwmChannelB2, 0);
}

void right(int speed) {
  Serial.println("Turning Right");
  ledcWrite(pwmChannelA1, speed);
  ledcWrite(pwmChannelA2, 0);
  ledcWrite(pwmChannelB1, 0);
  ledcWrite(pwmChannelB2, speed);
}

void stopMotor() {
  ledcWrite(pwmChannelA1, 0);
  ledcWrite(pwmChannelA2, 0);
  ledcWrite(pwmChannelB1, 0);
  ledcWrite(pwmChannelB2, 0);
  Serial.println("Motors Stopped");
// ========= GAME CONTROLLER ACTIONS SECTION ========= //
}
void processGamepad(ControllerPtr ctl) {

  int speedY = map(abs(ctl->axisY()), 0, 512, 0, 255);
  int speedX = map(abs(ctl->axisRX()), 0, 512, 0, 255);


  //== LEFT JOYSTICK - UP ==//
  if (ctl->axisY() <= -25) {
    backward(speedY);
    }

  //== LEFT JOYSTICK - DOWN ==//
  else if (ctl->axisY() >= 25) {
    forward(speedY);
  }

  //== RIGHT JOYSTICK - LEFT ==//
  else if (ctl->axisRX() <= -25) {
    left(speedX);
  }

  //== RIGHT JOYSTICK - RIGHT ==//
  else if (ctl->axisRX() >= 25) {
    right(speedX);
  }

  else {
  stopMotor();
  }

  dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {

  BP32.forgetBluetoothKeys();

  Serial.begin(115200);
  Serial.println("Starting setup...");

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // configure LEDC PWM
  // ledcAttachChannel(freq, resolution, pwmChannel);
  int freq = 30000;        // frekuensi PWM
  int resolution= 8;
   ledcSetup(pwmChannelA1, freq, resolution);
  ledcSetup(pwmChannelA2, freq, resolution);
  ledcSetup(pwmChannelB1, freq, resolution);
  ledcSetup(pwmChannelB2, freq, resolution);

  ledcAttachPin(motor1Pin1, pwmChannelA1);
  ledcAttachPin(motor1Pin2, pwmChannelA2);
  ledcAttachPin(motor2Pin1, pwmChannelB1);
  ledcAttachPin(motor2Pin2, pwmChannelB2);


  // koneksi bluetooth
  BP32.setup(&onConnectedController, &onDisconnectedController);
  Serial.println("Setup selesai, siap konek ke gamepad!");
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  delay(1);
}
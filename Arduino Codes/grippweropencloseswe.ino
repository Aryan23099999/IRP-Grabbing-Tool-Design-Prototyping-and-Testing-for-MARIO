// Motor pins
#define M1_PWM 5
#define M1_DIR 4
#define M2_PWM 7
#define M2_DIR 6

// Motor speed (0–255)
const int MOTOR_SPEED = 200;

void setup() {
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  Serial.begin(115200);
  while (!Serial) { } // wait for Serial on some boards

  stopMotors();
  Serial.println("Keyboard control ready:");
  Serial.println("  Press 'w' to CLOSE gripper");
  Serial.println("  Press 's' to OPEN gripper");
  Serial.println("  Press 'e' to STOP");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read(); // clear CR/LF

    if (c == 'w' || c == 'W') {
      closeGripper();
      Serial.println("Closing gripper...");
    }
    else if (c == 's' || c == 'S') {
      openGripper();
      Serial.println("Opening gripper...");
    }
    else if (c == 'e' || c == 'E') {
      stopMotors();
      Serial.println("Stopped.");
    }
  }
}

// ---------- Motor control helpers ----------
void openGripper() {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M1_PWM, MOTOR_SPEED);
  analogWrite(M2_PWM, MOTOR_SPEED);
}

void closeGripper() {
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  analogWrite(M1_PWM, MOTOR_SPEED);
  analogWrite(M2_PWM, MOTOR_SPEED);
}

void stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

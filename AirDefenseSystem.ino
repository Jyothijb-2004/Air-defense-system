#include <Servo.h>
#include <Stepper.h>
#define SERVO_PIN 9
#define TRIG_PIN 7
#define ECHO_PIN 6
#define LASER_PIN 8
const int STEPS_PER_REV = 2048; // 28BYJ-48 typical (change if needed)
const int STEPPER_RPM = 8; // lower rpm -> lower current spikes
const unsigned long SWEEP_STEP_DELAY = 30; // ms between servo angle increments
const unsigned long SERVO_SETTLE_MS = 8; // tiny settle after servo write
const int FIRE_DISTANCE_CM = 3; // laser fires when distance <= this
const int STEP_MOVE_DELAY_MS = 4; // delay between single-step moves (smoothness)
// *** NEW: only HALF revolution of stepper for full 0..180° servo sweep ***
const int STEPS_FOR_SWEEP = STEPS_PER_REV / 2; // 0..1024 steps (≈180°)
// ======== STEP PINS ========
// Option A: constructor order that worked in many setups (IN1, IN3, IN2, IN4)

Stepper stepper(STEPS_PER_REV, 2, 4, 3, 5);
// Option B (if needed):
// Stepper stepper(STEPS_PER_REV, 2, 3, 4, 5);
Servo radarServo;
int angle = 0;
int sweepDir = 1;
unsigned long lastSweepMillis = 0;
// track current absolute step position for 0..STEPS_FOR_SWEEP
int currentStepPos = 0;
void setup() {
 Serial.begin(115200);
 radarServo.attach(SERVO_PIN);
 pinMode(TRIG_PIN, OUTPUT);
 pinMode(ECHO_PIN, INPUT);
 pinMode(LASER_PIN, OUTPUT);
 digitalWrite(LASER_PIN, LOW);
 stepper.setSpeed(STEPPER_RPM);
 // initial sync to center
 radarServo.write(90);
 delay(50);
 currentStepPos = STEPS_FOR_SWEEP / 2; // middle of travel for 90°
}
void loop() {
 unsigned long now = millis();
 if (now - lastSweepMillis >= SWEEP_STEP_DELAY) {lastSweepMillis = now;
              // 1) Servo sweep step
radarServo.write(angle);
delay(SERVO_SETTLE_MS);
// 2) Read distance
int dist = readUltrasonicCM();
// 3) Print status
Serial.print("Angle: ");
Serial.print(angle);
Serial.print(" Dist: ");
if (dist == 999) Serial.println("Out");
else {
Serial.print(dist);
Serial.println(" cm");
}
// 4) Compute target step position
      // Servo 0° -> stepper 0 steps
// Servo 180°-> stepper STEPS_FOR_SWEEP steps (half revolution)
int targetSteps = map(angle, 0, 180, 0, STEPS_FOR_SWEEP);
// 5) Smoothly move stepper toward target
int diff = targetSteps - currentStepPos;
if (diff != 0) {
 int stepDirection = (diff > 0) ? 1 : -1;
 for (int s = 0; s < abs(diff); s++) {
 stepper.step(stepDirection);
 currentStepPos += stepDirection;
 delay(STEP_MOVE_DELAY_MS);
 }
 }
 // 6) Laser firing logic (only when <= FIRE_DISTANCE_CM)
 if (dist != 999 && dist <= FIRE_DISTANCE_CM) {
 digitalWrite(LASER_PIN, HIGH);
 delay(100);
 digitalWrite(LASER_PIN, LOW);
 } else {
 digitalWrite(LASER_PIN, LOW);
 }
 // 7) update sweep angle (bounce 0 <-> 180)
 angle += sweepDir;
 if (angle <= 0) {
 angle = 0;
sweepDir = 1;
 } else if (angle >= 180) {
 angle = 180;
sweepDir = -1;
 }
 }
}
// --- ultrasonic single reading (returns cm, 999 = timeout) ---
int readUltrasonicCM() {
 digitalWrite(TRIG_PIN, LOW);
 delayMicroseconds(2);
 digitalWrite(TRIG_PIN, HIGH);
 delayMicroseconds(10);
 digitalWrite(TRIG_PIN, LOW);
 unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout
 if (duration == 0) return 999;
 int cm = (int)(duration / 58.2);
return cm;
}
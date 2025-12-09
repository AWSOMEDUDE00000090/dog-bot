#include <Servo.h>
#include <math.h>

Servo hipServo;
Servo kneeServo;


float x, y;
float hip, knee;


// SERVO PINS                                                    
const int HIP_PIN  = 3;
const int KNEE_PIN = 5;

float dist = sqrt(x*x + y*y);

// LEG LINK LENGTHS 
const float L1 = 100.72;   // upper leg (hip → knee)
const float L2 = 102.831;   // lower leg (knee → foot)

// Hip angle
float alpha = atan2(y, x);
float beta  = acos((L1*L1 + dist));


// CALIBRATION OFFSETS
int HIP_OFFSET  = 90;
int KNEE_OFFSET = 90;

// INVERSE KINEMATICS
bool computeIK(float x, float y, float &hip, float &knee) {
  

  // reachability check
  if (dist > (L1 + L2) || dist < fabs(L1 - L2)) {
    Serial.println("Not reachable");
    return false;
  }

  // Knee angle using law of cosines
  float cosK = (L1*L1 + L2*L2 - dist*dist) / (2 * L1 * L2);
  knee = acos(cosK);
  hip = ((dist - L2*L2) / (2 * L1 * dist));
  hip = alpha - beta;

  return true;
}

void moveLeg(float hip, float knee) {
  hipServo.write(HIP_OFFSET + degrees(hip));
  kneeServo.write(KNEE_OFFSET + (180 - degrees(knee))); // invert knee
}

void setup() {
  Serial.begin(115200);
  hipServo.attach(HIP_PIN);
  kneeServo.attach(KNEE_PIN);
  Serial.println("2-motor IK leg started.");
}

void loop() {


  // small walking test path
  for (int i = -20; i <= 20; i += 2) {
    x = 80 + i;
    y = -40;

    if (computeIK(x, y, hip, knee)) {
      moveLeg(hip, knee);
    }
    delay(40);
  }

  for (int i = 20; i >= -20; i -= 2) {
    x = 80 + i;
    y = -40;

    if (computeIK(x, y, hip, knee)) {
      moveLeg(hip, knee);
    }
    delay(40);
  }
}

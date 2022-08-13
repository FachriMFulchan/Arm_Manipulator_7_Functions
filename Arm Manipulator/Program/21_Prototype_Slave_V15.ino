/* Program Slave Arm Robot
   Data didapat dari komunikasi serial dengan master
   Data yang didapat adalah degree
   Kemudian data di convert ke step dan di assign ke stepper (Microstepping = 1600)

  Kode :
  1 -> Pergerakan Joystick
  2 -> Pergerakan Kartesian

  Progress :
  - Konversi degree ke step untuk feeds ke stepper
  - Memakai library accel stepper
*/

// Library AccelStepper
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
int pinServo = 22;


//inisial pin Driver stepper TB6600
AccelStepper stepper1(AccelStepper::DRIVER, 3, 2); // step,DIR
AccelStepper stepper2(AccelStepper::DRIVER, 5, 4); // step,DIR
AccelStepper stepper3(AccelStepper::DRIVER, 7, 6); // step,DIR
AccelStepper stepper4(AccelStepper::DRIVER, 9, 8); // step,DIR
AccelStepper stepper5(AccelStepper::DRIVER, 11, 10); // step,DIR
AccelStepper stepper6(AccelStepper::DRIVER, 13, 12); // step,DIR
MultiStepper steppers;
Servo myServo; //Servo Gripper

// Pin Steppers
#define dirPinJ1 2
#define stepPinJ1 3
#define dirPinJ2 4
#define stepPinJ2 5
#define dirPinJ3 6
#define stepPinJ3 7
#define dirPinJ4 8
#define stepPinJ4 9
#define dirPinJ5 10
#define stepPinJ5 11
#define dirPinJ6 12
#define stepPinJ6 13

// lastState dari data
int lastStateJ1 = 0;
int lastStateJ2 = 0;
int lastStateJ3 = 0;
int lastStateJ4 = 0;
int lastStateJ5 = 0;
int lastStateJ6 = 0;

// currentState data sekarang
int currentStateJ1 = 0;
int currentStateJ2 = 0;
int currentStateJ3 = 0;
int currentStateJ4 = 0;
int currentStateJ5 = 0;
int currentStateJ6 = 0;

// Untuk memastikan perhitungan algoritma benar
int counter1 = 0;
int counter2 = 0;
int counter3 = 0;
int counter4 = 0;
int counter5 = 0;
int counter6 = 0;

// Parameter Angle
int maxAngleJ1 = 90;
int minAngleJ1 = -90;
int maxAngleJ2 = 90;
int minAngleJ2 = -45;
int maxAngleJ3 = 45;
int minAngleJ3 = -90;
int maxAngleJ4 = 180;
int minAngleJ4 = -180;
int maxAngleJ5 = 90;
int minAngleJ5 = -90;
int maxAngleJ6 = 360; // infinite
int minAngleJ6 = -360; // infinite

// Parameter step sampai sudut tertentu
//int batasJ1Plus = 1940; // 90 derajat J1
//int batasJ1Min = -1940; // -90 derajat J1
//int batasJ2Plus = 1600; // 90 derajat J2
//int batasJ2Min = -800; // -45 derajat J2
//int batasJ3Plus = 970; // 45 derajat J3
//int batasJ3Min = -1940; // 90 derajat J3
//int batasJ4Plus = 2224; // 180 derajat J4
//int batasJ4Min = -2224; // -180 derajat J4
//int batasJ5Plus = 820; // 90 derajat J5
//int batasJ5Min = -820; // -90 derajat J5
//int batasJ6Plus = 1600; // 180 derajat J6
//int batasJ6Min = -1600; // -180 derajat J6


// Parameter step sampai sudut tertentu (Versi Riska)
int batasJ1Plus = 1920; // 90 derajat J1
int batasJ1Min = -1920; // -90 derajat J1
int batasJ2Plus = 1600; // 90 derajat J2
int batasJ2Min = -800; // -45 derajat J2
int batasJ3Plus = 1000; // 45 derajat J3
int batasJ3Min = -2000; // 90 derajat J3
int batasJ4Plus = 2240; // 180 derajat J4
int batasJ4Min = -2240; // -180 derajat J4
int batasJ5Plus = 860; // 90 derajat J5
int batasJ5Min = -860; // -90 derajat J5
int batasJ6Plus = 1600; // 180 derajat J6
int batasJ6Min = -1600; // -180 derajat J6


// Variable untuk menampung data dari Master
int T1 = 0;
int T2 = 0;
int T3 = 0;
int T4 = 0;
int T5 = 0;
int T6 = 0;
int Grip = 0;
int modeKecepatan = 0;

// Inisiasi variable komdat
String sData, data[12];
int dataInt[12];
bool parsing = false;

// Keadaan untuk fold or home positions
bool stateFold = 0;


void setup() {
  // Inisiasi hubungan serial
  Serial.begin (115200);
  Serial3.begin(115200);

  // pinmode stepper motor
  pinMode(stepPinJ1, OUTPUT);
  pinMode(dirPinJ1, OUTPUT);
  pinMode (stepPinJ2, OUTPUT);
  pinMode (dirPinJ2, OUTPUT);
  pinMode(stepPinJ3, OUTPUT);
  pinMode(dirPinJ3, OUTPUT);
  pinMode (stepPinJ4, OUTPUT);
  pinMode (dirPinJ4, OUTPUT);
  pinMode(stepPinJ5, OUTPUT);
  pinMode(dirPinJ5, OUTPUT);
  pinMode (stepPinJ6, OUTPUT);
  pinMode (dirPinJ6, OUTPUT);

  // Configure each stepper Speed
  //  stepper1.setMaxSpeed(800);
  //  stepper2.setMaxSpeed(800);
  //  stepper3.setMaxSpeed(800);
  //  stepper4.setMaxSpeed(800);
  //  stepper5.setMaxSpeed(800);
  //  stepper6.setMaxSpeed(800);

  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  steppers.addStepper(stepper5);
  steppers.addStepper(stepper6);

  // pinMode Servo
  myServo.attach(pinServo);
}


void loop() {
  receiveEvent();
}


void receiveEvent() {
  // ===== KOMDAT
  // Baca dari serial (ASCII), convert ke char, masukin ke data
  while (Serial3.available() > 0) {
    delay (10);
    char c = Serial3.read();
    sData += c;

    if (c == '$') {
      parsing = true;
    }
  }

  //parsing data
  if (parsing) {
    //    Serial.print ("DATA MASUK:");
    //    Serial.println(sData);

    int q = 0;
    for (int i = 0; i < sData.length(); i++) {
      if (sData[i] == '#') {
        q++;
        data[q] = "";
      }
      else {
        data[q] += sData[i];
      }
    }

    // Konversi string ke integer
    for (int i = 0; i < 12; i++) {
      dataInt[i] = data[i].toInt();
    }


    // Bandingkan mode dengan pengkodean (kode 1 = joystick, kode 2 = kartesian)
    // ===== MODE JOYSTICK
    if (dataInt[1] == 1) {

      // --------- MAIN PROGRAM -----------
      //      Serial.print ("J1 1:");
      //      Serial.println (dataInt[2]);
      //      Serial.print ("J2 2:");
      //      Serial.println (dataInt[3]);
      //      Serial.print ("J3 3:");
      //      Serial.println (dataInt[4]);
      //      Serial.print ("J4 4:");
      //      Serial.println (dataInt[5]);
      //      Serial.print ("YawJ5:");
      //      Serial.println (dataInt[6]);
      //      Serial.print ("RollJ6:");
      //      Serial.println (dataInt[7]);
      //      Serial.print ("GripCount:");
      //      Serial.println (dataInt[8]);
      //      Serial.print ("Fold: ");
      //      Serial.println (dataInt[9]);
      //      Serial.print ("Mode Kecepatan: ");
      //      Serial.println (dataInt[10]);
      //      Serial.println();

      // ==== Menggerakan ke Fold or Home Positions
      // fold to home
      if (dataInt[9] == 1 and stateFold == 0) {
        foldToHome();
        stateFold = 1;
      }
      // home to fold
      else if (dataInt[9] == 0 and stateFold == 1) {
        homeToFold();
        stateFold = 0;
      }

      // ==== Assign data degree dari master
      T1 = dataInt[2];
      T2 = dataInt[3];
      T3 = dataInt[4];
      T4 = dataInt[5];
      T5 = dataInt[6];
      T6 = dataInt[7];
      Grip = dataInt[8];
      modeKecepatan = dataInt[10];

      // ==== convert data degree ke step
      currentStateJ1 = thetaConversions(T1, minAngleJ1, maxAngleJ1, batasJ1Min, batasJ1Plus);
      currentStateJ2 = thetaConversions(T2, minAngleJ2, maxAngleJ2, batasJ2Min, batasJ2Plus);
      currentStateJ3 = thetaConversions(T3, minAngleJ3, maxAngleJ3, batasJ3Min, batasJ3Plus);
      currentStateJ4 = thetaConversions(T4, minAngleJ4, maxAngleJ4, batasJ4Min, batasJ4Plus);
      currentStateJ5 = thetaConversions(T5, minAngleJ5, maxAngleJ5, batasJ5Min, batasJ5Plus);
      currentStateJ6 = thetaRollConversions(T6, batasJ6Min, batasJ6Plus);

      // === Menggerakan stepper sesuai dengan sinyal step
      //      rotateStepper (1, currentStateJ1);
      //      rotateStepper (2, currentStateJ2);
      //      rotateStepper (3, currentStateJ3);
      //      rotateStepper (4, currentStateJ4);
      //      rotateStepper (5, currentStateJ5);
      //      rotateStepper (6, currentStateJ6);

      long positions[6]; // Array of desired stepper positions
      positions[0] = currentStateJ1; // posisi pertama stepper 1
      positions[1] = currentStateJ2; // posisi pertama stepper 2
      positions[2] = currentStateJ3; // posisi pertama stepper 3
      positions[3] = currentStateJ4; // posisi pertama stepper 4
      positions[4] = currentStateJ5; // posisi pertama stepper 5
      positions[5] = currentStateJ6; // posisi pertama stepper 6

      // Low Scale
      if (modeKecepatan == 1) {
        stepper1.setMaxSpeed(300);
        stepper2.setMaxSpeed(300);
        stepper3.setMaxSpeed(300);
        stepper4.setMaxSpeed(300);
        stepper5.setMaxSpeed(300);
        stepper6.setMaxSpeed(300);
      }

      // Mid Scale
      else if (modeKecepatan == 0) {
        stepper1.setMaxSpeed(600);
        stepper2.setMaxSpeed(600);
        stepper3.setMaxSpeed(600);
        stepper4.setMaxSpeed(600);
        stepper5.setMaxSpeed(600);
        stepper6.setMaxSpeed(600);
      }

      // High Scale
      else if (modeKecepatan == 2) {
        stepper1.setMaxSpeed(900);
        stepper2.setMaxSpeed(900);
        stepper3.setMaxSpeed(900);
        stepper4.setMaxSpeed(900);
        stepper5.setMaxSpeed(900);
        stepper6.setMaxSpeed(900);
      }

      steppers.moveTo(positions);
      steppers.runSpeedToPosition(); // Blocks until all are in position
      myServo.write (Grip); //
    }


    // Kode 2 == Kartesian
    else if (dataInt[1] == 2) {
      Serial.println ("On Developing");
      //      Serial.print ("Counter X:");
      //      Serial.println (dataInt[2]);
      //      Serial.print ("Counter Y:");
      //      Serial.println (dataInt[3]);
      //      Serial.print ("Counter Z:");
      //      Serial.println (dataInt[4]);
      //      Serial.print ("Counter Yaw:");
      //      Serial.println (dataInt[5]);
      //      Serial.print ("Counter Pitch:");
      //      Serial.println (dataInt[6]);
      //      Serial.print ("Counter Roll:");
      //      Serial.println (dataInt[7]);
      //      Serial.print ("GripCount:");
      //      Serial.println (dataInt[8]);
      //      Serial.println();
    }

    parsing = false;
    sData = "";
  }
}


/* FUNCTIONS */

// ==== Fungsi untuk converting degree ke step ====
int thetaConversions (int angle, int minAngle, int maxAngle, int minStep, int maxStep) {
  int compare = 0;

  compare = map(angle, minAngle, maxAngle, minStep, maxStep);
  if (compare > maxStep) {
    compare = 0;
  }
  else if ( compare < minStep) {
    compare = 0;
  }
  return compare;
}


// ==== Fungsi untuk converting degree ke step khusus roll ====
int thetaRollConversions (int angle, int minStep, int maxStep) {
  float compare = 0;
  int result = 0;

  if (angle > 0) {
    compare = (angle / 360.0) * maxStep;
  }
  else if (angle < 0) {
    compare = (angle / -360.0) * minStep;
  }
  else if (angle == 0) {
    compare = 0.0;
  }

  // Casting float to integers
  result = int (compare);
  return result;
}


// ==== Menggerakan Arm Robot ke Home Positions ====
void foldToHome () {
  // go to the home position (all joints equal to 0)
  // joint #2
  digitalWrite(dirPinJ2, HIGH);
  int delValue = 4000;
  int incValue = 7;
  int accRate = 530;
  int totSteps = 1390;
  for (int i = 0; i < totSteps; i++)
  {
    if (totSteps > (2 * accRate + 1)) {
      if (i < accRate) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)) {
        //decceleration
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (i < ((totSteps - (totSteps % 2)) / 2)) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2)) / 2)) {
        //decceleration
        delValue = delValue + incValue;
      }
    }
    digitalWrite(stepPinJ2, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(stepPinJ2, LOW);
    delayMicroseconds(delValue);
  }

  // joint #3
  digitalWrite(dirPinJ3, LOW);
  delValue = 4000;
  incValue = 7;
  accRate = 530;
  totSteps = 1650;
  for (int i = 0; i < totSteps; i++)
  {
    if (totSteps > (2 * accRate + 1)) {
      if (i < accRate) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)) {
        //decceleration
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (i < ((totSteps - (totSteps % 2)) / 2)) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2)) / 2)) {
        //decceleration
        delValue = delValue + incValue;
      }
    }
    digitalWrite(stepPinJ3, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(stepPinJ3, LOW);
    delayMicroseconds(delValue);
  }
}


// ==== Menggerakan Arm Robot ke Fold Positions ====
void homeToFold() {
  // go to the fold position
  // joint #3
  digitalWrite(dirPinJ3, HIGH);
  int delValue = 4000;
  int incValue = 7;
  int accRate = 530;
  int totSteps = 1650;
  for (int i = 0; i < totSteps; i++)
  {
    if (totSteps > (2 * accRate + 1)) {
      if (i < accRate) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)) {
        //decceleration
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (i < ((totSteps - (totSteps % 2)) / 2)) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2)) / 2)) {
        //decceleration
        delValue = delValue + incValue;
      }
    }
    digitalWrite(stepPinJ3, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(stepPinJ3, LOW);
    delayMicroseconds(delValue);
  }

  // joint #2
  digitalWrite(dirPinJ2, LOW);
  delValue = 4000;
  incValue = 7;
  accRate = 530;
  totSteps = 1390;
  for (int i = 0; i < totSteps; i++)
  {
    if (totSteps > (2 * accRate + 1)) {
      if (i < accRate) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)) {
        //decceleration
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (i < ((totSteps - (totSteps % 2)) / 2)) {
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2)) / 2)) {
        //decceleration
        delValue = delValue + incValue;
      }
    }
    digitalWrite(stepPinJ2, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(stepPinJ2, LOW);
    delayMicroseconds(delValue);
  }

}




// ==== Menggerakan Arm Robot ke Positive Direction ====
void rotateCW (int dirPin, int stepPin) {
  digitalWrite(dirPin, HIGH); // putar searah jarum jam
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(1000); // ganti delay untuk mempercepat motor
  digitalWrite(stepPin, LOW);
  delayMicroseconds(1000); // ganti delay untuk mempercepat motor
}

// ==== Menggerakan Arm Robot ke Negative Direction ====
void rotateCCW (int dirPin, int stepPin) {
  digitalWrite(dirPin, LOW); // putar berlawanan searah jarum jam
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(1000); // ganti delay untuk mempercepat motor
  digitalWrite(stepPin, LOW);
  delayMicroseconds(1000); // ganti delay untuk mempercepat motor
}


// ==== Fungsi untuk menggenerate sinyal yang PRESISI ke stepper ====
void rotateStepper (int gerak, int currentState) {
  // Pergerakan J1
  if (gerak == 1) {
    if (currentState != lastStateJ1) {
      if (currentState > lastStateJ1) {
        int val = 0;
        int i;

        val = currentState - lastStateJ1;

        for (i = 0; i < val; i++) {
          counter1++;
          // Putar motor searah jarum jam sesuai dengan step yang diberikan
          rotateCW(dirPinJ1, stepPinJ1);
        }
      }
      else if (currentState < lastStateJ1) {
        int val = 0;
        int i;
        val = lastStateJ1 - currentState;
        for (i = 0; i < val; i++) {
          counter1--;
          rotateCCW(dirPinJ1, stepPinJ1);
        }
      }
    }
    lastStateJ1 = currentState;
  }


  // Pergerakan J2
  if (gerak == 2) {
    if (currentState != lastStateJ2) {
      if (currentState > lastStateJ2) {
        int val = 0;
        int i;

        val = currentState - lastStateJ2;

        for (i = 0; i < val; i++) {
          counter2++;
          // Putar motor searah jarum jam sesuai dengan step yang diberikan
          rotateCW(dirPinJ2, stepPinJ2);
        }
      }

      else if (currentState < lastStateJ2) {
        int val = 0;
        int i;
        val = lastStateJ2 - currentState;
        for (i = 0; i < val; i++) {
          counter2--;
          rotateCCW(dirPinJ2, stepPinJ2);
        }
      }
    }
    lastStateJ2 = currentState;
  }


  // Pergerakan J3
  if (gerak == 3) {
    if (currentState != lastStateJ3) {
      if (currentState > lastStateJ3) {
        int val = 0;
        int i;

        val = currentState - lastStateJ3;

        for (i = 0; i < val; i++) {
          counter3++;
          // Putar motor searah jarum jam sesuai dengan step yang diberikan
          rotateCW(dirPinJ3, stepPinJ3);
        }
      }

      else if (currentState < lastStateJ3) {
        int val = 0;
        int i;
        val = lastStateJ3 - currentState;
        for (i = 0; i < val; i++) {
          counter3--;
          rotateCCW(dirPinJ3, stepPinJ3);
        }
      }
    }
    lastStateJ3 = currentState;
  }

  // Pergerakan J4
  if (gerak == 4) {
    if (currentState != lastStateJ4) {
      if (currentState > lastStateJ4) {
        int val = 0;
        int i;

        val = currentState - lastStateJ4;

        for (i = 0; i < val; i++) {
          counter4++;
          // Putar motor searah jarum jam sesuai dengan step yang diberikan
          rotateCW(dirPinJ4, stepPinJ4);
        }
      }

      else if (currentState < lastStateJ4) {
        int val = 0;
        int i;
        val = lastStateJ4 - currentState;
        for (i = 0; i < val; i++) {
          counter4--;
          rotateCCW(dirPinJ4, stepPinJ4);
        }
      }
    }
    lastStateJ4 = currentState;
  }

  // Pergerakan J5
  if (gerak == 5) {
    if (currentState != lastStateJ5) {
      if (currentState > lastStateJ5) {
        int val = 0;
        int i;

        val = currentState - lastStateJ5;

        for (i = 0; i < val; i++) {
          counter5++;
          // Putar motor searah jarum jam sesuai dengan step yang diberikan
          rotateCW(dirPinJ5, stepPinJ5);
        }
      }

      else if (currentState < lastStateJ5) {
        int val = 0;
        int i;
        val = lastStateJ5 - currentState;
        for (i = 0; i < val; i++) {
          counter5--;
          rotateCCW(dirPinJ5, stepPinJ5);
        }
      }
    }
    lastStateJ5 = currentState;
  }

  // Pergerakan J6
  if (gerak == 6) {
    if (currentState != lastStateJ6) {
      if (currentState > lastStateJ6) {
        int val = 0;
        int i;

        val = currentState - lastStateJ6;

        for (i = 0; i < val; i++) {
          counter6++;
          // Putar motor searah jarum jam sesuai dengan step yang diberikan
          rotateCW(dirPinJ6, stepPinJ6);
        }
      }

      else if (currentState < lastStateJ6) {
        int val = 0;
        int i;
        val = lastStateJ6 - currentState;
        for (i = 0; i < val; i++) {
          counter6--;
          rotateCCW(dirPinJ6, stepPinJ6);
        }
      }
    }
    lastStateJ6 = currentState;
  }
}

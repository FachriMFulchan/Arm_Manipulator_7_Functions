/* V15
   Ini adalah program full joystick (forward kinematics)
   Program kartesian di comment semua agar tidak memberatkan (kembali ke V11 untuk melihat mode kartesian)

  UMUM
  Saklar toggle untuk 2 mode (0=netral 1=mode joystick 2=mode kartesian)
  2 Push button start dan freeze
  1 push button untuk home
  3 push button untuk low, mid, high

  MODE NEUTRAL
  Untuk indikator LOW in semua aja
  Persiapan konversi data Joystick ke kartesian dan sebaliknya

  MODE JOYSTICK
  Baud Rate = 115200
  Pengiriman data pada setiap 400ms (millis) -> Kalau bisa cepetin lagi
  4 Potensiometer (Smooth = ground dipisah)
  2 Push button gripper
  2 sinyal analog joystick (yaw dan roll)
  Data berurutan: J1, J2, J3, J4, yawJ5, rollJ6, GripCount, Fold, ModeKecepatan

  MODE KARTESIAN
  Pengiriman data setiap 400 ms (millis)
  2 push button X+X-
  2 push button Y+Y-
  2 push button Z+Z-
  2 push button Yaw+Yaw-
  2 push button Pitch+ Pitch-
  2 push button Roll+ Roll-
  2 push button untuk grip on dan grip off
  Data berurutan : CounterX, CounterY, CounterZ, CounterYaw, CounterPitch, CounterYaw, GripCount
  ### Notes : On Developing, jadi di comment semua

  Progress :
  - TAMBAH LCD untuk 2 mode
  - Home positioning untuk mode kartesian
  - Konversi ke derajat di mode joystick
  - Perbaikan ke function Kartesian value menjadi lebih singkat
  - Parameter pembatasan udah sama rehan diubah dan homepos nya jadi mantap
  - Penambahan fitur Fold to Home dan sebaliknya
  - Display jadi derajat, namun yang dikirim adalah konversi step nya
  - Mode kartesian di sisihkan (di comment)
  - HTM Matriks untuk Forward Kinematics ditambahkan
  - Penambahan fitur kecepatan low, mid, high pada mode Joystick
  - Pergerakan roll yang infinite
*/

/* ==== Libray LCD ==== */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd (0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Set the LCD I2C address


/* -------------- MODE JOYSTICK --------------- */
/* ==== ANALOG ==== */
// POTENSIOMETER
#define pinPot1 0
#define pinPot2 1
#define pinPot3 2
#define pinPot4 3
int potensio1 = 0;
int potensio2 = 0;
int potensio3 = 0;
int potensio4 = 0;

// JOYSTICK YAW & ROLL
#define pinJoyYaw 5
#define pinJoyRoll 4
int joyYaw = 0;
int joyRoll = 0;


/* ==== DIGITAL ==== */
// BUTTON GRIPPER
#define pinButtonGripper 22
#define pinButtonRelease 23
int gripCount = 0;

// State potensiometer
int lastStatePot1 = 0;
int lastStatePot2 = 0;
int lastStatePot3 = 0;
int lastStatePot4 = 0;
int currentStatePot1 = 0;
int currentStatePot2 = 0;
int currentStatePot3 = 0;
int currentStatePot4 = 0;

// Variabel Pergerakan Stepper Oleh Potensio (sebenarnya nanti hasilnya degree)
int steppJ1 = 0;
int steppJ2 = 0;
int steppJ3 = 0;
int steppJ4 = 0;
int yawJ5 = 0;
int rollJ6 = 0;

// Batas Max dan Min Mode Joystick (dalam derajat)
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


/* -------------- MODE Kartesian --------------- */
//// Push Button Kartesian
//#define pinButtonXPlus 30
//#define pinButtonXMin 31
//#define pinButtonYPlus 32
//#define pinButtonYMin 33
//#define pinButtonZPlus 34
//#define pinButtonZMin 35
//#define pinButtonYawPlus 36
//#define pinButtonYawMin 37
//#define pinButtonPitchPlus 38
//#define pinButtonPitchMin 39
//#define pinButtonRollPlus 40
//#define pinButtonRollMin 41
//#define pinButtonGripperKartesian 42
//#define pinButtonReleaseKartesian 43
//
//// Variable untuk menampung nilai push button
//bool buttonXPlusVal = 0;
//bool buttonXMinVal = 0;
//bool buttonYPlusVal = 0;
//bool buttonYMinVal = 0;
//bool buttonZPlusVal = 0;
//bool buttonZMinVal = 0;
//bool buttonYawPlusVal = 0;
//bool buttonYawMinVal = 0;
//bool buttonPitchPlusVal = 0;
//bool buttonPitchMinVal = 0;
//bool buttonRollPlusVal = 0;
//bool buttonRollMinVal = 0;
//
//// Counter nilai untuk masukan stepper
//int counterX = 0;
//int counterY = 0;
//int counterZ = 0;
//int counterYaw = 0;
//int counterPitch = 0;
//int counterRoll = 0;
//
//// Batas Max dan Min Mode Kartesian
//int batasXPlus = 200;
//int batasXMin = 0;
//int batasYPlus = 300;
//int batasYMin = -100;
//int batasZPlus = 200;
//int batasZMin = 0;
//int batasYawPlus = 200;
//int batasYawMin = 0;
//int batasPitchPlus = 200;
//int batasPitchMin = 0;
//int batasRollPlus = 200;
//int batasRollMin = 0;



/* -------------- UMUM --------------- */
// ==== BUTTTON START & FREEZE
#define pinStart 24
#define pinFreeze 25
bool mode = 0;

// ==== BUTTON HOME
#define pinButtonHome 29
int buttonHomeVal = 0;

// ==== TOGGLE
#define pinToggleLeft 2
#define pinToggleRight 3
byte modePergerakan = 0;

// ==== Variable pengkodean mode
int kode1 = 1; // Pergerakan joystick
int kode2 = 2; // Pergerakan Kartesian

// ==== BUTTON LOW, MID, HIGH
#define pinButtonLow 26
#define pinButtonMid 27
#define pinButtonHigh 28
byte modeScale = 0;

// ==== LED indicator
#define pinSendLED 12
#define pinStartLED 11
#define pinFreezeLED 10
#define pinScaleLowLED 9
#define pinScaleMidLED 8
#define pinScaleHighLED 7
#define pinHomeLED 6

// ==== Variable waktu untuk macam macam kebutuhan
unsigned long timeBefore = 0;  // buat komdat
unsigned long previousMillisRoll = 0; // buat roll mode joystick
unsigned long previousMillisYaw = 0; // buat yaw mode joystick
unsigned long previousMillisButton = 0; // buat buttons di mode kartesian
unsigned long previousMillisLCD = 0; // buat delay LCD
unsigned long previousMillisFrame = 0; // buat ganti frame ke FK di mode joystick
int interval = 200; // interval refresh LCD

// ==== Folding & frame parameter
unsigned long previousMillisFold = 0; // buat waktu folding
int fold = 0; // 0 ==  Folding mode, 1 == Home mode
int tanda = 0; // parameter bantu untuk folding
int frame = 0; // Buat ganti frame LCD di mode joystick


//  ==== Arm Robot Parameter
// Panjang link
float r1 = 4.70; //length of link r1 in cm
float r2 = 11.00; //length of link r2 in cm
float r3 = 2.60; //length of link r3 in cm
float d1 = 13.30; //length of link d1 in cm
float d4 = 11.75; //length of link d4 in cm
float d6 = 2.80; //length of link d6 in cm

float H0_2[4][4], H2_4[4][4], H4_6[4][4];
float H0_4[4][4], H0_6[4][4];



void setup() {
  // Serial Communication
  Serial.begin (115200);
  Serial3.begin (115200);

  // Buttons Joystick
  pinMode (pinButtonGripper, INPUT);
  pinMode (pinButtonRelease, INPUT);

  // Umum
  pinMode (pinStart, INPUT);
  pinMode (pinFreeze, INPUT);
  pinMode (pinButtonLow, INPUT);
  pinMode (pinButtonMid, INPUT);
  pinMode (pinButtonHigh, INPUT);
  pinMode (pinButtonHome, INPUT);
  pinMode (pinToggleLeft, INPUT);
  pinMode (pinToggleRight, INPUT);

  // Buttons Kartesians
  //  pinMode(pinButtonXPlus, INPUT);
  //  pinMode(pinButtonXMin, INPUT);
  //  pinMode(pinButtonYPlus, INPUT);
  //  pinMode(pinButtonYMin, INPUT);
  //  pinMode(pinButtonZPlus, INPUT);
  //  pinMode(pinButtonZMin, INPUT);
  //  pinMode(pinButtonYawPlus, INPUT);
  //  pinMode(pinButtonYawMin, INPUT);
  //  pinMode(pinButtonPitchPlus, INPUT);
  //  pinMode(pinButtonPitchMin, INPUT);
  //  pinMode(pinButtonRollPlus, INPUT);
  //  pinMode(pinButtonRollMin, INPUT);
  //  pinMode (pinButtonGripperKartesian, INPUT);
  //  pinMode (pinButtonReleaseKartesian, INPUT);

  // LED Indicators
  pinMode (pinSendLED, OUTPUT);
  pinMode (pinStartLED, OUTPUT);
  pinMode (pinFreezeLED, OUTPUT);
  pinMode (pinScaleLowLED, OUTPUT);
  pinMode (pinScaleMidLED, OUTPUT);
  pinMode (pinScaleHighLED, OUTPUT);
  pinMode (pinHomeLED, OUTPUT);

  // Generate last state dari potensiometer
  lastStatePot1 = generateStatePot(pinPot1, minAngleJ1, maxAngleJ1);
  lastStatePot2 = generateStatePot(pinPot2, minAngleJ2, maxAngleJ2);
  lastStatePot3 = generateStatePot(pinPot3, minAngleJ3, maxAngleJ3);
  lastStatePot4 = generateStatePot(pinPot4, minAngleJ4, maxAngleJ4);

  // LCD Initiation
  lcd.begin (20, 4);
  lcd.clear();

}

void loop() {
  // Timer millis komdat
  unsigned long timeAfter = millis();

  // Read Saklar Toggle
  modePergerakan = determineMovementMode(pinToggleLeft, pinToggleRight);


  /* ===== NEUTRAL MODE ====== */
  if (modePergerakan == 0) {
    // Pengenolan state
    mode = 0;   // mode Start atau Stop
    modeScale = 0; // mode Scale Low, Mid, High

    // LCD Neutral
    LCDNeutral();

    digitalWrite (pinStartLED, LOW);
    digitalWrite (pinFreezeLED, LOW);
    digitalWrite (pinScaleLowLED, LOW);
    digitalWrite (pinScaleMidLED, LOW);
    digitalWrite (pinScaleHighLED, LOW);
    Serial.println ("Neutral Mode");
  }


  /* ====== JOYSTICK MODE ======= */
  else if (modePergerakan == 1) {

    // Ketika di posisi FOLD
    if (fold == 0) {
      // Determine Fold or Home Position
      fold = determineFoldOrHome (pinButtonHome, fold);
      LCDFolding();

      // Pengenolan
      startFreezeIndicator (0); // Red LED is On
      modeScale = 0;
      digitalWrite (pinScaleLowLED, LOW);
      digitalWrite (pinScaleMidLED, LOW);
      digitalWrite (pinScaleHighLED, LOW);


      // Send Data to Slave (Untuk pengiriman data fold)
      if (timeAfter - timeBefore >= 700) {
        //  Assemble data dan kirim
        String out = "#" + String(kode1) + "#" + String(steppJ1) + "#" + String(steppJ2) + "#" + String(steppJ3) + "#" + String(steppJ4) + "#" + String(yawJ5) + "#" + String(rollJ6) + "#" + String(gripCount) + "#" + String(fold) + "#" + String(modeScale) + "#$";
        Serial.println (out);
        Serial3.print (out);
        timeBefore = millis();
      }
    }

    // Ketika sudah berada di posisi HOME
    else if (fold == 1) {
      // Start Mode or Freeze Mode
      mode = determineModeStartFreeze(pinStart, pinFreeze, mode);
      startFreezeIndicator(mode);

      // Generate current state dari potensio
      currentStatePot1 = generateStatePot(pinPot1, minAngleJ1, maxAngleJ1);
      currentStatePot2 = generateStatePot(pinPot2, minAngleJ2, maxAngleJ2);
      currentStatePot3 = generateStatePot(pinPot3, minAngleJ3, maxAngleJ3);
      currentStatePot4 = generateStatePot(pinPot4, minAngleJ4, maxAngleJ4);;

      // Read Joystick Yaw & Roll
      joyYaw = readJoystick(pinJoyYaw);
      joyRoll = readJoystick(pinJoyRoll);


      // START MODE
      if (mode == 1) {
        // Generate step for stepper (sebenernya diganti jadi generate degree sih)
        steppJ1 = generateStep (1, currentStatePot1, steppJ1);
        steppJ2 = generateStep (2, currentStatePot2, steppJ2);
        steppJ3 = generateStep (3, currentStatePot3, steppJ3);
        steppJ4 = generateStep (4, currentStatePot4, steppJ4);
        yawJ5 = determineYawValue(joyYaw, yawJ5, minAngleJ5, maxAngleJ5);
        rollJ6 = determineRollValue(joyRoll, rollJ6);

        // Read Gripper Value
        gripCount = determineGripperValue(pinButtonGripper, pinButtonRelease, gripCount);

        // Determine scaling mode
        modeScale = determineModeScale(pinButtonLow, pinButtonMid, pinButtonHigh);
        LowMidHighIndicator(modeScale);

        // Home Positioning
        homingPositionJoystick(pinButtonHome);

        // LCD Mode Joystick
        LCDJoystick(); // pake variable global

        // Determine End-Effector Position
        generateHTMWithDH (steppJ1, steppJ2, steppJ3, steppJ4, yawJ5, rollJ6);
        tampilMatriks (H0_6); // buat nampilin Matriks H0-6

        // Send Data to Slave
        if (timeAfter - timeBefore >= 700) {
          //  Assemble data dan kirim
          String out = "#" + String(kode1) + "#" + String(steppJ1) + "#" + String(steppJ2) + "#" + String(steppJ3) + "#" + String(steppJ4) + "#" + String(yawJ5) + "#" + String(rollJ6) + "#" + String(gripCount) + "#" + String(fold) + "#" + String(modeScale) + "#$";
          Serial.println (out);
          Serial3.print (out);
          timeBefore = millis();

          //Indikator pengiriman data (TX)
          SendLEDIndicator();
        }
      }


      // FREEZE MODE
      else if (mode == 0) {
        // pergerakan pot diabaikan dan tidak bisa menggenerate step stepper
        lastStatePot1 = currentStatePot1;
        lastStatePot2 = currentStatePot2;
        lastStatePot3 = currentStatePot3;
        lastStatePot4 = currentStatePot4;

        // Kembali ke Fold Position (ketika semua joint bernilai 0)
        if (steppJ1 == 0 and steppJ2 == 0 and steppJ3 == 0 and steppJ4 == 0 and yawJ5 == 0 and rollJ6 == 0) {
          fold = determineFoldOrHome(pinButtonHome, fold);
        }

        LCDJoystick();
        Serial.println ("Freeze in Joystick mode");

        // Send Data to Slave (Untuk pengiriman data fold)
        if (timeAfter - timeBefore >= 700) {
          //  Assemble data dan kirim
          String out = "#" + String(kode1) + "#" + String(steppJ1) + "#" + String(steppJ2) + "#" + String(steppJ3) + "#" + String(steppJ4) + "#" + String(yawJ5) + "#" + String(rollJ6) + "#" + String(gripCount) + "#" + String(fold) + "#" + String(modeScale) + "#$";
          Serial.println (out);
          Serial3.print (out);
          timeBefore = millis();
        }
      }
    }
  }


  /* ====== KARTESIAN MODE ====== */
  else if (modePergerakan == 2) {
    LCDKartesianDeveloping();

    // ====== ON DEVELOP =====
    //    // Start Mode or Freeze Mode
    //    mode = determineModeStartFreeze(pinStart, pinFreeze, mode);
    //    startFreezeIndicator(mode);
    //
    //    // Start Mode
    //    if (mode == 1) {
    //      // Determine scaling mode
    //      modeScale = determineModeScale(pinButtonLow, pinButtonMid, pinButtonHigh);
    //      LowMidHighIndicator(modeScale);
    //
    //      // Read button State
    //      buttonXPlusVal = digitalRead (pinButtonXPlus);
    //      buttonXMinVal = digitalRead (pinButtonXMin);
    //      buttonYPlusVal = digitalRead (pinButtonYPlus);
    //      buttonYMinVal = digitalRead (pinButtonYMin);
    //      buttonZPlusVal = digitalRead (pinButtonZPlus);
    //      buttonZMinVal = digitalRead (pinButtonZMin);
    //      buttonYawPlusVal = digitalRead (pinButtonYawPlus);
    //      buttonYawMinVal = digitalRead (pinButtonYawMin);
    //      buttonPitchPlusVal = digitalRead (pinButtonPitchPlus);
    //      buttonPitchMinVal = digitalRead (pinButtonPitchMin);
    //      buttonRollPlusVal = digitalRead (pinButtonRollPlus);
    //      buttonRollMinVal = digitalRead (pinButtonRollMin);
    //
    //      // Counter value
    //      counterX = determineButtonValue (buttonXPlusVal, buttonXMinVal, counterX, modeScale, batasXPlus, batasXMin);
    //      counterY = determineButtonValue (buttonYPlusVal, buttonYMinVal, counterY, modeScale, batasYPlus, batasYMin);
    //      counterZ = determineButtonValue (buttonZPlusVal, buttonZMinVal, counterZ, modeScale, batasZPlus, batasZMin);
    //      counterYaw = determineButtonValue (buttonYawPlusVal, buttonYawMinVal, counterYaw, modeScale, batasYawPlus, batasYawMin);
    //      counterPitch = determineButtonValue (buttonPitchPlusVal, buttonPitchMinVal, counterPitch, modeScale, batasPitchPlus, batasPitchMin);
    //      counterRoll = determineButtonValue (buttonRollPlusVal, buttonRollMinVal, counterRoll, modeScale, batasRollPlus, batasRollMin);
    //
    //      // Read Gripper Value
    //      gripCount = determineGripperValue(pinButtonGripperKartesian, pinButtonReleaseKartesian, gripCount);
    //
    //      // Home Positioning
    //      homingPositionKartesian(pinButtonHome);
    //
    //      // LCD Mode Kartesian
    //      LCDKartesian(); // pake variable global
    //
    //
    //      // Send Data to Slave
    //      if (timeAfter - timeBefore >= 700) {
    //        //  Assemble data dan kirim
    //        String out = "#" + String(kode2) + "#" + String(counterX) + "#" + String(counterY) + "#" + String(counterZ) + "#" + String(counterYaw) + "#" + String(counterPitch) + "#" + String(counterRoll) + "#" + String(gripCount) + "#$";
    //        Serial.println (out);
    //        Serial3.print (out);
    //        timeBefore = millis();
    //
    //        //Indikator pengiriman data (TX)
    //        SendLEDIndicator();
    //      }
    //    }
    //
    //    // Freeze Mode (gaboleh ada pergerakan samsek)
    //    else if (mode == 0) {
    //      // Determine scaling mode
    //      modeScale = determineModeScale(pinButtonLow, pinButtonMid, pinButtonHigh);
    //      LowMidHighIndicator(modeScale);
    //
    //      // LCD Mode Kartesian
    //      LCDKartesian(); //pake variable global
    //      Serial.println ("Freeze in Kartesian mode");
    //    }
  }
}



/* FUNCTIONS */

/* --------- FUNCTIONS MODE JOYSTICK ----------------- */
// ==== Determine Fold or Home Position ====
int determineFoldOrHome (int pinA, int fold) {
  unsigned long currentMillisFold = 0;
  bool buttonHome = digitalRead (pinA);

  // ----- FOLD TO HOME
  if (buttonHome == HIGH and fold == LOW and tanda == 0) {
    tanda = 1;
    previousMillisFold = millis();
  }

  else if (buttonHome == HIGH and fold == LOW and tanda == 1) {
    currentMillisFold = millis();
    if (currentMillisFold - previousMillisFold >= 7000) {
      tanda = 0;
      fold = 1;
    }
  }

  else if (buttonHome == LOW and fold == LOW) {
    tanda = 0;
    previousMillisFold = 0;
  }


  // ---- HOME TO FOLD
  if (buttonHome == HIGH and fold == HIGH and tanda == 0) {
    tanda = 1;
    previousMillisFold = millis();
  }

  else if (buttonHome == HIGH and fold == HIGH and tanda == 1) {
    currentMillisFold = millis();
    if (currentMillisFold - previousMillisFold >= 7000) {
      tanda = 0;
      fold = 0;
    }
  }

  else if (buttonHome == LOW and fold == HIGH) {
    tanda = 0;
    previousMillisFold = 0;
  }
  return fold;
}


// ==== Determine Movement mode (0=netral 1=mode joystick 2=mode kartesian) ====
int determineMovementMode (int pinToggleA, int pinToggleB) {
  byte toggleValA = digitalRead (pinToggleA);
  byte toggleValB = digitalRead (pinToggleB);

  if (toggleValA == LOW and toggleValB == LOW) {
    modePergerakan = 0;
  }
  else if (toggleValA == HIGH and toggleValB == LOW) {
    modePergerakan = 1;
  }
  else if (toggleValA == LOW and toggleValB == HIGH) {
    modePergerakan = 2;
  }
  return modePergerakan;
}


// ==== Potensiometer Reading & Smoothing Functions =====
int readPotensiometer(int pinPot) {
  int i;
  int sval = 0;
  int sampel = 16;

  for (i = 0; i < sampel; i++) {
    sval = sval + analogRead(pinPot);
  }

  sval = sval / sampel;  //average (scale 0-1023)
  //  sval = sval / 4;   //scale to 8 bits (0-255)
  return sval;
}

int readPotensiometerInvert (int pinPot) {
  int i;
  int sval = 0;
  int sampel = 16;

  for (i = 0; i < sampel; i++) {
    sval = sval + analogRead(pinPot);
  }

  sval = sval / sampel;  //average (scale 0-1023)
  sval = 1023 - sval;
  return sval;
}

// ==== Mapping Potensio to Joint Degree Range ======
// Mapping nilai potensiometer ke range yang sesuai dengan masing2 joint nya
int generateStatePot (int pinPot, int batasMin, int batasMax) {
  int potensioVal = 0;
  int stateVal = 0;

  // Untuk joint 2 (karena elektriknya terbalik)
  if (pinPot == 1) {
    potensioVal = readPotensiometerInvert(pinPot);
  }
  else {
    potensioVal = readPotensiometer(pinPot);
  }

  stateVal = map (potensioVal, 0, 1023, batasMin, batasMax);
  return stateVal;
}


// ===== Generate Step Value from Potensio =====
int generateStep (int gerak, int currentStatePot, int stepp) {

  // Potensio 1
  if (gerak == 1) {
    if (currentStatePot != lastStatePot1) {
      if (currentStatePot > lastStatePot1) {
        int val = 0;
        int i;
        val = currentStatePot - lastStatePot1;

        // kalo mode start baru jalanin stepper
        for (i = 0; i < val; i++) {
          stepp++;
          stepp = BatasMax (stepp, maxAngleJ1);
        }

      }

      else if (currentStatePot < lastStatePot1) {
        int val = 0;
        int i;
        val = lastStatePot1 - currentStatePot;
        for (i = 0; i < val; i++) {
          stepp--;
          stepp = BatasMin (stepp, minAngleJ1);
        }
      }
    }
    lastStatePot1 = currentStatePot;
  }

  // Potensio 2
  else if (gerak == 2) {
    if (currentStatePot != lastStatePot2) {
      if (currentStatePot > lastStatePot2) {
        int val = 0;
        int i;
        val = currentStatePot - lastStatePot2;

        // kalo mode start baru jalanin stepper
        for (i = 0; i < val; i++) {
          stepp++;
          stepp = BatasMax (stepp, maxAngleJ2);
        }

      }

      else if (currentStatePot < lastStatePot2) {
        int val = 0;
        int i;
        val = lastStatePot2 - currentStatePot;
        for (i = 0; i < val; i++) {
          stepp--;
          stepp = BatasMin (stepp, minAngleJ2);
        }

      }
    }
    lastStatePot2 = currentStatePot;
  }

  // Potensio 3
  else if (gerak == 3) {
    if (currentStatePot != lastStatePot3) {
      if (currentStatePot > lastStatePot3) {
        int val = 0;
        int i;
        val = currentStatePot - lastStatePot3;

        // kalo mode start baru jalanin stepper
        for (i = 0; i < val; i++) {
          stepp++;
          stepp = BatasMax (stepp, maxAngleJ3);
        }

      }

      else if (currentStatePot < lastStatePot3) {
        int val = 0;
        int i;
        val = lastStatePot3 - currentStatePot;
        for (i = 0; i < val; i++) {
          stepp--;
          stepp = BatasMin (stepp, minAngleJ3);
        }

      }
    }
    lastStatePot3 = currentStatePot;
  }

  // Potensio 4
  else if (gerak == 4) {
    if (currentStatePot != lastStatePot4) {
      if (currentStatePot > lastStatePot4) {
        int val = 0;
        int i;
        val = currentStatePot - lastStatePot4;

        // kalo mode start baru jalanin stepper
        for (i = 0; i < val; i++) {
          stepp++;
          stepp = BatasMax (stepp, maxAngleJ4);
        }

      }

      else if (currentStatePot < lastStatePot4) {
        int val = 0;
        int i;
        val = lastStatePot4 - currentStatePot;
        for (i = 0; i < val; i++) {
          stepp--;
          stepp = BatasMin (stepp, minAngleJ4);
        }
      }
    }
    lastStatePot4 = currentStatePot;
  }
  return stepp;
}



// ==== Gripper Value Function =====
int determineGripperValue (int pinButtonGrip, int pinReleaseGrip, int gripCount) {
  bool btnGrip = 0;
  bool btnRelease = 0;
  btnGrip = digitalRead (pinButtonGrip);
  btnRelease = digitalRead (pinReleaseGrip);


  //Determine gripValue (nanti kasih delay)
  if (btnGrip == HIGH && btnRelease == LOW) {
    gripCount++;
    delay (5);
    if (gripCount >= 255) {
      gripCount = 255;
    }
  }

  else if (btnGrip == LOW && btnRelease == HIGH) {
    gripCount--;
    delay (5);
    if (gripCount <= 0) {
      gripCount = 0;
    }
  }
  return gripCount;
}


// ==== Reading Analog Joystick Sensor =====
int readJoystick(int pinJoy) {
  int i;
  int sval = 0;
  int sampel = 16;

  for (i = 0; i < sampel; i++) {
    sval = sval + analogRead(pinJoy);
  }

  sval = sval / sampel;  //average
  //  sval = sval / 4;
  return sval;
}


// ====== Generate Yaw Joystick to Step Value =====
int determineYawValue (int joyVal, int yaw, int minAngle, int maxAngle) {
  unsigned long currentMillis = millis();

  // Pertambahan
  if (joyVal >= 650) {
    if (currentMillis - previousMillisYaw >= 5) {
      yaw ++;
      yaw = BatasMax (yaw, maxAngleJ5);
      previousMillisYaw = currentMillis;
    }
  }

  // Pengurangan
  else if (joyVal <= 400) {
    if (currentMillis - previousMillisYaw >= 5) {
      yaw --;
      yaw = BatasMin (yaw, minAngleJ5);
      previousMillisYaw = currentMillis;
    }
  }

  return yaw;
}



// ====== Generate Roll Joystick to Step Value =====
int determineRollValue (int joyVal, int roll) {
  unsigned long currentMillis = millis();

  // Pertambahan Cepat
  if (joyVal >= 650) {
    if (currentMillis - previousMillisRoll >= 5) {
      roll++;
      previousMillisRoll = currentMillis;
    }
  }

  // Pengurangan Cepat
  else if (joyVal <= 400) {
    if (currentMillis - previousMillisRoll >= 5) {
      roll--;
      previousMillisRoll = currentMillis;
    }
  }
  return roll;
}



/* ----------- FUNCTION UMUM -------------- */
// ==== Mode Start / Freeze Function ====
int determineModeStartFreeze (int pinButtonStart, int pinButtonFreeze, int mode) {
  bool btnStartVal = digitalRead (pinButtonStart);
  bool btnFreezeVal = digitalRead (pinButtonFreeze);

  if (btnStartVal == HIGH && btnFreezeVal == LOW) {
    mode = 1;
  }
  else if (btnStartVal == LOW && btnFreezeVal == HIGH) {
    mode = 0;
  }
  return mode;
}

// ==== Indicator LED Start / Freeze Mode ====
void startFreezeIndicator(bool mode) {
  if (mode == HIGH) {
    digitalWrite (pinStartLED, HIGH);
    digitalWrite (pinFreezeLED, LOW);
  }
  else if (mode == LOW) {
    digitalWrite (pinStartLED, LOW);
    digitalWrite (pinFreezeLED, HIGH);
  }
}

// ==== Determine Mode Low, Mid, High ====
int determineModeScale (int pinButtonScaleLow, int pinButtonScaleMid, int pinButtonScaleHigh) {
  bool buttonScaleLowVal = digitalRead(pinButtonScaleLow);
  bool buttonScaleMidVal = digitalRead(pinButtonScaleMid);
  bool buttonScaleHighVal = digitalRead(pinButtonScaleHigh);

  if (buttonScaleLowVal == HIGH && buttonScaleMidVal == LOW && buttonScaleHighVal == LOW) {
    modeScale = 1;
  }
  else if (buttonScaleLowVal == LOW && buttonScaleMidVal == HIGH && buttonScaleHighVal == LOW) {
    modeScale = 0;
  }
  else if (buttonScaleLowVal == LOW && buttonScaleMidVal == LOW && buttonScaleHighVal == HIGH) {
    modeScale = 2;
  }
  return modeScale;
}

// ==== Indicator LED for Low, Mid, High Scale Mode ====
void LowMidHighIndicator(byte modeScale) {
  if (modeScale == 1) {
    digitalWrite (pinScaleLowLED, HIGH);
    digitalWrite (pinScaleMidLED, LOW);
    digitalWrite (pinScaleHighLED, LOW);
  }
  else if (modeScale == 0) {
    digitalWrite (pinScaleLowLED, LOW);
    digitalWrite (pinScaleMidLED, HIGH);
    digitalWrite (pinScaleHighLED, LOW);
  }
  else if (modeScale == 2) {
    digitalWrite (pinScaleLowLED, LOW);
    digitalWrite (pinScaleMidLED, LOW);
    digitalWrite (pinScaleHighLED, HIGH);
  }
}

// ==== Send Data LED Indicator ====
void SendLEDIndicator() {
  digitalWrite (pinSendLED, HIGH);
  delay (1);
  digitalWrite (pinSendLED, LOW);
}

// ==== homingIndicator ====
void homingIndicator (int buttonVal) {
  if (buttonVal == HIGH) {
    digitalWrite (pinHomeLED, HIGH);
  }
  else if (buttonVal == LOW) {
    digitalWrite (pinHomeLED, LOW);
  }
}


/* ----------- FUNCTION HOMING dan BATAS -------------- */
// ==== Homing Position Joystick Function ====
void homingPositionJoystick (int pinHoming) {
  bool buttonValue = digitalRead (pinHoming);
  homingIndicator (buttonValue);

  if (buttonValue == HIGH) {
    steppJ1 = 0;
    steppJ2 = 0;
    steppJ3 = 0;
    steppJ4 = 0;
    yawJ5 = 0;
    rollJ6 = 0;
  }
}


// ==== Fungsi untuk menentukan batas maksimal degree ====
int BatasMax (int counter, int batas) {
  if (counter >= batas) {
    counter = batas;
  }
  else {
    //pass
  }
  return counter;
}

// ==== Fungsi untuk menentukan batas minimal degree ====
int BatasMin (int counter, int batas) {
  if (counter <= batas) {
    counter = batas;
  }
  else {
    //pass
  }
  return counter;
}





/* --------- FUNCTION INDICATOR LCD ---------- */
// ==== Display LCD Mode Joystick ====
void LCDJoystick () {

  unsigned long currentMillisLCD = millis();

  // Tampilan Sudut Joint
  if (currentMillisLCD - previousMillisLCD >= interval and frame == 0) {
    lcd.clear();
    lcd.setCursor (3, 0);
    lcd.print ("MODE JOYSTICK");

    lcd.setCursor (0, 1);
    lcd.print ("J1:");
    lcd.setCursor (3, 1);
    lcd.print (steppJ1);

    lcd.setCursor (0, 2);
    lcd.print ("J2:");
    lcd.setCursor (3, 2);
    lcd.print (steppJ2);

    lcd.setCursor (0, 3);
    lcd.print ("J3:");
    lcd.setCursor (3, 3);
    lcd.print (steppJ3);

    lcd.setCursor (7, 1);
    lcd.print ("J4:");
    lcd.setCursor (10, 1);
    lcd.print (steppJ4);

    lcd.setCursor (7, 2);
    lcd.print ("J5:");
    lcd.setCursor (10, 2);
    lcd.print (yawJ5);

    lcd.setCursor (7, 3);
    lcd.print ("J6:");
    lcd.setCursor (10, 3);
    lcd.print (rollJ6);

    lcd.setCursor (14, 1);
    lcd.print ("Gr:");
    lcd.setCursor (17, 1);
    lcd.print (gripCount);

    previousMillisLCD = currentMillisLCD;

    if (currentMillisLCD - previousMillisFrame >= 1000) {
      frame = 1;
      previousMillisFrame = currentMillisLCD;
    }
  }

  // Tampilan Rotasi dan Posisi FK
  else if (currentMillisLCD - previousMillisLCD >= 200 and frame == 1) {
    lcd.clear();

    /* Rotasi nanti pake Yaw Pitch Roll aja */
    //    lcd.setCursor (0, 0);
    //    lcd.print ("XX:");
    //    lcd.setCursor (3, 0);
    //    lcd.print (H0_6[0][0], 1);
    //
    //    lcd.setCursor (0, 1);
    //    lcd.print ("XY:");
    //    lcd.setCursor (3, 1);
    //    lcd.print (H0_6[1][0], 1);
    //
    //    lcd.setCursor (0, 2);
    //    lcd.print ("XZ:");
    //    lcd.setCursor (3, 2);
    //    lcd.print (H0_6[2][0], 1);
    //
    //    lcd.setCursor (7, 0);
    //    lcd.print ("YX:");
    //    lcd.setCursor (10, 0);
    //    lcd.print (H0_6[0][1], 1);
    //
    //    lcd.setCursor (7, 1);
    //    lcd.print ("YY:");
    //    lcd.setCursor (10, 1);
    //    lcd.print (H0_6[1][1], 1);
    //
    //    lcd.setCursor (7, 2);
    //    lcd.print ("YZ:");
    //    lcd.setCursor (10, 2);
    //    lcd.print (H0_6[2][1], 1);
    //
    //    lcd.setCursor (14, 0);
    //    lcd.print ("ZX:");
    //    lcd.setCursor (17, 0);
    //    lcd.print (H0_6[0][2], 1);
    //
    //    lcd.setCursor (14, 1);
    //    lcd.print ("ZY:");
    //    lcd.setCursor (17, 1);
    //    lcd.print (H0_6[1][2], 1);
    //
    //    lcd.setCursor (14, 2);
    //    lcd.print ("ZZ:");
    //    lcd.setCursor (17, 2);
    //    lcd.print (H0_6[2][2], 1);


    /* Posisi End-Effector */
    lcd.setCursor (4, 0);
    lcd.print ("END EFF POS");

    lcd.setCursor (0, 1);
    lcd.print ("X:");
    lcd.setCursor (2, 1);
    lcd.print (H0_6[0][3]);

    lcd.setCursor (0, 2);
    lcd.print ("Y:");
    lcd.setCursor (2, 2);
    lcd.print (H0_6[1][3]);

    lcd.setCursor (0, 3);
    lcd.print ("Z:");
    lcd.setCursor (2, 3);
    lcd.print (H0_6[2][3]);



    previousMillisLCD = currentMillisLCD;

    if (currentMillisLCD - previousMillisFrame >= 1000) {
      frame = 0;
      previousMillisFrame = currentMillisLCD;
    }
  }
}



// ==== Display LCD Mode Neutral ====
void LCDNeutral() {
  unsigned long currentMillisLCD = millis();

  if (currentMillisLCD - previousMillisLCD >= interval) {
    lcd.clear();
    lcd.setCursor (2, 0);
    lcd.print ("ARM MANIPULATOR &");

    lcd.setCursor (2, 1);
    lcd.print ("MASTER CONTROLLER");

    lcd.setCursor (4, 2);
    lcd.print ("DEVELOPED BY");

    lcd.setCursor (0, 3);
    lcd.print ("RAIHAN, RISKA, KOPLO");

    previousMillisLCD = currentMillisLCD;
  }
}


// ==== Display LCD Mode Folding ====
void LCDFolding() {
  unsigned long currentMillisLCD = millis();

  if (currentMillisLCD - previousMillisLCD >= interval) {
    lcd.clear();
    lcd.setCursor (2, 0);
    lcd.print ("THE ROBOT IS IN");

    lcd.setCursor (3, 1);
    lcd.print ("FOLD POSITION");

    lcd.setCursor (0, 2);
    lcd.print ("PLEASE CHANGE IT TO");

    lcd.setCursor (3, 3);
    lcd.print ("HOME POSITION");

    previousMillisLCD = currentMillisLCD;
  }
}

// ==== Display LCD Kartesian Developing ====
void LCDKartesianDeveloping() {
  unsigned long currentMillisLCD = millis();

  if (currentMillisLCD - previousMillisLCD >= interval) {
    lcd.clear();

    lcd.setCursor (3, 1);
    lcd.print ("MODE KARTESIAN");

    lcd.setCursor (2, 2);
    lcd.print ("IS ON DEVELOPMENT");



    previousMillisLCD = currentMillisLCD;
  }
}



/* --------- FUNCTION MATRIKS UNTUK FORWARD KINEMATICS ---------- */
// Generate H0_6 menggunakan DH parameter
void generateHTMWithDH (float T1, float T2, float T3, float T4, float T5, float T6) {

  // Degree to radiant
  T1 = (T1 / 180.00) * PI;
  T2 = (T2 / 180.00) * PI;
  T3 = (T3 / 180.00) * PI;
  T4 = (T4 / 180.00) * PI;
  T5 = (T5 / 180.00) * PI;
  T6 = (T6 / 180.00) * PI;

  // Parameter DH
  float PT [6][4] {
    { T1, (-90.00 / 180.00)*PI, r1, d1},
    { ((-90.00 / 180.00)*PI) + T2, 0, r2, 0},
    { T3, (-90.00 / 180.00)*PI, r3, 0},
    { T4, (90.00 / 180.00)*PI, 0, d4},
    { T5, (-90.00 / 180.00)*PI, 0, 0},
    { T6, 0, 0, d6}
  };

  // Generate Homogeneous Transformation Matrix
  int i = 0;
  float H0_1[4][4] = {
    {cos(PT[i][0]), -sin(PT[i][0]) * cos(PT[i][1]), sin(PT[i][0]) * sin(PT[i][1]), PT[i][2] * cos(PT[i][0])},
    {sin(PT[i][0]), cos(PT[i][0]) * cos(PT[i][1]), -cos(PT[i][0]) * sin(PT[i][1]), PT[i][2] * sin(PT[i][0])},
    {0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]},
    {0, 0, 0, 1}
  };

  int j = 1;
  float H1_2[4][4] = {
    {cos(PT[j][0]), -sin(PT[j][0]) * cos(PT[j][1]), sin(PT[j][0]) * sin(PT[j][1]), PT[j][2] * cos(PT[j][0])},
    {sin(PT[j][0]), cos(PT[j][0]) * cos(PT[j][1]), -cos(PT[j][0]) * sin(PT[j][1]), PT[j][2] * sin(PT[j][0])},
    {0, sin(PT[j][1]), cos(PT[j][1]), PT[j][3]},
    {0, 0, 0, 1}
  };

  int k = 2;
  float H2_3[4][4] = {
    {cos(PT[k][0]), -sin(PT[k][0]) * cos(PT[k][1]), sin(PT[k][0]) * sin(PT[k][1]), PT[k][2] * cos(PT[k][0])},
    {sin(PT[k][0]), cos(PT[k][0]) * cos(PT[k][1]), -cos(PT[k][0]) * sin(PT[k][1]), PT[k][2] * sin(PT[k][0])},
    {0, sin(PT[k][1]), cos(PT[k][1]), PT[k][3]},
    {0, 0, 0, 1}
  };

  int m = 3;
  float H3_4[4][4] = {
    {cos(PT[m][0]), -sin(PT[m][0]) * cos(PT[m][1]), sin(PT[m][0]) * sin(PT[m][1]), PT[m][2] * cos(PT[m][0])},
    {sin(PT[m][0]), cos(PT[m][0]) * cos(PT[m][1]), -cos(PT[m][0]) * sin(PT[m][1]), PT[m][2] * sin(PT[m][0])},
    {0, sin(PT[m][1]), cos(PT[m][1]), PT[m][3]},
    {0, 0, 0, 1}
  };

  int n = 4;
  float H4_5[4][4] = {
    {cos(PT[n][0]), -sin(PT[n][0]) * cos(PT[n][1]), sin(PT[n][0]) * sin(PT[n][1]), PT[n][2] * cos(PT[n][0])},
    {sin(PT[n][0]), cos(PT[n][0]) * cos(PT[n][1]), -cos(PT[n][0]) * sin(PT[n][1]), PT[n][2] * sin(PT[n][0])},
    {0, sin(PT[n][1]), cos(PT[n][1]), PT[n][3]},
    {0, 0, 0, 1}
  };

  int o = 5;
  float H5_6[4][4] = {
    {cos(PT[o][0]), -sin(PT[o][0]) * cos(PT[o][1]), sin(PT[o][0]) * sin(PT[o][1]), PT[o][2] * cos(PT[o][0])},
    {sin(PT[o][0]), cos(PT[o][0]) * cos(PT[o][1]), -cos(PT[o][0]) * sin(PT[o][1]), PT[o][2] * sin(PT[o][0])},
    {0, sin(PT[o][1]), cos(PT[o][1]), PT[o][3]},
    {0, 0, 0, 1}
  };

  dotProductMatriks (1, H0_1, H1_2);
  dotProductMatriks (2, H2_3, H3_4);
  dotProductMatriks (3, H4_5, H5_6);

  dotProductMatriks (4, H0_2, H2_4);
  dotProductMatriks (5, H0_4, H4_6);
}


// Perkalian Matrix 4x4
// format parameter: mark, leftMatrix, rightMatrix
void dotProductMatriks (int mark, float matriksA[4][4], float matriksB[4][4]) {
  int a = 0 ;
  int b = 0 ;
  int c = 0 ;
  float hasil = 0.00;

  // for H0_2 multiplication
  if (mark == 1) {
    while (a < 4) {
      for (b = 0; b < 4; b++) {
        hasil += matriksA [a][b] * matriksB [b][c];
      }
      H0_2[a][c] = hasil;
      hasil = 0.00;

      // ganti kolom
      c++;
      // ganti baris
      if (c == 4) {
        a ++;
        c = 0;
      }
    }
  }

  // for H2_4 multiplication
  else if (mark == 2) {
    while (a < 4) {
      for (b = 0; b < 4; b++) {
        hasil += matriksA [a][b] * matriksB [b][c];
      }
      H2_4[a][c] = hasil;
      hasil = 0.00;

      // ganti kolom
      c++;
      // ganti baris
      if (c == 4) {
        a ++;
        c = 0;
      }
    }
  }

  // for H4_6 multiplication
  else if (mark == 3) {
    while (a < 4) {
      for (b = 0; b < 4; b++) {
        hasil += matriksA [a][b] * matriksB [b][c];
      }
      H4_6[a][c] = hasil;
      hasil = 0.00;

      // ganti kolom
      c++;
      // ganti baris
      if (c == 4) {
        a ++;
        c = 0;
      }
    }
  }

  // for H0_4 multiplication
  else if (mark == 4) {
    while (a < 4) {
      for (b = 0; b < 4; b++) {
        hasil += matriksA [a][b] * matriksB [b][c];
      }
      H0_4[a][c] = hasil;
      hasil = 0.00;

      // ganti kolom
      c++;
      // ganti baris
      if (c == 4) {
        a ++;
        c = 0;
      }
    }
  }

  // for H0_6 multiplication
  else if (mark == 5) {
    while (a < 4) {
      for (b = 0; b < 4; b++) {
        hasil += matriksA [a][b] * matriksB [b][c];
      }
      H0_6[a][c] = hasil;
      hasil = 0.00;

      // ganti kolom
      c++;
      // ganti baris
      if (c == 4) {
        a ++;
        c = 0;
      }
    }
  }

}

// Fungsi untuk menampilkan matriks 4x4
void tampilMatriks (float matriks[4][4]) {
  Serial.println();
  Serial.println("Matriks 4x4");

  // Menampilkan matriks
  int a, b;
  for (a = 0; a < 4; a++) {
    for (b = 0; b < 4; b++) {
      Serial.print (matriks[a][b], 4);
      Serial.print(' ');
    }
    Serial.println();
  }
}


/* --------- FUNCTION MODE KARTESIAN (Yang disisihkan) ---------- */

/* --------- FUNCTIONS MODE KARTESIAN ----------------- */
// ==== Button Kartesian Function ====
// Fungsi untuk menentukan button step value, delay mode scale, dan pembatasan positif  negatif
//int determineButtonValue (int buttonPlus, int buttonMin, int counter, int modeScale, int batasanPlus, int batasanMin) {
//  unsigned long currentMillisButton = millis();
//
//  // Waktu Tunda dari Scale low, mid high
//  int tunda = 0;
//
//  // LOW Mode Scale
//  if (modeScale == 1) {
//    tunda = 300;
//  }
//  // MID Mode Scale
//  else if (modeScale == 0) {
//    tunda = 100;
//  }
//  // HIGH Mode Scale
//  else if (modeScale == 2) {
//    tunda = 20;
//  }
//
//
//  // Pergerakan
//  if (currentMillisButton - previousMillisButton >= tunda) {
//    if (buttonPlus == HIGH and buttonMin == LOW) {
//      counter++;
//      counter = BatasMax(counter, batasanPlus);
//      previousMillisButton = currentMillisButton;
//    }
//    else if (buttonMin == HIGH and buttonPlus == LOW) {
//      counter--;
//      counter = BatasMin (counter, batasanMin);
//      previousMillisButton = currentMillisButton;
//    }
//    else if (buttonPlus == HIGH and buttonMin == HIGH) {
//      counter = counter;
//    }
//  }
//
//  return counter;
//}

// ==== Display LCD Homing Mode Kartesian ====
//void LCDHomingKartesian () {
//  unsigned long currentMillisLCD = millis();
//
//  if (currentMillisLCD - previousMillisLCD >= interval) {
//    lcd.clear();
//    lcd.setCursor (6, 0);
//    lcd.print ("HOMING");
//
//    lcd.setCursor (0, 1);
//    lcd.print ("X:");
//    lcd.setCursor (2, 1);
//    lcd.print (counterX);
//
//    lcd.setCursor (0, 2);
//    lcd.print ("Y:");
//    lcd.setCursor (2, 2);
//    lcd.print (counterY);
//
//    lcd.setCursor (0, 3);
//    lcd.print ("Z:");
//    lcd.setCursor (2, 3);
//    lcd.print (counterZ);
//
//    lcd.setCursor (6, 1);
//    lcd.print ("Yaw:");
//    lcd.setCursor (10, 1);
//    lcd.print (counterYaw);
//
//    lcd.setCursor (6, 2);
//    lcd.print ("Pit:");
//    lcd.setCursor (10, 2);
//    lcd.print (counterPitch);
//
//    lcd.setCursor (6, 3);
//    lcd.print ("Rol:");
//    lcd.setCursor (10, 3);
//    lcd.print (counterRoll);
//
//    lcd.setCursor (14, 1);
//    lcd.print ("Gr:");
//    lcd.setCursor (17, 1);
//    lcd.print (gripCount);
//
//    previousMillisLCD = currentMillisLCD;
//  }
//}


// ==== Display LCD Mode Kartesian ====
//void LCDKartesian () {
//  unsigned long currentMillisLCD = millis();
//
//
//  if (currentMillisLCD - previousMillisLCD >= interval) {
//    lcd.clear();
//    lcd.setCursor (2, 0);
//    lcd.print ("MODE KARTESIAN");
//
//    lcd.setCursor (0, 1);
//    lcd.print ("X:");
//    lcd.setCursor (2, 1);
//    lcd.print (counterX);
//
//    lcd.setCursor (0, 2);
//    lcd.print ("Y:");
//    lcd.setCursor (2, 2);
//    lcd.print (counterY);
//
//    lcd.setCursor (0, 3);
//    lcd.print ("Z:");
//    lcd.setCursor (2, 3);
//    lcd.print (counterZ);
//
//    lcd.setCursor (6, 1);
//    lcd.print ("Yaw:");
//    lcd.setCursor (10, 1);
//    lcd.print (counterYaw);
//
//    lcd.setCursor (6, 2);
//    lcd.print ("Pit:");
//    lcd.setCursor (10, 2);
//    lcd.print (counterPitch);
//
//    lcd.setCursor (6, 3);
//    lcd.print ("Rol:");
//    lcd.setCursor (10, 3);
//    lcd.print (counterRoll);
//
//    lcd.setCursor (14, 1);
//    lcd.print ("Gr:");
//    lcd.setCursor (17, 1);
//    lcd.print (gripCount);
//
//    previousMillisLCD = currentMillisLCD;
//  }
//}

// ==== Homing Position Kartesian Function ====
//void homingPositionKartesian (int pinHoming) {
//  bool buttonValue = 0;
//  bool homing = 0;
//  byte index = 0;
//  buttonValue = digitalRead (pinHoming);
//
//  if (buttonValue == HIGH) {
//    homing = 1;
//  }
//
//  while (homing == 1) {
//    unsigned long timeAfter = millis();
//
//    // Homing Counter X
//    if (index == 0) {
//      if (counterX > 0) {
//        counterX --;
//        //rotateCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterX < 0) {
//        counterX ++;
//        //rotateCCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterX == 0) {
//        index++;
//      }
//    }
//
//    // Homing Counter Y
//    else if (index == 1) {
//      if (counterY > 0) {
//        counterY --;
//        //rotateCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterY < 0) {
//        counterY ++;
//        //rotateCCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterY == 0) {
//        index++;
//      }
//    }
//
//    // Homing Counter Z
//    else if (index == 2) {
//      if (counterZ > 0) {
//        counterZ --;
//        //rotateCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterZ < 0) {
//        counterZ ++;
//        //rotateCCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterZ == 0) {
//        index++;
//      }
//    }
//
//    // Homing Counter Yaw
//    else if (index == 3) {
//      if (counterYaw > 0) {
//        counterYaw --;
//        //rotateCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterYaw < 0) {
//        counterYaw ++;
//        //rotateCCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterYaw == 0) {
//        index++;
//      }
//    }
//
//
//    // Homing Counter Pitch
//    else if (index == 4) {
//      if (counterPitch > 0) {
//        counterPitch --;
//        //rotateCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterPitch < 0) {
//        counterPitch ++;
//        //rotateCCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterPitch == 0) {
//        index++;
//      }
//    }
//
//
//    // Homing Counter Roll
//    else if (index == 5) {
//      if (counterRoll > 0) {
//        counterRoll --;
//        //rotateCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterRoll < 0) {
//        counterRoll ++;
//        //rotateCCW (pinMotor)
//        delay (10);
//      }
//
//      else if (counterRoll == 0) {
//        index++;
//
//        // Keluar dari looping while
//        if (index == 6) {
//          index = 0;
//          homing = 0;
//        }
//      }
//    }
//
//    // LCD and LED Indicator
//    LCDHomingKartesian();
//    digitalWrite (pinHomeLED, HIGH);
//
//
//    // Send Data to Slave
//    if (timeAfter - timeBefore >= 700) {
//      //  Assemble data dan kirim
//      String out = "#" + String(kode2) + "#" + String(counterX) + "#" + String(counterY) + "#" + String(counterZ) + "#" + String(counterYaw) + "#" + String(counterPitch) + "#" + String(counterRoll) + "#" + String(gripCount) + "#$";
//      Serial.println (out);
//      Serial3.print (out);
//      timeBefore = millis();
//
//      //Indikator pengiriman data (TX)
//      SendLEDIndicator();
//    }
//  }
//
//  // Memadamkan LED
//  digitalWrite (pinHomeLED, LOW);
//}

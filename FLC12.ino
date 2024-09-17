#include <IBusBM.h>
#include <Fuzzy.h>
IBusBM IBus;

int otomatis = 14;
int motorRightPin1 = 12;
int motorRightPin2 = 13;

int motorLeftPin1 = 18;
int motorLeftPin2 = 19;
// Fan DC
int relay3Pin = 23;
// Indikator mode manual
int ledKuning = 32;
// Indikator mode otomatis
int ledHijau = 33;

///durasi belok
int turnDuration = 7500;
int motorSpeed;
int motorTurn;

String receivedData;
float errorValue;

// Membuat objek Fuzzy
Fuzzy *fuzzy = new Fuzzy();

// Input membership function untuk error
FuzzySet *Kiri_Tajam = new FuzzySet(-150, -150, -35, -30);
FuzzySet *Kiri = new FuzzySet(-35, -30, -20, -15);
FuzzySet *Lurus = new FuzzySet(-20, -15, 15, 20);
FuzzySet *Kanan = new FuzzySet(15, 20, 30, 35);
FuzzySet *Kanan_Tajam = new FuzzySet(30, 35, 150, 150);

// Input membership function untuk delta_error
FuzzySet *Delta_Kiri_Tajam = new FuzzySet(-150, -150, -35, -30);
FuzzySet *Delta_Kiri = new FuzzySet(-35, -30, -20, -15);
FuzzySet *Delta_Lurus = new FuzzySet(-20, -15, 15, 20);
FuzzySet *Delta_Kanan = new FuzzySet(15, 20, 30, 35);
FuzzySet *Delta_Kanan_Tajam = new FuzzySet(30, 35, 150, 150);

// Output membership function (kecepatan motor kanan)
FuzzySet *RightStop = new FuzzySet(0, 5, 15, 20);
FuzzySet *RightPelan = new FuzzySet(15, 20, 30, 35);
FuzzySet *RightSedang = new FuzzySet(30, 35, 45, 50);
FuzzySet *RightCepat = new FuzzySet(45, 50, 60, 65);
FuzzySet *RightMax = new FuzzySet(60, 65, 75, 80);

// Output membership function (kecepatan motor kiri)
FuzzySet *LeftStop = new FuzzySet(0, 5, 15, 20);
FuzzySet *LeftPelan = new FuzzySet(15, 20, 30, 35);
FuzzySet *LeftSedang = new FuzzySet(30, 35, 45, 50);
FuzzySet *LeftCepat = new FuzzySet(45, 50, 60, 65);
FuzzySet *LeftMax = new FuzzySet(60, 65, 75, 80);

float previousError = 0; // Menyimpan nilai error sebelumnya

// Channel FS
int ch_3, ch_1, ch_5, ch_6, ch_7, ch_8;

void addFuzzyRule(int ruleNumber, FuzzySet* input1, FuzzySet* input2, FuzzySet* output1, FuzzySet* output2) {
  FuzzyRuleAntecedent *antecedent = new FuzzyRuleAntecedent();
  antecedent->joinWithAND(input1, input2);

  FuzzyRuleConsequent *consequent = new FuzzyRuleConsequent();
  consequent->addOutput(output1);
  consequent->addOutput(output2);

  FuzzyRule *fuzzyRule = new FuzzyRule(ruleNumber, antecedent, consequent);
  fuzzy->addFuzzyRule(fuzzyRule);
}


void maju(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPin1, abs(leftSpeed));
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, abs(rightSpeed));
  analogWrite(motorRightPin2, 0);
}

void mundur(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, abs(leftSpeed));
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, abs(rightSpeed));
}

void kanan(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPin1, abs(leftSpeed));
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, abs(rightSpeed));
}

void kiri(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, abs(leftSpeed));
  analogWrite(motorRightPin1, abs(rightSpeed));
  analogWrite(motorRightPin2, 0);
}

void kananUJ() {
  analogWrite(motorLeftPin1, 70);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 20);
}

void kiriUJ() {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 20);
  analogWrite(motorRightPin1, 70);
  analogWrite(motorRightPin2, 0);
}

void majuU() {
  analogWrite(motorLeftPin1, 30);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 30);
  analogWrite(motorRightPin2, 0);
}

void stopp() {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 0);
}

void setup() {
  Serial.begin(115200);
  IBus.begin(Serial2, 1);
  pinMode(relay3Pin, OUTPUT);
  pinMode(ledKuning, OUTPUT);
  pinMode(ledHijau, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  digitalWrite(relay3Pin, LOW);
  digitalWrite(ledKuning, LOW);
  digitalWrite(ledHijau, LOW);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 0);
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 0);

  // Menambahkan FuzzyInput untuk error
  FuzzyInput *error = new FuzzyInput(1);
  error->addFuzzySet(Kiri_Tajam);
  error->addFuzzySet(Kiri);
  error->addFuzzySet(Lurus);
  error->addFuzzySet(Kanan);
  error->addFuzzySet(Kanan_Tajam);
  fuzzy->addFuzzyInput(error);

  // Menambahkan FuzzyInput untuk delta_error
  FuzzyInput *delta_error = new FuzzyInput(2);
  delta_error->addFuzzySet(Delta_Kiri_Tajam);
  delta_error->addFuzzySet(Delta_Kiri);
  delta_error->addFuzzySet(Delta_Lurus);
  delta_error->addFuzzySet(Delta_Kanan);
  delta_error->addFuzzySet(Delta_Kanan_Tajam);
  fuzzy->addFuzzyInput(delta_error);

  // Menambahkan FuzzyOutput untuk motor kanan dan kiri
  FuzzyOutput *MotorKiri = new FuzzyOutput(1);
  MotorKiri->addFuzzySet(LeftStop);
  MotorKiri->addFuzzySet(LeftPelan);
  MotorKiri->addFuzzySet(LeftSedang);
  MotorKiri->addFuzzySet(LeftCepat);
  MotorKiri->addFuzzySet(LeftMax);
  fuzzy->addFuzzyOutput(MotorKiri);

  FuzzyOutput *MotorKanan = new FuzzyOutput(2); // Output 1 untuk Motor Kanan
  MotorKanan->addFuzzySet(RightStop);
  MotorKanan->addFuzzySet(RightPelan);
  MotorKanan->addFuzzySet(RightSedang);
  MotorKanan->addFuzzySet(RightCepat);
  MotorKanan->addFuzzySet(RightMax);
  fuzzy->addFuzzyOutput(MotorKanan);

  // Menambahkan aturan fuzzy menggunakan fungsi
  addFuzzyRule(1, Kiri_Tajam, Delta_Kiri_Tajam, RightMax, LeftPelan); //Kiri_Tajam
  addFuzzyRule(2, Kanan_Tajam, Delta_Kanan_Tajam, RightPelan, LeftMax); //Kanan_Tajam
  //////////////// Lurus
  addFuzzyRule(3, Kiri, Delta_Kanan_Tajam, RightSedang, LeftPelan);
  addFuzzyRule(4, Lurus, Delta_Kiri, RightSedang, LeftSedang);
  addFuzzyRule(5, Lurus, Delta_Lurus, RightSedang, LeftSedang);
  addFuzzyRule(6, Lurus, Delta_Kanan, RightSedang, LeftSedang);
  addFuzzyRule(7, Kanan, Delta_Kiri_Tajam, RightSedang, LeftSedang);
  //////////////// Kiri
  addFuzzyRule(8, Kiri_Tajam, Delta_Kiri, RightMax, LeftPelan); //Belok Kiri Tajam
  addFuzzyRule(9, Kiri_Tajam, Delta_Lurus, RightMax, LeftPelan); //Belok Kiri Tajam
  addFuzzyRule(10, Kiri_Tajam, Delta_Kanan, RightCepat, LeftStop);
  addFuzzyRule(11, Kiri_Tajam, Delta_Kanan_Tajam, RightCepat, LeftStop);
  addFuzzyRule(12, Kiri, Delta_Kiri_Tajam, RightCepat, LeftStop);
  addFuzzyRule(13, Kiri, Delta_Kiri, RightCepat, LeftStop);
  addFuzzyRule(14, Kiri, Delta_Lurus, RightCepat, LeftStop);
  addFuzzyRule(15, Kiri, Delta_Kanan_Tajam, RightCepat, LeftStop);
  addFuzzyRule(16, Lurus, Delta_Kiri_Tajam, RightCepat, LeftStop);
  //////////////// Kanan
  addFuzzyRule(17, Lurus, Delta_Kanan_Tajam, RightStop, LeftCepat);
  addFuzzyRule(18, Kanan, Delta_Kiri, RightStop, LeftCepat);
  addFuzzyRule(19, Kanan, Delta_Lurus, RightStop, LeftCepat);
  addFuzzyRule(20, Kanan, Delta_Kanan, RightStop, LeftCepat);
  addFuzzyRule(21, Kanan, Delta_Kanan_Tajam, RightStop, LeftCepat);
  addFuzzyRule(22, Kanan_Tajam, Delta_Kiri_Tajam, RightStop, LeftCepat);
  addFuzzyRule(23, Kanan_Tajam, Delta_Kiri, RightStop, LeftCepat);
  addFuzzyRule(24, Kanan_Tajam, Delta_Lurus, RightPelan, LeftMax); //belok kanan tajam
  addFuzzyRule(25, Kanan_Tajam, Delta_Kanan, RightPelan, LeftMax); //belok kanan tajam
}

void loop() {
  // Maju mundur
  ch_3 = IBus.readChannel(2);

  // Belok kanan kiri
  ch_1 = IBus.readChannel(0);

  // Auto manual
  ch_5 = IBus.readChannel(4);

  // Fan DC
  ch_6 = IBus.readChannel(5);

  // Pompa 1
  ch_7 = IBus.readChannel(6);

  // Pompa 2
  ch_8 = IBus.readChannel(7);

  motorSpeed = map(ch_3, 1000, 2000, -80, 80);
  motorTurn = map(ch_1, 1000, 2000, -80, 80);

  
  if (ch_5 < 1500) {
    digitalWrite(otomatis, HIGH);
    digitalWrite(ledKuning, LOW);
    digitalWrite(ledHijau, HIGH);
    ch_1 = 1500;
    ch_3 = 1500;
    ch_5 = 1500;
    ch_6 = 1500;
    ch_7 = 1500;
    ch_8 = 1500;

    if (Serial.available() > 0) {
        receivedData = Serial.readStringUntil('\n');
        
        if (receivedData.startsWith("error_")) {
          String errorString = receivedData.substring(6); // Mengambil substring setelah "error_"
          float currentError = errorString.toFloat(); // Mengkonversi substring ke nilai float
          float deltaError = currentError - previousError; // Menghitung delta error dari error aktual - error sebelumnya

          fuzzy->setInput(1, currentError); // Mengatur input fuzzy untuk error
          fuzzy->setInput(2, deltaError); // Mengatur input fuzzy untuk delta_error

          fuzzy->fuzzify(); // Melakukan fuzzifikasi

          // Mendefuzzifikasi output
          float output1 = fuzzy->defuzzify(1);
          float output2 = fuzzy->defuzzify(2);

          // Mengatur kecepatan motor berdasarkan hasil defuzzifikasi dan aturan fuzzy
          int pwmValueKanan = 0;
          int pwmValueKiri = 0;

         if (output1 >= 15 && output1 <= 35 && output2 >= 60) { // Belok Kiri Tajam
            pwmValueKiri = 25; //Left Pelan
            pwmValueKanan = 70; //Right Max
            analogWrite(motorLeftPin1, 0);
            analogWrite(motorLeftPin2, abs(pwmValueKiri));
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 0 && output1 <= 20 && output2 >= 45 && output2 <= 65) { // Belok Kiri
            pwmValueKiri = 10; // Left Stop
            pwmValueKanan = 55; // Right Cepat
            analogWrite(motorLeftPin1, 0);
            analogWrite(motorLeftPin2, abs(pwmValueKiri));
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 30 && output1 <= 50 && output2 >= 30 && output2 <= 50) { // Maju Lurus
            pwmValueKiri = 40; //Left Sedang
            pwmValueKanan = 40; // Right Sedang
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 45 && output1 <= 65 && output2 >= 0 && output2 <= 20) { // Belok Kanan
            pwmValueKiri = 55; // Left Cepat
            pwmValueKanan = 10; // Right Stop
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, 0);
            analogWrite(motorRightPin2, abs(pwmValueKanan));

          } else if (output1 >= 60 && output2 >= 15 && output2 <= 35) { // Belok Kanan Tajam
            pwmValueKiri = 70; //Left Max
            pwmValueKanan = 25; //Right Pelan
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, 0);
            analogWrite(motorRightPin2, abs(pwmValueKanan));
          }
          previousError = currentError;

          Serial.print("error = ")    ;Serial.print(currentError)    ;Serial.print(" ");
          Serial.print("delta error = ")    ;Serial.print(deltaError)    ;Serial.print(" ");
          Serial.print("output1 = ")    ;Serial.print(output1)    ;Serial.print(" ");
          Serial.print("output2 = ")    ;Serial.print(output2)    ;Serial.print(" ");
          Serial.print("Motor Kiri = ")    ;Serial.print(pwmValueKiri)    ;Serial.print(" ");
          Serial.print("Motor Kanan = ")    ;Serial.print(pwmValueKanan)    ;Serial.println(" ");
        }

        else if(receivedData.startsWith("MAJU")) {
          String command1 = receivedData;
          Serial.print("command: ");
          Serial.println(command1);
          majuU();
        }
        else if (receivedData.startsWith("KIRI")){
          String command2 = receivedData;
          Serial.print("command: ");
          Serial.println(command2);
          kiriUJ();
          delay(turnDuration);
        }
        else if(receivedData.startsWith("KANAN")){
          String command3 = receivedData;
          Serial.print("command: ");
          Serial.println(command3);
          kananUJ();
          delay(turnDuration);
        }
      } 
    }
  else {
    digitalWrite(ledKuning, HIGH);
    digitalWrite(ledHijau, LOW);
    if (motorTurn > 5) {
      kanan(motorTurn, motorTurn);
    } else if (motorTurn < -5) {
      kiri(motorTurn, motorTurn);
    } else if (motorSpeed < 5 && motorSpeed > -5 && motorTurn > -5 && motorTurn < 5) {
      stopp();
    } else if (motorSpeed > 5 && motorTurn > -5 && motorTurn < 5) {
      maju(motorSpeed, motorSpeed);
    } else if (motorSpeed < 5 && motorTurn > -5 && motorTurn < 5) {
      mundur(motorSpeed, motorSpeed);
    }

    if (ch_6 > 1500) {
      digitalWrite(relay3Pin, HIGH);
    } else {
      digitalWrite(relay3Pin, LOW);
    }
  }

  delay(100);
  
}

void manual() {
  if (ch_5 < 1500) {
    stopp();
  }
}

  

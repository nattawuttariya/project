#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>  

#define ON  LOW
#define OFF HIGH

#define WATER_FULL 1
#define WATER_EMPTY 0

#define EEPROM_ADDR 0
#define EEPROM_MAGIC_ADDR 50
#define EEPROM_MAGIC_VALUE 0x42
#define EEPROM_CUSTOM_TIME_ADDR 60
#define EEPROM_USE_CUSTOM_FLAG_ADDR 70

const int sensor = 2;
const int bt = 7;
const int relaypump = 3;
const int relaygreenlight = 5;
const int relayfrontlight = 6;
const int relayredlight = 4;
const int Floatpin = 8;

const int modeBtn = 9;
const int upBtn = 10;
const int downBtn = 11;
const int saveBtn = 12;


volatile unsigned long lastInterruptTime = 0;

bool isPumping = false;
bool puse = false; 
unsigned long pumpStartTime = 0;
unsigned long pumpDuration = 0;
unsigned long pausedElapsed = 0;

volatile bool ignoreCoin = false;

int timePerCredit = 1000;
int pumpingCredit = 0;

int settingMode = 0;
bool isTiming = false;
unsigned long timingStart = 0;
unsigned long customPumpTime = 1000;
bool useCustomTime = false;

bool lastBtState = HIGH;

float volumePerCredit_ml = 0;
float pricePerCredit = 5.0;
float pricePerMl = 0;

volatile bool coinDetected = false;
volatile int lastCoinValue = 1;

// 13 (RX), A0 (TX)
SoftwareSerial espSerial(13, A0); 

String amount = "";
String timeStr = "";
int coin = 0;
volatile bool sendCoinFlag = false;
volatile int coinCount = 0;

void coinISR() {
  unsigned long now = millis();
  if (ignoreCoin) return;
  if (now - lastInterruptTime > 60) {
    if (digitalRead(Floatpin) == WATER_FULL) {
      lastCoinValue = 1;
      coinDetected = true;
      sendCoinFlag = true;
    }
    lastInterruptTime = now;
  }
}

#include <LiquidCrystal_I2C.h>

// กำหนด address LCD และขนาด 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long pumpStopTime = 0;
bool isWaitingAfterPump = false;


void setup() {
  pinMode(sensor, INPUT_PULLUP);
  pinMode(bt, INPUT_PULLUP);
  pinMode(Floatpin, INPUT_PULLUP);

  pinMode(relaypump, OUTPUT);
  pinMode(relaygreenlight, OUTPUT);
  pinMode(relayfrontlight, OUTPUT);
  pinMode(relayredlight, OUTPUT);

  pinMode(modeBtn, INPUT_PULLUP);
  pinMode(upBtn, INPUT_PULLUP);
  pinMode(downBtn, INPUT_PULLUP);
  pinMode(saveBtn, INPUT_PULLUP);

  digitalWrite(relaypump, OFF);
  digitalWrite(relayfrontlight, OFF);

  attachInterrupt(digitalPinToInterrupt(sensor), coinISR, FALLING);

  Serial.begin(115200);
  espSerial.begin(9600);
  Serial.println(F("เริ่มต้นระบบ"));

  settingMode = 0;
  handleEEPROM();

  // เริ่มต้นจอ LCD
  lcd.init();
  lcd.backlight();

  // แสดงโหมดเริ่มต้นบนจอ LCD
  showModeOnLCD(settingMode, 0);
}

// ประกาศตัวแปร global (ข้างนอก loop)
int coinCredit = 0;   // เก็บเครดิตจากเหรียญ
int espCredit = 0;    // เก็บเครดิตจาก ESP32
int credit = 0;       // เครดิตรวม = coinCredit + espCredit




bool isPauseTimeoutCheck = false;
unsigned long pauseTimeoutStart = 0;

// ตัวแปร global ด้านบนสุดของไฟล์
String currentPhoneNumber = "";  // เก็บเบอร์โทรจาก ESP32


void loop() {
  static int lastSentCredit = 0;
  static bool refundedCreditSent = false;  // ป้องกันการส่งเครดิตซ้ำ

  checkModeButtons();

  if (espSerial.available()) {
    readDataFromESP32();
  }

  controlLightByTime();

  // โหมดจับเวลา
  if (settingMode == 3) {
    ignoreCoin = true;
    showModeOnLCD(settingMode, customPumpTime);

    if (digitalRead(bt) == LOW) {
      Serial.println("กดปุ่ม bt ตรวจพบ");
      delay(200);
      if (!isTiming) {
        isTiming = true;
        timingStart = millis();
        digitalWrite(relaypump, ON);
        Serial.println(F("เริ่มจับเวลาและเปิดปั๊ม"));
      } else {
        isTiming = false;
        digitalWrite(relaypump, OFF);
        customPumpTime = millis() - timingStart;
        Serial.print(F("หยุดจับเวลา: "));
        Serial.print(customPumpTime / 1000);
        Serial.println(F(" วินาที"));
      }
      delay(100);
    }

    if (digitalRead(saveBtn) == LOW) {
      useCustomTime = true;
      EEPROM.put(EEPROM_USE_CUSTOM_FLAG_ADDR, useCustomTime);
      EEPROM.put(EEPROM_CUSTOM_TIME_ADDR, customPumpTime);
      Serial.println(F("บันทึกเวลาที่จับได้แล้ว"));
      delay(1000);
    }
    return;
  }

  // โหมดปรับเวลา
  if (settingMode == 2) {
    ignoreCoin = true;
    showModeOnLCD(settingMode, 0);

    if (digitalRead(upBtn) == LOW) {
      timePerCredit += 100;
      if (timePerCredit > 10000) timePerCredit = 10000;
      Serial.print(F("ปรับเวลา: "));
      Serial.println(timePerCredit);
      delay(100);
    }

    if (digitalRead(downBtn) == LOW) {
      timePerCredit -= 100;
      if (timePerCredit < 100) timePerCredit = 100;
      Serial.print(F("ปรับเวลา: "));
      Serial.println(timePerCredit);
      delay(100);
    }

    if (digitalRead(saveBtn) == LOW) {
      EEPROM.put(EEPROM_ADDR, timePerCredit);
      EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
      EEPROM.put(EEPROM_USE_CUSTOM_FLAG_ADDR, false);
      EEPROM.put(EEPROM_CUSTOM_TIME_ADDR, (unsigned long)0);
      Serial.println(F("บันทึกค่าเวลาแล้ว"));
      delay(1000);
    }
    return;
  }

  ignoreCoin = false;
  showModeOnLCD(settingMode, 0);

  int floatState = digitalRead(Floatpin);
  if (!handleWaterLevel(floatState)) return;

  bool currentBt = digitalRead(bt);
  if (lastBtState == HIGH && currentBt == LOW && isPumping) {
    togglePausePump();
    if (puse) {
      pauseTimeoutStart = millis();
      isPauseTimeoutCheck = true;
    } else {
      isPauseTimeoutCheck = false;
    }
  }
  lastBtState = currentBt;

  if (currentBt == LOW && !isPumping && !puse) {
    if (settingMode == 1 && credit >= 1) {
      startPumpingoffline();
      refundedCreditSent = false;  // reset เมื่อเริ่มใหม่
    } else if (settingMode == 0 && (coinCredit + espCredit) >= 1) {
      credit = coinCredit + espCredit;
      startPumpingonline();
      refundedCreditSent = false;  // reset เมื่อเริ่มใหม่
    }
  }

  if (isPumping && !puse) {
    unsigned long elapsed = millis() - pumpStartTime;
    int totalSeconds = pumpDuration / 1000;
    int remaining = totalSeconds - (elapsed / 1000);
    static int lastShown = -1;

    if (remaining != lastShown && remaining >= 0) {
      Serial.print(F("เหลือ: "));
      Serial.print(remaining);
      Serial.println(F(" วินาที"));
      lastShown = remaining;
    }

    if (elapsed >= pumpDuration) {
      stopPumping();
      lastShown = -1;
      pumpStopTime = millis();
      isWaitingAfterPump = true;

      if (settingMode == 1 && credit > 0) {
        credit--;
      } else if (settingMode == 0) {
        if (coinCredit > 0) {
          coinCredit--;                                                                                                                                                                                                                                                         

        } else if (espCredit > 0) {
          espCredit--;
        }
        sendCoinFlag = true;
      }
    }
  }

  // ตรวจจับเหรียญ
  noInterrupts();
  bool coinFlag = coinDetected;
  coinDetected = false;
  int coinValue = lastCoinValue;
  interrupts();

  if ((settingMode == 0 || settingMode == 1) && coinFlag) {
    if (isWaitingAfterPump && (millis() - pumpStopTime < 5000)) {
      Serial.println(F("เหรียญเด้ง"));
    } else {
      if (settingMode == 1) {
        credit += coinValue;
        Serial.print("รับเหรียญ (offline): ");
        Serial.println(credit);
      } else {
        coinCredit += coinValue;
        sendCoinFlag = true;
        Serial.print("รับเหรียญ (online): ");
        Serial.println(coinCredit);
      }
      isWaitingAfterPump = false;
    }
  }

  // ส่งเครดิตไป ESP32
  noInterrupts();
  bool sendFlag = sendCoinFlag;
  sendCoinFlag = false;
  interrupts();

  if (settingMode == 0 && sendFlag) {
    credit = coinCredit + espCredit;
    if (credit > 0 && credit != lastSentCredit) {
      Serial.print("ส่งเครดิตไป ESP32: ");
      Serial.println(credit);

      espSerial.println("1");
      espSerial.print("credit:");
      espSerial.println(credit);
      delay(10);

      lastSentCredit = credit;
      espCredit = 0;
    }
  }

  // เช็ค timeout pause > 15 วิ
  if (isPumping && puse && isPauseTimeoutCheck) {
    unsigned long pauseElapsed = millis() - pauseTimeoutStart;
    if (pauseElapsed >= 15000 && !refundedCreditSent) {
      unsigned long usedTime = pausedElapsed;
      unsigned long totalTime = pumpDuration;
      unsigned long remainingTime = totalTime > usedTime ? totalTime - usedTime : 0;

      int refundedCredit = 0;
      if (useCustomTime && customPumpTime > 0) {
        refundedCredit = remainingTime / customPumpTime;
      } else if (timePerCredit > 0) {
        refundedCredit = remainingTime / timePerCredit;
      }

      if (currentPhoneNumber.length() >= 11) {
        Serial.println(F("User ID ยาวมาก ไม่คืนเครดิต"));
        stopPumping();
        isPauseTimeoutCheck = false;
        refundedCreditSent = true;
      } else if (refundedCredit > 0) {
        Serial.print(F("หมดเวลา pause. คืนเครดิต: "));
        Serial.println(refundedCredit);

        if (settingMode == 0) {
          coinCredit += refundedCredit;
          sendCoinFlag = true;
        } else if (settingMode == 1) {
          credit += refundedCredit;
        }

        if (currentPhoneNumber.length() > 0) {
          float amountNet = (float)refundedCredit;

          espSerial.print("Amount (net): ");
          espSerial.println(amountNet, 2);
          delay(10);

          espSerial.print("User ID: ");
          espSerial.println(currentPhoneNumber);
          delay(10);

          Serial.println(F("ส่งคืนเครดิต:"));
          Serial.print(F("refunCredit: "));
          Serial.println(amountNet, 2);
          Serial.print(F("User ID: "));
          Serial.println(currentPhoneNumber);
        }

        refundedCreditSent = true;
        isPauseTimeoutCheck = false;
        stopPumping();
      }
    }
  }

  delay(30);
}




// ฟังก์ชันอ่านข้อมูลจาก ESP32 รับแค่ 2 บรรทัด: AMOUNT กับ User ID
bool readDataFromESP32() {
  if (!espSerial.available()) return false;

  String line = espSerial.readStringUntil('\n');
  line.trim();

  Serial.println("ข้อมูลที่ได้รับจาก ESP32:");
  Serial.println(line);

  // ตรวจสอบว่าบรรทัดนี้เป็นเวลา HH:MM
  if (line.length() == 5 && line.charAt(2) == ':') {
    Serial.print("เวลาที่ได้รับ: ");
    Serial.println(line);
    timeStr = line;  // เก็บไว้ใช้ controlLightByTime()
    return true;
  }

  // ตรวจสอบ AMOUNT
  if (line.startsWith("AMOUNT:")) {
    String valueStr = line.substring(7);
    valueStr.trim();
    float amountValue = valueStr.toFloat();

    if (amountValue != 0 || valueStr == "0") {
      amount = valueStr;
      coin = (int)amountValue;
      espCredit += coin;
      credit = coinCredit + espCredit;

      Serial.print("เครดิตจาก ESP32 เพิ่ม: ");
      Serial.println(espCredit);
      Serial.print("เครดิตรวม: ");
      Serial.println(credit);
      Serial.println("กรุณากดปุ่มเพื่อเริ่มการจ่ายน้ำยา");
    } else {
      Serial.println(F("รูปแบบข้อมูล AMOUNT ไม่ถูกต้อง"));
    }
    return true;
  }

  // ตรวจสอบ User ID
  if (line.startsWith("User ID:")) {
    currentPhoneNumber = line.substring(8);
    currentPhoneNumber.trim();
    Serial.print("User ID จาก ESP32: ");
    Serial.println(currentPhoneNumber);
    return true;
  }

  // ถ้าไม่ตรงรูปแบบใด ๆ
  Serial.println(F("รูปแบบข้อมูลไม่รู้จัก"));
  return true;
}





void startPumpingonline() {
  if (espCredit <= 0) return;  // ใช้ espCredit เป็นตัวตัดสิน

  isPumping = true;
  puse = false;
  ignoreCoin = true;
  pumpStartTime = millis();
  pausedElapsed = 0;

  pumpingCredit = espCredit;  // ใช้ espCredit เป็นเครดิตที่ใช้จริง

  // คำนวณเวลาตาม espCredit เท่านั้น
  if (useCustomTime) {
    pumpDuration = (unsigned long)customPumpTime * (unsigned long)espCredit;
  } else {
    pumpDuration = (unsigned long)timePerCredit * (unsigned long)espCredit;
  }

  Serial.print(F("เริ่มเปิดปั๊ม "));
  Serial.print(pumpingCredit);
  Serial.print(F(" เครดิต ใช้เวลา "));
  Serial.print(pumpDuration / 1000);
  Serial.println(F(" วินาที"));

  credit = 0;  // รีเซ็ตเครดิตรวม
  coinCredit = 0;  // ถ้าไม่เอา coinCredit มาใช้ ก็เคลียร์ด้วยเพื่อป้องกันค้าง
  espCredit = 0;   // รีเซ็ต espCredit

  digitalWrite(relaypump, ON); // เปิดปั๊ม
}


unsigned long pauseStartTime = 0;  // ประกาศตัวแปรเก็บเวลาที่เริ่มหยุดปั๊ม (global)

void togglePausePump() {
  if (!isPumping) return;

  puse = !puse;

  if (puse) {
    pausedElapsed = millis() - pumpStartTime;
    pauseStartTime = millis();
    pauseTimeoutStart = millis();        // เริ่มจับเวลา timeout
    isPauseTimeoutCheck = true;
    digitalWrite(relaypump, OFF);
    Serial.println(F("หยุดจ่ายน้ำชั่วคราว"));
  } else {
    pumpStartTime = millis() - pausedElapsed;
    isPauseTimeoutCheck = false;         // ยกเลิก timeout ถ้ากลับมาทำงาน
    digitalWrite(relaypump, ON);
    Serial.println(F("เริ่มจ่ายน้ำต่อ"));
  }
}









void showModeOnLCD(int mode, unsigned long saveTime) {
  lcd.clear();
  lcd.setCursor(0, 0);
  
  switch (mode) {
    case 0:  // ออนไลน์
      lcd.print("Online Mode");
      lcd.setCursor(0, 1);
      lcd.print("espCredit :");
      lcd.print(espCredit);
      break;

    case 1:  // ออฟไลน์
      lcd.print("Offline Mode");
      lcd.setCursor(0, 1);
      lcd.print("credit :");
      lcd.print(credit);
      break;

    case 2:  // ปรับเวลา
      lcd.print("Setting Mode 1");
      lcd.setCursor(0, 1);
      lcd.print("No time saved");
      break;

    case 3:  // จับเวลา
      lcd.print("Timer Mode");
      lcd.setCursor(0, 1);
      lcd.print("Time: ");
      lcd.print(saveTime / 1000);
      lcd.print("s");
      break;

    default:
      lcd.print("working");
  }
}





void controlLightByTime() {
  if (timeStr.length() > 0) {
    int colonIndex = timeStr.indexOf(":");
    if (colonIndex > 0) {
      int hour = timeStr.substring(0, colonIndex).toInt();
      int minute = timeStr.substring(colonIndex + 1).toInt();

      int totalMinutes = hour * 60 + minute;

      const int onTime =  22* 60 + 40;  
      const int offTime = 14 * 60 + 21; 

      if (totalMinutes >= onTime && totalMinutes < offTime) {
        digitalWrite(relayfrontlight, ON);
      } else {
        digitalWrite(relayfrontlight, OFF);
      }
    } else {
      digitalWrite(relayfrontlight, OFF); 
    }
  } else {
    digitalWrite(relayfrontlight, OFF);    
  }
}


void handleEEPROM() {
  byte magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic != EEPROM_MAGIC_VALUE) {
    timePerCredit = 1000;
    EEPROM.put(EEPROM_ADDR, timePerCredit);
    EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
    useCustomTime = false;
    EEPROM.put(EEPROM_USE_CUSTOM_FLAG_ADDR, useCustomTime);
    EEPROM.put(EEPROM_CUSTOM_TIME_ADDR, (unsigned long)0);
    Serial.println("EEPROM ถูกกำหนดค่าเริ่มต้นใหม่");
  } else {
    EEPROM.get(EEPROM_ADDR, timePerCredit);
    EEPROM.get(EEPROM_USE_CUSTOM_FLAG_ADDR, useCustomTime);
    EEPROM.get(EEPROM_CUSTOM_TIME_ADDR, customPumpTime);

    if (timePerCredit < 100 || timePerCredit > 10000) {
      timePerCredit = 1000;
      Serial.println("EEPROM ข้อมูลผิดพลาด รีเซ็ตเป็นค่า default");
    }
  }

  // คำนวณปริมาณน้ำต่เครดิต
  if (customPumpTime > 0) {
    volumePerCredit_ml = (100.0 * timePerCredit) / customPumpTime;
    pricePerMl = pricePerCredit / volumePerCredit_ml;
  } else {
    volumePerCredit_ml = 0;
    pricePerMl = 0;
  }

  Serial.print("Time/Credit: ");
  Serial.print(timePerCredit);
  Serial.println(" ms");

  Serial.print("เวลาจับจาก EEPROM: ");
  Serial.print(customPumpTime / 1000);
  Serial.println(" s");

  Serial.print("โหมดการใช้งานล่าสุด: ");
  Serial.println(useCustomTime ? "โหมดจับเวลา (custom)" : "โหมดเครดิต");


}

bool handleWaterLevel(int floatState) {
  if (floatState == WATER_EMPTY) {
    digitalWrite(relaygreenlight, OFF);
    digitalWrite(relayredlight, ON);
    if (isPumping) {
      digitalWrite(relaypump, OFF);
      isPumping = false;
      ignoreCoin = false;
      Serial.println("หยุดปั๊ม: น้ำไม่เต็ม");
    }
    return false;
  }

  digitalWrite(relaygreenlight, ON);
  digitalWrite(relayredlight, OFF);
  return true;
}

void startPumpingoffline() {
  if (credit <= 0) return;

  isPumping = true;
  puse = false;
  ignoreCoin = true;
  pumpStartTime = millis();
  pausedElapsed = 0;
  pumpingCredit = credit;  // ใช้ credit รวม (โหมดออฟไลน์ใช้ตัวเดียว)

  // คำนวณระยะเวลาเปิดปั๊มตามจำนวนเครดิต (ms)
  if (useCustomTime) {
    pumpDuration = (unsigned long)customPumpTime * (unsigned long)credit;
  } else {
    pumpDuration = (unsigned long)timePerCredit * (unsigned long)credit;
  }

  Serial.print(F("เริ่มเปิดปั๊ม "));
  Serial.print(pumpingCredit);
  Serial.print(F(" เครดิต ใช้เวลา "));
  Serial.print(pumpDuration / 1000);
  Serial.println(F(" วินาที"));

  credit = 0; // รีเซ็ตเครดิตหลังเริ่ม

  digitalWrite(relaypump, ON); // เปิดปั๊ม
}







void stopPumping() {
  isPumping = false;
  puse = false;

  digitalWrite(relaypump, OFF);
  Serial.println(F("จ่ายน้ำเสร็จสิ้น"));

  // เคลียร์ flag เพื่อไม่ให้เหรียญเด้งหลอก
  noInterrupts();
  coinDetected = false;
  sendCoinFlag = false;
  interrupts();

  delay(500);  // หน่วงเวลาเล็กน้อยก่อนเปิดรับเหรียญใหม่
  ignoreCoin = false;
}




void checkModeButtons() {
  if (digitalRead(modeBtn) == LOW) {
    delay(300);

    settingMode++;
    if (settingMode > 3) settingMode = 0;  // หมุนวน 0-3

    switch (settingMode) {
      case 0:
        Serial.println(F("โหมด0: ออนไลน์"));
        ignoreCoin = false;
        espSerial.println("online");
        break;
      case 1:
        Serial.println(F("โหมด1: ออฟไลน์"));
        ignoreCoin = false;
        espSerial.println("offline");
        break;
      case 2:
        Serial.println(F("โหมด2: ปรับเวลา"));
   
        ignoreCoin = true;
        break;
      case 3:
        Serial.println(F("โหมด3: จับเวลา"));
        ignoreCoin = true;
        break;
    }

    showModeOnLCD(settingMode, customPumpTime);
    
  }
}


void sendCashAmountToArduino(float amount) {
  float totalAmount = amount + credit;  // บวกเครดิตเข้าไปกับยอดเงิน

  Serial.println("=== CASH PAYMENT ===");
  Serial.print("Amount to send (amount + credit): ");
  Serial.println(totalAmount, 2);

  espSerial.print(totalAmount, 2);  // ใช้ espSerial แทน Serial2
  espSerial.print("\n");
  espSerial.flush();
}


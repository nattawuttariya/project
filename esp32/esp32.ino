

#include <TFT_eSPI.h>
#include <SPI.h>
#include "aes_encryption.h.ino"  // ไฟล์ที่คุณทำ AESLib ไว้
#include <WiFi.h>
#include <HTTPClient.h>
#include <stdlib.h>  // สำหรับ atof()
#include <FS.h>           // สำหรับ file system (SPIFFS หรือ LittleFS)
using namespace fs;       // ให้สามารถใช้ FS แทน fs::FS ได้
#include <esp_system.h>  // สำหรับการใช้ esp_random()
#include <ArduinoJson.h>
#include <WebServer.h>    // ใส่หลังจากที่ประกาศ namespace แล้ว

#include <WiFiUdp.h>
#include <NTPClient.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000);

TFT_eSPI tft = TFT_eSPI();
TFT_eSPI_Button creditBtn;  // ประกาศปุ่ม Credit
#define CALIBRATION_FILE "/TouchCalData1"
#define REPEAT_CAL true

// ปรับตำแหน่งปุ่มให้ใหญ่ขึ้นและกระจายพื้นที่มากขึ้นสำหรับจอ 320x480
#define KEY_X 65          // เริ่มจากซ้ายมากขึ้น ให้ปุ่มไม่ชิดขอบ
#define KEY_Y 150         // เลื่อนลงมา
#define KEY_W 70          // ปรับให้ปุ่มใหญ่ขึ้น (กว้าง)
#define KEY_H 50          // ปรับให้ปุ่มใหญ่ขึ้น (สูง)
#define KEY_SPACING_X 25  // ระยะห่างระหว่างปุ่มแนวนอนเพิ่มขึ้น
#define KEY_SPACING_Y 20  // ระยะห่างระหว่างปุ่มแนวตั้งเพิ่มขึ้น
#define KEY_TEXTSIZE 1    // ตัวหนังสือใหญ่ขึ้น อ่านง่ายขึ้น

#define DISP_X 0         //เริ่มต้นห่างจากขอบซ้าย 20
#define DISP_Y 50         //เริ่มจากด้านบนหน้าจอ 50 พิกเซล
#define DISP_W 440        //กว้าง
#define DISP_H 60         //สูง
#define DISP_TSIZE 5      //ขนาด
#define DISP_TCOLOR TFT_CYAN //กำหนดสีตัวอักษร
#define NUM_LEN 12        //กำหนดบัฟเฟอร์รับเลขสูงสุด 12 ตัว

char numberBuffer[NUM_LEN + 1] = "";

uint8_t numberIndex = 0;

#define STATUS_X 240
#define STATUS_Y 110

char keyLabel[14][10] = {
  "1", "2", "3", "4", "5", "6", "7", "8", "9", "Del",
  "0", "Send", "Skip", "cancel"
};

uint16_t keyColor[14] = {
  TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_BLUE,
  TFT_BLUE, TFT_BLUE, TFT_BLUE, TFT_DARKGREY, TFT_BLUE,
  TFT_DARKGREEN, TFT_ORANGE, TFT_PURPLE
};

TFT_eSPI_Button key[14];

void drawKeypad() {
  for (uint8_t b = 0; b < 14; b++) {
    int x = KEY_X + (b % 3) * (KEY_W + KEY_SPACING_X);
    int y = KEY_Y + (b / 3) * (KEY_H + KEY_SPACING_Y);
    int w = KEY_W;
    int h = KEY_H;
    int textSize = KEY_TEXTSIZE;

    // ขยายขนาดเฉพาะปุ่ม cancel
    if (strcmp(keyLabel[b], "cancel") == 0) {
      w = KEY_W * 1 + KEY_SPACING_X;  // กว้าง 2 ช่อง + ระยะห่าง
      h = KEY_H + 7;                 // สูงเพิ่มอีกเล็กน้อย (ปรับได้)
      x = KEY_X + (b % 3) * (KEY_W + KEY_SPACING_X); // อิงจากเดิม
      textSize = KEY_TEXTSIZE + 0.5;   // เพิ่มขนาดตัวอักษร
    }

    key[b].initButton(&tft, x, y, w, h, TFT_WHITE, keyColor[b], TFT_WHITE, keyLabel[b], textSize);
    key[b].drawButton();
  }
}




const char *ssid = "Nut_SC";
const char *password = "0920420694";
const char *serverUrl = "https://172.20.10.6:3000/save-transaction";

String selectedPaymentMethod = "";
float amountValue = 0.0;


char pointsBuffer[NUM_LEN + 1] = "";
uint8_t pointsIndex = 0;

char phoneNumber[NUM_LEN + 1] = "";
uint8_t phoneNumberIndex = 0;

bool isPhoneNumberEntered = false;
bool isAmountEntered = false;
bool isPointsEntered = false;
bool isPaymentMethodSelected = false;  // New flag for payment method selection
bool isPaymentMethodScan = false;     // Flag for selecting scan payment


// สร้าง instance ของ WebServer บนพอร์ต 3000
WebServer server(3000);

// ประกาศล่วงหน้าก่อนเรียกใช้งาน
void playAudio(int trackNumber);

static bool hasPlayedThankYouAudio = false;




#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>

HardwareSerial myDFPlayerSerial(1);  // ใช้ UART1 สำหรับ DFPlayer
DFRobotDFPlayerMini myDFPlayer;

#define ARDUINO_RX 16  // ESP32 รับข้อมูลจาก Arduino TX
#define ARDUINO_TX 17  // ESP32 ส่งข้อมูลไป Arduino RX

#define DFPLAYER_RX 26  // ESP32 รับข้อมูลจาก DFPlayer TX
#define DFPLAYER_TX 27  // ESP32 ส่งข้อมูลไป DFPlayer RX


#include <EEPROM.h>

#define EEPROM_SIZE 10  // 5 ค่า x 2 bytes = 10 bytes

// เพิ่ม global ด้านบนสุดของไฟล์
bool hasPlayedPhoneStartAudio = false;
bool hasPlayedPointsAudio = false;
bool hasPlayedAmountAudio = false;
bool hasShownThankYou = false;
bool isAtStartScreen = true;
// สมมติว่ามีตัวแปร global ที่ต้องใช้ใน handlePaymentMethodSelection()
bool paymentMethodSelected = false;
bool buttonsDrawn = false;

bool waitingForRelease = false;  // ตัวแปร global


bool skipPoints = false;  // ตัวแปรควบคุมการข้ามกรอกแต้ม

bool sentMorning = false;
bool sentEvening1 = false;  // สำหรับตี 1:50
bool sentEvening2 = false;  // สำหรับตี 2:00

bool isWaitingForCredit = false;
bool creditReceived = false;
int pointsValue = 0; 
int lastShownCredit = -1;  // เริ่มจากค่าที่ไม่ตรงกับอะไรเลย
int currentCredit = 0;     // เก็บค่าเครดิตจาก Arduino
String serialBuffer = "";  // เก็บข้อความที่อ่านมาจาก Serial2


// ฟังก์ชันเขียน 16-bit ลง EEPROM
void EEPROM_writeUShort(int address, uint16_t value) {
  EEPROM.write(address, value & 0xFF);
  EEPROM.write(address + 1, (value >> 8) & 0xFF);
}

// ฟังก์ชันอ่าน 16-bit จาก EEPROM
uint16_t EEPROM_readUShort(int address) {
  uint16_t value = EEPROM.read(address);
  value |= (EEPROM.read(address + 1) << 8);
  return value;
}

void saveCalibration(uint16_t *calData) {
  for (int i = 0; i < 5; i++) {
    EEPROM_writeUShort(i * 2, calData[i]);
  }
  EEPROM.commit();
  Serial.println("Calibration saved to EEPROM");
}

bool loadCalibration(uint16_t *calData) {
  bool valid = true;
  for (int i = 0; i < 5; i++) {
    calData[i] = EEPROM_readUShort(i * 2);
    // เช็คค่าว่างหรือผิดปกติ (0xFFFF หรือ 0)
    if (calData[i] == 0xFFFF || calData[i] == 0) {
      valid = false;
    }
  }
  return valid;
}

// ประกาศ calData ไว้ที่ระดับ global เพื่อใช้ใน setup และส่วนอื่น ๆ
uint16_t calData[5];

void setup() {
  Serial.begin(115200);  // Debug Serial Monitor

  // เริ่ม EEPROM
  EEPROM.begin(EEPROM_SIZE);  // อย่าลืม define EEPROM_SIZE เช่น 512

  // เริ่ม Serial2 สำหรับสื่อสารกับ Arduino
  Serial2.begin(9600, SERIAL_8N1, ARDUINO_RX, ARDUINO_TX);  // อย่าลืม define ARDUINO_RX/TX เช่น 16, 17

  // เริ่ม Serial กับ DFPlayer
  myDFPlayerSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);  // กำหนดขา DFPLAYER_RX/TX

  if (!myDFPlayer.begin(myDFPlayerSerial)) {
    Serial.println("DFPlayer ไม่พร้อม");
  } else {
    Serial.println("DFPlayer เริ่มทำงานแล้ว");
    myDFPlayer.volume(30);  // ปรับเสียงระดับ 0–30
  }

  // เริ่มหน้าจอ TFT
  tft.init();
  tft.setRotation(0);

  // โหลดค่าคาลิเบรตจอสัมผัส
  if (loadCalibration(calData)) {
    Serial.println("โหลดค่า calibration จาก EEPROM สำเร็จ");
    tft.setTouch(calData);  // ตั้งค่าจอสัมผัส
  } else {
    Serial.println("ค่า calibration ไม่ถูกต้อง ต้องคาลิเบรตใหม่");
    tft.calibrateTouch(calData, TFT_WHITE, TFT_BLACK, 15);
    saveCalibration(calData);
  }

  // แสดงหน้าเริ่มต้น
  showStartScreen();

  // เชื่อมต่อ WiFi
  connectWiFi();  // ควรมี retry หรือ timeout ด้วย

  // เริ่ม sync เวลา NTP
  timeClient.begin();

  // ตั้งค่า HTTP Endpoint
  server.on("/your-endpoint", HTTP_POST, handleESPPost);
  server.begin();

  Serial.println("ESP32 HTTP server running");
}



float lastAmount = 0.0;

// ตัวแปรสถานะแบบ global
bool isCreditScreenShown = false;
bool isAmountScreenShown = false;
unsigned long paymentTimeoutStart = 0;
const unsigned long paymentTimeoutDuration = 30000; // 30 วินาที


#define MAX_LINE_LENGTH 128

String lineBuffer = "";
int returnCredit = 0;
String currentUserID = "";
int currentRefund = 0;

String arduinoBuffer = "";  // Buffer สะสมข้อความจาก Arduino
bool isWiFiConnected = true;  // เพิ่มตัวแปรสถานะ WiFi
bool isOfflineMessageShown = false;  // กันไม่ให้แสดงซ้ำทุก loop
int pointsUsed = 0;      // ใส่ค่าที่ต้องการเริ่มต้น

float creditValue = 0.0;  // เครดิตปัจจุบัน

void loop() {
  server.handleClient();

  // ===== เช็คเวลาสำหรับส่งข้อความ =====
  timeClient.update();
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();

  if (currentHour == 22 && currentMinute == 30 && !sentMorning) {
    Serial2.println("22:30");
    Serial.println("Sent time: 22:30");
    sentMorning = true;
  }
  if (currentHour == 22 && currentMinute == 40 && !sentEvening1) {
    Serial2.println("22:40");
    Serial.println("Sent time: 22:40");
    sentEvening1 = true;
  }
  if (currentHour == 2 && currentMinute == 49 && !sentEvening2) {
    Serial2.println("02:49");
    Serial.println("Sent time: 02:49");
    sentEvening2 = true;
  }
  if (currentHour == 0 && currentMinute == 1) {
    sentMorning = false;
    sentEvening1 = false;
    sentEvening2 = false;
  }

  // ===== อ่านข้อมูลจาก Arduino =====
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n' || c == '\r') {
      if (lineBuffer.length() > 0) {
        lineBuffer.trim();
        parseLine(lineBuffer);
        lineBuffer = "";
      }
    } else {
      if (lineBuffer.length() < MAX_LINE_LENGTH) {
        lineBuffer += c;
      } else {
        Serial.println("Warning: lineBuffer overflow");
        lineBuffer = "";
      }
    }
  }

  // ===== ถ้าออฟไลน์ =====
  if (!isWiFiConnected) {
    if (!isOfflineMessageShown) {
      showStartScreen();
      isOfflineMessageShown = true;
    }
    return;
  } else {
    isOfflineMessageShown = false;
  }

  // ===== ตรวจสอบการทัชหน้าจอ =====
  uint16_t t_x = 0, t_y = 0;
  bool pressed = tft.getTouch(&t_x, &t_y);

  // ===== หน้าจอเริ่มต้น =====
  if (isAtStartScreen) {
    if (pressed && !waitingForRelease) {
      resetInput();
      isAtStartScreen = false;
      showPhoneNumberScreen();
      delay(300);
      waitingForRelease = true;
    } else if (!pressed) {
      waitingForRelease = false;
    }
    return;
  }

  // ===== หน้าจอกรอกเครดิต =====
  if (isCreditScreenShown) {
    for (uint8_t b = 0; b < 14; b++) {
      key[b].press(pressed && key[b].contains(t_x, t_y));
    }

    // ตรวจจับ keypad
    for (uint8_t b = 0; b < 14; b++) {
      if (key[b].justReleased()) key[b].drawButton();
      if (!key[b].justPressed()) continue;

      key[b].drawButton(true);

      if (strcmp(keyLabel[b], "cancel") == 0) {
        isCreditScreenShown = false;
        handleCancel();
        return;
      }
      if (b == 11 || b == 12) { // Send / Skip
        isCreditScreenShown = false;
        isAmountScreenShown = true;
        showAmountInputScreen();
        playAudio(3);
        return;
      }
      if (b <= 10) { // ตัวเลข
        handleCreditScreenInput(b);
        return;
      }
    }

    return;
  }

  // ===== หน้าจอกรอกจำนวนเงิน (เพิ่มปุ่ม Credit) =====
  if (isAmountScreenShown) {
    for (uint8_t b = 0; b < 14; b++) {
      key[b].press(pressed && key[b].contains(t_x, t_y));
    }
    creditBtn.press(pressed && creditBtn.contains(t_x, t_y)); // ตรวจจับปุ่ม Credit ที่นี่

    // ตรวจจับ keypad
    for (uint8_t b = 0; b < 14; b++) {
      if (key[b].justReleased()) key[b].drawButton();
      if (!key[b].justPressed()) continue;

      key[b].drawButton(true);

      if (strcmp(keyLabel[b], "cancel") == 0) {
        isAmountScreenShown = false;
        handleCancel();
        return;
      }
      if (b <= 10) { // ตัวเลข
        handleAmountInput(b);
        return;
      }
      if (b == 11 || b == 12) { // Send / Skip
        isAmountScreenShown = false;
        isPointsEntered = false;
        if (skipPoints) {
          showPaymentMethodScreen();
        }
        return;
      }
    }

    // ตรวจจับปุ่ม Credit
    if (creditBtn.justReleased()) creditBtn.drawButton();
    if (creditBtn.justPressed()) {
      creditBtn.drawButton(true);
      Serial.println("Credit button pressed!");

      // อัปเดต creditValue ก่อนส่ง
      updateCreditValue();

      // ส่งข้อมูลไป server
      sendCreditData(
        phoneNumber,    // เบอร์โทรศัพท์
        creditValue     // เครดิตที่ผู้ใช้กรอก
      );

      playAudio(6); // เสียงตอบรับ
      
    //รีเซ็ต input และกลับหน้าเริ่มต้น
      resetInput();
      return;
    }

    return;
  }

  // ===== แสดงเครดิต =====
  checkCreditFromArduino();
  static int lastDisplayedCredit = -1;
  if (!isPaymentMethodScan && isPhoneNumberEntered && isAmountEntered && isPointsEntered && isPaymentMethodSelected && !hasShownThankYou && currentCredit != lastDisplayedCredit) {
    lastDisplayedCredit = currentCredit;
    showCurrentCredit();
  }

  // ===== เสียงกรอกเบอร์ครั้งแรก =====
  if (!isPhoneNumberEntered && !hasPlayedPhoneStartAudio) {
    playAudio(1);
    hasPlayedPhoneStartAudio = true;
  }

  // ===== ตรวจสอบ keypad สำหรับหน้าทั่วไป =====
  for (uint8_t b = 0; b < 14; b++) {
    key[b].press(pressed && key[b].contains(t_x, t_y));
  }
  for (uint8_t b = 0; b < 14; b++) {
    if (key[b].justReleased()) key[b].drawButton();
    if (!key[b].justPressed()) continue;

    key[b].drawButton(true);

    if (strcmp(keyLabel[b], "cancel") == 0) {
      tft.fillRect(60, 100, 200, 40, TFT_BLACK);
      tft.fillRect(60, 160, 200, 40, TFT_BLACK);
      tft.fillRect(60, 220, 200, 40, TFT_BLACK);
      tft.fillRect(60, 270, 250, 30, TFT_BLACK);
      updateDisplay("");
      handleCancel();
      return;
    }

    if (b == 12 && !isPhoneNumberEntered && !isAmountEntered && !isPaymentMethodSelected) {
      isPhoneNumberEntered = true;
      isPointsEntered = true;
      skipPoints = true;
      playAudio(3);
      tft.fillRect(0, 20, 320, 30, tft.color565(230, 230, 255));
      tft.setTextColor(tft.color565(70, 30, 120), tft.color565(230, 230, 255));
      tft.setTextFont(4);
      tft.setCursor(80, 20);
      tft.print("Enter amount");
      updateDisplay("");
      return;
    }

    if (!isPhoneNumberEntered) {
      handlePhoneNumberInput(b);
      if (isPhoneNumberEntered) {
        playAudio(2);
        updateDisplay("");
        isCreditScreenShown = true;
        showCreditScreen();
        return;
      }
    } else if (!isAmountEntered) {
      handleAmountInput(b);
      if (isAmountEntered && !hasPlayedAmountAudio) {
        hasPlayedAmountAudio = true;
        if (skipPoints) {
          isPointsEntered = true;
          playAudio(5);
          showPaymentMethodScreen();
        } else {
          playAudio(4);
        }
        return;
      }
    } else if (!isPointsEntered && !skipPoints) {
      handlePointsInput(b);
      return;
    } else if (!isPaymentMethodSelected) {
      handlePaymentMethodSelection();
      if (paymentMethodSelected) {
        isPaymentMethodSelected = true;
        paymentTimeoutStart = millis();
        return;
      }
    }

    if (isPhoneNumberEntered && isAmountEntered && isPointsEntered && isPaymentMethodSelected && !hasShownThankYou) {
      hasShownThankYou = true;
      showThankYouAndReset();
      return;
    }
  }

  if (!hasShownThankYou && isPaymentMethodSelected && !isPaymentMethodScan && currentCredit == lastAmount) {
    delay(500);
    sendCashAmountToArduino(amountValue);
    hasShownThankYou = true;
    showThankYouAndReset();
    return;
  }

  // ===== ตรวจ timeout =====
  if (isPaymentMethodSelected && !hasShownThankYou && paymentTimeoutStart != 0 && (millis() - paymentTimeoutStart > paymentTimeoutDuration)) {
    Serial.println("Timeout! กลับไปหน้าแรก");
    handleCancel();
    return;
  }
}




// ฟังก์ชันจัดการสถานะ WiFi ตามโหมดที่รับจาก Arduino
void handleModeFromArduino(const String &mode) {
  Serial.print("handleModeFromArduino called with mode: ");
  Serial.println(mode);

  if (mode == "online") {
    Serial.println("Mode is online");
    if (!isWiFiConnected) {
      Serial.println("Connecting WiFi...");
      WiFi.mode(WIFI_STA);
      connectWiFi();
      isWiFiConnected = (WiFi.status() == WL_CONNECTED);
      Serial.print("WiFi connected: ");
      Serial.println(isWiFiConnected);
    }
  } else if (mode == "offline") {
    Serial.println("Mode is offline");
    if (isWiFiConnected) {
      Serial.println("Disconnecting WiFi...");
      WiFi.disconnect(true);
      delay(100);
      WiFi.mode(WIFI_OFF);
      isWiFiConnected = false;
      Serial.println("WiFi disconnected");
      Serial.print("WiFi status: ");
      Serial.println(WiFi.status());
    } else {
      Serial.println("WiFi already disconnected");
    }
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);         // ตั้งโหมด STA
  WiFi.begin(ssid, password);  // เริ่มเชื่อมต่อ
  Serial.println("Connecting to WiFi...");

  int retryCount = 0;
  const int maxRetries = 20;

  while (WiFi.status() != WL_CONNECTED && retryCount < maxRetries) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}






// ========= แยกข้อมูลแต่ละบรรทัด =========

float parsedRefundAmount = 0.0;
bool readyToAddCredit = false;

void parseLine(String line) {
  Serial.print("Received line: ");
  Serial.println(line);

  if (line.startsWith("Amount (net):")) {
    parsedRefundAmount = line.substring(13).toFloat();
    Serial.print("Parsed refundAmount: ");
    Serial.println(parsedRefundAmount, 2);
    readyToAddCredit = true;  // เตรียมส่งหลังได้ User ID
  } 
  else if (line.startsWith("User ID:")) {
    currentUserID = line.substring(8);
    currentUserID.trim();
    Serial.print("Parsed currentUserID: ");
    Serial.println(currentUserID);

    // ถ้ารับ amount มาก่อนหน้าแล้ว ให้เรียก addcredit
    if (readyToAddCredit && parsedRefundAmount > 0 && currentUserID.length() > 0) {
      Serial.println("==> Calling addcredit with refund data");
      addcredit(currentUserID, parsedRefundAmount);

      // reset flag
      readyToAddCredit = false;
      parsedRefundAmount = 0;
    }
  }
  else if (line.equalsIgnoreCase("online")) {
    handleModeFromArduino("online");
  }
  else if (line.equalsIgnoreCase("offline")) {
    handleModeFromArduino("offline");
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(line);
  }
}


void readDataFromArduino() {
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.print(c);  // Debug แสดงข้อมูลที่รับแบบเรียลไทม์

    if (c == '\n' || c == '\r') {
      if (lineBuffer.length() > 0) {
        lineBuffer.trim();
        parseLine(lineBuffer);
        lineBuffer = "";
      }
    } else {
      if (lineBuffer.length() < MAX_LINE_LENGTH) {
        lineBuffer += c;
      } else {
        Serial.println("Warning: lineBuffer overflow, clearing buffer");
        lineBuffer = "";
      }
    }
  }
}






void sendModeStatusToArduino(const String &mode) {
  if (mode == "online" || mode == "offline") {
    Serial2.println(mode);
    Serial.print("Sent to Arduino: ");
    Serial.println(mode);
  } else {
    Serial.print("Invalid mode: ");
    Serial.println(mode);
  }
}

// ฟังก์ชันตรวจรับข้อความจาก Arduino ต่อเนื่องตลอดเวลา
void checkArduinoMessage() {
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n') {  // ข้อความจบที่ \n
      arduinoBuffer.trim();

      if (arduinoBuffer == "online") {
        Serial.println("Received online from Arduino");
        handleModeFromArduino("online");
      } else if (arduinoBuffer == "offline") {
        Serial.println("Received offline from Arduino");
        handleModeFromArduino("offline");
      } else {
        Serial.print("Unknown message from Arduino: ");
        Serial.println(arduinoBuffer);
      }

      arduinoBuffer = "";  // ล้างบัฟเฟอร์หลังอ่าน
    } else {
      arduinoBuffer += c;  // เก็บตัวอักษรเพิ่มในบัฟเฟอร์
    }
  }
}






void handleCancel() {
  Serial.println("ยกเลิกข้อมูลและกลับหน้าแรก");

  // รีเซ็ตสถานะทั้งหมด
  isPhoneNumberEntered = false;
  isAmountEntered = false;
  isPointsEntered = false;
  isPaymentMethodSelected = false;
  paymentMethodSelected = false;
  skipPoints = false;

  hasPlayedPhoneStartAudio = false;
  hasPlayedPointsAudio = false;
  hasPlayedAmountAudio = false;
  hasShownThankYou = false;

  buttonsDrawn = false;  // รีเซ็ตสถานะปุ่มเพื่อให้วาดใหม่ได้
  paymentTimeoutStart = 0;  //  รีเซ็ตตัวจับเวลาด้วย
  amountValue = 0;
  pointsValue = 0;
  currentCredit = 0;

  // ล้างข้อมูลในบัฟเฟอร์
  phoneNumber[0] = '\0';
  pointsBuffer[0] = '\0';
  numberBuffer[0] = '\0';
  pointsIndex = 0;
  phoneNumberIndex = 0;
  numberIndex = 0;

  // ล้างหน้าจอทั้งหมด (หรือเฉพาะส่วนที่ต้องการ)
  tft.fillScreen(TFT_BLACK);

  // เคลียร์ปุ่มทั้งหมด (ถ้ามีฟังก์ชันเคลียร์ปุ่ม ให้เรียกที่นี่)
  clearKeypad();   // สมมติว่าฟังก์ชันนี้ลบปุ่มทั้งหมด

  // เคลียร์ข้อความหรือช่องอื่น ๆ ที่อาจเหลือ
  updateDisplay("");

  // ตั้งสถานะว่าอยู่หน้าเริ่มต้น
  isAtStartScreen = true;

  // แสดงหน้าจอเริ่มต้นใหม่
  showStartScreen();
}


void handleESPPost() {
  if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    Serial.println("Data received from Flask:");
    Serial.println(body);

    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, body);
    if (error) {
      Serial.println("JSON parsing failed");
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }

    // 🔹 อ่านค่า amount จาก JSON
    // ถ้ามี "finally_amount" ใช้ค่านั้น (สำหรับเครดิต)
    // ถ้าไม่มี แต่มี "amount" ใช้ค่านั้น (สำหรับ POS)
    float amountValue = 0.0;
    if (doc.containsKey("finally_amount")) {
      amountValue = doc["finally_amount"] | 0.0;
    } else if (doc.containsKey("amount")) {
      amountValue = doc["amount"] | 0.0;
    }

    const char* userId = doc["phone"] | "";  // รับ user_id จากฟิลด์ "phone"
    lastAmount = amountValue;

    // Debug ผ่าน Serial Monitor
    Serial.println("---------------------------------");
    Serial.println("Next to Arduino");
    Serial.print("Amount (net): ");
    Serial.println(lastAmount, 2);
    Serial.print("User ID: ");
    Serial.println(userId);
    Serial.println("---------------------------------");

    // ส่งข้อมูลไป Arduino
    Serial2.print("AMOUNT:");
    Serial2.println(lastAmount, 2);

    Serial2.print("User ID:");
    Serial2.println(userId);
    
    Serial2.flush();

    // ส่ง response กลับ Flask พร้อมรายละเอียด
    DynamicJsonDocument resDoc(256);
    resDoc["message"] = "ESP32 received the data.";
    resDoc["received_amount"] = lastAmount;
    resDoc["user_id"] = userId;

    String resStr;
    serializeJson(resDoc, resStr);
    server.send(200, "application/json", resStr);

    showThankYouAndReset();
    
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

//////////////////////////////////////////////////////////////////////////


// เพิ่มพารามิเตอร์ amount (float หรือแปลงเป็น float) เพื่อส่งไปที่ Arduino
void sendCashAmountToArduino(float amount) {
  float totalAmount = amount + creditValue;  // บวกเครดิตเข้าไปกับยอดเงิน

  Serial.println("=== CASH PAYMENT ===");
  Serial.print("Amount to send (amount + credit): ");
  Serial.println(totalAmount, 2);  // แสดงยอดรวมก่อนส่ง

  Serial2.print("AMOUNT: ");
  Serial2.println(totalAmount, 2);

  Serial2.print("User ID: ");
  Serial2.println(phoneNumber);  // ต้องแน่ใจว่า currentUserID มีค่าก่อนเรียกฟังก์ชันนี้

  Serial2.flush();
}


/////////////////////////////////////////////////////////////////////////////



// สมมติฟังก์ชันนี้ต้องสร้างขึ้นเพื่อแสดงหน้าชำระเงิน
void showPaymentMethodScreen() {
  // ล้างหน้าจอ, รีเซ็ตสถานะวาดปุ่ม, วาดปุ่มวิธีชำระเงินใหม่
  buttonsDrawn = false;
  paymentMethodSelected = false;
  isPaymentMethodSelected = false;

  // เรียกฟังก์ชันที่วาดหน้าชำระเงิน
  handlePaymentMethodSelection();
}


void showStartScreen() {
  // เคลียร์หน้าจอทั้งหมดก่อนวาดใหม่
  tft.fillScreen(TFT_WHITE);

  // วาดกรอบและพื้นหลัง
  uint16_t backgroundColor = tft.color565(230, 230, 255);
  uint16_t frameColor = tft.color565(140, 110, 190);
  tft.fillRoundRect(10, 10, 300, 460, 40, backgroundColor);
  tft.drawRoundRect(10, 10, 300, 460, 40, frameColor);

  // ฟังก์ชันวาดฟองสบู่ (nested inline)
  auto drawBubble = [&](int x, int y, int radius) {
    uint16_t bubbleColor = tft.color565(135, 206, 250);       // ฟ้าอ่อน (LightSkyBlue)
    uint16_t bubbleHighlight = tft.color565(224, 255, 255);   // ฟ้าใส (LightCyan)

    tft.fillCircle(x, y, radius, bubbleColor);
    tft.fillCircle(x - radius/3, y - radius/3, radius / 2, bubbleHighlight);
    tft.drawCircle(x, y, radius, TFT_WHITE);
  };

  // ฟังก์ชันวาดผีเสื้อ
  auto drawButterfly = [&](int x, int y, int size) {
    uint16_t wingBaseColor = tft.color565(200, 100, 180);
    uint16_t wingHighlight = tft.color565(255, 180, 220);
    uint16_t bodyColor = tft.color565(100, 30, 150);
    uint16_t outlineColor = TFT_BLACK;

    tft.fillCircle(x, y, size / 6, bodyColor);
    tft.drawCircle(x, y, size / 6, outlineColor);

    tft.fillTriangle(x, y, x - size / 2, y - size / 2, x - size / 3, y - size / 5, wingBaseColor);
    tft.fillTriangle(x - size / 2 + 4, y - size / 2 + 4, x - size / 3, y - size / 5, x - size / 2 + 10, y - size / 5, wingHighlight);

    tft.fillTriangle(x, y, x - size / 2, y + size / 2, x - size / 3, y + size / 5, wingBaseColor);
    tft.fillTriangle(x - size / 2 + 4, y + size / 2 - 4, x - size / 3, y + size / 5, x - size / 2 + 10, y + size / 5, wingHighlight);

    tft.fillTriangle(x, y, x + size / 2, y - size / 2, x + size / 3, y - size / 5, wingBaseColor);
    tft.fillTriangle(x + size / 2 - 4, y - size / 2 + 4, x + size / 3, y - size / 5, x + size / 2 - 10, y - size / 5, wingHighlight);

    tft.fillTriangle(x, y, x + size / 2, y + size / 2, x + size / 3, y + size / 5, wingBaseColor);
    tft.fillTriangle(x + size / 2 - 4, y + size / 2 - 4, x + size / 3, y + size / 5, x + size / 2 - 10, y + size / 5, wingHighlight);

    tft.drawLine(x, y, x - size / 2, y - size / 2, outlineColor);
    tft.drawLine(x, y, x - size / 2, y + size / 2, outlineColor);
    tft.drawLine(x, y, x + size / 2, y - size / 2, outlineColor);
    tft.drawLine(x, y, x + size / 2, y + size / 2, outlineColor);
  };

  // ฟังก์ชันวาดขวดน้ำยา (ปรับสวยขึ้นด้วย gradient ง่ายๆ)
  auto drawBottle = [&](int x, int y, int width, int height) {
    uint16_t outlineColor = TFT_BLACK;

    for (int i = 0; i < height; i++) {
      uint8_t r = 70 + (uint8_t)(120 * i / height);
      uint8_t g = 110 + (uint8_t)(140 * i / height);
      uint8_t b = 220 + (uint8_t)(35 * i / height);
      uint16_t gradColor = tft.color565(r, g, b);
      tft.drawFastHLine(x, y + height - i, width, gradColor);
    }

    tft.drawRoundRect(x, y, width, height, 12, outlineColor);

    int capHeight = height / 7;
    for (int i = 0; i < capHeight; i++) {
      uint8_t r = 255;
      uint8_t g = 180 + (uint8_t)(40 * i / capHeight);
      uint8_t b = 210 + (uint8_t)(45 * i / capHeight);
      uint16_t capColor = tft.color565(r, g, b);
      tft.drawFastHLine(x + width/4, y - capHeight + i, width/2, capColor);
    }
    tft.drawRoundRect(x + width/4, y - capHeight, width/2, capHeight, 6, outlineColor);

    int labelHeight = height / 3;
    for (int i = 0; i < labelHeight; i++) {
      uint8_t r = 240 + (uint8_t)(15 * i / labelHeight);
      uint8_t g = 230 + (uint8_t)(20 * i / labelHeight);
      uint8_t b = 255;
      uint16_t labelColor = tft.color565(r, g, b);
      tft.drawFastHLine(x + width/8, y + height/3 + i, width*3/4, labelColor);
    }
    tft.drawRoundRect(x + width/8, y + height/3, width*3/4, labelHeight, 7, outlineColor);

    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(2);
    tft.setTextColor(tft.color565(80, 30, 130));
    tft.drawString("Softener", x + width/2, y + height/2 + labelHeight/5);

    for (int i = 0; i < height; i++) {
      if (i % 2 == 0) {
        tft.drawPixel(x + width - 3, y + i, tft.color565(255, 255, 255));
      }
    }
  };

  // วาดฟองสบู่ข้างบน
  drawBubble(40, 60, 15);
  drawBubble(90, 80, 10);
  drawBubble(130, 50, 12);
  drawBubble(180, 70, 14);
  drawBubble(230, 60, 10);
  drawBubble(270, 80, 12);

  // วาดฟองสบู่ข้างล่าง
  drawBubble(50, 400, 20);
  drawBubble(100, 450, 15);
  drawBubble(200, 420, 25);
  drawBubble(250, 460, 18);
  drawBubble(270, 430, 12);
  drawBubble(80, 430, 10);

  // วาดผีเสื้อ 2 ตัว
  drawButterfly(70, 150, 40);
  drawButterfly(230, 130, 50);

  // วาดขวดน้ำยา
  drawBottle(130, 230, 80, 140);

  // หัวข้อหลัก
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(4);
  tft.setTextColor(tft.color565(70, 30, 120));
  tft.drawString("Softener", 160, 110);

  // คำโปรยใต้หัวข้อ
  tft.setTextFont(2);
  tft.setTextColor(tft.color565(120, 120, 140));
  tft.drawString("Feel the softness and freshness", 160, 180);

  // วาดกล่องข้อความ (ไม่ใช่ปุ่มกด)
  uint16_t buttonColor = tft.color565(255, 180, 210);
  uint16_t buttonBorder = tft.color565(160, 60, 120);
  tft.fillRoundRect(60, 380, 200, 40, 10, buttonColor);
  tft.drawRoundRect(60, 380, 200, 40, 10, buttonBorder);

  tft.setTextColor(tft.color565(70, 30, 120));
  tft.setTextFont(4);
  tft.drawString("Touch screen ", 160, 400);

  buttonsDrawn = false;
}


// ฟังก์ชันเล่นเสียง
void playAudio(int trackNumber) {
  myDFPlayer.play(trackNumber);
}

float amount = 0.0;

float getAmountToPay() {
  return amount;
}




void showPhoneNumberScreen() {
  // สีพื้นหลังและฟองสบู่
  uint16_t backgroundColor = tft.color565(230, 230, 255);  // ฟ้าพาสเทล
  uint16_t frameColor = tft.color565(140, 110, 190);       // ม่วงพาสเทล
  uint16_t textColor = tft.color565(70, 30, 120);          // ม่วงเข้ม
  uint16_t labelColor = tft.color565(120, 120, 140);       // เทานุ่ม
  uint16_t bubbleColor = tft.color565(200, 240, 255);      // ฟองฟ้าอ่อน
  uint16_t bubbleOutline = tft.color565(150, 200, 255);    // ขอบฟอง

  // วาดพื้นหลังเต็มจอ
  tft.fillScreen(frameColor);
  tft.fillRect(0, 0, 320, 480, backgroundColor);

  // วาดฟองสบู่หลายฟอง
  int bubbleX[] = {40, 90, 140, 200, 260, 300, 70, 130, 190, 240, 20, 280};
  int bubbleY[] = {100, 50, 80, 30, 60, 110, 200, 250, 300, 350, 400, 420};
  int bubbleR[] = {15, 10, 12, 8, 10, 14, 20, 16, 12, 18, 10, 13};

  for (int i = 0; i < 12; i++) {
    tft.fillCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleColor);
    tft.drawCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleOutline);
  }

  // หัวข้อ
  tft.setTextColor(textColor, backgroundColor);
  tft.setTextFont(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Enter Phone Number", 160, 40);

  // คำแนะนำ
  tft.setTextFont(2);
  tft.setTextColor(labelColor);
  tft.drawString("Used to collect points", 160, 80);

  // แสดงช่องกรอกเบอร์
  updateDisplay("");


  // วาดปุ่มกดตัวเลข
  drawKeypad();

}


void showThankYouAndReset() {
  uint16_t backgroundColor = tft.color565(230, 230, 255);
  uint16_t bubbleColor = tft.color565(200, 240, 255);
  uint16_t bubbleOutline = tft.color565(150, 200, 255);
  uint16_t textColor = tft.color565(70, 30, 120);

  tft.fillScreen(backgroundColor);

  int bubbleX[] = {40, 90, 140, 200, 260, 300, 70, 130, 190, 240, 20, 280};
  int bubbleY[] = {100, 50, 80, 30, 60, 110, 200, 250, 300, 350, 400, 420};
  int bubbleR[] = {15, 10, 12, 8, 10, 14, 20, 16, 12, 18, 10, 13};

  for (int i = 0; i < 12; i++) {
    tft.fillCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleColor);
    tft.drawCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleOutline);
  }

  int centerX = 160;
  int centerY = 240;

  // วาดเงา (shadow) สีดำ เลื่อนลงขวา 2 px
  tft.setTextColor(TFT_BLACK, backgroundColor);
  tft.setTextFont(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Thank You!", centerX + 2, centerY + 2);

  // วาดข้อความจริงสี textColor กึ่งกลาง
  tft.setTextColor(textColor, backgroundColor);
  tft.drawString("Thank You!", centerX, centerY);

  // วาดกรอบสี่เหลี่ยมมนล้อมรอบข้อความ
  // โดยประมาณข้อความสูง 30 px กว้าง 140 px
  int rectW = 140;
  int rectH = 40;
  int rectX = centerX - rectW / 2;
  int rectY = centerY - rectH / 2;
  tft.drawRoundRect(rectX, rectY, rectW, rectH, 10, textColor);

  playAudio(6);

  delay(3000);

  resetInput();

  isAtStartScreen = true;
  showStartScreen();
}










void handlePointsInput(uint8_t b)
{ 
  if (b >= 0 && b <= 8)
  {
    if (pointsIndex < NUM_LEN)
    {
      pointsBuffer[pointsIndex] = keyLabel[b][0];
      pointsIndex++;
      pointsBuffer[pointsIndex] = 0;
    }
    updateDisplay(pointsBuffer);
  }
  if (b == 9 && pointsIndex > 0)
  {
    pointsIndex--;
    pointsBuffer[pointsIndex] = 0;
    updateDisplay(pointsBuffer);
  }
  if (b == 10)
  {
    if (pointsIndex < NUM_LEN)
    {
      pointsBuffer[pointsIndex] = '0';
      pointsIndex++;
      pointsBuffer[pointsIndex] = 0;
    }
    updateDisplay(pointsBuffer);
  }
  if (b == 11)  // กดปุ่ม Skip
{   
    isPointsEntered = true;

    clearKeypad(); // เคลียร์คีย์แพด (ลบปุ่มตัวเลข)

    // ตั้งสถานะให้พร้อมสำหรับเลือกวิธีชำระเงิน
    isPaymentMethodSelected = false;

    // เล่นเสียงที่ต้องการตอนกด Skip
 

    // เรียกฟังก์ชันเลือกวิธีชำระเงิน
    handlePaymentMethodSelection();
}

}




// ตัวแปรเก็บข้อมูล

int Creditdata = 100; // ตัวอย่างเครดิต

void handlePhoneNumberInput(uint8_t b)
{
  if (b >= 0 && b <= 8)
  {
    if (phoneNumberIndex < NUM_LEN)
    {
      phoneNumber[phoneNumberIndex] = keyLabel[b][0];
      phoneNumberIndex++;
      phoneNumber[phoneNumberIndex] = 0;
    }
    updateDisplay(phoneNumber);
  }
  else if (b == 9 && phoneNumberIndex > 0)
  {
    phoneNumberIndex--;
    phoneNumber[phoneNumberIndex] = 0;
    updateDisplay(phoneNumber);
  }
  else if (b == 10)
  {
    if (phoneNumberIndex < NUM_LEN)
    {
      phoneNumber[phoneNumberIndex] = '0';
      phoneNumberIndex++;
      phoneNumber[phoneNumberIndex] = 0;
    }
    updateDisplay(phoneNumber);
  }
  else if (b == 11)
  {
    // กด Next หลังกรอกเบอร์
    isPhoneNumberEntered = true;
    isCreditScreenShown = true;
    isAmountScreenShown = false;
  }
}



char creditBuffer[10] = {0};
int creditIndex = 0;

void handleCreditScreenInput(uint8_t b)
{
  if (b >= 0 && b <= 8)  // ปุ่ม 1-9
  {
    if (creditIndex < 9)
    {
      creditBuffer[creditIndex] = keyLabel[b][0];
      creditIndex++;
      creditBuffer[creditIndex] = 0;
    }
    updateDisplay(creditBuffer);
  }
  else if (b == 9 && creditIndex > 0)  // ปุ่มลบ
  {
    creditIndex--;
    creditBuffer[creditIndex] = 0;
    updateDisplay(creditBuffer);
  }
  else if (b == 10)  // ปุ่ม 0
  {
    if (creditIndex < 9)
    {
      creditBuffer[creditIndex] = '0';
      creditIndex++;
      creditBuffer[creditIndex] = 0;
    }
    updateDisplay(creditBuffer);
  }
}



// ฟังก์ชันแสดงหน้าจอเครดิต พร้อมเรียกแสดงเครดิตล่าสุด
void showCreditScreen() {
  uint16_t backgroundColor = tft.color565(230, 230, 255);
  uint16_t frameColor = tft.color565(140, 110, 190);
  uint16_t textColor = tft.color565(70, 30, 120);
  uint16_t bubbleColor = tft.color565(200, 240, 255);
  uint16_t bubbleOutline = tft.color565(150, 200, 255);

  // วาดพื้นหลัง
  tft.fillScreen(frameColor);
  tft.fillRect(0, 0, 320, 480, backgroundColor);

  // วาดวงกลมตกแต่ง
  int bubbleX[] = {40, 90, 140, 200, 260, 300, 70, 130, 190, 240, 20, 280};
  int bubbleY[] = {100, 50, 80, 30, 60, 110, 200, 250, 300, 350, 400, 420};
  int bubbleR[] = {15, 10, 12, 8, 10, 14, 20, 16, 12, 18, 10, 13};
  for (int i = 0; i < 12; i++) {
    tft.fillCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleColor);
    tft.drawCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleOutline);
  }

  // แสดงหัวข้อ
  tft.setTextColor(textColor, backgroundColor);
  tft.setTextFont(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Current Credit", 90, 40);

  // แสดงเครดิตปัจจุบัน
  fetchCreditFromServer(phoneNumber);
  displayCurrentCredit();


  // แสดง keypad
  drawKeypad();

  // เคลียร์กล่องแสดงข้อมูล
  updateDisplay("");
}

void updateCreditValue() {
  // ถ้าผู้ใช้กรอกค่าเครดิต
  if (strlen(creditBuffer) > 0) {
    creditValue = atof(creditBuffer);  // แปลง string เป็น float
  } else {
    creditValue = 0.0;  // ถ้ายังว่าง ให้เครดิตเป็น 0
  }
}





void showAmountInputScreen() {
  uint16_t backgroundColor = tft.color565(230, 230, 255);
  uint16_t frameColor = tft.color565(140, 110, 190);
  uint16_t textColor = tft.color565(70, 30, 120);
  uint16_t bubbleColor = tft.color565(200, 240, 255);
  uint16_t bubbleOutline = tft.color565(150, 200, 255);

  // วาดพื้นหลัง
  tft.fillScreen(frameColor);
  tft.fillRect(0, 0, 320, 480, backgroundColor);

  // วาดวงกลมตกแต่ง
  int bubbleX[] = {40, 90, 140, 200, 260, 300, 70, 130, 190, 240, 20, 280};
  int bubbleY[] = {100, 50, 80, 30, 60, 110, 200, 250, 300, 350, 400, 420};
  int bubbleR[] = {15, 10, 12, 8, 10, 14, 20, 16, 12, 18, 10, 13};
  for (int i = 0; i < 12; i++) {
    tft.fillCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleColor);
    tft.drawCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleOutline);
  }

  // แสดงหัวข้อ
  tft.setTextColor(textColor, backgroundColor);
  tft.setTextFont(4);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Enter Amount", 160, 40);

  // วาด keypad
  drawKeypad();

  // วางปุ่ม Credit ที่นี่แทนหน้า Credit
  int creditX = KEY_X + 2 * (KEY_W + KEY_SPACING_X);  // ตัวอย่างวางขวาสุด
  int creditY = KEY_Y + 4 * (KEY_H + KEY_SPACING_Y);  // ตัวอย่างวางแถวใหม่
  creditBtn.initButton(&tft, creditX, creditY, KEY_W, KEY_H, TFT_WHITE, TFT_BLUE, TFT_WHITE, "Credit", KEY_TEXTSIZE);
  creditBtn.drawButton();

  // เคลียร์กล่องแสดงข้อมูล
  updateDisplay("");
}


int currentCreditServer = 0; // เครดิตปัจจุบันจากเซิร์ฟเวอร์

// ฟังก์ชันแสดงเครดิตปัจจุบันบนจอ
void displayCurrentCredit() {
  // ล้างพื้นหลังบริเวณข้อความ credit: (เลือกตำแหน่งและขนาดให้พอดี)
  tft.fillRect(180, 10, 140, 50, TFT_BLACK);  // เคลียร์พื้นเก่า

  // ตั้งสีข้อความเหลือง และพื้นหลังสีดำ
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(190, 20);
  tft.setTextFont(4);
  tft.print("credit: ");
  tft.print(currentCreditServer);

  updateDisplay("");
}



// ฟังก์ชันอัปเดตเครดิตจากเซิร์ฟเวอร์
void updateCreditFromServer(int newCredit) {
  currentCreditServer = newCredit;
  displayCurrentCredit();
}

void fetchCreditFromServer(String userId) {
  if (userId.length() == 0) {
    Serial.println("User ID is empty. Skipping server request.");
    return;
  }

  // ไม่เข้ารหัส userId แล้ว ส่งตรงๆ
  String serverUrl = "https://172.20.10.6:3000/getCredit?user_id=" + userId;

  HTTPClient http;

  Serial.println("Connecting to: " + serverUrl);

  http.begin(serverUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println("Server Response: " + payload);

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      if (doc.containsKey("credit")) {
        int credit = doc["credit"];
        Serial.print("Credit from server: ");
        Serial.println(credit);
        updateCreditFromServer(credit);  // ฟังก์ชันนี้คุณต้องเขียนเองเพื่ออัปเดตค่าเครดิตในระบบ
      } else {
        Serial.println("Key 'credit' not found in JSON response.");
      }
    } else {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
    }

  } else {
    Serial.println("Failed to connect to server");
    Serial.print("HTTP Code: ");
    Serial.println(httpCode);
  }

  http.end();
}







int currentPoints = 0; // คะแนนปัจจุบัน

// ฟังก์ชันแสดงคะแนนปัจจุบันบนจอ
void displayCurrentPoints() {
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(170, 20);
  tft.print("points: ");
  tft.print(currentPoints);
  updateDisplay("");
}

// ฟังก์ชันอัปเดตคะแนนจากเซิร์ฟเวอร์
void updatePointsFromServer(int newPoints) {
  currentPoints = newPoints;
  displayCurrentPoints();
}




void fetchPointsFromServer(String userId) {
  if (userId.length() == 0) {
    Serial.println("User ID is empty. Skipping server request.");
    return;
  }

  // ไม่เข้ารหัส userId แล้ว ส่งตรงๆ
  String serverUrl = "https://172.20.10.6:3000/getPoints?user_id=" + userId;

  HTTPClient http;

  Serial.println("Connecting to: " + serverUrl);

  http.begin(serverUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println("Server Response: " + payload);

    // แปลง JSON
    StaticJsonDocument<200> doc;  // กำหนดขนาด buffer ให้เหมาะสม
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      if (doc.containsKey("points")) {
        int points = doc["points"];
        Serial.print("Points from server: ");
        Serial.println(points);
        updatePointsFromServer(points);
      } else {
        Serial.println("Key 'points' not found in JSON response.");
      }
    } else {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.c_str());
    }

  } else {
    Serial.println("Failed to connect to server");
    Serial.print("HTTP Code: ");
    Serial.println(httpCode);
  }
  http.end();
}



void handleAmountInput(uint8_t b)
{
  if (b >= 0 && b <= 8)
  {
    if (numberIndex < NUM_LEN)
    {
      numberBuffer[numberIndex] = keyLabel[b][0];
      numberIndex++;
      numberBuffer[numberIndex] = 0;
    }
    updateDisplay(numberBuffer);
  }
  else if (b == 9 && numberIndex > 0)
  {
    numberIndex--;
    numberBuffer[numberIndex] = 0;
    updateDisplay(numberBuffer);
  }
  else if (b == 10)
  {
    if (numberIndex < NUM_LEN)
    {
      numberBuffer[numberIndex] = '0';
      numberIndex++;
      numberBuffer[numberIndex] = 0;
    }
    updateDisplay(numberBuffer);
  }
  else if (b == 11) // ปุ่ม OK
  {
    amountValue = atof(numberBuffer);
    isAmountEntered = true;

    // === วาดหน้าจอสไตล์ฟองสบู่ใหม่ ===
    uint16_t backgroundColor = tft.color565(230, 230, 255);  // ฟ้าพาสเทล
    uint16_t frameColor = tft.color565(140, 110, 190);       // ม่วงพาสเทล
    uint16_t textColor = tft.color565(70, 30, 120);          // ม่วงเข้ม
    uint16_t labelColor = tft.color565(120, 120, 140);       // เทานุ่ม
    uint16_t bubbleColor = tft.color565(200, 240, 255);      // ฟองฟ้าอ่อน
    uint16_t bubbleOutline = tft.color565(150, 200, 255);    // ขอบฟอง

    tft.fillScreen(frameColor);
    tft.fillRect(0, 0, 320, 480, backgroundColor);

    // ฟองสบู่
    int bubbleX[] = {40, 90, 140, 200, 260, 300, 70, 130, 190, 240, 20, 280};
    int bubbleY[] = {100, 50, 80, 30, 60, 110, 200, 250, 300, 350, 400, 420};
    int bubbleR[] = {15, 10, 12, 8, 10, 14, 20, 16, 12, 18, 10, 13};
    for (int i = 0; i < 12; i++) {
      tft.fillCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleColor);
      tft.drawCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleOutline);
    }

    // หัวข้อ
    tft.setTextColor(textColor, backgroundColor);
    tft.setTextFont(4);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Enter points", 80, 40);

    // ช่องกรอกแต้ม (ล่าง)
    tft.fillRect(160, 10, 150, 50, TFT_BLACK);
    tft.drawRect(160, 10, 150, 50, textColor);

    // แสดง Keypad เหมือนเดิม
    drawKeypad();

    // เรียก API เพื่อใช้เบอร์โทร
    fetchPointsFromServer(phoneNumber);

    // ล้าง buffer การแสดงผล
    updateDisplay("");
  }
}




// ตัวแปร global (อยู่นอกฟังก์ชัน)
bool skip = false;

unsigned long lastPressTime = 0;
void clearKeypad() {
  // สีพื้นหลังและฟองสบู่ (ปรับให้ตรงกับธีมหลัก)
  uint16_t backgroundColor = tft.color565(230, 230, 255);
  uint16_t bubbleColor = tft.color565(200, 240, 255);
  uint16_t bubbleOutline = tft.color565(150, 200, 255);

  // เคลียร์เฉพาะบริเวณ keypad ด้านล่างของจอ (เริ่มจาก y = 40)
  tft.fillRect(0, 40, 320, 440, backgroundColor);  // จาก y=40 ถึงล่างสุด (480)

  // วาดฟองสบู่แบบสุ่ม (หรือกำหนดตายตัว)
  int bubbleX[] = {40, 90, 140, 200, 260, 300, 70, 130, 190, 240, 20, 280};
  int bubbleY[] = {60, 100, 150, 180, 220, 260, 300, 340, 380, 420, 440, 460};
  int bubbleR[] = {15, 10, 12, 8, 10, 14, 20, 16, 12, 18, 10, 13};

  for (int i = 0; i < 12; i++) {
    tft.fillCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleColor);
    tft.drawCircle(bubbleX[i], bubbleY[i], bubbleR[i], bubbleOutline);
  }
}

enum Screen {
  NONE,
  PAYMENT_METHOD_SELECTION,
  // ... เพิ่มหน้าจออื่น ๆ ตามต้องการ
};





Screen currentScreen = NONE;

/////////////////////////////////////////////////////////////

void handlePaymentMethodSelection() {
  static bool cancelPressed = false;
  currentScreen = PAYMENT_METHOD_SELECTION;

  const int btnW = 120;
  const int btnH = 50;
  const int btnX = 150;
  const int btnY1 = 100;
  const int btnY2 = 185;
  const int btnY3 = 270;

  uint16_t backgroundColor = tft.color565(230, 230, 255);
  uint16_t frameColor = tft.color565(140, 110, 190);
  uint16_t bubbleColor = tft.color565(200, 240, 255);
  uint16_t bubbleOutline = tft.color565(150, 200, 255);
  uint16_t textColor = tft.color565(70, 30, 120);

  auto redrawBubbleBackgroundArea = [&](int x, int y, int w, int h) {
    tft.fillRect(x, y, w, h, backgroundColor);
    int bubbleX[] = {40, 90, 140, 200, 260, 300, 70, 130, 190, 240, 20, 280};
    int bubbleY[] = {100, 50, 80, 30, 60, 110, 200, 250, 300, 350, 400, 420};
    int bubbleR[] = {15, 10, 12, 8, 10, 14, 20, 16, 12, 18, 10, 13};
    for (int i = 0; i < 12; i++) {
      int bx = bubbleX[i], by = bubbleY[i], br = bubbleR[i];
      if (bx + br >= x && bx - br <= x + w && by + br >= y && by - br <= y + h) {
        tft.fillCircle(bx, by, br, bubbleColor);
        tft.drawCircle(bx, by, br, bubbleOutline);
      }
    }
  };

  if (!buttonsDrawn) {
    clearKeypad();
    cancelPressed = false;

    tft.fillScreen(frameColor);
    tft.fillRect(0, 0, 320, 480, backgroundColor);
    tft.fillRect(0, 60, 320, 150, backgroundColor);
    redrawBubbleBackgroundArea(0, 0, 320, 480);

    tft.setTextColor(textColor, backgroundColor);
    tft.setTextFont(4);
    tft.setCursor(10, 20);
    tft.print("Choose payment method");

    key[11].initButton(&tft, btnX, btnY1, btnW, btnH, TFT_WHITE, TFT_GREEN, TFT_WHITE, "Scan", 1);
    key[11].drawButton();
    key[10].initButton(&tft, btnX, btnY2, btnW, btnH, TFT_WHITE, TFT_BLUE, TFT_WHITE, "Cash", 1);
    key[10].drawButton();
    key[9].initButton(&tft, btnX, btnY3, btnW, btnH, TFT_WHITE, TFT_RED, TFT_WHITE, "Cancel", 1);
    key[9].drawButton();

    playAudio(5);
    buttonsDrawn = true;
  }

  uint16_t x, y;
  bool pressed = tft.getTouch(&x, &y);

  if (pressed && (millis() - lastPressTime > 100)) {
    lastPressTime = millis();

    if (!paymentMethodSelected) {
      int points = atoi(pointsBuffer);
      creditValue = atoi(creditBuffer);  // อัปเดต creditValue จาก creditBuffer (ไม่ต้องประกาศใหม่)

      if (key[11].contains(x, y)) {  // Scan
        paymentMethodSelected = true;
        isPaymentMethodScan = true;
        selectedPaymentMethod = "Scan";
        skip = false;

        Serial.println("======= Sending Transaction Data [SCAN] =======");
        Serial.print("Phone Number: "); Serial.println(phoneNumber);
        Serial.print("Amount: "); Serial.println(numberBuffer);
        Serial.print("Points: "); Serial.println(points);
        Serial.print("isScanPayment: "); Serial.println(isPaymentMethodScan ? "true" : "false");
        Serial.print("skip: "); Serial.println(skip ? "true" : "false");
        Serial.print("creditBuffer: "); Serial.println(creditBuffer);
        Serial.print("creditValue: "); Serial.println(creditValue);
        Serial.println("===============================================");

        redrawBubbleBackgroundArea(btnX - btnW / 2, btnY1 - btnH / 2, btnW, btnH);
        redrawBubbleBackgroundArea(btnX - btnW / 2, btnY2 - btnH / 2, btnW, btnH);
        redrawBubbleBackgroundArea(btnX - btnW / 2, btnY3 - btnH / 2, btnW, btnH);
        redrawBubbleBackgroundArea(0, 60, 320, 100);

        sendTransactionData(phoneNumber, numberBuffer, points, isPaymentMethodScan, skip, creditValue);
        buttonsDrawn = false;

      } else if (key[10].contains(x, y)) {  // Cash
        paymentMethodSelected = true;
        isPaymentMethodScan = false;
        selectedPaymentMethod = "Cash";
        skip = false;

        lastAmount = atoi(numberBuffer);
        currentCredit = 0;

        Serial.println("======= Sending Transaction Data [CASH] =======");
        Serial.print("Phone Number: "); Serial.println(phoneNumber);
        Serial.print("Amount: "); Serial.println(numberBuffer);
        Serial.print("Points: "); Serial.println(points);
        Serial.print("isScanPayment: "); Serial.println(isPaymentMethodScan ? "true" : "false");
        Serial.print("skip: "); Serial.println(skip ? "true" : "false");
        Serial.print("creditBuffer: "); Serial.println(creditBuffer);
        Serial.print("creditValue: "); Serial.println(creditValue);
        Serial.println("===============================================");

        redrawBubbleBackgroundArea(btnX - btnW / 2, btnY1 - btnH / 2, btnW, btnH);
        redrawBubbleBackgroundArea(btnX - btnW / 2, btnY2 - btnH / 2, btnW, btnH);
        redrawBubbleBackgroundArea(btnX - btnW / 2, btnY3 - btnH / 2, btnW, btnH);
        redrawBubbleBackgroundArea(0, 60, 320, 100);

        sendTransactionData(phoneNumber, numberBuffer, points, isPaymentMethodScan, skip, creditValue);
        buttonsDrawn = false;

        if (lastAmount > 0) {
          tft.setTextColor(TFT_BLACK, backgroundColor);
          tft.setTextFont(4);
          tft.setCursor(40, 70);
          tft.print("Last Amount: ");
          tft.print(lastAmount);
          tft.print(" Baht");

          if (currentScreen == PAYMENT_METHOD_SELECTION) {
            showCurrentCredit();
          }

          if (currentCredit == lastAmount) {
            delay(500);
            showThankYouAndReset();
          }
        } else {
          redrawBubbleBackgroundArea(0, 60, 320, 100);
        }

      } else if (!cancelPressed && key[9].contains(x, y)) {  // Cancel
        cancelPressed = true;
        tft.fillScreen(TFT_BLACK);
        buttonsDrawn = false;
        handleCancel();
      }
    }
  }
}





void checkCreditFromArduino() {
  static String inputLine = "";
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n' || c == '\r') {
      inputLine.trim();
      if (inputLine.length() > 0 && inputLine.startsWith("credit:")) {
        String creditStr = inputLine.substring(7);
        int creditVal = creditStr.toInt();
        currentCredit = creditVal;
        Serial.println("Updated currentCredit: " + String(currentCredit));
      }
      inputLine = "";
    } else {
      inputLine += c;
    }
  }
}


void showCurrentCredit() {
  // สีพื้นหลังฟองสบู่ (สีฟ้าอ่อนที่คุณใช้ในธีม)
  uint16_t backgroundColor = tft.color565(230, 230, 255);  // ฟ้าพาสเทล
  uint16_t textColor = tft.color565(70, 30, 120);          // ม่วงเข้ม

  // เคลียร์พื้นก่อนเขียนใหม่
  tft.fillRect(60, 270, 250, 30, backgroundColor);  // ล้างพื้นที่แสดงเครดิต

  tft.setCursor(10, 270);
  tft.setTextColor(textColor, backgroundColor);    // ตัวหนังสือสีม่วงเข้ม บนพื้นฟ้าพาสเทล
  tft.setTextFont(4);
  tft.print("Current Credit: ");
  tft.print(currentCredit);
}



void resetPaymentSelection() {
  paymentMethodSelected = false;
  buttonsDrawn = false;
  skip = false;
  lastPressTime = 0;
}







// ฟังก์ชันส่งข้อมูลการทำรายการไปยังเซิร์ฟเวอร์
String generateUUID()
{
  char uuid[33]; // 32 chars + null terminator
  for (int i = 0; i < 32; i++) {
    int r = esp_random() % 16;
    uuid[i] = "0123456789abcdef"[r];
  }
  uuid[32] = '\0';
  return String(uuid);
}

void sendTransactionData(const char* phoneNumber, const char* amount, int points, bool isScanPayment, bool isSkip, int credit)
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Disconnected");
    return;
  }

  HTTPClient http;
  http.begin("https://172.20.10.6:3000/save-transaction");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-KEY", "mysecretapikey12345");

  // กำหนด paymentMethod ตามเงื่อนไข
  String paymentMethod = isSkip ? "skip" : (isScanPayment ? "Scan" : "Cash");

  bool hasPhone = strlen(phoneNumber) > 0;
  String user_id = hasPhone ? String(phoneNumber) : generateUUID();

  // ตรวจสอบ amount ถ้าว่าง ให้ตั้งเป็น "0"
  String sendAmount = (strlen(amount) == 0) ? "0" : String(amount);

  // สร้าง JSON object สำหรับส่งข้อมูล
  DynamicJsonDocument doc(512);
  doc["user_id"] = user_id;
  doc["amount"] = sendAmount;   // ใช้ค่า 0 ถ้าว่าง
  doc["points"] = points;
  doc["paymentMethod"] = paymentMethod;
  doc["credit"] = credit;       // ส่งค่าเครดิตเพิ่มเข้าไป

  // แปลง JSON เป็น String
  String requestBody;
  serializeJson(doc, requestBody);

  // ส่ง POST request
  int httpResponseCode = http.POST(requestBody);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Raw server response (POST): " + response);

    DynamicJsonDocument resDoc(1024);
    DeserializationError error = deserializeJson(resDoc, response);

    if (!error) {
      Serial.println("-------- Server Response --------");
      Serial.println("Message: " + String(resDoc["message"].as<const char*>()));
      Serial.println("Payment Method: " + String(resDoc["paymentMethod"].as<const char*>()));
      Serial.println("Amount: " + String(resDoc["amount"].as<const char*>()));

      if (resDoc.containsKey("last_amount")) {
        if (resDoc["last_amount"].is<int>()) {
          lastAmount = resDoc["last_amount"].as<int>();
        } else if (resDoc["last_amount"].is<const char*>()) {
          lastAmount = atoi(resDoc["last_amount"].as<const char*>());
        }
      }

      Serial.println("Total Points: " + String(resDoc["total_points"].as<int>()));

      if (resDoc.containsKey("user_id")) {
        Serial.println("User ID: " + String(resDoc["user_id"].as<const char*>()));
      }

      if (resDoc.containsKey("activityLog") && resDoc["activityLog"].is<JsonArray>()) {
        JsonArray logs = resDoc["activityLog"].as<JsonArray>();
        for (JsonObject log : logs) {
          Serial.println("Activity Log:");
          Serial.println(" - Type: " + String(log["type"].as<const char*>()));
          Serial.println(" - Points: " + String(log["points"].as<int>()));
          Serial.println(" - Time: " + String(log["timestamp"].as<const char*>()));
          Serial.println(" - Note: " + String(log["note"].as<const char*>()));
        }
      }
      Serial.println("---------------------------------");
    } else {
      Serial.print("JSON Parse Error (POST): ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.println("Error sending data: " + String(httpResponseCode));
  }

  http.end();
}



void sendCreditData(const char* phoneNumber, int credit) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Disconnected");
    return;
  }

  HTTPClient http;
  http.begin("https://172.20.10.6:3000/save-credit");  // endpoint สำหรับเครดิต
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-KEY", "mysecretapikey12345");

  // ตรวจสอบเบอร์โทร ถ้าว่างให้ใช้ UUID
  bool hasPhone = strlen(phoneNumber) > 0;
  String user_id = hasPhone ? String(phoneNumber) : generateUUID();

  // สร้าง JSON object สำหรับส่งข้อมูล (มีแค่ user_id + credit)
  DynamicJsonDocument doc(256);
  doc["user_id"] = user_id;   // ใช้ user_id ที่ตรวจสอบแล้ว
  doc["credit"] = credit;     // ส่ง credit ตาม parameter

  // แปลง JSON เป็น String
  String requestBody;
  serializeJson(doc, requestBody);

  // แสดงข้อมูลที่จะส่งออกไป
  Serial.println("-------- Sending Request --------");
  Serial.println("URL: https://172.20.10.6:3000/save-credit");
  Serial.println("Headers: Content-Type=application/json, X-API-KEY=mysecretapikey12345");
  Serial.println("Request Body: " + requestBody);
  Serial.println("---------------------------------");

  // ส่ง POST request
  int httpResponseCode = http.POST(requestBody);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Raw server response (POST): " + response);

    DynamicJsonDocument resDoc(512);
    DeserializationError error = deserializeJson(resDoc, response);

    if (!error) {
      Serial.println("-------- Server Response --------");
      if (resDoc.containsKey("message")) {
        Serial.println("Message: " + String(resDoc["message"].as<const char*>()));
      }
      if (resDoc.containsKey("finally_amount")) {   // 🔹 เปลี่ยนจาก amount/credit
        float finalAmount = resDoc["finally_amount"].as<float>();
        Serial.println("Amount (net): " + String(finalAmount));
        lastAmount = finalAmount;  // อัปเดตตัวแปร global ใช้ต่อ
      }
      if (resDoc.containsKey("esp32_response")) {
        Serial.println("ESP32 Response: " + String(resDoc["esp32_response"].as<const char*>()));
      }
      Serial.println("---------------------------------");
    } else {
      Serial.print("JSON Parse Error (POST): ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.println("Error sending credit data: " + String(httpResponseCode));
  }

  http.end();
}



// ฟังก์ชันนี้จะถูกเรียกใช้เมื่อ ESP32 ได้รับข้อมูลเคดิตจาก Arduino
void addcredit(String userId, float credit) {
  HTTPClient http;

  // URL สำหรับเพิ่มเครดิต
  String serverUrl = "https://172.20.10.6:3000/add-credit";

  http.begin(serverUrl);
  http.addHeader("Content-Type", "application/json");

  // เพิ่ม API Key
  http.addHeader("X-API-KEY", "mysecretapikey12345");

  // สร้าง JSON payload ส่ง user_id กับ credit
  String jsonPayload = "{\"user_id\":\"" + userId + "\", \"credit\":" + String(credit, 2) + "}";

  int httpResponseCode = http.POST(jsonPayload);

  if (httpResponseCode == 200) {
    String response = http.getString();
    Serial.println("Add Credit Success: " + response);
  } else {
    Serial.println("Error in POST request. HTTP Response code: " + String(httpResponseCode));
  }

  http.end();
}



void resetInput()
{
    phoneNumberIndex = 0;
    numberIndex = 0;
    pointsIndex = 0;
    creditIndex = 0;
    memset(creditBuffer, 0, sizeof(creditBuffer));

    isPhoneNumberEntered = false;
    isAmountScreenShown = false;
    isAmountEntered = false;
    isPointsEntered = false;
    isPaymentMethodSelected = false;
    isPaymentMethodScan = false;
    skip = false;

    hasPlayedPhoneStartAudio = false;
    hasPlayedAmountAudio = false;
    hasPlayedPointsAudio = false;

    hasShownThankYou = false;

    lastAmount = 0;
    currentCredit = 0;

    memset(phoneNumber, 0, sizeof(phoneNumber));
    memset(numberBuffer, 0, sizeof(numberBuffer));
    memset(pointsBuffer, 0, sizeof(pointsBuffer));

    isAtStartScreen = true;

    tft.fillScreen(TFT_BLACK);
    showStartScreen();
    updateDisplay("");

    buttonsDrawn = false;
}


void updateDisplay(const char *text)
{
  // ล้างพื้นหลังกล่องแสดงเบอร์
  tft.fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // ตัวอักษรขาว พื้นหลังดำ
  tft.setTextFont(4);                      // ขนาดฟอนต์ใหญ่พอสมควร
  tft.setTextDatum(TL_DATUM);              // ยึดมุมบนซ้ายเป็นจุดเริ่ม
  tft.setCursor(DISP_X + 10, DISP_Y + 10); // ขยับเข้ามานิดหน่อย
  tft.print(text);                         // แสดงข้อความ (เบอร์โทร)
}


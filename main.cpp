#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_sleep.h"

// Cài đặt OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define OLED_SDA 10
#define OLED_SCL 1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Khai báo các chân
#define RAIN_ANALOG 19
#define RAIN_DIGITAL 5
#define BUTTON_PIN 4
#define POT_PIN 0
#define STEP_PIN 2
#define DIR_PIN 3
#define MOTOR_EN_PIN 8

long totalRoundX = 3;
long pulsePerRoundX = 200;
float defaultSpeedX = 60;

bool rainSensorEnabled = true;
bool lastRainState = HIGH;
volatile bool buttonPressed = false;

unsigned long lastRainCheckTime = 0;  // Thời gian kiểm tra cảm biến mưa lần cuối
const unsigned long rainCheckInterval = 1000;  // Thời gian kiểm tra cảm biến mưa (1 giây)

void setup() {
  Serial.begin(9600);
  delay(1000);

  // Khởi tạo I2C
  Wire.begin(OLED_SDA, OLED_SCL);

  // Kiểm tra OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED initialization failed"));
    for (;;);
  }
  display.display();
  delay(100);
  display.clearDisplay();

  // Khởi tạo chân
  pinMode(RAIN_ANALOG, INPUT);
  pinMode(RAIN_DIGITAL, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MOTOR_EN_PIN, OUTPUT);

  digitalWrite(MOTOR_EN_PIN, LOW); // Kích hoạt động cơ

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);

  // Hiển thị trạng thái khởi tạo
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Rain Sensor Initialized");
  display.display();
  delay(2000);
  display.clearDisplay();

  Serial.println("System initialized.");
}

void loop() {
  unsigned long currentMillis = millis();

   
  // Xử lý nút bấm
  if (buttonPressed) {
    buttonPressed = false;
    rainSensorEnabled = !rainSensorEnabled;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Rain Sensor: ");
    display.println(rainSensorEnabled ? "ON" : "OFF");
    display.display();

    Serial.println(rainSensorEnabled ? "Rain sensor ON" : "Rain sensor OFF");
    moveX(totalRoundX, rainSensorEnabled ? LOW : HIGH, defaultSpeedX);
    if (rainSensorEnabled){
      enterDeepSleep();
    }
  }

  // Kiểm tra cảm biến mưa mỗi 1 giây
  if (rainSensorEnabled && (currentMillis - lastRainCheckTime >= rainCheckInterval)) {
    lastRainCheckTime = currentMillis;  // Cập nhật thời gian kiểm tra

    bool currentRainState = digitalRead(RAIN_DIGITAL);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.print("Rain Sensor: ");
    display.println(rainSensorEnabled ? "ON" : "OFF");

    if (currentRainState == LOW) {
      display.setCursor(0, 20);
      display.println("Rain Detected!");
      display.setCursor(0, 40);
      display.print("Action: Closing...");
    } else {
      display.setCursor(0, 20);
      display.println("No Rain Detected.");
      display.setCursor(0, 40);
      display.print("Action: Opening...");
    }
    display.display();

    if (currentRainState != lastRainState) {
      if (currentRainState == LOW) {
        moveX(totalRoundX, HIGH, defaultSpeedX);
      } else {
        moveX(totalRoundX, LOW, defaultSpeedX);
        delay(500); // Đảm bảo động cơ có đủ thời gian để chuyển động
        enterDeepSleep();  
      }
      lastRainState = currentRainState;
    }

    // Nếu trời không mưa, vào chế độ deep sleep
    // if (currentRainState == HIGH) {
    //   enterDeepSleep();
    // }
  }

  delay(100);  // Giới hạn độ trễ tổng thể để tránh tắc nghẽn
}

// Hàm xử lý nút bấm
void onButtonPress() {
  buttonPressed = true;
}

// Hàm điều khiển động cơ
void moveX(long totalRound, bool dir, float speed_) {
  long stepsCount = 0;
  long totalPulse = totalRound * pulsePerRoundX;
  float period = 1000000.0 / (speed_ / 60.0 * pulsePerRoundX);

  digitalWrite(DIR_PIN, dir);

  while (stepsCount < totalPulse) {
    stepsCount++;
    int analogValue = analogRead(POT_PIN);
    speed_ = map(analogValue, 0, 4095, 20, 240);
    period = 1000000.0 / (speed_ / 60.0 * pulsePerRoundX);

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(0.2 * period);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(0.8 * period);
  }
}

// Hàm vào chế độ Deep Sleep
void enterDeepSleep() {
  Serial.println("Entering Deep Sleep...");
  display.clearDisplay();
  display.setCursor(0, 20);
  display.println("Entering Deep Sleep...");
  display.display();
  delay(1000); // Hiển thị thông báo trước khi đi ngủ


  
  // Tắt màn hình OLED
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  esp_deep_sleep_enable_gpio_wakeup(1 << RAIN_DIGITAL, ESP_GPIO_WAKEUP_GPIO_LOW);  // Wake up on rain sensor LOW state
  
  esp_deep_sleep_enable_gpio_wakeup(1 << BUTTON_PIN , ESP_GPIO_WAKEUP_GPIO_LOW);
  
  
  esp_deep_sleep_start();
  display.println("Entering Deep Sleepzsfscxzs...");
}

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// =====================================================
// ADC
// =====================================================

const int pinMinus = A0;
const int pinPlus  = A1;

const float VREF = 5.0;
const float ADC_MAX = 16383.0;


// =====================================================
// Thresholds
// =====================================================

// Press触发阈值
const float PRESS_THRESHOLD = 0.30;

// Release触发阈值
const float RELEASE_THRESHOLD = 0.30;

// 零点附近
const float ZERO_THRESHOLD = 0.10;

// LCD显示死区
const float DISPLAY_DEADBAND = 0.05;


// =====================================================
// Timing
// =====================================================

// Press以后至少等待20 ms
// 避免同一个Press瞬态振铃被误认为Release
const unsigned long MIN_RELEASE_DELAY = 20;

// Release峰值捕捉窗口
const unsigned long RELEASE_WINDOW = 60;

// Result保持5秒
const unsigned long HOLD_TIME = 5000;

// LCD刷新
const unsigned long DISPLAY_TIME = 50;


// =====================================================
// States
// =====================================================

enum State
{
  WAITING,
  PRESSED,
  CAPTURE_RELEASE,
  HOLDING
};

State state = WAITING;


// =====================================================
// Signal variables
// =====================================================

float zeroOffset = 0.0;

float signedDiff = 0.0;

float displayDiff = 0.0;

float pressPeak = 0.0;

float releasePeak = 0.0;

// Press方向
// +1 = 正方向
// -1 = 负方向
int pressSign = 0;


// =====================================================
// Result阶段重新触发控制
// =====================================================

// Release完成以后，必须先回到零点附近
// 才允许识别下一次Press
bool readyForNewPress = true;


// =====================================================
// Time variables
// =====================================================

unsigned long pressStartTime = 0;

unsigned long releaseStartTime = 0;

unsigned long holdStartTime = 0;

unsigned long lastDisplayTime = 0;


// =====================================================
// Start new event
// =====================================================

void startNewEvent()
{
  // 自动判断本次Press方向
  if (signedDiff >= 0.0)
  {
    pressSign = 1;
  }
  else
  {
    pressSign = -1;
  }

  pressPeak = fabs(signedDiff);

  releasePeak = 0.0;

  pressStartTime = millis();

  readyForNewPress = false;

  state = PRESSED;
}


// =====================================================
// Setup
// =====================================================

void setup()
{
  Serial.begin(115200);

  analogReadResolution(14);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");


  // ===================================================
  // Startup calibration
  // 校准时不要触碰传感器
  // ===================================================

  float sum = 0.0;

  for (int i = 0; i < 300; i++)
  {
    int rawMinus = analogRead(pinMinus);
    int rawPlus  = analogRead(pinPlus);

    float vMinus =
      rawMinus * VREF / ADC_MAX;

    float vPlus =
      rawPlus * VREF / ADC_MAX;

    sum += (vPlus - vMinus);

    delay(3);
  }

  zeroOffset = sum / 300.0;

  lcd.clear();
}


// =====================================================
// Main loop
// =====================================================

void loop()
{
  // ===================================================
  // 1. Read ADC
  // ===================================================

  int rawMinus = analogRead(pinMinus);
  int rawPlus  = analogRead(pinPlus);

  float vMinus =
    rawMinus * VREF / ADC_MAX;

  float vPlus =
    rawPlus * VREF / ADC_MAX;


  // ===================================================
  // 2. Signed differential voltage
  // ===================================================

  signedDiff =
    (vPlus - vMinus) - zeroOffset;


  // ===================================================
  // 3. LCD deadband
  // ===================================================

  if (fabs(signedDiff) < DISPLAY_DEADBAND)
  {
    displayDiff = 0.0;
  }
  else
  {
    displayDiff = signedDiff;
  }


  // ===================================================
  // 4. State machine
  // ===================================================

  switch (state)
  {

    // =================================================
    // WAITING
    // =================================================

    case WAITING:

      if (fabs(signedDiff) >= PRESS_THRESHOLD)
      {
        startNewEvent();
      }

      break;


    // =================================================
    // PRESSED
    //
    // 已经检测到Press
    // 持续更新Press Peak
    // 同时直接等待相反方向Release
    // =================================================

    case PRESSED:
    {
      float normalized =
        signedDiff * pressSign;


      // -----------------------------------------------
      // 更新Press Peak
      // -----------------------------------------------

      if (normalized > pressPeak)
      {
        pressPeak = normalized;
      }


      // -----------------------------------------------
      // 检测Release
      //
      // Release方向必须与Press相反
      // -----------------------------------------------

      if (
        millis() - pressStartTime >= MIN_RELEASE_DELAY &&
        normalized <= -RELEASE_THRESHOLD
      )
      {
        releasePeak = -normalized;

        releaseStartTime = millis();

        state = CAPTURE_RELEASE;
      }

      break;
    }


    // =================================================
    // CAPTURE_RELEASE
    //
    // Release一出现就立即有数值
    // 再用60ms寻找真正的Release Peak
    // =================================================

    case CAPTURE_RELEASE:
    {
      float normalized =
        signedDiff * pressSign;


      // -----------------------------------------------
      // 更新Release Peak
      // -----------------------------------------------

      if (normalized < 0.0)
      {
        float releaseValue =
          -normalized;

        if (releaseValue > releasePeak)
        {
          releasePeak = releaseValue;
        }
      }


      // -----------------------------------------------
      // 60 ms后直接进入Result
      // 不再等待信号长时间稳定
      // -----------------------------------------------

      if (
        millis() - releaseStartTime >= RELEASE_WINDOW
      )
      {
        holdStartTime = millis();

        readyForNewPress = false;

        state = HOLDING;
      }

      break;
    }


    // =================================================
    // HOLDING
    //
    // 显示Press和Release结果
    // =================================================

    case HOLDING:

      // -----------------------------------------------
      // Release尾部先回到零点附近
      // -----------------------------------------------

      if (!readyForNewPress)
      {
        if (fabs(signedDiff) <= ZERO_THRESHOLD)
        {
          readyForNewPress = true;
        }
      }


      // -----------------------------------------------
      // 5秒内再次Press
      // 立即开始新的事件
      // -----------------------------------------------

      if (
        readyForNewPress &&
        fabs(signedDiff) >= PRESS_THRESHOLD
      )
      {
        startNewEvent();
      }


      // -----------------------------------------------
      // 5秒内没有新的Press
      // 清除结果
      // -----------------------------------------------

      else if (
        millis() - holdStartTime >= HOLD_TIME
      )
      {
        pressPeak = 0.0;

        releasePeak = 0.0;

        pressSign = 0;

        readyForNewPress = true;

        state = WAITING;
      }

      break;
  }


  // ===================================================
  // 5. LCD
  // ===================================================

  if (
    millis() - lastDisplayTime >= DISPLAY_TIME
  )
  {
    lastDisplayTime = millis();


    // =================================================
    // Line 1
    // Diff
    // =================================================

    lcd.setCursor(0, 0);

    lcd.print("Diff:");

    if (displayDiff > 0.0)
    {
      lcd.print("+");
    }
    else if (displayDiff < 0.0)
    {
      lcd.print("-");
    }
    else
    {
      lcd.print(" ");
    }

    lcd.print(fabs(displayDiff), 3);

    lcd.print(" V       ");


    // =================================================
    // Line 2
    // Press
    // =================================================

    lcd.setCursor(0, 1);

    lcd.print("Press:");

    lcd.print(pressPeak, 3);

    lcd.print(" V      ");


    // =================================================
    // Line 3
    // Release
    // =================================================

    lcd.setCursor(0, 2);

    lcd.print("Release:");

    lcd.print(releasePeak, 3);

    lcd.print(" V    ");


    // =================================================
    // Line 4
    // State
    // =================================================

    lcd.setCursor(0, 3);

    if (state == WAITING)
    {
      lcd.print("State: Waiting      ");
    }

    else if (state == PRESSED)
    {
      lcd.print("State: Pressed      ");
    }

    else if (state == CAPTURE_RELEASE)
    {
      lcd.print("State: Release      ");
    }

    else
    {
      lcd.print("State: Result       ");
    }


    // =================================================
    // Serial
    // =================================================

    Serial.print("Diff:");
    Serial.print(signedDiff, 3);

    Serial.print(",Press:");
    Serial.print(pressPeak, 3);

    Serial.print(",Release:");
    Serial.print(releasePeak, 3);

    Serial.print(",State:");

    if (state == WAITING)
    {
      Serial.println("WAITING");
    }

    else if (state == PRESSED)
    {
      Serial.println("PRESSED");
    }

    else if (state == CAPTURE_RELEASE)
    {
      Serial.println("RELEASE");
    }

    else
    {
      Serial.println("RESULT");
    }
  }
}
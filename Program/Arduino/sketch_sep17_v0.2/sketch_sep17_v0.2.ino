#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

const int pinMinus = A0;
const int pinPlus  = A1;

const float VREF = 5.0;

// 开始事件
const float START_THRESHOLD = 0.3;

// 结束事件
const float END_THRESHOLD = 0.10;

// 低于结束阈值持续300 ms
const unsigned long END_TIME = 150;

// 结果保持5秒
const unsigned long HOLD_TIME = 5000;

// LCD刷新周期
const unsigned long DISPLAY_TIME = 200;

// 低通滤波系数
// 越小越平滑
const float ALPHA = 0.1;


enum State
{
  WAITING,
  CAPTURING,
  HOLDING
};

State state = WAITING;


// 静态零点
float zeroOffset = 0.0;

// 原始实时差值
float diffValue = 0.0;

// 滤波后的差值
float filteredDiff = 0.0;

// 本次事件最大原始差值
float maxDiff = 0.0;


// 时间
unsigned long belowThresholdStart = 0;
unsigned long holdStartTime = 0;
unsigned long lastDisplayTime = 0;


void setup()
{
  Serial.begin(115200);

  analogReadResolution(14);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");


  // =========================
  // 开机零点校准
  // =========================

  float sum = 0.0;

  for (int i = 0; i < 200; i++)
  {
    int rawMinus = analogRead(pinMinus);
    int rawPlus  = analogRead(pinPlus);

    float vMinus = rawMinus * VREF / 16383.0;
    float vPlus  = rawPlus  * VREF / 16383.0;

    sum += (vPlus - vMinus);

    delay(5);
  }

  zeroOffset = sum / 200.0;

  filteredDiff = 0.0;

  lcd.clear();
}


void loop()
{
  // =====================================
  // 1. 读取A0、A1
  // =====================================

  int rawMinus = analogRead(pinMinus);
  int rawPlus  = analogRead(pinPlus);

  float vMinus = rawMinus * VREF / 16383.0;
  float vPlus  = rawPlus  * VREF / 16383.0;


  // =====================================
  // 2. 原始绝对差值
  // =====================================

  diffValue = fabs((vPlus - vMinus) - zeroOffset);


  // =====================================
  // 3. 对Diff进行低通滤波
  // 只用于状态判断
  // =====================================

  filteredDiff =
      ALPHA * diffValue
      + (1.0 - ALPHA) * filteredDiff;


  // =====================================
  // 4. 状态机
  // =====================================

  switch (state)
  {

    // ---------------------------------
    // 等待按压
    // ---------------------------------

    case WAITING:

      if (filteredDiff > START_THRESHOLD)
      {
        maxDiff = diffValue;

        belowThresholdStart = 0;

        state = CAPTURING;
      }

      break;


    // ---------------------------------
    // 正在测量
    // ---------------------------------

    case CAPTURING:

      // Max始终使用原始数据
      if (diffValue > maxDiff)
      {
        maxDiff = diffValue;
      }


      // 使用滤波值判断结束
      if (filteredDiff <= END_THRESHOLD)
      {
        // 第一次进入低于阈值区域
        if (belowThresholdStart == 0)
        {
          belowThresholdStart = millis();
        }

        // 连续300 ms低于阈值
        else if (millis() - belowThresholdStart >= END_TIME)
        {
          holdStartTime = millis();

          belowThresholdStart = 0;

          state = HOLDING;
        }
      }
      else
      {
        // 明显重新出现信号才重新计时
        belowThresholdStart = 0;
      }

      break;


    // ---------------------------------
    // 显示结果5秒
    // ---------------------------------

    case HOLDING:

      // 5秒内再次出现新的按压
      if (filteredDiff > START_THRESHOLD)
      {
        maxDiff = diffValue;

        belowThresholdStart = 0;

        state = CAPTURING;
      }

      // 没有新的按压
      else if (millis() - holdStartTime >= HOLD_TIME)
      {
        maxDiff = 0.0;

        state = WAITING;
      }

      break;
  }


  // =====================================
  // 5. LCD显示
  // =====================================

  if (millis() - lastDisplayTime >= DISPLAY_TIME)
  {
    lastDisplayTime = millis();


    lcd.setCursor(0, 0);
    lcd.print("Piezo Sensor        ");


    // 显示原始实时差值
    lcd.setCursor(0, 1);
    lcd.print("Diff: ");
    lcd.print(diffValue, 3);
    lcd.print(" V       ");


    // 最大差值
    lcd.setCursor(0, 2);
    lcd.print("Max : ");
    lcd.print(maxDiff, 3);
    lcd.print(" V       ");


    // 状态
    lcd.setCursor(0, 3);

    if (state == WAITING)
    {
      lcd.print("Waiting             ");
    }
    else if (state == CAPTURING)
    {
      lcd.print("Measuring           ");
    }
    else
    {
      lcd.print("Result              ");
    }


    // =====================================
    // Serial调试
    // =====================================

    Serial.print("Diff:");
    Serial.print(diffValue, 3);

    Serial.print(",Filtered:");
    Serial.print(filteredDiff, 3);

    Serial.print(",Max:");
    Serial.print(maxDiff, 3);

    Serial.print(",State:");

    if (state == WAITING)
      Serial.println("WAITING");

    else if (state == CAPTURING)
      Serial.println("CAPTURING");

    else
      Serial.println("HOLDING");
  }
}
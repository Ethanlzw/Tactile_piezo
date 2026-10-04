#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdint.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);
const int pinMinus = A0;
const int pinPlus = A1;
const float VREF = 5.0;
const float ALPHA = 0.02; // 保留原程序滤波系数
const unsigned long DISPLAY_TIME = 200; // 保留原程序刷新周期

// BEGIN PAIR LOGIC
// 原程序的通用捕峰器：不判断正负，只把连续两次事件分别保存。
struct PeakPair {
  enum State { WAITING, CAPTURING, HOLDING };
  State state = WAITING;
  static constexpr float START_THRESHOLD = 0.20f;
  static constexpr float END_THRESHOLD = 0.10f;
  static constexpr uint32_t END_TIME = 300;
  static constexpr uint32_t HOLD_TIME = 5000;
  // 初始调参值：有效按压峰须达到0.50 V；释放沿用原触发阈值。
  static constexpr float MIN_PRESS_PEAK = 0.30f;
  // 从确认第一峰开始计时；正常长按必须短于此值，0表示关闭超时。
  static constexpr uint32_t PAIR_TIMEOUT = 10000;
  float maxDiff = 0, pressPeak = 0, releasePeak = 0;
  bool nextIsPress = true, capturingPress = true;
  bool peakAccepted = false, waitForQuiet = false;
  bool peakSeparated = false, belowThreshold = false;
  uint32_t belowThresholdStart = 0, holdStartTime = 0, pairStartTime = 0;
  uint32_t rejectedPeaks = 0, expiredPairs = 0;

  constexpr void recordPeak(float raw, uint32_t now) {
    if (raw > maxDiff) maxDiff = raw;
    if (!peakAccepted) {
      if (capturingPress && maxDiff < MIN_PRESS_PEAK) return;
      peakAccepted = true;
      nextIsPress = !capturingPress; // 只有有效峰才推进配对顺序
      if (capturingPress) {
        releasePeak = 0;
        pairStartTime = now;
      }
    }
    if (capturingPress) pressPeak = maxDiff;
    else releasePeak = maxDiff;
  }

  constexpr void startPeak(float raw, uint32_t now) {
    capturingPress = nextIsPress;
    maxDiff = raw;
    peakAccepted = false;
    peakSeparated = belowThreshold = false;
    state = CAPTURING;
    recordPeak(raw, now);
  }

  constexpr void process(float raw, float filtered, uint32_t now) {
    if (waitForQuiet) {
      if (filtered <= END_THRESHOLD) waitForQuiet = false;
      return; // 超时后先等本次信号回落，不把残余信号立刻当成新峰
    }
    if (state != CAPTURING) {
      if (filtered > START_THRESHOLD) startPeak(raw, now);
      else if (state == HOLDING && nextIsPress && uint32_t(now - holdStartTime) >= HOLD_TIME) {
        pressPeak = releasePeak = maxDiff = 0;
        state = WAITING;
      }
      return;
    }

    // 已回落过，再次超过原来的开始阈值就是新事件。
    // 必须先分开事件，再更新最大值，防止新峰覆盖上一行。
    if (peakSeparated && filtered > START_THRESHOLD) {
      startPeak(raw, now);
      return;
    }
    if (!peakSeparated) {
      recordPeak(raw, now);
    }

    if (filtered <= END_THRESHOLD) {
      if (!peakAccepted) {
        ++rejectedPeaks;
        maxDiff = 0;
        // 无效第一峰不进入等释放，上一组已完成的结果仍可保留。
        state = pressPeak > 0 ? HOLDING : WAITING;
        peakSeparated = belowThreshold = false;
        return;
      }
      peakSeparated = true;
      if (!belowThreshold) {
        belowThreshold = true;
        belowThresholdStart = now;
      }
      if (uint32_t(now - belowThresholdStart) >= END_TIME) {
        holdStartTime = now;
        state = HOLDING;
      }
    } else belowThreshold = false;
  }

  constexpr void update(float raw, float filtered, uint32_t now) {
    // 当本次采样恰好出现有效第二峰时优先完成配对。
    process(raw, filtered, now);
    if (PAIR_TIMEOUT != 0 && !nextIsPress && uint32_t(now - pairStartTime) >= PAIR_TIMEOUT) {
      ++expiredPairs;
      pressPeak = releasePeak = maxDiff = 0;
      nextIsPress = capturingPress = true;
      peakAccepted = peakSeparated = belowThreshold = false;
      waitForQuiet = filtered > END_THRESHOLD;
      state = WAITING; // 超时只清除孤立峰，不生成释放结果
    }
  }
};
// END PAIR LOGIC

PeakPair peaks;
float zeroOffset = 0, diffValue = 0, filteredDiff = 0;
unsigned long lastDisplayTime = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(14);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  float sum = 0;
  // 与原程序相同：开机1秒内完全松开传感器。
  for (int i = 0; i < 200; ++i) {
    int rawMinus = analogRead(pinMinus);
    int rawPlus = analogRead(pinPlus);
    sum += (rawPlus - rawMinus) * VREF / 16383.0;
    delay(5);
  }
  zeroOffset = sum / 200.0;
  lcd.clear();
}

void loop() {
  int rawMinus = analogRead(pinMinus);
  int rawPlus = analogRead(pinPlus);
  float vMinus = rawMinus * VREF / 16383.0;
  float vPlus = rawPlus * VREF / 16383.0;
  diffValue = fabs((vPlus - vMinus) - zeroOffset);
  filteredDiff = ALPHA * diffValue + (1.0 - ALPHA) * filteredDiff;
  const unsigned long now = millis();
  peaks.update(diffValue, filteredDiff, now);

  if (now - lastDisplayTime >= DISPLAY_TIME) {
    lastDisplayTime = now;
    lcd.setCursor(0, 0);
    lcd.print("Diff: "); lcd.print(diffValue, 3); lcd.print(" V       ");
    lcd.setCursor(0, 1);
    lcd.print("Press: "); lcd.print(peaks.pressPeak, 3); lcd.print(" V      ");
    lcd.setCursor(0, 2);
    lcd.print("Release: "); lcd.print(peaks.releasePeak, 3); lcd.print(" V    ");
    lcd.setCursor(0, 3);
    if (peaks.waitForQuiet) lcd.print("Wait signal quiet   ");
    else if (peaks.state == PeakPair::WAITING) lcd.print("Waiting             ");
    else if (peaks.state == PeakPair::CAPTURING && !peaks.peakAccepted) lcd.print("Checking peak       ");
    else if (peaks.state == PeakPair::CAPTURING && !peaks.peakSeparated) {
      if (peaks.capturingPress) lcd.print("Measuring Press     ");
      else lcd.print("Measuring Release   ");
    } else if (!peaks.nextIsPress) lcd.print("Wait next peak      ");
    else lcd.print("Result              ");

    Serial.print("Diff:"); Serial.print(diffValue, 3);
    Serial.print(",Filtered:"); Serial.print(filteredDiff, 3);
    Serial.print(",Press:"); Serial.print(peaks.pressPeak, 3);
    Serial.print(",Release:"); Serial.print(peaks.releasePeak, 3);
    Serial.print(",Separated:"); Serial.print(peaks.peakSeparated);
    Serial.print(",Rejected:"); Serial.print(peaks.rejectedPeaks);
    Serial.print(",Expired:"); Serial.print(peaks.expiredPairs);
    Serial.print(",Next:"); Serial.print(peaks.nextIsPress ? "PRESS" : "RELEASE");
    Serial.print(",State:");
    if (peaks.state == PeakPair::WAITING) Serial.println("WAITING");
    else if (peaks.state == PeakPair::CAPTURING) Serial.println("CAPTURING");
    else Serial.println("HOLDING");
  }
}

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
  float START_THRESHOLD = 0.30f;
  float END_THRESHOLD = 0.10f;
  static constexpr uint32_t END_TIME = 300;
  static constexpr uint32_t HOLD_TIME = 5000;
  // 初始调参值：有效按压峰须达到0.50 V；释放沿用原触发阈值。
  float MIN_PRESS_PEAK = 0.50f;
  // 从确认第一峰开始计时；正常长按必须短于此值，0表示关闭超时。
  static constexpr uint32_t PAIR_TIMEOUT = 10000;
  float maxDiff = 0, pressPeak = 0, releasePeak = 0;
  bool nextIsPress = true, capturingPress = true;
  bool peakAccepted = false, waitForQuiet = true;
  bool peakSeparated = false, belowThreshold = false;
  uint32_t belowThresholdStart = 0, holdStartTime = 0, pairStartTime = 0;
  uint32_t rejectedPeaks = 0, expiredPairs = 0;
  unsigned candidateSamples = 0;
  float candidatePeak = 0;
  uint32_t candidateLastAt = 0, quietSince = 0;
  bool quietTiming = false;

  constexpr void clearCandidate() { candidateSamples = 0; candidatePeak = 0; }

  constexpr void observeCandidate(float raw, uint32_t now) {
    // 不跨越 LCD 写屏等长采样间隙拼接确认；单点毛刺不触发。
    if (candidateSamples != 0 && uint32_t(now - candidateLastAt) > 5) clearCandidate();
    if (raw > START_THRESHOLD) {
      candidateLastAt = now;
      if (candidateSamples < 3) ++candidateSamples;
      if (raw > candidatePeak) candidatePeak = raw;
    } else clearCandidate();
  }

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
    clearCandidate();
  }

  constexpr void process(float raw, float filtered, uint32_t now) {
    if (waitForQuiet) {
      if (filtered <= END_THRESHOLD && raw <= END_THRESHOLD) {
        if (!quietTiming) { quietTiming = true; quietSince = now; }
        if (uint32_t(now - quietSince) >= 300) {
          waitForQuiet = false;
          quietTiming = false;
        }
      } else quietTiming = false;
      return; // 超时后先等本次信号回落，不把残余信号立刻当成新峰
    }
    if (state != CAPTURING || peakSeparated) observeCandidate(raw, now);
    if (state != CAPTURING) {
      if (filtered > START_THRESHOLD && candidateSamples >= 3) startPeak(candidatePeak, now);
      else if (state == HOLDING && nextIsPress && uint32_t(now - holdStartTime) >= HOLD_TIME) {
        pressPeak = releasePeak = maxDiff = 0;
        state = WAITING;
      }
      return;
    }

    // 已回落过，再次超过原来的开始阈值就是新事件。
    // 必须先分开事件，再更新最大值，防止新峰覆盖上一行。
    if (peakSeparated && filtered > START_THRESHOLD && candidateSamples >= 3) {
      startPeak(candidatePeak, now);
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
      if (!peakSeparated) clearCandidate();
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
      waitForQuiet = true;
      quietTiming = false;
      clearCandidate();
      state = WAITING; // 超时只清除孤立峰，不生成释放结果
    }
  }
};
// END PAIR LOGIC

PeakPair peaks;
float zeroOffset = 0, diffValue = 0, filteredDiff = 0;
float noisePeak = 0, noiseSigma = 0, latestSigned = 0;
int latestMinus = 0, latestPlus = 0;
float previous1 = 0, previous2 = 0;
bool medianReady = false;
uint32_t lastSampleUs = 0, maxGapUs = 0;
unsigned long lastDisplayTime = 0;

float readMedianDifference() {
  latestMinus = analogRead(pinMinus);
  latestPlus = analogRead(pinPlus);
  latestSigned = (latestPlus - latestMinus) * VREF / 16383.0;
  // 对连续三个带符号差值取中值，再取绝对值，抑制孤立ADC毛刺。
  if (!medianReady) {
    previous1 = previous2 = latestSigned;
    medianReady = true;
  }
  float a = latestSigned, b = previous1, c = previous2, temp;
  if (a > b) { temp = a; a = b; b = temp; }
  if (b > c) { temp = b; b = c; c = temp; }
  if (a > b) { temp = a; a = b; b = temp; }
  previous2 = previous1;
  previous1 = latestSigned;
  return b;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(14);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  // 先等待电路初步稳定，再测量约2秒；校准期间完全松开。
  delay(500);
  float mean = 0, sumSquares = 0, low = 100, high = -100;
  for (unsigned n = 1; n <= 1000; ++n) {
    const float value = readMedianDifference();
    const float delta = value - mean;
    mean += delta / n;
    sumSquares += delta * (value - mean);
    low = fminf(low, value); high = fmaxf(high, value);
    delay(2);
  }
  zeroOffset = mean;
  noiseSigma = sqrtf(sumSquares / 999);
  noisePeak = fmaxf(high - mean, mean - low);
  peaks.START_THRESHOLD = fmaxf(0.30f, fmaxf(6 * noiseSigma, 1.25f * noisePeak + 0.02f));
  peaks.END_THRESHOLD = fmaxf(0.10f, fmaxf(3 * noiseSigma, noisePeak + 0.01f));
  peaks.MIN_PRESS_PEAK = fmaxf(0.50f, 1.25f * peaks.START_THRESHOLD);
  filteredDiff = 0;
  medianReady = false;
  lcd.clear();
}

void loop() {
  const uint32_t sampleAt = micros();
  if (lastSampleUs != 0) {
    const uint32_t gap = sampleAt - lastSampleUs;
    if (gap > maxGapUs) maxGapUs = gap;
    if (gap > 5000) medianReady = false;
  }
  lastSampleUs = sampleAt;
  diffValue = fabs(readMedianDifference() - zeroOffset);
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
    Serial.print(",Noise:"); Serial.print(noisePeak, 3);
    Serial.print(",Start:"); Serial.print(peaks.START_THRESHOLD, 3);
    Serial.print(",End:"); Serial.print(peaks.END_THRESHOLD, 3);
    Serial.print(",MinPress:"); Serial.print(peaks.MIN_PRESS_PEAK, 3);
    Serial.print(",A0:"); Serial.print(latestMinus);
    Serial.print(",A1:"); Serial.print(latestPlus);
    Serial.print(",Raw:"); Serial.print(latestSigned - zeroOffset, 3);
    Serial.print(",GapUs:"); Serial.print(maxGapUs);
    Serial.print(",Next:"); Serial.print(peaks.nextIsPress ? "PRESS" : "RELEASE");
    Serial.print(",State:");
    if (peaks.state == PeakPair::WAITING) Serial.println("WAITING");
    else if (peaks.state == PeakPair::CAPTURING) Serial.println("CAPTURING");
    else Serial.println("HOLDING");
  }
}

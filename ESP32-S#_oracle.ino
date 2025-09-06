// Advanced Curvature Analysis Oscilloscope for ESP32-S3
// Specialized for signal pattern analysis and sign flip detection

#define NUM_CHANNELS 6
#define SAMPLE_RATE_MS 20   // 50Hz sampling
#define HISTORY_SIZE 20     // Enough for sign flip window
#define DISPLAY_HEIGHT 12
#define DISPLAY_WIDTH 80
#define SMOOTHING_WINDOW 7
#define STATE_SPACE 999999

// !!!! UPDATE THESE TO MATCH YOUR ESP32-S3 PINOUT !!!!
const int analogPins[NUM_CHANNELS] = {1, 2, 3, 4, 5, 6}; // <--- Replace with ADC-capable GPIOs

float channelHistory[NUM_CHANNELS][HISTORY_SIZE];
float smoothedData[NUM_CHANNELS][HISTORY_SIZE];
float curvatureData[HISTORY_SIZE];
int historyIndex = 0;
unsigned long lastSample = 0;
int selectedChannel = 0;
unsigned long analysisCount = 0;

int flipCount = 0;

// For sign flip detection
float signFlips[HISTORY_SIZE];

void setup() {
  Serial.begin(115200);

  // Optional: Set attenuation (ADC_11db gives 0-3.3V range)
  #if defined(ARDUINO_ARCH_ESP32)
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    analogSetPinAttenuation(analogPins[ch], ADC_11db); // 0–3.3V
  }
  #endif

  // Initialize arrays
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    for (int i = 0; i < HISTORY_SIZE; i++) {
      channelHistory[ch][i] = 0.0;
      smoothedData[ch][i] = 0.0;
    }
  }
  for (int i = 0; i < HISTORY_SIZE; i++) {
    curvatureData[i] = 0.0;
    signFlips[i] = 0.0;
  }

  Serial.println("CURVATURE ANALYSIS OSCILLOSCOPE");
  Serial.println("================================");
  Serial.println("Commands: 0-5 (channel), r (reset), c (curvature stats)");
  Serial.println("Specialized for pattern analysis and sign flip detection");
  Serial.println("========================================================");
}

void loop() {
  unsigned long currentTime = millis();

  // Handle serial commands
  handleSerialCommands();

  if (currentTime - lastSample >= SAMPLE_RATE_MS) {
    // Sample all channels
    sampleAllChannels();

    // Apply moving average smoothing
    applySmoothingFilter();

    // Perform curvature analysis
    performCurvatureAnalysis();

    // Detect sign flips & rate of change
    detectSignFlips();

    // Display updated stats
    displayCurvatureOscilloscope();

    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    analysisCount++;
    lastSample = currentTime;
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd >= '0' && cmd < ('0' + NUM_CHANNELS)) {
      selectedChannel = cmd - '0';
      Serial.print("■ SELECTED CHANNEL ");
      Serial.println(selectedChannel);
    } else if (cmd == 'r') {
      resetAnalysis();
      Serial.println("■ ANALYSIS RESET");
    } else if (cmd == 'c') {
      displayCurvatureStats();
    }
    // Ignore unrecognized commands
  }
}

void sampleAllChannels() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    int rawValue = analogRead(analogPins[ch]);
    // ESP32-S3: 12-bit ADC (0..4095), 3.3V range
    channelHistory[ch][historyIndex] = (rawValue / 4095.0) * 3.3;

    // Basic noise filtering
    if (historyIndex > 0) {
      float diff = abs(channelHistory[ch][historyIndex] -
                       channelHistory[ch][(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE]);
      if (diff > 0.5) { // Noise threshold
        channelHistory[ch][historyIndex] =
          (channelHistory[ch][historyIndex] +
           channelHistory[ch][(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE]) / 2.0;
      }
    }
  }
}

void applySmoothingFilter() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float sum = 0;
    int count = 0;
    for (int i = 0; i < SMOOTHING_WINDOW && i < HISTORY_SIZE; i++) {
      int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
      sum += channelHistory[ch][idx];
      count++;
    }
    if (count > 0) {
      smoothedData[ch][historyIndex] = sum / count;
    }
  }
}

void performCurvatureAnalysis() {
  // Curvature: second finite difference
  int curr = historyIndex;
  int prev = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
  int prev2 = (historyIndex - 2 + HISTORY_SIZE) % HISTORY_SIZE;

  float currentVal = smoothedData[selectedChannel][curr];
  float prevVal = smoothedData[selectedChannel][prev];
  float prev2Val = smoothedData[selectedChannel][prev2];

  float secondDeriv = currentVal - 2 * prevVal + prev2Val;
  curvatureData[historyIndex] = abs(secondDeriv) * 100; // Scaled for visibility
}

void detectSignFlips() {
  flipCount = 0;
  const int window = 15; // how many points to examine
  if (analysisCount < window) return;

  float diffs[window] = {0};
  float signs[window - 1] = {0};

  for (int i = 0; i < window; i++) {
    int idx1 = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
    int idx2 = (historyIndex - i - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    diffs[i] = smoothedData[selectedChannel][idx1] - smoothedData[selectedChannel][idx2];
  }
  for (int i = 0; i < window - 1; i++) {
    if (diffs[i] > 0.01) signs[i] = 1;
    else if (diffs[i] < -0.01) signs[i] = -1;
    else signs[i] = 0;
  }
  for (int i = 0; i < window - 2; i++) {
    if (signs[i] != signs[i + 1] && signs[i] != 0 && signs[i + 1] != 0) {
      flipCount++;
    }
  }

  // Extra pattern analysis (with arbitrary check as in original)
  if (analysisCount * analysisCount < STATE_SPACE && flipCount > 1) {
    performAdvancedAnalysis();
  }
}

void performAdvancedAnalysis() {
  // Calculate rate of change analysis
  float rateOfChange = 0;
  if (flipCount > 0) {
    rateOfChange = (float)flipCount * curvatureData[historyIndex] * analysisCount;
  }

  Serial.print("■ PATTERN DETECTED: Flips=");
  Serial.print(flipCount);
  Serial.print(" Rate=");
  Serial.print(rateOfChange, 4);
  Serial.print(" Curve=");
  Serial.println(curvatureData[historyIndex], 4);
}

void displayCurvatureOscilloscope() {
  // For text/UART, print stats only. (Displays for serial terminals.)
  displayEnhancedStats();
}

void displayEnhancedStats() {
  float currentVal = getCurrentValue(selectedChannel);
  float peakVal = getPeakValue(selectedChannel);
  float rmsVal = getRMSValue(selectedChannel);
  float curvatureAvg = getAverageCurvature();

  Serial.print("CURRENT: ");
  Serial.print(currentVal, 3);
  Serial.print("V │ PEAK: ");
  Serial.print(peakVal, 3);
  Serial.print("V │ RMS: ");
  Serial.print(rmsVal, 3);
  Serial.print(" CURVATURE: ");
  Serial.print(curvatureAvg, 4);
  Serial.print(" │ FLIPS: ");
  Serial.print(flipCount);
  Serial.print(" │ SAMPLES: ");
  Serial.println(analysisCount);
}

float getCurrentValue(int channel) {
  int currentIndex = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
  return channelHistory[channel][currentIndex];
}

float getPeakValue(int channel) {
  float peak = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    if (abs(channelHistory[channel][i]) > peak)
      peak = abs(channelHistory[channel][i]);
  }
  return peak;
}

float getRMSValue(int channel) {
  float sum = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    sum += channelHistory[channel][i] * channelHistory[channel][i];
  }
  return sqrt(sum / HISTORY_SIZE);
}

float getAverageCurvature() {
  float sum = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    sum += curvatureData[i];
  }
  return sum / HISTORY_SIZE;
}

void resetAnalysis() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    for (int i = 0; i < HISTORY_SIZE; i++) {
      channelHistory[ch][i] = 0;
      smoothedData[ch][i] = 0;
    }
  }
  for (int i = 0; i < HISTORY_SIZE; i++) {
    curvatureData[i] = 0;
  }
  analysisCount = 0;
  flipCount = 0;
}

void displayCurvatureStats() {
  Serial.println("█████ CURVATURE ANALYSIS REPORT █████");
  Serial.print("Total Samples: ");
  Serial.println(analysisCount);
  Serial.print("Sign Flips: ");
  Serial.println(flipCount);
  Serial.print("Average Curvature: ");
  Serial.println(getAverageCurvature(), 4);
  Serial.println("██████████████████████████████████████");
}

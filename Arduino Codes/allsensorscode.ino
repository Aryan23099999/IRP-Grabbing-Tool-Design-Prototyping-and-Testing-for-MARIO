// Read 8 FSR sensors (A0..A7) with per-channel Kalman filters
// Live plot: 8 traces (k0..k7) for Kalman-filtered values
// Snapshot: after settling, averages Kalman outputs for ~1 s and prints all 8

#include <SimpleKalmanFilter.h>

// -------- Config --------
const uint8_t FSR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

const unsigned long SETTLE_MS       = 5000;  // wait after placing weights
const unsigned long PLOT_PERIOD_MS  = 20;    // ~50 Hz live sampling
const int           SNAP_SAMPLES    = 100;   // ~1 s at 100 Hz during snapshot

// Kalman parameters (tune these to your noise)
const float KF_MEAS_NOISE = 8.0f;   // measurement noise stdev (counts)
const float KF_EST_ERROR  = 8.0f;   // initial estimate error
const float KF_PROCESS_Q  = 0.05f;  // process noise (responsiveness)

// ------------------------

SimpleKalmanFilter kf[8] = {
  SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
  SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
  SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
  SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
  SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
  SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
  SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
  SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
};

bool plotMode = false;
bool kfPrimed = false;
unsigned long lastPlotMs = 0;

void primeKalman() {
  // Give high initial uncertainty so the first updates grab quickly
  for (int i = 0; i < 8; i++) {
    kf[i].setEstimateError(1000.0f);
  }
  kfPrimed = false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB serial if needed */ }

  // Inputs (A6/A7 are analog-only; pinMode not required but harmless)
  for (int i = 0; i < 8; i++) pinMode(FSR_PINS[i], INPUT);

  Serial.println("8x FSR reader ready.");
  Serial.println("Press 'P' to toggle live plot (KF k0..k7).");
  Serial.println("Press Enter for SNAPSHOT: settle 5 s, then average KF for ~1 s and print.");
}

void loop() {
  // ---- Serial commands ----
  if (Serial.available()) {
    int c = Serial.read();
    while (Serial.available()) Serial.read(); // clear any CR/LF extras

    if (c == 'p' || c == 'P') {
      plotMode = !plotMode;
      primeKalman();
      Serial.println(plotMode ? "PLOT_START" : "PLOT_STOP");
      if (plotMode) Serial.println("Plotting k0..k7 (Kalman per sensor).");
    } else {
      // Snapshot for all 8 sensors using Kalman-averaged values
      Serial.println("Settling...");
      unsigned long t0 = millis();
      while (millis() - t0 < SETTLE_MS) delay(10);

      // Local Kalman instances for snapshot (so live state isn't disturbed)
      SimpleKalmanFilter kfs[8] = {
        SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
        SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
        SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
        SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
        SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
        SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
        SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
        SimpleKalmanFilter(KF_MEAS_NOISE, KF_EST_ERROR, KF_PROCESS_Q),
      };
      for (int i = 0; i < 8; i++) kfs[i].setEstimateError(1000.0f);

      double sum[8] = {0,0,0,0,0,0,0,0};

      for (int n = 0; n < SNAP_SAMPLES; n++) {
        for (int i = 0; i < 8; i++) {
          int raw = analogRead(FSR_PINS[i]);
          float est = kfs[i].updateEstimate((float)raw);
          sum[i] += est;  // average *Kalman* outputs
        }
        delay(10); // 100 Hz effective snapshot rate
      }

      Serial.println("SNAPSHOT (KF-averaged ADC):");
      for (int i = 0; i < 8; i++) {
        float avg = (float)(sum[i] / SNAP_SAMPLES);
        Serial.print("k"); Serial.print(i); Serial.print("_avg = ");
        Serial.println(avg, 2);
      }
      Serial.println("Change/remove weights, press Enter again for another snapshot.");
    }
  }

  // ---- Live plot (k0..k7) ----
  if (plotMode && (millis() - lastPlotMs >= PLOT_PERIOD_MS)) {

    // Prime live KFs on first pass
    if (!kfPrimed) {
      // Feed each KF a couple of initial reads to anchor
      for (int i = 0; i < 2; i++) {
        for (int s = 0; s < 8; s++) {
          int raw = analogRead(FSR_PINS[s]);
          kf[s].updateEstimate((float)raw);
        }
      }
      // Return to normal estimate error
      for (int s = 0; s < 8; s++) kf[s].setEstimateError(KF_EST_ERROR);
      kfPrimed = true;
    }

    // Read all sensors, update KFs, and print one plot line with 8 series
    for (int s = 0; s < 8; s++) {
      int raw = analogRead(FSR_PINS[s]);
      float est = kf[s].updateEstimate((float)raw);

      // Label:value pairs separated by tabs for Serial Plotter
      Serial.print("k"); Serial.print(s); Serial.print(":");
      Serial.print(est, 2);
      if (s < 7) Serial.print('\t');
    }
    Serial.println();

    lastPlotMs = millis();
  }
}

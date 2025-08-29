// Read 8 FSR sensors (A0..A7) with per-channel Kalman filters
// Live plot: 2 traces (Group1 = A0..A3 avg, Group2 = A4..A7 avg)
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
  for (int i = 0; i < 8; i++) {
    kf[i].setEstimateError(1000.0f); // big initial uncertainty
  }
  kfPrimed = false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB serial if needed */ }

  for (int i = 0; i < 8; i++) pinMode(FSR_PINS[i], INPUT);

  Serial.println("8x FSR reader ready.");
  Serial.println("Press 'P' to toggle live plot (Group1 vs Group2).");
  Serial.println("Press Enter for SNAPSHOT: settle 5 s, then average KF for ~1 s and print.");
}

void loop() {
  // ---- Serial commands ----
  if (Serial.available()) {
    int c = Serial.read();
    while (Serial.available()) Serial.read(); // clear CR/LF extras

    if (c == 'p' || c == 'P') {
      plotMode = !plotMode;
      primeKalman();
      Serial.println(plotMode ? "PLOT_START" : "PLOT_STOP");
      if (plotMode) Serial.println("Plotting Group1 (A0–A3) and Group2 (A4–A7).");
    } else {
      // ---- Snapshot mode ----
      Serial.println("Settling...");
      unsigned long t0 = millis();
      while (millis() - t0 < SETTLE_MS) delay(10);

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
          sum[i] += est;
        }
        delay(10); // ~100 Hz
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

  // ---- Live plot (Group1 = A0..A3 avg, Group2 = A4..A7 avg) ----
  if (plotMode && (millis() - lastPlotMs >= PLOT_PERIOD_MS)) {
    if (!kfPrimed) {
      for (int i = 0; i < 2; i++) {
        for (int s = 0; s < 8; s++) {
          int raw = analogRead(FSR_PINS[s]);
          kf[s].updateEstimate((float)raw);
        }
      }
      for (int s = 0; s < 8; s++) kf[s].setEstimateError(KF_EST_ERROR);
      kfPrimed = true;
    }

    float group1 = 0, group2 = 0;

    for (int s = 0; s < 8; s++) {
      int raw = analogRead(FSR_PINS[s]);
      float est = kf[s].updateEstimate((float)raw);
      if (s < 4) group1 += est;
      else       group2 += est;
    }

    group1 /= 4.0;
    group2 /= 4.0;

    Serial.print("Group1:"); Serial.print(group1, 2);
    Serial.print('\t');
    Serial.print("Group2:"); Serial.println(group2, 2);

    lastPlotMs = millis();
  }
}


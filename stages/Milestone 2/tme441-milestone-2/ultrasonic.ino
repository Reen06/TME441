/* HC-SR04: >=10 us trigger, pulse width / 58 gives cm, >60 ms between pings.
 * Zero-duration echo is INVALID DATA, never a zero-distance arrival.
 */
UltrasonicReading cached_ultrasonic = {ULTRASONIC_NOT_READY, NAN, 0, NAN};
bool ultrasonic_sampled = false;

void initUltrasonic() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  cached_ultrasonic = {ULTRASONIC_NOT_READY, NAN, 0, NAN};
  ultrasonic_sampled = false;
}

UltrasonicReading readUltrasonic() {
  if (ultrasonic_sampled &&
      millis() - cached_ultrasonic.sampled_ms < ULTRASONIC_PING_INTERVAL_MS) {
    return cached_ultrasonic;
  }
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  const unsigned long echo_us =
      pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_ECHO_TIMEOUT_US);
  cached_ultrasonic.sampled_ms = millis();
  ultrasonic_sampled = true;
  cached_ultrasonic.distance_cm = NAN;
  cached_ultrasonic.maximum_cm = NAN;
  if (echo_us == 0) {
    cached_ultrasonic.status = ULTRASONIC_NO_ECHO;
    return cached_ultrasonic;
  }
  const float cm = echo_us / 58.0f;
  if (!isfinite(cm) || cm < ULTRASONIC_MIN_CM || cm > ULTRASONIC_MAX_CM) {
    cached_ultrasonic.status = ULTRASONIC_INVALID;
    return cached_ultrasonic;
  }
  cached_ultrasonic.status = ULTRASONIC_OK;
  cached_ultrasonic.distance_cm = cm;
  cached_ultrasonic.maximum_cm = cm;
  return cached_ultrasonic;
}

bool ultrasonicReadingFresh(const UltrasonicReading &reading) {
  return reading.status == ULTRASONIC_OK && isfinite(reading.distance_cm) &&
      millis() - reading.sampled_ms <= ULTRASONIC_FRESHNESS_MS;
}

UltrasonicReading readStableUltrasonic() {
  float samples[5];
  unsigned long sample_times[5];
  unsigned int valid_count = 0, attempts = 0;
  bool seen = false;
  unsigned long last_sample_ms = 0;
  const unsigned long started_ms = millis();
  UltrasonicReading reading = {ULTRASONIC_NOT_READY, NAN, started_ms, NAN};
  while (attempts < 5 && millis() - started_ms < ULTRASONIC_CONFIRM_TIMEOUT_MS) {
    reading = readUltrasonic();
    if (!seen || reading.sampled_ms != last_sample_ms) {
      seen = true;
      last_sample_ms = reading.sampled_ms;
      ++attempts;
      if (ultrasonicReadingFresh(reading)) {
        unsigned int pos = valid_count++;
        samples[pos] = reading.distance_cm;
        sample_times[pos] = reading.sampled_ms;
        while (pos > 0 && samples[pos] < samples[pos - 1]) {
          const float t = samples[pos];
          const unsigned long stamp = sample_times[pos];
          samples[pos] = samples[pos - 1]; sample_times[pos] = sample_times[pos - 1];
          samples[--pos] = t; sample_times[pos] = stamp;
        }
        for (unsigned int i = 0; i + 2 < valid_count; ++i) {
          if (samples[i + 2] - samples[i] <= ULTRASONIC_STABLE_SPREAD_CM &&
              millis() - sample_times[i] <= ULTRASONIC_FRESHNESS_MS &&
              millis() - sample_times[i + 1] <= ULTRASONIC_FRESHNESS_MS &&
              millis() - sample_times[i + 2] <= ULTRASONIC_FRESHNESS_MS) {
            reading.distance_cm = samples[i + 1];
            reading.maximum_cm = samples[i + 2];
            return reading;
          }
        }
      }
    }
    delay(1);
  }
  reading.status = ULTRASONIC_INVALID;
  reading.distance_cm = NAN;
  reading.maximum_cm = NAN;
  return reading;
}

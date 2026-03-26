#ifndef READS_H
#define READS_H

// Start LED sensor (photoresistor) configuration.
static const uint8_t PHOTORESISTOR_PIN = 4;
static const int START_LED_THRESHOLD = 1800;
static const bool START_LED_ACTIVE_HIGH = true;

inline void initStartLedSensor() {
	pinMode(PHOTORESISTOR_PIN, INPUT);
}

inline int readStartLedRaw() {
	return analogRead(PHOTORESISTOR_PIN);
}

inline bool isStartLedDetected() {
	const int raw = readStartLedRaw();
	if (START_LED_ACTIVE_HIGH) {
		return raw >= START_LED_THRESHOLD;
	}
	return raw <= START_LED_THRESHOLD;
}

#endif
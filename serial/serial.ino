/*
 * Create a serial bridge from the Teensy to the FPGA.
 */
#define BAUD 3000000

void setup()
{
	Serial.begin(BAUD);
	Serial1.begin(BAUD);
	Serial.setTimeout(0);
	Serial1.setTimeout(0);
	pinMode(13, OUTPUT);
}

char buf[32];

void loop()
{
	size_t incoming;

	if (Serial.available()) {
		incoming = Serial.readBytes(buf, sizeof(buf));
		if (incoming > 0)
			Serial1.write(buf, incoming);
	}

	if (Serial1.available()) {
		incoming = Serial1.readBytes(buf, sizeof(buf));
		if (incoming > 0)
		{
			Serial.write(buf, incoming);
			digitalWriteFast(13, 1);
		}
	} else {
		digitalWriteFast(13, 0);
	}
}

int test = 0;

void setup() {
	Serial.begin(9600);
}

void loop() {
	test++;
	Serial.println("hello!");
	//Serial.flush();
	delay(500);
}

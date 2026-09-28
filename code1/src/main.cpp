#include <Arduino.h>

void setup() {
    Serial1.begin(115200);
}

void loop() {
    Serial1.println("Running...");
    delay(1000);
}
#include <Arduino.h>

#include "JoystickReader.h"


void JoystickReader::init() {
    pinMode(SW_PIN, INPUT_PULLUP);
}

void JoystickReader::getInputs(int& x, int& y, bool& switchOn) {
    x = analogRead(X_PIN);
    y = analogRead(Y_PIN);
    switchOn = (digitalRead(SW_PIN) == 0);
}
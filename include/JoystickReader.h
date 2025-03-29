#pragma once

const int SW_PIN = 2;   // digital pin connected to switch output
const int X_PIN = 0;    // analog pin connected to X output
const int Y_PIN = 1;    // analog pin connected to Y output


class JoystickReader {
    public:
        static void init();
        static void getInputs(int& x, int& y, bool& switchOn);

        // Disallow instantiation of this class
        JoystickReader() = delete;
};

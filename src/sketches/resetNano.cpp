#include <Arduino.h>

#include "KVStore.h"
#include "kvstore_global_api.h"
#include "mbed_error.h"

void setup() {
    Serial.begin(9600);
    while (!Serial);

    int status = kv_reset("/kv/");
    if (status == MBED_SUCCESS) {
        Serial.println("KVStore successfully resetted.");
    } else {
        Serial.println("KVStore could not be resetted.");
    }
}

void loop() {
    // Nothing to be done here.
}
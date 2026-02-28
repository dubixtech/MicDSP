#pragma once
#include <Arduino.h>
#include "driver/i2s.h"

extern "C" {
#include <noise_suppression.h>
#include "agc.h"
#include <fvad.h>          // libfvad
}

class MicDSP {
public:
    MicDSP() = default;

    // Begin: always initialize AGC, NS, VAD
    bool begin(i2s_port_t port,
               uint32_t sampleRate,
               int bclkPin,
               int wsPin,
               int dataPin);

    // Read with per-call AGC toggle
    esp_err_t read(int16_t *output,
                   size_t samples,
                   size_t *samples_read = nullptr,
                   bool *vad = nullptr,
                   bool agcEnabled = true,          // <-- new per-call toggle
                   TickType_t ticks_to_wait = portMAX_DELAY);

private:
    bool _agcEnabled;        // Tracks AGC state (always allocated)
    i2s_port_t _port;
    uint32_t _sampleRate;

    NsHandle* _ns = nullptr;
    void* _agc = nullptr;
    Fvad* _fvad = nullptr;

    bool initI2S(int bclkPin, int wsPin, int dataPin);
    bool initNS();
    bool initAGC();
    bool initVAD();

    void log(const char* msg); // always prints
};
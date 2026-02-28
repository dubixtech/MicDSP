#include "MicDSP.h"

#define FRAME_SIZE 160   // 10ms @16kHz

void MicDSP::log(const char* msg) {
    Serial.println(msg);
}

// ---------------- BEGIN ----------------
bool MicDSP::begin(i2s_port_t port,
                   uint32_t sampleRate,
                   int bclkPin,
                   int wsPin,
                   int dataPin)
{
    _port = port;
    _sampleRate = sampleRate;

    log("MicDSP: Initializing I2S...");
    if (!initI2S(bclkPin, wsPin, dataPin)) {
        log("MicDSP ERROR: I2S init failed");
        return false;
    }

    log("MicDSP: Purging initial DC offset (200ms)...");
    size_t bytes_discarded = 0;
    int32_t dummy_buf[FRAME_SIZE];
    for (int i = 0; i < 20; i++)
        i2s_read(_port, dummy_buf, sizeof(dummy_buf), &bytes_discarded, portMAX_DELAY);

    if (!initNS())   { log("MicDSP ERROR: NS init failed"); return false; }
    if (!initAGC())  { log("MicDSP ERROR: AGC init failed"); return false; }
    if (!initVAD())  { log("MicDSP ERROR: VAD init failed"); return false; }

    log("MicDSP: Initialization complete");
    return true;
}

// ---------------- I2S ----------------
bool MicDSP::initI2S(int bclkPin, int wsPin, int dataPin)
{
    i2s_config_t config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = _sampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = FRAME_SIZE,
        .use_apll = false
    };

    if (i2s_driver_install(_port, &config, 0, NULL) != ESP_OK)
        return false;

    i2s_pin_config_t pins = {
        .bck_io_num = bclkPin,
        .ws_io_num = wsPin,
        .data_out_num = -1,
        .data_in_num = dataPin
    };

    if (i2s_set_pin(_port, &pins) != ESP_OK)
        return false;

    i2s_zero_dma_buffer(_port);
    log("MicDSP: I2S ready");
    return true;
}

// ---------------- NS ----------------
bool MicDSP::initNS()
{
    _ns = WebRtcNs_Create();
    if (!_ns) return false;
    if (WebRtcNs_Init(_ns, _sampleRate) != 0) return false;
    WebRtcNs_set_policy(_ns, 3);
    log("MicDSP: NS ready");
    return true;
}

// ---------------- AGC ----------------
bool MicDSP::initAGC()
{
    _agc = WebRtcAgc_Create();
    if (!_agc) return false;
    if (WebRtcAgc_Init(_agc, 0, 255, 3, _sampleRate) != 0) return false;

    WebRtcAgcConfig config;
    config.targetLevelDbfs = 3;
    config.compressionGaindB = 20;
    config.limiterEnable = 1;
    WebRtcAgc_set_config(_agc, config);

    log("MicDSP: AGC ready");
    return true;
}

// ---------------- VAD ----------------
bool MicDSP::initVAD()
{
    _fvad = fvad_new();
    if (!_fvad) return false;
    if (fvad_set_sample_rate(_fvad, _sampleRate) < 0) return false;
    fvad_set_mode(_fvad, 3);
    log("MicDSP: VAD ready");
    return true;
}

// ---------------- READ ----------------
esp_err_t MicDSP::read(int16_t *output,
                       size_t samples,
                       size_t *samples_read,
                       bool *vad,
                       bool agcEnabled,
                       TickType_t ticks_to_wait)
{
    static int32_t raw32[FRAME_SIZE];
    static int16_t nsIn[FRAME_SIZE];

    int16_t* nsInPtr[1]  = { nsIn };
    int16_t* nsOutPtr[1] = { output };

    size_t bytesRead = 0;
    esp_err_t err = i2s_read(_port,
                             raw32,
                             samples * sizeof(int32_t),
                             &bytesRead,
                             ticks_to_wait);
    if (err != ESP_OK) {
        Serial.println("MicDSP ERROR: i2s_read failed");
        return err;
    }

    size_t count = bytesRead / sizeof(int32_t);

    // Convert 32 -> 16
    for (size_t i = 0; i < count; i++)
        nsIn[i] = (int16_t)(raw32[i] >> 8);

    // NS
    WebRtcNs_Analyze(_ns, nsIn);
    WebRtcNs_Process(_ns,
                     (const int16_t *const *)nsInPtr,
                     1,
                     nsOutPtr);

    // AGC (toggle per read)
    if (agcEnabled) {
        int32_t inMicLevel = 0, outMicLevel = 0;
        uint8_t saturationWarning = 0;
        int16_t echo = 0;

        WebRtcAgc_Process(_agc,
                          (const int16_t *const *)nsOutPtr,
                          1,
                          count,
                          (int16_t *const *)nsOutPtr,
                          inMicLevel,
                          &outMicLevel,
                          echo,
                          &saturationWarning);
    }

    // VAD
    if (vad)
        *vad = (fvad_process(_fvad, output, count) == 1);

    if (samples_read) *samples_read = count;
    return ESP_OK;
}
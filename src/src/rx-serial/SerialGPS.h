#pragma once

#include "SerialIO.h"

#include "device.h"

typedef struct {
    // Latitude in decimal degrees
    uint32_t lat;
    // Longitude in decimal degrees
    uint32_t lon;
    // Altitude in meters
    uint32_t alt;
    // Speed in km/h
    uint32_t speed;
    // Heading in degrees, positive. 0 is north.
    uint32_t heading;
    // Number of satellites
    uint8_t satellites;
} GpsData;

class SerialGPS final : public SerialIO {
public:
    explicit SerialGPS(HardwareSerial &stream, const int8_t rxPin, const int8_t txPin) : SerialIO(&stream, 115200, SERIAL_8N1, rxPin, txPin, false) {}
    ~SerialGPS() override = default;

    void sendQueuedData(uint32_t maxBytesToSend) override;

    int32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override { return DURATION_IMMEDIATELY; }
private:
    void processBytes(uint8_t *bytes, uint16_t size) override;
    void sendTelemetryFrame() const;
    void processSentence(uint8_t *sentence, uint8_t size);
    GpsData gpsData = {0};
};

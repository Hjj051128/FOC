#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <Arduino.h>

struct VisionFrame
{
    int16_t delta_x;
    int16_t delta_y;
    uint8_t confidence;
    bool target_found;
};

class VisionPacketParser
{
public:
    VisionPacketParser();

    // Consumes all currently available bytes and returns the newest valid frame.
    bool poll(Stream &port, VisionFrame &frame);
    void reset();

private:
    static constexpr uint8_t PACKET_SIZE = 11;
    static constexpr uint8_t HEADER = 0xAA;
    static constexpr uint8_t TAIL = 0xFF;

    bool pushByte(uint8_t value, VisionFrame &frame);
    bool decode(VisionFrame &frame) const;
    void resyncAfterInvalidPacket();

    uint8_t buffer[PACKET_SIZE];
    uint8_t length;
};

#endif

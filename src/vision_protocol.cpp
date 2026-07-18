#include "vision_protocol.h"

VisionPacketParser::VisionPacketParser()
    : buffer{}
    , length(0)
{
}

void VisionPacketParser::reset()
{
    length = 0;
}

bool VisionPacketParser::poll(Stream &port, VisionFrame &frame)
{
    bool frame_received = false;

    while (port.available() > 0) {
        int value = port.read();
        if (value < 0) {
            break;
        }

        VisionFrame candidate = {};
        if (pushByte(static_cast<uint8_t>(value), candidate)) {
            frame = candidate;
            frame_received = true;
        }
    }

    return frame_received;
}

bool VisionPacketParser::pushByte(uint8_t value, VisionFrame &frame)
{
    if (length == 0) {
        if (value == HEADER) {
            buffer[0] = value;
            length = 1;
        }
        return false;
    }

    if (length == 1) {
        if (value == HEADER) {
            buffer[1] = value;
            length = 2;
        }
        else {
            length = 0;
        }
        return false;
    }

    buffer[length++] = value;
    if (length < PACKET_SIZE) {
        return false;
    }

    if (decode(frame)) {
        length = 0;
        return true;
    }

    resyncAfterInvalidPacket();
    return false;
}

bool VisionPacketParser::decode(VisionFrame &frame) const
{
    if (
        buffer[0] != HEADER ||
        buffer[1] != HEADER ||
        buffer[9] != TAIL ||
        buffer[10] != TAIL ||
        buffer[2] > 1 ||
        buffer[3] > 100
    ) {
        return false;
    }

    uint8_t checksum = 0;
    for (uint8_t index = 0; index <= 7; ++index) {
        checksum = static_cast<uint8_t>(
            checksum + buffer[index]
        );
    }

    if (checksum != buffer[8]) {
        return false;
    }

    uint16_t raw_x =
        static_cast<uint16_t>(buffer[4]) |
        (static_cast<uint16_t>(buffer[5]) << 8);
    uint16_t raw_y =
        static_cast<uint16_t>(buffer[6]) |
        (static_cast<uint16_t>(buffer[7]) << 8);

    frame.delta_x = static_cast<int16_t>(raw_x);
    frame.delta_y = static_cast<int16_t>(raw_y);
    frame.confidence = buffer[3];
    frame.target_found = buffer[2] == 1;
    return true;
}

void VisionPacketParser::resyncAfterInvalidPacket()
{
    for (uint8_t start = 1; start + 1 < PACKET_SIZE; ++start) {
        if (
            buffer[start] == HEADER &&
            buffer[start + 1] == HEADER
        ) {
            uint8_t remaining = PACKET_SIZE - start;
            for (uint8_t index = 0; index < remaining; ++index) {
                buffer[index] = buffer[start + index];
            }
            length = remaining;
            return;
        }
    }

    if (buffer[PACKET_SIZE - 1] == HEADER) {
        buffer[0] = HEADER;
        length = 1;
    }
    else {
        length = 0;
    }
}

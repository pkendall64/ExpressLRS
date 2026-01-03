#include "targets.h"
#include "FEC.h"
#include <cstring>

/*
 * Optimization: LUT for Simultaneous Encode + Interleave (Spread)
 * This table maps a 4-bit nibble (0-15) to a 64-bit 'spread' word.
 * * Logic: Bit j of the Hamming Code is mapped to Byte j of the uint64_t.
 * This allows the transpose operation to correctly reconstruct the
 * codewords in the decoder.
 */
static constexpr uint64_t DRAM_ATTR hammingEncodeSpread[16] = {
    0x0000000000000000ULL, // nibble 0x0 (code 0x00)
    0x0001010100000001ULL, // nibble 0x1 (code 0x71)
    0x0001010000000100ULL, // nibble 0x2 (code 0x62)
    0x0000000100000101ULL, // nibble 0x3 (code 0x13)
    0x0001000100010000ULL, // nibble 0x4 (code 0x54)
    0x0000010000010001ULL, // nibble 0x5 (code 0x25)
    0x0000010100010100ULL, // nibble 0x6 (code 0x36)
    0x0001000000010101ULL, // nibble 0x7 (code 0x47)
    0x0000010101000000ULL, // nibble 0x8 (code 0x38)
    0x0001000001000001ULL, // nibble 0x9 (code 0x49)
    0x0001000101000100ULL, // nibble 0xA (code 0x5A)
    0x0000010001000101ULL, // nibble 0xB (code 0x2B)
    0x0001010001010000ULL, // nibble 0xC (code 0x6C)
    0x0000000101010001ULL, // nibble 0xD (code 0x1D)
    0x0000000001010100ULL, // nibble 0xE (code 0x0E)
    0x0001010101010101ULL  // nibble 0xF (code 0x7F)
};

// Helper: Transpose 8x8 bit matrix stored in uint64_t
static ICACHE_RAM_ATTR uint64_t transpose64(uint64_t x) {
    uint64_t t;
    t = (x ^ (x >> 7)) & 0x00AA00AA00AA00AAULL;
    x = x ^ t ^ (t << 7);
    t = (x ^ (x >> 14)) & 0x0000CCCC0000CCCCULL;
    x = x ^ t ^ (t << 14);
    t = (x ^ (x >> 28)) & 0x00000000F0F0F0F0ULL;
    x = x ^ t ^ (t << 28);
    return x;
}

void ICACHE_RAM_ATTR FECEncode(const uint8_t *incomingData, uint8_t *FECBuffer)
{
    // We process input as two parallel streams of 8 nibbles each
    // Stream A: Nibbles 0-7 (from Input Bytes 0-3) -> Even output bytes
    // Stream B: Nibbles 8-15 (from Input Bytes 4-7) -> Odd output bytes
    uint64_t streamA = 0;
    uint64_t streamB = 0;

    for (int i = 0; i < 4; i++) {
        // Handle Stream A (Input 0-3)
        streamA |= (hammingEncodeSpread[incomingData[i] & 0x0F] << (i * 2)) |    // LSB -> Column 0, 2, 4, 6
                   (hammingEncodeSpread[incomingData[i] >> 4]   << (i * 2 + 1)); // MSB -> Column 1, 3, 5, 7

        // Handle Stream B (Input 4-7)
        streamB |= (hammingEncodeSpread[incomingData[i + 4] & 0x0F] << (i * 2)) |    // LSB -> Column 0, 2, 4, 6
                   (hammingEncodeSpread[incomingData[i + 4] >> 4]   << (i * 2 + 1)); // MSB -> Column 1, 3, 5, 7
    }

    // Map streams to interleaved FECBuffer
    // Byte 0-6 of streamA go to FECBuffer[0, 2, 4, 6, 8, 10, 12]
    // Byte 0-6 of streamB go to FECBuffer[1, 3, 5, 7, 9, 11, 13]
    const auto pA = (uint8_t*)&streamA;
    const auto pB = (uint8_t*)&streamB;

    for (int i = 0; i < 7; i++) {
        FECBuffer[i * 2 + 0] = pA[i];
        FECBuffer[i * 2 + 1] = pB[i];
    }
}

/**
 * Optimization: Unpacked Hamming Decode LUT
 * This table maps a 7-bit codeword directly to its 4-bit data nibble.
 * This removes the division/modulo and bit-shifting logic from the decoder loop.
 */
static constexpr uint8_t DRAM_ATTR hammingDecodeTable[128] = {
    0x0, 0x0, 0x0, 0x3, 0x0, 0x5, 0xE, 0x7, 0x0, 0x9, 0xE, 0xB, 0xE, 0xD, 0xE, 0xE,
    0x0, 0x3, 0x3, 0x3, 0x4, 0xD, 0x6, 0x3, 0x8, 0xD, 0xA, 0x3, 0xD, 0xD, 0xE, 0xD,
    0x0, 0x5, 0x2, 0xB, 0x5, 0x5, 0x6, 0x5, 0x8, 0xB, 0xB, 0xB, 0xC, 0x5, 0xE, 0xB,
    0x8, 0x1, 0x6, 0x3, 0x6, 0x5, 0x6, 0x6, 0x8, 0x8, 0x8, 0xB, 0x8, 0xD, 0x6, 0xF,
    0x0, 0x9, 0x2, 0x7, 0x4, 0x7, 0x7, 0x7, 0x9, 0x9, 0xA, 0x9, 0xC, 0x9, 0xE, 0x7,
    0x4, 0x1, 0xA, 0x3, 0x4, 0x4, 0x4, 0x7, 0xA, 0x9, 0xA, 0xA, 0x4, 0xD, 0xA, 0xF,
    0x2, 0x1, 0x2, 0x2, 0xC, 0x5, 0x2, 0x7, 0xC, 0x9, 0x2, 0xB, 0xC, 0xC, 0xC, 0xF,
    0x1, 0x1, 0x2, 0x1, 0x4, 0x1, 0x6, 0xF, 0x8, 0x1, 0xA, 0xF, 0xC, 0xF, 0xF, 0xF
};

void ICACHE_RAM_ATTR FECDecode(const uint8_t *incomingFECBuffer, uint8_t *outgoingData) {
    uint64_t streamA = 0, streamB = 0;
    const auto pA = (uint8_t*)&streamA, pB = (uint8_t*)&streamB;

    // Load interleaved data into 64-bit blocks
    for (int i = 0; i < 7; i++) {
        pA[i] = incomingFECBuffer[i * 2 + 0];
        pB[i] = incomingFECBuffer[i * 2 + 1];
    }

    // De-interleave bits
    streamA = transpose64(streamA);
    streamB = transpose64(streamB);

    // Fast Decode using Unpacked LUT
    for (int i = 0; i < 4; i++) {
        outgoingData[i] =  hammingDecodeTable[pA[i * 2 + 0] & 0x7F] | (hammingDecodeTable[pA[i * 2 + 1] & 0x7F] << 4);
        outgoingData[i + 4] = hammingDecodeTable[pB[i * 2 + 0] & 0x7F] | (hammingDecodeTable[pB[i * 2 + 1] & 0x7F] << 4);
    }
}
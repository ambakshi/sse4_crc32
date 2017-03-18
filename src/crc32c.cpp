/**
 * @file crc32c.cpp
 * @brief Node.js bindings for CRC-32C calculation using hardware-acceleration, when available.
 *
 * The code below provides the bindings for the node-addon allowing for interfacing of C/C++ code with
 * JavaScript. It chooses between two versions of the CRC-32C calculator:
 * - The hardware-accelerated version that uses Intel's SSE 4.2 instructions, implemented in crc32c_sse42.cpp
 * - A table-lookup based CRC calculated implemented in software for non-Nehalam-based architectures
 *
 * NOTES:
 * - This code, though originally designed for little-endian hardware, should work for all platforms.
 * - Table-based CRC-32C implementation based on code by Mark Adler at http://stackoverflow.com/a/17646775.
 *
 * @author Anand Suresh <anandsuresh@gmail.com>
 */

#include <stdint.h>
#include <nan.h>

#include "crc32c.h"



using namespace v8;
using namespace node;



// Bit-mask for the SSE 4.2 flag in the CPU ID
#define SSE4_2_FLAG         0x100000

// The CRC-32C polynomial in reversed bit order
#define CRC32C_POLYNOMIAL   0x82f63b78



// Stores the CRC-32 lookup table for the software-fallback implementation
static uint32_t crc32cTable[8][256];

/* Copyright (c) 2014, Matt Stancliff <matt@genges.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of Redis nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE. */

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

#define POLY UINT64_C(0xad93d23594c935a9)
static uint64_t crc64tab[8][256];
static bool isInit = false;

/**
 * Reflect all bits of a \a data word of \a data_len bytes.
 *
 * \param data         The data word to be reflected.
 * \param data_len     The width of \a data expressed in number of bits.
 * \return             The reflected data.
 *****************************************************************************/
static inline uint_fast64_t crc_reflect(uint_fast64_t data, size_t data_len) {
    uint_fast64_t ret = data & 0x01;

    for (size_t i = 1; i < data_len; i++) {
        data >>= 1;
        ret = (ret << 1) | (data & 0x01);
    }

    return ret;
}

/**
 *  Update the crc value with new data.
 *
 * \param crc      The current crc value.
 * \param data     Pointer to a buffer of \a data_len bytes.
 * \param data_len Number of bytes in the \a data buffer.
 * \return         The updated crc value.
 ******************************************************************************/
uint64_t crc64slow(uint_fast64_t crc, const void *in_data, const uint64_t len) {
    const uint8_t *data = (uint8_t *)in_data;
    bool bit;

    for (uint64_t offset = 0; offset < len; offset++) {
        uint8_t c = data[offset];
        for (uint_fast8_t i = 0x01; i & 0xff; i <<= 1) {
            bit = crc & 0x8000000000000000;
            if (c & i) {
                bit = !bit;
            }

            crc <<= 1;
            if (bit) {
                crc ^= POLY;
            }
        }

        crc &= 0xffffffffffffffff;
    }

    crc = crc & 0xffffffffffffffff;
    return crc_reflect(crc, 64) ^ 0x0000000000000000;
}


/* Fill in a CRC constants table. */
void crc64s_init(void) {
    uint64_t crc;

    assert(!isInit);

    /* generate CRCs for all single byte sequences */
    for (int n = 0; n < 256; n++) {
        crc64tab[0][n] = crc64slow(0, &n, 1);
    }

    /* generate nested CRC crc64tab for future slice-by-8 lookup */
    for (int n = 0; n < 256; n++) {
        crc = crc64tab[0][n];
        for (int k = 1; k < 8; k++) {
            crc = crc64tab[0][crc & 0xff] ^ (crc >> 8);
            crc64tab[k][n] = crc;
        }
    }

    isInit = true;
}

/* Calculate a non-inverted CRC multiple bytes at a time on a little-endian
 * architecture. If you need inverted CRC, invert *before* calling and invert
 * *after* calling.
 * 64 bit crc = process 8 bytes at once;
 */
uint64_t crc64s(uint64_t crc, void *buf, size_t len) {
    unsigned char *next = (unsigned char *)buf;

    assert(isInit);
    /* process individual bytes until we reach an 8-byte aligned pointer */
    while (len && ((uintptr_t)next & 7) != 0) {
        crc = crc64tab[0][(crc ^ *next++) & 0xff] ^ (crc >> 8);
        len--;
    }

    /* fast middle processing, 8 bytes (aligned!) per loop */
    while (len >= 8) {
        crc ^= *(uint64_t *)next;
        crc = crc64tab[7][crc & 0xff] ^
              crc64tab[6][(crc >> 8) & 0xff] ^
              crc64tab[5][(crc >> 16) & 0xff] ^
              crc64tab[4][(crc >> 24) & 0xff] ^
              crc64tab[3][(crc >> 32) & 0xff] ^
              crc64tab[2][(crc >> 40) & 0xff] ^
              crc64tab[1][(crc >> 48) & 0xff] ^
              crc64tab[0][crc >> 56];
        next += 8;
        len -= 8;
    }

    /* process remaining bytes (can't be larger than 8) */
    while (len) {
        crc = crc64tab[0][(crc ^ *next++) & 0xff] ^ (crc >> 8);
        len--;
    }

    return crc;
}


/**
 * Cross-platform CPU feature set detection to check for availability of hardware-based CRC-32C
 */
void cpuid(uint32_t op, uint32_t reg[4]) {
#if defined(_WIN64) || defined(_WIN32)
    #include <intrin.h>
    __cpuid((int *)reg, 1);
#elif defined(__x86_64__)
    __asm__ volatile(
        "pushq %%rbx       \n\t"
        "cpuid             \n\t"
        "movl  %%ebx, %1   \n\t"
        "popq  %%rbx       \n\t"
        : "=a"(reg[0]), "=r"(reg[1]), "=c"(reg[2]), "=d"(reg[3])
        : "a"(op)
        : "cc");
#elif defined(__i386__)
    __asm__ volatile(
        "pushl %%ebx       \n\t"
        "cpuid             \n\t"
        "movl  %%ebx, %1   \n\t"
        "popl  %%ebx       \n\t"
        : "=a"(reg[0]), "=r"(reg[1]), "=c"(reg[2]), "=d"(reg[3])
        : "a"(op)
        : "cc");
#else
    reg[0] = reg[1] = reg[2] = reg[3] = 0;
#endif
}


/**
 * Returns whether or not Intel's Streaming SIMD Extensions 4.2 is available on the hardware
 *
 * @return true if Intel's Streaming SIMD Extensions 4.2 are present; otherwise false
 */
bool isSSE42Available() {
    uint32_t reg[4];

    cpuid(1, reg);
    return ((reg[2] >> 20) & 1) == 1;
}


/**
 * Initializes the CRC-32C lookup table for software-based CRC calculation
 */
void initCrcTable() {
    uint32_t i, j, crc;

    for (i = 0; i < 256; i++) {
        crc = i;
        crc = crc & 1 ? (crc >> 1) ^ CRC32C_POLYNOMIAL : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ CRC32C_POLYNOMIAL : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ CRC32C_POLYNOMIAL : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ CRC32C_POLYNOMIAL : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ CRC32C_POLYNOMIAL : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ CRC32C_POLYNOMIAL : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ CRC32C_POLYNOMIAL : crc >> 1;
        crc = crc & 1 ? (crc >> 1) ^ CRC32C_POLYNOMIAL : crc >> 1;
        crc32cTable[0][i] = crc;
    }

    for (i = 0; i < 256; i++) {
        crc = crc32cTable[0][i];
        for (j = 1; j < 8; j++) {
            crc = crc32cTable[0][crc & 0xff] ^ (crc >> 8);
            crc32cTable[j][i] = crc;
        }
    }
}


/**
 * Calculates CRC-32C using the lookup table
 *
 * @param initialCrc The initial CRC to use for the operation
 * @param buf The buffer that stores the data whose CRC is to be calculated
 * @param len The size of the buffer
 * @return The CRC-32C of the data in the buffer
 */
uint32_t swCrc32c(uint32_t initialCrc, const char *buf, size_t len) {
    const char *next = buf;
    uint64_t crc = initialCrc;


    // If the string is empty, return 0
    if (len == 0) return (uint32_t)crc;

    // XOR the initial CRC with INT_MAX
    crc ^= 0xFFFFFFFF;

    // Process byte-by-byte until aligned to 8-byte boundary
    while (len && ((uintptr_t) next & 7) != 0) {
        crc = crc32cTable[0][(crc ^ *next++) & 0xff] ^ (crc >> 8);
        len--;
    }

    // Process 8 bytes at a time
    while (len >= 8) {
        crc ^= *(uint64_t *) next;
        crc = crc32cTable[7][(crc >>  0) & 0xff] ^ crc32cTable[6][(crc >>  8) & 0xff]
            ^ crc32cTable[5][(crc >> 16) & 0xff] ^ crc32cTable[4][(crc >> 24) & 0xff]
            ^ crc32cTable[3][(crc >> 32) & 0xff] ^ crc32cTable[2][(crc >> 40) & 0xff]
            ^ crc32cTable[1][(crc >> 48) & 0xff] ^ crc32cTable[0][(crc >> 56)];
        next += 8;
        len -= 8;
    }

    // Process any remaining bytes
    while (len) {
        crc = crc32cTable[0][(crc ^ *next++) & 0xff] ^ (crc >> 8);
        len--;
    }

    // XOR again with INT_MAX
    return (uint32_t)(crc ^= 0xFFFFFFFF);
}


/**
 * Returns whether or not hardware support is available for CRC calculation
 */
NAN_METHOD(isHardwareCrcSupported) {
    Nan::HandleScope scope;
    info.GetReturnValue().Set(Nan::New<Boolean>(isSSE42Available()));
}


/**
 * Calculates CRC-32C for the specified string/buffer
 */
NAN_METHOD(calculateCrc) {
    Nan::HandleScope scope;
    uint32_t initCrc;
    uint32_t crc;
    bool useHardwareCrc;

    // Ensure an argument is passed
    if (info.Length() < 1) {
        info.GetReturnValue().Set(Nan::New<Integer>(0));
    } else if (info.Length() > 3) {
        Nan::ThrowTypeError("Invalid number of arguments!");
        return;
    }

    // Check if the table-lookup is required
    if (!info[0]->IsBoolean()) {
        Nan::ThrowTypeError("useHardwareCrc isn't a boolean value as expected!");
        return;
    }
    useHardwareCrc = info[0]->BooleanValue();

    // Check for any initial CRC passed to the function
    if (info.Length() > 2) {
        if (!(info[2]->IsUint32())) {
            Nan::ThrowTypeError("Initial CRC-32C is not an integer value as expected!");
            return;
        }
        initCrc = info[2]->Uint32Value();
    } else {
        initCrc = 0;
    }

    // Ensure the argument is a buffer or a string
    if (node::Buffer::HasInstance(info[1])) {
        Local<Object> buf = info[1]->ToObject();

        if (useHardwareCrc) {
            crc = hwCrc32c(initCrc, (const char *)Buffer::Data(buf), (size_t)Buffer::Length(buf));
        } else {
            crc = crc64s(initCrc, (void *)Buffer::Data(buf), (size_t)Buffer::Length(buf));
        }
    } else if (info[1]->IsObject()) {
        Nan::ThrowTypeError("Cannot compute CRC-32C for objects!");
        return;
    } else {
        Local<String> strInput = info[1]->ToString();

        if (useHardwareCrc) {
            crc = hwCrc32c(initCrc, (const char *)(*String::Utf8Value(strInput)), (size_t)strInput->Utf8Length());
        } else {
            crc = crc64s(initCrc, (void *)(*String::Utf8Value(strInput)), (size_t)strInput->Utf8Length());
        }
    }

    // Calculate the 32-bit CRC
    info.GetReturnValue().Set(Nan::New<Uint32>(crc));
}



/**
 * Initialize the module
 */
void init(Local<Object> exports) {
    crc64s_init();

    Nan::SetMethod(exports, "isHardwareCrcSupported", isHardwareCrcSupported);
    Nan::SetMethod(exports, "calculateCrc", calculateCrc);
}


NODE_MODULE(sse4_crc32, init)

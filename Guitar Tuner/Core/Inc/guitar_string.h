/*
 * String.h
 *
 */

#ifndef INC_GUITAR_STRING_H_
#define INC_GUITAR_STRING_H_

#include "global_header.h"

typedef enum {
    VERY_FLAT,
    FLAT,
    SLIGHTLY_FLAT,
    ON_PITCH,
    SLIGHTLY_SHARP,
    SHARP,
    VERY_SHARP
} TuningOffset;

typedef struct {
    int number;           // String number (0 = low e, 5 = high e)
    float frequency;      // Current frequency of the string
    float targetFrequency; // Desired tuning frequency
    const char* note;     // Corresponding musical note (e.g., "E", "A")
    TuningOffset tuning_offset;
} GuitarString;


// Function declarations
GuitarString createGuitarString(int number, float frequency, float targetFrequency, const char* note, TuningOffset tuning_offset);
void calculateTuningOffset(GuitarString* string, char* msg, size_t msg_size);
void initializeGuitarStrings(GuitarString strings[], GuitarString** currentString);
void switchString(GuitarString strings[], GuitarString** currentString);
void yin_detect_frequency(float32_t *buffer, uint32_t length, uint32_t sample_rate, GuitarString* string);


#endif /* INC_GUITAR_STRING_H_ */

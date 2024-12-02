/*
 * String.h
 *
 */

#ifndef INC_GUITAR_STRING_H_
#define INC_GUITAR_STRING_H_

#include "global_header.h"

typedef struct {
    int number;           // String number (0 = low e, 5 = high e)
    float frequency;      // Current frequency of the string
    float targetFrequency; // Desired tuning frequency
    const char* note;     // Corresponding musical note (e.g., "E", "A")
} GuitarString;

// Function declarations
GuitarString createGuitarString(int number, float frequency, float targetFrequency, const char* note);
float calculateTuningOffset(const GuitarString* string);
void initializeGuitarStrings(GuitarString strings[], GuitarString** currentString);
void switchString(GuitarString strings[], GuitarString** currentString);
void yin_detect_frequency(float32_t *buffer, uint32_t length, uint32_t sample_rate, GuitarString* string);


#endif /* INC_GUITAR_STRING_H_ */

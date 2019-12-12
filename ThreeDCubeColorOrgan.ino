/*
 * Three D Cube Library by Craig A. Lindley
 *
 * This code coupled with custom hardware drives the Seeed
 * Studio's RGB LED Cube. Driving the LEDs is accomplished using
 * two TCL5940 chips multiplexed into eight rows for a total driving
 * capacity of 64 RGB leds. Sound frequency determination is done
 * using an MSGEQ7 chip. Computing power is provided by an Arduino Uno.
 * Power is supplied via a 5VDC 3A wall wort type power supply.
 *
 * Version: 1.0 
 * Last Update: 12/25/2011
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "Types.h"

// #define DEBUG

/*****************************************************************************/
// // Base Hardware Constants 
/*****************************************************************************/

#define NUM_TLCS           2
#define CHANNELS_PER_TLC   16
#define NUM_TLC_CHANNELS   (NUM_TLCS * CHANNELS_PER_TLC)
#define DATA_BYTES_PER_TLC ((CHANNELS_PER_TLC * 3) / 2)
#define DATA_BYTES_PER_ROW (DATA_BYTES_PER_TLC * NUM_TLCS)
#define NUM_ROWS           8

#define TLC_PWM_CLOCKS     4096
#define TLC_GSCLK_CLOCKS   1

#define NUM_RGB_LEDS        64
#define COLOR_COMPONENT_MIN 0
#define COLOR_COMPONENT_MAX 4095
#define RED_ADJ             2000

#define MAX_DIMENSION       4
#define MIN_INDEX           0
#define MAX_INDEX           (MAX_DIMENSION - 1)

/*****************************************************************************/
// Base Hardware I/O Pin Definitions 
/*****************************************************************************/

#define SPI_SCLK_PIN        13
#define SPI_MOSI_PIN        11
#define SPI_SS_PIN          10

#define TIMER2_GSCLK_PIN    3

#define TLC_XLAT_PIN        5
#define TLC_BLANK_PIN       6

#define DECODER_ENABLE_PIN  2
#define DECODER_A_PIN       4
#define DECODER_B_PIN       7
#define DECODER_C_PIN       8

/*****************************************************************************/
// Color Organ Constants 
/*****************************************************************************/

#define NOISE_THRESHOLD_60   140
#define NOISE_THRESHOLD_NORM  50
#define GAMMA               2.5
#define LEDS_PER_FREQ       9
#define MSGEQ7_MAX_LEVEL    1023

// MSGEQ7 has seven frequency bands
#define BANDS_TOTAL         7
#define BAND_INDEX_63HZ     0
#define BAND_INDEX_160HZ    1
#define BAND_INDEX_400HZ    2
#define BAND_INDEX_1000HZ   3
#define BAND_INDEX_2500HZ   4
#define BAND_INDEX_6250HZ   5
#define BAND_INDEX_16000HZ  6

// Our color organ has four channels.
#define COLOR_ORGAN_CHANNELS 4
#define BAND_LOW_INDEX      0
#define BAND_LOW_MID_INDEX  1
#define BAND_HIGH_MID_INDEX 2
#define BAND_HIGH_INDEX     3

// Assign a hue for each channel
#define HUE_LOW             359.0  // Low freqs are red
#define HUE_LOW_MID         60.0   // Low mid freqs are yellow
#define HUE_HIGH_MID        120.0  // High mid freqs are green
#define HUE_HIGH            240.0  // High freqs are blue

/*****************************************************************************/
// MSGEQ7 I/O Pin Definitions 
/*****************************************************************************/
#define MSGEQ7_RESET_PIN    9
#define MSGEQ7_STROBE_PIN   10
#define MSGEQ7_IN_PIN       A0

/*****************************************************************************/
// Misc I/O Pin Definitions 
/*****************************************************************************/
#define FUNC_SWITCH_PIN     A1

/*****************************************************************************/
// Static Color Data
/*****************************************************************************/
RGB BLK = {COLOR_COMPONENT_MIN, COLOR_COMPONENT_MIN, COLOR_COMPONENT_MIN};
RGB BLU = {COLOR_COMPONENT_MIN, COLOR_COMPONENT_MIN, COLOR_COMPONENT_MAX};
RGB GRN = {COLOR_COMPONENT_MIN, COLOR_COMPONENT_MAX, COLOR_COMPONENT_MIN};
RGB RED = {COLOR_COMPONENT_MAX, COLOR_COMPONENT_MIN, COLOR_COMPONENT_MIN};
RGB YEL = {COLOR_COMPONENT_MAX, COLOR_COMPONENT_MAX, COLOR_COMPONENT_MIN};
RGB WHT = {COLOR_COMPONENT_MAX, COLOR_COMPONENT_MAX, COLOR_COMPONENT_MAX};

/*****************************************************************************/
// Base Hardware Global Data
/*****************************************************************************/

// Array for storage of LED data which is shifted out to TLC5940s
// 12 bits (a byte and a half) are used for each LED to support
// 12 bit PWM. Each TLC has 16 channels which requiring 12 bits of data each
byte LEDData[NUM_ROWS][DATA_BYTES_PER_ROW];

// 3D mapping of Seeed Studios Cube LEDs to our LED numbers
// Array is [z][y][x]
byte CubeLEDNumber[MAX_DIMENSION][MAX_DIMENSION][MAX_DIMENSION] = {
  {{56,57,58,59},{63,62,61,60},{24,25,26,27},{31,30,29,28}},
  {{48,49,50,51},{55,54,53,52},{16,17,18,19},{23,22,21,20}},
  {{40,41,42,43},{47,46,45,44},{ 8, 9,10,11},{15,14,13,12}},
  {{32,33,34,35},{39,38,37,36},{ 0, 1, 2, 3},{ 7, 6, 5, 4}}
};

/*****************************************************************************/
// Color Organ Global Data
/*****************************************************************************/

// Array of audio data returned from MSGEQ7
word AudioSpectrumData[BANDS_TOTAL];

// Array of normalized audio data
float AudioSpectrumDataNormalized[COLOR_ORGAN_CHANNELS];

// Arrays of LEDs per frequency
byte lowFreqLEDs[LEDS_PER_FREQ];
byte lowMidFreqLEDs[LEDS_PER_FREQ];
byte highMidFreqLEDs[LEDS_PER_FREQ];
byte highFreqLEDs[LEDS_PER_FREQ];

/*****************************************************************************/
// Utility Routines
/*****************************************************************************/

#ifdef DEBUG
// printf equivalent
void p(char *fmt, ... ) {
  char tmp[80]; // resulting string limited to 80 chars
  va_list args;
  va_start (args, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end (args);
  tmp[sizeof(tmp)/sizeof(tmp[0])-1]='\0';
  Serial.print(tmp);
}

extern unsigned int __bss_end;
extern void *__brkval;

// Print the amount of free memory
void printFreeMemory(char tag) {
  int free_memory;

  if((int)__brkval == 0) {
     free_memory = ((int)&free_memory) - ((int)&__bss_end);
  }  else  {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  p("RAM-%c: %d\n", tag, free_memory);
}
#endif

// Map an input value into an output value
float mmap(float in, float inMin, float inMax, float outMin, float outMax) {
    return (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
}

// Read the function switch
boolean readFunctionSwitch() {
  return (digitalRead(FUNC_SWITCH_PIN) == HIGH);
}

/*****************************************************************************/
// Color and Palette Functions
/*****************************************************************************/

// Input a value 0 to 95 to get a color value.
// The colors are a transition r - g - b - back to r
void colorWheel(word wheelPos, RGB *color) {
  
  wheelPos %= 96;
  
  switch(wheelPos >> 5) {
    
  case 0:
    color->red   = 31 - wheelPos % 32; // Red down
    color->green = wheelPos % 32;      // Green up
    color->blue  = 0;                  // Blue off
    break; 
  case 1:
    color->red   = 0;                  // Red off
    color->green = 31 - wheelPos % 32; // Green down
    color->blue  = wheelPos % 32;      // Blue up
    break; 
  case 2:
    color->red   = wheelPos % 32;      // Red up
    color->green = 0;                  // Green off
    color->blue  = 31 - wheelPos % 32; // Blue down 
    break; 
  }
  // Scale to COLOR_COMPONENT_MAX
  color->red   *= 128;
  color->green *= 128;
  color->blue  *= 128;
}

// Color space conversion from HSV to RGB
// Input arguments
// hue in degrees (0 - 360.0)
// saturation in percent (0.0 - 1.0)
// value in percent (0.0 - 1.0)
// Output arguments
// red, green blue (0.0 - 1.0)
void HSVtoRGB(float hue, float saturation, float value, float *red, float *green, float *blue) {

  word i;
  float f, p, q, t;
  if (saturation == 0) {
    // achromatic (grey)
    *red = *green = *blue = value;
    return;
  }
  hue /= 60;			// sector 0 to 5
  i = floor(hue);
  f = hue - i;			// factorial part of h
  p = value * (1 - saturation);
  q = value * (1 - saturation * f);
  t = value * (1 - saturation * (1 - f));
  switch(i) {
  case 0:
    *red = value;
    *green = t;
    *blue = p;
    break;
  case 1:
    *red = q;
    *green = value;
    *blue = p;
    break;
  case 2:
    *red = p;
    *green = value;
    *blue = t;
    break;
  case 3:
    *red = p;
    *green = q;
    *blue = value;
    break;
  case 4:
    *red = t;
    *green = p;
    *blue = value;
    break;
  default:		// case 5:
    *red = value;
    *green = p;
    *blue = q;
    break;
  }
}

void HSVtoRGB(float hue, float saturation, float value, RGB *color) {
  float r, g, b;

  // Generate an RGB color
  HSVtoRGB(hue, saturation, value, &r, &g, &b);
  color->red   = (word) (r * COLOR_COMPONENT_MAX);
  color->green = (word) (g * COLOR_COMPONENT_MAX);
  color->blue =  (word) (b * COLOR_COMPONENT_MAX);
}

// Convert HSV to RGB returning values in range 0 .. COLOR_COMPONENT_MAX
void HSVtoRGB(float hue, float value, word *red, word *green, word *blue) {
  float r, g, b;

  // Generate a full saturated RGB color
  HSVtoRGB(hue, 1.0, value, &r, &g, &b);
  *red   = (word) (r * COLOR_COMPONENT_MAX);
  *green = (word) (g * COLOR_COMPONENT_MAX);
  *blue =  (word) (b * COLOR_COMPONENT_MAX);
}

void HSVtoRGB(float hue, float value, RGB *color) {
  float r, g, b;

  // Generate a full saturated RGB color
  HSVtoRGB(hue, 1.0, value, &r, &g, &b);
  color->red   = (word) (r * COLOR_COMPONENT_MAX);
  color->green = (word) (g * COLOR_COMPONENT_MAX);
  color->blue =  (word) (b * COLOR_COMPONENT_MAX);
}

// Create a fully saturated HSV color
void createHSVColor(byte divisions, byte index, RGB *color) {
 
   float hueAngle = (360.0 * index) / divisions;
   HSVtoRGB(hueAngle, 1.0, color);
}

/*****************************************************************************/
// Program initialization entry point
/*****************************************************************************/

void setup() {  

#ifdef DEBUG
  // Initialize serial interface for monitor
  Serial.begin(115200);
#endif

  // Seed random number generator with floating analog input
  randomSeed(analogRead(A5));

  // Clear the LEDData array
  clearLEDData();
  
  // Define SPI SCLK output as LOW
  pinMode(SPI_SCLK_PIN, OUTPUT);
  digitalWrite(SPI_SCLK_PIN, LOW);
  
  // Define SPI MOSI output as LOW
  pinMode(SPI_MOSI_PIN, OUTPUT);
  digitalWrite(SPI_MOSI_PIN, LOW);
   
  // Define SPI SS output as LOW
  // pinMode(SPI_SS_PIN, OUTPUT);
  // digitalWrite(SPI_SS_PIN, LOW);
   
  // Define Timer 2 output as LOW
  pinMode(TIMER2_GSCLK_PIN, OUTPUT);
  digitalWrite(TIMER2_GSCLK_PIN, LOW); 
  
  // Define TLC XLAT output as LOW
  pinMode(TLC_XLAT_PIN, OUTPUT);
  digitalWrite(TLC_XLAT_PIN, LOW); 
  
  // Define TLC BLANK output as HIGH
  pinMode(TLC_BLANK_PIN, OUTPUT);
  digitalWrite(TLC_BLANK_PIN, HIGH); 
  
  // Define DECODER enable as output as LOW
  pinMode(DECODER_ENABLE_PIN, OUTPUT);
  digitalWrite(DECODER_ENABLE_PIN, HIGH); 
 
  // Define DECODER A as output as LOW
  pinMode(DECODER_A_PIN, OUTPUT);
  digitalWrite(DECODER_A_PIN, LOW); 
 
  // Define DECODER B as output as LOW
  pinMode(DECODER_B_PIN, OUTPUT);
  digitalWrite(DECODER_B_PIN, LOW); 
 
  // Define DECODER C as output as LOW
  pinMode(DECODER_C_PIN, OUTPUT);
  digitalWrite(DECODER_C_PIN, LOW); 
 
  // Color Organ initialization
  
  // Define MSGEQ7 input with pullup
  pinMode(MSGEQ7_IN_PIN, INPUT);
  digitalWrite(MSGEQ7_IN_PIN, HIGH);
  
  // Define MSGEQ7 reset output as HIGH
  pinMode(MSGEQ7_RESET_PIN, OUTPUT);
  digitalWrite(MSGEQ7_RESET_PIN, HIGH);
  
  // Define MSGEQ7 strobe output as HIGH
  pinMode(MSGEQ7_STROBE_PIN, OUTPUT);
  digitalWrite(MSGEQ7_STROBE_PIN, HIGH);
  
  // Define function switch input with pullup
  pinMode(FUNC_SWITCH_PIN, INPUT);
  digitalWrite(FUNC_SWITCH_PIN, HIGH);

  // SPI initialization
  SPCR = _BV(SPE) | _BV(MSTR);   // SPI enabled in master mode
  SPSR = _BV(SPI2X);             // SPI double speed

  // 16 bit Timer 1 initialization
  // Timer 1 counts from 0 to TLC_PWM_CLOCKS and
  // generates an interrupt when it gets there.
  // Timer 1 is not connected to any output pins of processor
  TCCR1A = 0;            // Clear timer register on general principles
  TCCR1B = _BV(WGM13);   // Phase/freq correct PWM, ICR1 is TOP
  ICR1 = TLC_PWM_CLOCKS; // Set ICR1 to TOP value
  TIFR1 |= _BV(TOV1);    // Enable overflow flag
  TIMSK1 = _BV(TOIE1);   // Enable mask bit to generate interrupt on overflow
    
  // 8 bit Timer 2 initialization
  // Timer 2 generates the GSCLK pulse stream
  // Output is on pin DP3 and is set on BOTTOM and cleared on TOP (OCR2A)
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Fast pwm with OCR2A top
  TCCR2B = _BV(WGM22);      // Fast pwm with OCR2A top
  OCR2B = 0;
  OCR2A = TLC_GSCLK_CLOCKS; // Set TOP
  TCCR2B |= _BV(CS20);      // Start Timer 2 with no prescale

  TCCR1B |= _BV(CS10);      // Start Timer 1 with no prescale
}

/*****************************************************************************/
// Interrupt Service Routine
// ISR called each time the TLC9540's PWM cycle has been completed. That is
// when Timer2 has output 4096 clocks.
/*****************************************************************************/

ISR(TIMER1_OVF_vect) {
  
  static byte outputRow = 0;
  
  // Shift out the data for specified row
  byte *ptr = &LEDData[outputRow][0];

  // Shift out each byte of row data
  for (word i = 0; i < DATA_BYTES_PER_ROW; i++) {
    
    // Begin transmission via SPI
    SPDR = *ptr++;
    while (! (SPSR & _BV(SPIF)))
      ; // wait for transmission complete
  }
  // Lower 74LS138 enable to disable all LED row selects turning all LEDs off
  digitalWrite(DECODER_ENABLE_PIN, LOW);

  // Set BLANK high
  digitalWrite(TLC_BLANK_PIN, HIGH);
    
  // Set XLAT high
  digitalWrite(TLC_XLAT_PIN, HIGH);
    
  // Set XLAT low
  digitalWrite(TLC_XLAT_PIN, LOW);
    
  // Set BANK low
  digitalWrite(TLC_BLANK_PIN, LOW);
 
  // Select the appropriate decoder outputs
  word a = ((outputRow & 0x01) != 0) ? HIGH : LOW;
  word b = ((outputRow & 0x02) != 0) ? HIGH : LOW;
  word c = ((outputRow & 0x04) != 0) ? HIGH : LOW;
      
  digitalWrite(DECODER_A_PIN, a);
  digitalWrite(DECODER_B_PIN, b);
  digitalWrite(DECODER_C_PIN, c);
  
  // Raise 74LS138 enable to enable specified LED row select
  digitalWrite(DECODER_ENABLE_PIN, HIGH);
 
  // Setup for next interrupt
  if (++outputRow >= NUM_ROWS) {
    outputRow = 0;
  }
}

/*****************************************************************************/
// Utility functions
/*****************************************************************************/

// Given a 3D coordinate from the LED Cube, return the led number it
// corresponds to.
byte getLEDNum(byte x, byte y, byte z) {
  return CubeLEDNumber[z][y][x];
}

// Return the blue channel number of the specified LED number
// NOTE: this function takes into account we are only driving
// 4 RGB LEDs per TLC5940.
byte channelFromLEDNum(byte ledNumber) {

  byte col = ledNumber % 8;
  
  switch(col) {
    case 0: return 0;
    case 1: return 3;
    case 2: return 6;
    case 3: return 9;
    case 4: return 16;
    case 5: return 19;
    case 6: return 22;
    case 7: return 25;
  }
  return 0;
}

// Set LED at row/channel to grayscale value (0..4095). Three such sets are required
// to define color of an RGB LED. This code is complicated because each value spans more
// than one byte in LEDData array.
void setLEDRowChannelValue(byte row, byte channel, word value) {
  
  byte ch = NUM_TLC_CHANNELS - 1 - channel;
  byte i = ch * 3 / 2;
  if ((ch % 2) == 0) {
      LEDData[row][i] = (value >> 4);
      i++;
      LEDData[row][i] = (LEDData[row][i] & 0x0F) | (byte)(value << 4);
  }  else  {
      LEDData[row][i] = (LEDData[row][i] & 0xF0) | (value >> 8);
      i++;
      LEDData[row][i] = (byte) value;
  }
}

// Get the grayscale value (0..4095) for the LED at row/channel. Three such gets are required
// to retrieve the color of an RGB LED. This code is complicated because each value spans more
// than one byte in LEDData array.
void getLEDRowChannelValue(byte row, byte channel, word *value) {
  
  word temp;
  byte ch = NUM_TLC_CHANNELS - 1 - channel;
  byte i = ch * 3 / 2;
  if ((ch % 2) == 0) {
      temp = LEDData[row][i] << 4;
      i++;
      temp |= (LEDData[row][i] & 0xF0) >> 4;
  }  else  {
      temp = (LEDData[row][i] & 0x0F) << 8;
      i++;
      temp |= LEDData[row][i];
  }
  *value = temp;
}

// Set the specifed LED to the specified Red, Green, Blue values
void setLEDRGB(byte ledNum, word red, word green, word blue) {

  // Calculate row LED is in
  byte row = ledNum / 8;

  // Calculate blue channel of LED
  byte channel = channelFromLEDNum(ledNum);
  
  // Downward adjust the red component as it seems brighter than either green or blue
  int newRed = red;
  
  newRed -= RED_ADJ;
  if (newRed < 0) {
    newRed = 0;
  }
  red = (word) newRed;

  // Set the three consecutive channels for this LED
  setLEDRowChannelValue(row, channel,   blue);
  setLEDRowChannelValue(row, channel+1, green);
  setLEDRowChannelValue(row, channel+2, red);
}

// Return the current color for the specifed LED
void getLEDRGB(byte ledNum, RGB *color) {

  word red, green, blue;
  
  // Calculate row LED is in
  byte row = ledNum / 8;

  // Calculate blue channel of LED
  byte channel = channelFromLEDNum(ledNum);
  
  // Set the three consecutive channels for this LED
  getLEDRowChannelValue(row, channel,   &blue);
  getLEDRowChannelValue(row, channel+1, &green);
  getLEDRowChannelValue(row, channel+2, &red);
  
  color->red = red;
  color->green = green;
  color->blue = blue;
}

// Set the specifed LED to the color specified in the RGB struct
void setLEDRGB(byte ledNum, RGB color) {
  setLEDRGB(ledNum, color.red, color.green, color.blue);
}

// Turn specified LED off (color is black)
void setLEDOff(byte ledNum) {
  setLEDRGB(ledNum, 0, 0, 0);
}

// Set the specifed 3D LED to the specified Red, Green, Blue values
void set3DLEDRGB(byte x, byte y, byte z, word red, word green, word blue) {
  byte ledNum = getLEDNum(x, y, z);
  setLEDRGB(ledNum, red, green, blue);
}

// Set the specifed 3D LED to the specified Red, Green, Blue values
void set3DLEDRGB(POINT3D led, word red, word green, word blue) {
  byte ledNum = getLEDNum(led.x, led.y, led.z);
  setLEDRGB(ledNum, red, green, blue);
}

// Set the specifed 3D LED to the color specified in RGB struct
void set3DLEDRGB(byte x, byte y, byte z, RGB color) {
  byte ledNum = getLEDNum(x, y, z);
  setLEDRGB(ledNum, color);
}

// Set the specifed 3D LED to the color specified in RGB struct
void set3DLEDRGB(POINT3D led, RGB color) {
  byte ledNum = getLEDNum(led.x, led.y, led.z);
  setLEDRGB(ledNum, color);
}

// Turn off the specified 3D LED (color is black)
void set3DLEDOff(byte x, byte y, byte z) {
  byte ledNum = getLEDNum(x, y, z);
  setLEDRGB(ledNum, 0, 0, 0);
}

// Turn off the specified 3D LED (color is black)
void set3DLEDOff(POINT3D led) {
  byte ledNum = getLEDNum(led.x, led.y, led.z);
  setLEDRGB(ledNum, 0, 0, 0);
}

// Clear the entire LEDData array
void clearLEDData(void) { 
  memset(LEDData, 0, sizeof(LEDData));
}

// Set cube to single color
void setCubeToColor(RGB color) {
  for (byte i = 0; i < NUM_RGB_LEDS; i++) {
    setLEDRGB(i, color);
  }
}

#define FADE_DELAY_MS  10
#define FADE_PAUSE_DELAY_MS  500
#define FADE_INCREMENT 40

// Gradually fade cube to black 
void fadeToBlack() {
  
  RGB color;
  int t;
  boolean allBlack = false;
  
  while (! allBlack) {
    allBlack = true;
    for (byte n = 0; n < NUM_RGB_LEDS; n++) {
      // Get color of LED
      getLEDRGB(n, &color);
      
      if ((color.red != 0) || (color.green != 0) || (color.blue != 0)) {
        allBlack = false;
        
        // Process red
        t = color.red;
        t -= FADE_INCREMENT / 2;
        if (t < 0) {
          t = 0;
        }
        color.red = t;
        
        // Process green
        t = color.green;
        t -= FADE_INCREMENT;
        if (t < 0) {
          t = 0;
        }
        color.green = t;
        
        // Process blue
        t = color.blue;
        t -= FADE_INCREMENT;
        if (t < 0) {
          t = 0;
        }
        color.blue = t;
        
        // Set dimmed color
        setLEDRGB(n, color);
      }
    }
    delay(FADE_DELAY_MS);
  }
  delay(FADE_PAUSE_DELAY_MS);
}

/*****************************************************************************/
// Color Organ Code
/*****************************************************************************/

// Array keeps track of which LEDs have been allocated to a color organ band
byte AllocatedLEDArray[NUM_RGB_LEDS];

// Return an unallocated LED from the 64 available
byte allocateLED() {
  
  byte ledNum;
  
  while (1) {
    // Get a random LED number
    ledNum = random(NUM_RGB_LEDS);
    
    // Was this LED previously allocated ?
    if (AllocatedLEDArray[ledNum] == 0) {
      // Not previously allocated so allocate it now and return it
      AllocatedLEDArray[ledNum] = 1;
      return ledNum;
    }
  }
}

// Allocate the LEDs to be used for each freq band
void allocateLEDs() {
  
  // Clear allocated LED array
  memset(AllocatedLEDArray, 0, sizeof(AllocatedLEDArray));
  
  // Allocate low freq LEDs
  for (byte n = 0; n < LEDS_PER_FREQ; n++) {
    lowFreqLEDs[n] = allocateLED();
  }
  
  // Allocate low mid freq LEDs
  for (byte n = 0; n < LEDS_PER_FREQ; n++) {
    lowMidFreqLEDs[n] = allocateLED();
  }
  
  // Allocate high mid freq LEDs
  for (byte n = 0; n < LEDS_PER_FREQ; n++) {
    highMidFreqLEDs[n] = allocateLED();
  }
  
  // Allocate high freq LEDs
  for (byte n = 0; n < LEDS_PER_FREQ; n++) {
    highFreqLEDs[n] = allocateLED();
  }  
}

// Acquire audio data from the MSGEQ7 chip and threshold it to remove noise.
// Return true if sound data was detected; false if not
boolean acquireAudioData() {
  
  // Read the audio spectrum from the MSGEQ7
  // Reset the MSGEQ7 to start the process
  digitalWrite(MSGEQ7_RESET_PIN, HIGH);
  __asm__("nop\n\t""nop\n\t");  // Reset pulse width 100ns min
  digitalWrite(MSGEQ7_RESET_PIN, LOW);
  delayMicroseconds(80);        // Reset to strobe 72 us min
 
  for (byte ch = 0; ch < BANDS_TOTAL; ch++) {
    digitalWrite(MSGEQ7_STROBE_PIN, LOW);
    delayMicroseconds(40);      // Output settling time 36 us min
    AudioSpectrumData[ch] = analogRead(MSGEQ7_IN_PIN);
    digitalWrite(MSGEQ7_STROBE_PIN, HIGH);    
    delayMicroseconds(80);      // Strobe to strobe 72 us min
  } 
  // Reduce the noise in the data
  // Process the 60Hz band separately
  if (AudioSpectrumData[BAND_INDEX_63HZ] < NOISE_THRESHOLD_60) {
    AudioSpectrumData[BAND_INDEX_63HZ] = 0;
  }
  // Process the rest of the bands
  for (word band = 1; band < BANDS_TOTAL; band++) {
    if (AudioSpectrumData[band] < NOISE_THRESHOLD_NORM) {
      AudioSpectrumData[band] = 0;
    }
  }
/* 
  p("1: %d, 2: %d, 3: %d, 4: %d, 5: %d, 6: %d, 7: %d\n",
    AudioSpectrumData[0],
    AudioSpectrumData[1],
    AudioSpectrumData[2],
    AudioSpectrumData[3],
    AudioSpectrumData[4],
    AudioSpectrumData[5],
    AudioSpectrumData[6]);
*/    
  // Is there data to display ?
  if ((AudioSpectrumData[0] == 0) &&
      (AudioSpectrumData[1] == 0) &&
      (AudioSpectrumData[2] == 0) &&
      (AudioSpectrumData[3] == 0) &&
      (AudioSpectrumData[4] == 0) &&
      (AudioSpectrumData[5] == 0) &&
      (AudioSpectrumData[6] == 0)) {
    // Return indication of no data
    return false;
  }
  // Return indication of data
  return true;
}

#define AUDIO_FADE_COUNT 8

// Detect whether sound is present
boolean detectSound() {
  
  // Read repeatedly to fade the data provided by MSGEQ7
  for (byte i = 0; i < AUDIO_FADE_COUNT; i++) {
    acquireAudioData();
  }
  
  // After data is faded, do the real detection
  return acquireAudioData();
}

// Color Organ Code - Uses distributed random LEDs per channel
void colorOrganCode() {
  
  byte ledNum;
  word red, green, blue;
  
  // Allocate the LEDs in the cube for each frequency band
  allocateLEDs();
  
  while (1) {

    // Check the function switch every loop
    if (! readFunctionSwitch()) {
      return;
    }
    
    // Is there data to acquire and process ?
    if (! acquireAudioData()) {
      // Turn off all LEDs
      clearLEDData();
      continue;
    }
    // Normalize the data into range 0.0 .. 1.0    
    // Low band is combination of two lowest frequencies
    float level1 = AudioSpectrumData[BAND_INDEX_63HZ];
    float level2 = AudioSpectrumData[BAND_INDEX_160HZ];
    float newLevel = (level1 + level2) / 2.0;
    AudioSpectrumDataNormalized[BAND_LOW_INDEX] = newLevel / MSGEQ7_MAX_LEVEL;

    // Low Mid band
    AudioSpectrumDataNormalized[BAND_LOW_MID_INDEX] = ((float) AudioSpectrumData[BAND_INDEX_400HZ]) / MSGEQ7_MAX_LEVEL;
    
    // High Mid band
    AudioSpectrumDataNormalized[BAND_HIGH_MID_INDEX] = ((float) AudioSpectrumData[BAND_INDEX_2500HZ]) / MSGEQ7_MAX_LEVEL;
    
    // High band is combination of two highest frequencies
    level1 = AudioSpectrumData[BAND_INDEX_6250HZ];
    level2 = AudioSpectrumData[BAND_INDEX_16000HZ];
    newLevel = (level1 + level2) / 2.0;
    AudioSpectrumDataNormalized[BAND_HIGH_INDEX] = newLevel / MSGEQ7_MAX_LEVEL;
     
    // Gamma correct the color brightness value
    float oneOverGamma = 1.0 / GAMMA;
    for (word ch = 0; ch < COLOR_ORGAN_CHANNELS; ch++) {
      AudioSpectrumDataNormalized[ch] = pow(AudioSpectrumDataNormalized[ch], oneOverGamma);
    }
    
    // Generate color for low band
    HSVtoRGB(HUE_LOW, AudioSpectrumDataNormalized[BAND_LOW_INDEX], &red, &green, &blue);
     
    // Light all low freq LEDs with calculated color
    for (byte led = 0; led < LEDS_PER_FREQ; led++) {
      ledNum = lowFreqLEDs[led];
      setLEDRGB(ledNum, red, green, blue);
    }
       
    // Generate color for low mid band
    HSVtoRGB(HUE_LOW_MID, AudioSpectrumDataNormalized[BAND_LOW_MID_INDEX], &red, &green, &blue);
     
    // Light all low mid freq LEDs with calculated color
    for (byte led = 0; led < LEDS_PER_FREQ; led++) {
      ledNum = lowMidFreqLEDs[led];
      setLEDRGB(ledNum, red, green, blue);
    }
       
    // Generate color for high mid band
    HSVtoRGB(HUE_HIGH_MID, AudioSpectrumDataNormalized[BAND_HIGH_MID_INDEX], &red, &green, &blue);
     
    // Light all high mid freq LEDs with calculated color
    for (byte led = 0; led < LEDS_PER_FREQ; led++) {
      ledNum = highMidFreqLEDs[led];
      setLEDRGB(ledNum, red, green, blue);
    }
       
    // Generate color for high band
    HSVtoRGB(HUE_HIGH, AudioSpectrumDataNormalized[BAND_HIGH_INDEX], &red, &green, &blue);
     
    // Light all high freq LEDs with calculated color
    for (byte led = 0; led < LEDS_PER_FREQ; led++) {
      ledNum = highFreqLEDs[led];
      setLEDRGB(ledNum, red, green, blue);
    }
  }
}

/*****************************************************************************/
// Blue Green Sweep Code
/*****************************************************************************/

void blueGreenSweepCode(void) {

  byte x1, y1, z1, loops = 3;

  while (loops--) {
    for (byte z = 0; z < MAX_DIMENSION; z++) {
      for (byte y = 0; y < MAX_DIMENSION; y++) {
        for (byte x = 0; x < MAX_DIMENSION; x++) {
          x1 = MAX_INDEX - x;
          y1 = MAX_INDEX - y;
          z1 = MAX_INDEX - z;
        
          set3DLEDRGB(x, y, z, BLU);
          set3DLEDRGB(x1, y1, z1, GRN);
          delay(100);
          set3DLEDOff(x, y, z);
          set3DLEDOff(x1, y1, z1);
        }
      }
    }
  }
}

/*****************************************************************************/
// Walls Code
/*****************************************************************************/

void drawFrontWithColor(RGB color) {
  for (byte z = 0; z < MAX_DIMENSION; z++) {
    for (byte x = 0; x < MAX_DIMENSION; x++) {
      set3DLEDRGB(x, 0, z, color);
    }
  }
}  

void drawRightWithColor(RGB color) {
  for (byte z = 0; z < MAX_DIMENSION; z++) {
    for (byte y = 0; y < MAX_DIMENSION; y++) {
      set3DLEDRGB(3, y, z, color);
    }
  }
}  

void drawLeftWithColor(RGB color) {
  for (byte z = 0; z < MAX_DIMENSION; z++) {
    for (byte y = 0; y < MAX_DIMENSION; y++) {
      set3DLEDRGB(0, y, z, color);
    }
  }
}

void drawBackWithColor(RGB color) {
  for (byte z = 0; z < MAX_DIMENSION; z++) {
    for (byte x = 0; x < MAX_DIMENSION; x++) {
      set3DLEDRGB(x, 3, z, color);
    }
  }
}  

void drawTopWithColor(RGB color) {
  for (byte y = 0; y < MAX_DIMENSION; y++) {
    for (byte x = 0; x < MAX_DIMENSION; x++) {
      set3DLEDRGB(x, y, 3, color);
    }
  }
}  

void drawBottomWithColor(RGB color) {
  for (byte y = 0; y < MAX_DIMENSION; y++) {
    for (byte x = 0; x < MAX_DIMENSION; x++) {
      set3DLEDRGB(x, y, 0, color);
    }
  }
}  

RGB RW_Select_Color() {
  
  static byte index = 0;
  
  switch(index++ % 5) {
    case 0: return RED;
    case 1: return GRN;
    case 2: return BLU;
    case 3: return YEL;
    case 4: return WHT;
  }
  return BLU;
}

#define DELAY_MAX 400
#define DELAY_MIN  10
#define DELAY_INC  10

word currentDelay = 0;
boolean up = true;

void RW_Select_Delay() {
  
  if (up) {
    currentDelay += DELAY_INC;
    if (currentDelay > DELAY_MAX) {
      currentDelay = DELAY_MAX;
      up = false;
    }
  }  else  {
    currentDelay -= DELAY_INC;
    if (currentDelay < DELAY_MIN) {
      currentDelay = DELAY_MIN;
      up = true;
    }
  }
  delay(currentDelay);
}

void wallsCode() {
  
  byte loops = 20;
  while(loops--) {
    
    // Draw wall
    drawFrontWithColor(RW_Select_Color());
    RW_Select_Delay();
    clearLEDData();
  
    // Draw wall
    drawRightWithColor(RW_Select_Color());
    RW_Select_Delay();
    clearLEDData();
  
    // Draw wall
    drawBackWithColor(RW_Select_Color());
    RW_Select_Delay();
    clearLEDData();
  
    // Draw wall
    drawLeftWithColor(RW_Select_Color());
    RW_Select_Delay();
    clearLEDData();
  
    // Draw wall
    drawFrontWithColor(RW_Select_Color());
    RW_Select_Delay();
    clearLEDData();
  
    // Draw wall
    drawTopWithColor(RW_Select_Color());
    RW_Select_Delay();
    clearLEDData();  

    // Draw wall
    drawBackWithColor(RW_Select_Color());
    RW_Select_Delay();
    clearLEDData();  
    
    // Draw wall
    drawBottomWithColor(RW_Select_Color());
    RW_Select_Delay();
    clearLEDData();  
  }  
}

void walls2Code() {
  
  clearLEDData();
  
  byte loops = 50;
  while(loops--) {
    
    // Draw wall
    drawFrontWithColor(RW_Select_Color());
    delay(50);
    clearLEDData();
  
    // Draw wall
    drawRightWithColor(RW_Select_Color());
    delay(50);
    clearLEDData();
  
    // Draw wall
    drawBackWithColor(RW_Select_Color());
    delay(50);
    clearLEDData();
  
    // Draw wall
    drawLeftWithColor(RW_Select_Color());
    delay(50);
    clearLEDData();
  
    // Draw wall
    drawFrontWithColor(RW_Select_Color());
    delay(50);
    clearLEDData();
  
    // Draw wall
    drawTopWithColor(RW_Select_Color());
    delay(50);
    clearLEDData();  

    // Draw wall
    drawBackWithColor(RW_Select_Color());
    delay(50);
    clearLEDData();  
    
    // Draw wall
    drawBottomWithColor(RW_Select_Color());
    delay(50);
    clearLEDData();  
  }  
}

/*****************************************************************************/
// Chaser Code
/*****************************************************************************/

POINT3D track [] = {
  {0,0,0}, {0,0,1}, {0,0,2}, {0,0,3},
  {1,0,3}, {1,0,2}, {1,0,1}, {1,0,0},
  {2,0,0}, {2,0,1}, {2,0,2}, {2,0,3},
  {3,0,3}, {3,0,2}, {3,0,1}, {3,0,0},
  {3,1,0}, {3,1,1}, {3,1,2}, {3,1,3},
  {3,2,3}, {3,2,2}, {3,2,1}, {3,2,0},
  {3,3,0}, {3,3,1}, {3,3,2}, {3,3,3},
  {2,3,3}, {2,3,2}, {2,3,1}, {2,3,0},
  {1,3,0}, {1,3,1}, {1,3,2}, {1,3,3},
  {0,3,3}, {0,3,2}, {0,3,1}, {0,3,0},
  {0,2,0}, {0,2,1}, {0,2,2}, {0,2,3},
  {0,1,3}, {0,1,2}, {0,1,1}, {0,1,0},
  {0,0,0}, {0,0,1}, {0,0,2}, {0,0,3},

  // Top
  {1,0,3}, {2,0,3}, {3,0,3},
  {3,1,3}, {2,1,3}, {1,1,3}, {0,1,3},
  {0,2,3}, {1,2,3}, {2,2,3}, {3,2,3},
  {3,3,3}, {2,3,3}, {1,3,3}, {0,3,3},

  {0,3,2}, {0,3,1}, {0,3,0},

  // Bottom
  {1,3,0}, {2,3,0}, {3,3,0},
  {3,2,0}, {2,2,0}, {1,2,0}, {0,2,0},
  {0,1,0}, {1,1,0}, {2,1,0}, {3,1,0},
  {3,0,0}, {2,0,0}, {1,0,0}, {0,0,0},

  // Inside
  {1,1,0}, {2,1,0}, {2,2,0}, {1,2,0},
  {1,2,1}, {1,1,1}, {2,1,1}, {2,2,1},
  {2,2,2}, {1,2,2}, {1,1,2}, {2,1,2},
  {2,1,3}, {2,2,3}, {1,2,3}, {1,1,3}  
};

#define NUMBER_OF_LEDS (sizeof(track) / sizeof(POINT3D))   

#define CHASER_DELAY 12

void chaserCode() {
  
  RGB hueColor, hueColor1, hueColor2, hueColor3;
  byte loops = 3;
  while (loops--) {
    for (byte i = 0; i < NUMBER_OF_LEDS; i++) {
    
      createHSVColor(NUMBER_OF_LEDS, i,   &hueColor);
      createHSVColor(NUMBER_OF_LEDS, i-1, &hueColor1);
      createHSVColor(NUMBER_OF_LEDS, i-2, &hueColor2);
      createHSVColor(NUMBER_OF_LEDS, i-3, &hueColor3);
    
      POINT3D led = track[i];
      set3DLEDRGB(led, hueColor);
      delay(CHASER_DELAY);
    
      if (i >= 1) {
        led = track[i-1];
        set3DLEDRGB(led, hueColor1);
        delay(CHASER_DELAY);
      }
    
      if (i >= 2) {
        led = track[i-2];
        set3DLEDRGB(led, hueColor2);
        delay(CHASER_DELAY);
      }
    
      if (i >= 3) {
        led = track[i-3];
        set3DLEDRGB(led, hueColor3);
        delay(CHASER_DELAY);
      }
    
      if (i >= 4) {
        led = track[i-4];
        set3DLEDOff(led);
        delay(CHASER_DELAY);
      }
    }
    clearLEDData();
  
    byte i1 = 0;
    for (byte i = NUMBER_OF_LEDS - 1; i > 0; i--) {
      createHSVColor(NUMBER_OF_LEDS, i1,   &hueColor);
      createHSVColor(NUMBER_OF_LEDS, i1+1, &hueColor1);
      createHSVColor(NUMBER_OF_LEDS, i1+2, &hueColor2);
      createHSVColor(NUMBER_OF_LEDS, i1+3, &hueColor3);
      i1++;
    
      POINT3D led = track[i];
      set3DLEDRGB(led, hueColor);
      delay(CHASER_DELAY);
    
      if (i + 1 < NUMBER_OF_LEDS) {
        led = track[i+1];
        set3DLEDRGB(led, hueColor1);
        delay(CHASER_DELAY);
      }
    
      if (i + 2 < NUMBER_OF_LEDS) {
        led = track[i+2];
        set3DLEDRGB(led, hueColor2);
        delay(CHASER_DELAY);
      }
    
      if (i + 3 < NUMBER_OF_LEDS) {
        led = track[i+3];
        set3DLEDRGB(led, hueColor3);
        delay(CHASER_DELAY);
      }
    
      if (i + 4 < NUMBER_OF_LEDS) {
        led = track[i+4];
        set3DLEDOff(led);
        delay(CHASER_DELAY);
      }
    }
  }
}
/*****************************************************************************/
// Slices Code
/*****************************************************************************/

void xSlice(byte x, RGB color) {
  
  for (byte z = 0; z < MAX_DIMENSION; z++) {
    for (byte y = 0; y < MAX_DIMENSION; y++) {
      set3DLEDRGB(x, y, z, color);
    }
  }
}

void ySlice(byte y, RGB color) {
  
  for (byte z = 0; z < MAX_DIMENSION; z++) {
    for (byte x = 0; x < MAX_DIMENSION; x++) {
      set3DLEDRGB(x, y, z, color);
    }
  }
}

void zSlice(byte z, RGB color) {
  
  for (byte y = 0; y < MAX_DIMENSION; y++) {
    for (byte x = 0; x < MAX_DIMENSION; x++) {
      set3DLEDRGB(x, y, z, color);
    }
  }
}

#define SLICE_DELAY 100

void slicesCode() {
  
  RGB color;
    
  byte count, colorIndex = 0;
  
  byte loops = 2;
  while (loops--) {
    
    count = 4;
    while (count--) {
      // Do x slices
      for (byte x = 0; x < MAX_DIMENSION; x++) {
        colorWheel(colorIndex++, &color);
        xSlice(x, color);
        delay(SLICE_DELAY);
        xSlice(x, BLK);
      }
    
      for (byte x = MAX_INDEX; x > 0; x--) {
        colorWheel(colorIndex++, &color);
        xSlice(x, color);
        delay(SLICE_DELAY);
        xSlice(x, BLK);
      }
    }
    
    count = 4;
    while (count--) {    
      // Do z slices
      for (byte z = 0; z < MAX_DIMENSION; z++) {
        colorWheel(colorIndex++, &color);
        zSlice(z, color);
        delay(SLICE_DELAY);
        zSlice(z, BLK);
      }
    
      for (byte z = MAX_INDEX; z > 0; z--) {
        colorWheel(colorIndex++, &color);
        zSlice(z, color);
        delay(SLICE_DELAY);
        zSlice(z, BLK);
      }
    }
    
    count = 4;
    while (count--) {    
      // Do y slices
      for (byte y = 0; y < MAX_DIMENSION; y++) {
        colorWheel(colorIndex++, &color);
        ySlice(y, color);
        delay(SLICE_DELAY);
        ySlice(y, BLK);
      }
    
      for (byte y = MAX_INDEX; y > 0; y--) {
        colorWheel(colorIndex++, &color);
        ySlice(y, color);
        delay(SLICE_DELAY);
        ySlice(y, BLK);
      }
    }
  }
}

void slices2Code() {
  
  RGB color;
  
  byte count, colorIndex = 0;
  
  byte loops = 2;
  while (loops--) {
    
    count = 4;
    while (count--) {
      // Do x slices
      for (byte x = 0; x < MAX_DIMENSION; x++) {
        colorWheel(colorIndex++, &color);
        xSlice(x, color);
        delay(SLICE_DELAY);
      }
    
      for (byte x = 0; x < MAX_DIMENSION; x++) {
        xSlice(x, BLK);
        delay(SLICE_DELAY);
      }
    }
    
    count = 4;
    while (count--) {    
      // Do z slices
      for (byte z = 0; z < MAX_DIMENSION; z++) {
        colorWheel(colorIndex++, &color);
        zSlice(z, color);
        delay(SLICE_DELAY);
      }
    
      for (byte z = 0; z < MAX_DIMENSION; z++) {
        zSlice(z, BLK);
        delay(SLICE_DELAY);
      }
    }
    
    count = 4;
    while (count--) {    
      // Do y slices
      for (byte y = 0; y < MAX_DIMENSION; y++) {
        colorWheel(colorIndex++, &color);
        ySlice(y, color);
        delay(SLICE_DELAY);
      }
    
      for (byte y = 0; y < MAX_DIMENSION; y++) {
        ySlice(y, BLK);
        delay(SLICE_DELAY);
      }
    }
  }
}

/*****************************************************************************/
// Random Code
/*****************************************************************************/

void randomCode() {
  
  RGB color;

  word loops = 500;
  while (loops--) {
    // Pick a random LED
    byte n = random(64);  
      
    // Pick a random color
    byte colorIndex = random(96);
    colorWheel(colorIndex, &color);
    setLEDRGB(n, color);
    delay(20);
  }
}  

/*****************************************************************************/
// Flash Code
/*****************************************************************************/

#define FLASH_ON_DELAY  150
#define FLASH_OFF_DELAY 800

void flashCode() {
  
  byte count = 5;
  while (count--) {
    setCubeToColor(RED);
    delay(FLASH_ON_DELAY);
    setCubeToColor(BLK);
    delay(FLASH_OFF_DELAY);
    setCubeToColor(GRN);
    delay(FLASH_ON_DELAY);
    setCubeToColor(BLK);
    delay(FLASH_OFF_DELAY);
    setCubeToColor(BLU);
    delay(FLASH_ON_DELAY);
    setCubeToColor(BLK);
    delay(FLASH_OFF_DELAY);
  }
}

/*****************************************************************************/
// Rainbow Code
/*****************************************************************************/

void rainbowCode() {
  
  RGB color;
  word index = 0;
  
  for (word count = 0; count < 96 * 3; count++) {
    index = 0;
    for (byte y = 0; y < MAX_DIMENSION; y++) {
      for (byte z = 0; z < MAX_DIMENSION; z++) {
        for (byte x = 0; x < MAX_DIMENSION; x++) {
          // Pick a color
          byte colorIndex = (index++ + count) % 96;
          colorWheel(colorIndex, &color);
          set3DLEDRGB(x, y, z, color);
          delay(2);
        }
      }
    }
  }
}

void rainbow2Code() {
  
  byte index, colorIndex = 0;
  RGB color;
  
  word loops = 96 * 10;  
  while (loops-- > 0) {
    index = colorIndex;
    for (byte led = 0; led < 64; led++) {
      createHSVColor(64, index, &color);
      setLEDRGB(led, color);
      if (++index > 63) {
        index = 0;
      }
    }
    delay(25);
    if (++colorIndex > 63) {
      colorIndex = 0;
    }
  }
}

/*****************************************************************************/
// 2D Sin wave display pattern
/*****************************************************************************/

void shadowXZ(byte x, byte z, RGB color) {  
  for (byte y = 0; y < MAX_DIMENSION; y++) {
    set3DLEDRGB(x, y, z, color);
  }
}

void shadowYZ(byte y, byte z, RGB color) {  
  for (byte x = 0; x < MAX_DIMENSION; x++) {
    set3DLEDRGB(x, y, z, color);
  }
}

void shadowXY(byte x, byte y, RGB color) {  
  for (byte z = 0; z < MAX_DIMENSION; z++) {
    set3DLEDRGB(x, y, z, color);
  }
}

#define SIN_FREQ_HZ       5.5
#define OMEGA             (2.0 * 3.14159 * SIN_FREQ_HZ)
#define TIME_INC_SECONDS  0.05

void twoDSinCodeXZ() {

  byte loops = 100;  
  float amplitude, z, t = 0.0;
  
  while (loops--) {
    clearLEDData();
    for (byte x = 0; x < MAX_DIMENSION; x++) {
      amplitude = sin(OMEGA * t);      
      z = mmap(amplitude, -1, +1, 0, 3);
      shadowXZ(x, ((word) round(z)), RED);
      t += TIME_INC_SECONDS;
    }
    delay(100);
  }
}

void twoDSinCodeYZ() {
  
  byte loops = 100;  
  float amplitude, z, t = 0.0;
  
  while (loops--) {
    clearLEDData();
    for (byte y = 0; y < MAX_DIMENSION; y++) {
      amplitude = sin(OMEGA * t);      
      z = mmap(amplitude, -1, +1, 0, 3);
      shadowYZ(y, ((word) round(z)), GRN);
      t += TIME_INC_SECONDS;
    }
    delay(100);
  }
}

void twoDSinCodeXY() {
  
  byte loops = 100;  
  float amplitude, y, t = 0.0;
  
  while (loops--) {
    clearLEDData();
    for (byte x = 0; x < MAX_DIMENSION; x++) {
      amplitude = sin(OMEGA * t);      
      y = mmap(amplitude, -1, +1, 0, 3);
      shadowXY(x, ((word) round(y)), BLU);
      t += TIME_INC_SECONDS;
    }
    delay(100);
  }
}

/*****************************************************************************/
// 3D Sin wave display pattern
/*****************************************************************************/

void threeDSinCode() {
  
  word z;
  byte loops = 100;
  float Z, t = 0;
  
  while (loops--) {
    clearLEDData();
    for(byte x = 0; x < MAX_DIMENSION; x++){
       for(byte y = 0; y < MAX_DIMENSION; y++){
         Z = sin((OMEGA * t) + sqrt(pow(mmap(x,0,MAX_INDEX,-PI,PI),2) + pow(mmap(y,0,MAX_INDEX,-PI,PI),2)));
         Z = round(mmap(Z,-1,1,0,MAX_INDEX));
         z = ((word) Z);
         set3DLEDRGB(x, y, z, BLU);
       }
    }
    t += TIME_INC_SECONDS;
    delay(200);
  }
}

/*****************************************************************************/
// Random 3D lines display pattern
/*****************************************************************************/

/*
 * Line3D uses Bresenham's algorithm to generate the 3 dimensional points on a
 * line from (x1, y1, z1) to (x2, y2, z2)
 */
 
// Determine sign of a, either -1, 0, or 1
#define sgn(a) (((a)<0) ? -1 : (a)>0 ? 1 : 0)

void line3D(int x1, int y1, int z1, int x2, int y2, int z2, RGB color) {
  int x,y,z,xd,yd,zd,ax,ay,az,sx,sy,sz,dx,dy,dz;
  
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;

  ax = abs(dx) << 1;
  ay = abs(dy) << 1;
  az = abs(dz) << 1;

  sx = sgn(dx);
  sy = sgn(dy);
  sz = sgn(dz);
  
  x = x1;
  y = y1;
  z = z1;
  
  if (ax >= max(ay, az)) {    // x dominant
    yd = ay - (ax >> 1);
    zd = az - (ax >> 1);
    for (;;) {
      set3DLEDRGB(x, y, z, color);
      if (x == x2) {
        return;
      }
      if (yd >= 0) {
        y += sy;
        yd -= ax;
      }
      if (zd >= 0) {
        z += sz;
        zd -= ax;
      }
      x += sx;
      yd += ay;
      zd += az;
    }
  }  else if (ay >= max(ax, az)) {    // y dominant
     xd = ax - (ay >> 1);
     zd = az - (ay >> 1);
     for (;;) {
       set3DLEDRGB(x, y, z, color);
       if (y == y2) {
         return;
       }
       if (xd >= 0) {
         x += sx;
         xd -= ay;
       }
       if (zd >= 0) {
         z += sz;
         zd -= ay;
       }
       y += sy;
       xd += ax;
       zd += az;
     }
  }  else if (az >= max(ax, ay)) {    // z dominant
     xd = ax - (az >> 1);
     yd = ay - (az >> 1);
     for (;;) {
       set3DLEDRGB(x, y, z, color);
       if (z == z2) {
         return;
       }
       if (xd >= 0) {
         x += sx;
         xd -= az;
       }
       if (yd >= 0) {
         y += sy;
         yd -= az;
       }
       z += sz;
       xd += ax;
       yd += ay;
     }
  }
}

void randomLinesCode() {
  
  RGB color;
  byte x1, y1, z1, x2, y2, z2;
  byte loops = 100;  
  while (loops--) {    
    // Chose random endpoints
    x1 = random(MAX_DIMENSION);
    y1 = random(MAX_DIMENSION);
    z1 = random(MAX_DIMENSION);
    
    x2 = random(MAX_DIMENSION);
    y2 = random(MAX_DIMENSION);
    z2 = random(MAX_DIMENSION);
    
   createHSVColor(64, random(64), &color);

    line3D(x1, y1, z1, x2, y2, z2, color);
    delay(200);
    line3D(x1, y1, z1, x2, y2, z2, BLK);
  }
}

/*****************************************************************************/
// Cube Palette Code
/*****************************************************************************/
#define PALETTE_CHANGE_DELAY 300

void cubePaletteCode() {
  
  RGB color;
  byte loops = 6;
  while (loops--) {
    for (byte i = 0; i < 64; i++) {
      createHSVColor(64, i, &color);
      setCubeToColor(color);
      delay(PALETTE_CHANGE_DELAY);
    }
  }
}

void cubePalette2Code() {
  
  RGB color;
  float hue, sat, val, t = 0;
  
  // Pick a random period
  float huePeriod = random(1, 10);
  float satPeriod = random(1, 10);
  float valPeriod = random(1, 10);
  
  float hueOmega = ((2.0 * 3.14159) / huePeriod);
  float satOmega = ((2.0 * 3.14159) / satPeriod);
  float valOmega = ((2.0 * 3.14159) / valPeriod);
  
  word loops = 1000;
  while (loops--) {
    // Get the sine wave values for three color components
    hue = sin(hueOmega * t);
    sat = sin(satOmega * t);
    val = sin(valOmega * t);
    
    // Scale the values
    // Hue in range 0.0 .. 360.0, sat in range 0.0 .. 1.0, val in range 0.0 .. 1.0
    hue = ((hue + 1.0) / 2.0) * 360.0;
    sat = (sat + 1.0) / 2.0;
    val = (val + 1.0) / 2.0;
    val = max(val, 0.2);
    
    // Generate the appropriate color 
    HSVtoRGB(hue, sat, val, &color);
    setCubeToColor(color);
    delay(50);
    t += TIME_INC_SECONDS;
  }
}

/*****************************************************************************/
// Moving boxes display pattern
/*****************************************************************************/

POINT3D box1Locations[] = {
  {0,0,0}, {1,0,0}, {2,0,0},
  {2,1,0}, {2,2,0}, 
  {1,2,0}, {0,2,0}, {0,1,0}, {0,0,0},
  
  {0,0,1}, {1,0,1}, {2,0,1},
  {2,1,1}, {2,2,1}, 
  {1,2,1}, {0,2,1}, {0,1,1}, {0,0,1},
  
  {0,0,2}, {1,0,2}, {2,0,2},
  {2,1,2}, {2,2,2}, 
  {1,2,2}, {0,2,2}, {0,1,2}, {0,0,2}
};

POINT3D box2Locations[] = {
  {2,2,2}, {2,1,2}, {2,0,2},
  {1,0,2}, {0,0,2},
  {0,1,2}, {0,2,2}, {1,2,2}, {2,2,2},
  
  {2,2,1}, {2,1,1}, {2,0,1},
  {1,0,1}, {0,0,1},
  {0,1,1}, {0,2,1}, {1,2,1}, {2,2,1},
  
  {2,2,0}, {2,1,0}, {2,0,0},
  {1,0,0}, {0,0,0},
  {0,1,0}, {0,2,0}, {1,2,0}, {2,2,0},
};  

#define NUMBER_OF_LOCATIONS (sizeof(box1Locations) / sizeof(POINT3D))

// Draw a one unit box of specified color at specificed location
// Location is lower left corner of box
void drawBoxAtXYZ(byte x, byte y, byte z, RGB color) {
  set3DLEDRGB(x,   y,   z, color);
  set3DLEDRGB(x+1, y,   z, color);
  set3DLEDRGB(x,   y+1, z, color);
  set3DLEDRGB(x+1, y+1, z, color);
  set3DLEDRGB(x,   y,   z+1, color);
  set3DLEDRGB(x+1, y,   z+1, color);
  set3DLEDRGB(x,   y+1, z+1, color);
  set3DLEDRGB(x+1, y+1, z+1, color);
}

void drawBox(POINT3D location, RGB color) {
  drawBoxAtXYZ(location.x, location.y, location.z, color);
}

void movingBoxesCode() {

  byte loops = 3;
  while(loops--) {    
    for (byte i = 0; i < NUMBER_OF_LOCATIONS; i++) {
      drawBox(box1Locations[i], RED);
      drawBox(box2Locations[i], GRN);
      delay(200);
      drawBox(box1Locations[i], BLK);
      drawBox(box2Locations[i], BLK);
      delay(200);
    }
    for (byte i = NUMBER_OF_LOCATIONS - 1; i > 0; i--) {
      drawBox(box1Locations[i], RED);
      drawBox(box2Locations[i], GRN);
      delay(200);
      drawBox(box1Locations[i], BLK);
      drawBox(box2Locations[i], BLK);
      delay(200);
    }
  }
}  

void movingBoxes2Code() {

  RGB color;
  
  byte loops = 10;
  while(loops--) {    
    for (byte i = 0; i < NUMBER_OF_LOCATIONS; i++) {
      createHSVColor(NUMBER_OF_LOCATIONS, random(NUMBER_OF_LOCATIONS), &color);
      drawBox(box1Locations[i], color);
      delay(30);
      drawBox(box1Locations[i], BLK);
    }
    for (byte i = NUMBER_OF_LOCATIONS - 1; i > 0; i--) {
      createHSVColor(NUMBER_OF_LOCATIONS, random(NUMBER_OF_LOCATIONS), &color);
      drawBox(box1Locations[i], color);
      delay(50);
      drawBox(box1Locations[i], BLK);
    }
  }
}  

void movingBoxes3Code() {

  RGB color;
  
  byte x, y, z, colorIndex = 0;
  byte loops = 150;
  while(loops--) {
    x = random(MAX_INDEX);
    y = random(MAX_INDEX);
    z = random(MAX_INDEX);
    
    createHSVColor(32, colorIndex++, &color);
    drawBoxAtXYZ(x, y, z, color);
    delay(200);
    drawBoxAtXYZ(x, y, z, BLK);
    colorIndex %= 32;
  }
}  

/*****************************************************************************/
// Matrix display pattern
/*****************************************************************************/

#define MATRIX_BACKGROUND_COLOR BLU
#define MATRIX_RAIN_COLOR       RED
#define MATRIX_CELL_ENERGY      11
#define MATRIX_DELAY            250
#define NUMBER_OF_CELLS         10

struct CELL {
  byte x, y, z;
  byte speed;
  byte energy;
};

struct CELL matrix[NUMBER_OF_CELLS];

byte allocatedCells[MAX_DIMENSION][MAX_DIMENSION];

void allocateCell(struct CELL *cell) {
    
  while(1) {
    // Randomly place cell and determine its energy
    byte x = random(MAX_DIMENSION);
    byte y = random(MAX_DIMENSION);
    byte energy = random(1, MATRIX_CELL_ENERGY);
    
    // See if cell already allocated
    if (allocatedCells[x][y] == 0) {
       allocatedCells[x][y] = 1;
       
       // Initialize cell
       cell->x = x;
       cell->y = y;
       cell->z = MAX_INDEX;
       cell->speed = cell->energy = energy;
       return;
    }
  }
}
       
void processTheMatrix() {
  int z;
  
  for (byte i = 0; i < NUMBER_OF_CELLS; i++) {
    
    if (matrix[i].speed-- == 0) {
      matrix[i].speed = matrix[i].energy;
      z = ((int) matrix[i].z) - 1;
      if (z < 0) {
        z = MAX_INDEX;
      }
      matrix[i].z = (byte) z;
    }
  }
}

void displayTheMatrix() {
  
  byte x, y, z;

  // Set cube's color
  setCubeToColor(MATRIX_BACKGROUND_COLOR);
  
  for (byte i = 0; i < NUMBER_OF_CELLS; i++) {
    x = matrix[i].x;
    y = matrix[i].y;
    z = matrix[i].z;
    
    set3DLEDRGB(x, y, z, MATRIX_RAIN_COLOR);
  }
}

void matrixCode() {
  
  struct CELL cell;
  
  // Clear allocation array
  memset(allocatedCells, 0, sizeof(allocatedCells));
  
  // Set cube's color
  setCubeToColor(MATRIX_BACKGROUND_COLOR);
  
  // Allocate the cells in the matrix
  for (byte i = 0; i < NUMBER_OF_CELLS; i++) {
    allocateCell(&cell);
    matrix[i] = cell;
  }
  
  byte loops = 200;
  while (loops--) {
    // Display the matrix
    displayTheMatrix();

    // Process the matrix
    processTheMatrix();

    // Delay for next generation
    delay(MATRIX_DELAY);    
  }
}
   
/*****************************************************************************/
// Color Test display pattern
/*****************************************************************************/

void colorTestCode() {
  
  for (byte z = 0; z < MAX_DIMENSION; z++) {
    for (byte y = 0; y < MAX_DIMENSION; y++) {
      for (byte x = 0; x < MAX_DIMENSION; x++) {
        set3DLEDRGB(x, y, z, RED) ;
        delay(125);
        
        set3DLEDRGB(x, y, z, GRN) ;
        delay(125);
        
        set3DLEDRGB(x, y, z, BLU) ;
        delay(125);

        set3DLEDRGB(x, y, z, BLK) ;
        delay(125);
      }
    }
  }
}

/*****************************************************************************/
// Rotating Slices Code
/*****************************************************************************/

POINT3D Z3Slice[] = {
  {0,0,3}, {1,0,3}, {2,0,3}, {3,0,3},
  {3,1,3}, {3,2,3}, {3,3,3},
  {2,3,3}, {1,3,3}, {0,3,3},
  {0,2,3}, {0,1,3}
};

POINT3D Z2Slice[] = {
  {0,0,2}, {1,0,2}, {2,0,2}, {3,0,2},
  {3,1,2}, {3,2,2}, {3,3,2},
  {2,3,2}, {1,3,2}, {0,3,2},
  {0,2,2}, {0,1,2}
};

POINT3D Z1Slice[] = {
  {0,0,1}, {1,0,1}, {2,0,1}, {3,0,1},
  {3,1,1}, {3,2,1}, {3,3,1},
  {2,3,1}, {1,3,1}, {0,3,1},
  {0,2,1}, {0,1,1}
};

POINT3D Z0Slice[] = {
  {0,0,0}, {1,0,0}, {2,0,0}, {3,0,0},
  {3,1,0}, {3,2,0}, {3,3,0},
  {2,3,0}, {1,3,0}, {0,3,0},
  {0,2,0}, {0,1,0}
};

#define COLOR_LAG    1
#define RSLICE_DELAY 20

void rotatingSlicesCode() {
  
  RGB color;

  // Paint whole cube white
  setCubeToColor(WHT);
  
  byte z3ColorIndex = 0;
  byte z2ColorIndex = COLOR_LAG;
  byte z1ColorIndex = z2ColorIndex + COLOR_LAG;
  byte z0ColorIndex = z1ColorIndex + COLOR_LAG;
  
  byte loops = 200;
  while (loops--) {
    
    // Draw Z3 slice
    for (byte i = 0; i < 12; i++) {
      createHSVColor(12, z3ColorIndex++, &color);
      set3DLEDRGB(Z3Slice[i], color);
      z3ColorIndex %= 12;
    }
    z3ColorIndex++;
    
    delay(RSLICE_DELAY);
    
    // Draw Z2 slice
    for (byte i = 0; i < 12; i++) {
      createHSVColor(12, z2ColorIndex++, &color);
      set3DLEDRGB(Z2Slice[i], color);
      z2ColorIndex %= 12;
    }
    z2ColorIndex++;
    delay(RSLICE_DELAY);
    
     // Draw Z1 slice
    for (byte i = 0; i < 12; i++) {
      createHSVColor(12, z1ColorIndex++, &color);
      set3DLEDRGB(Z1Slice[i], color);
      z1ColorIndex %= 12;
    }
    z1ColorIndex++;
    delay(RSLICE_DELAY);
   
     // Draw Z0 slice
    for (byte i = 0; i < 12; i++) {
      createHSVColor(12, z0ColorIndex++, &color);
      set3DLEDRGB(Z0Slice[i], color);
      z0ColorIndex %= 12;
    }
    z0ColorIndex++;
    delay(RSLICE_DELAY);
  }
}
        
/*****************************************************************************/
// Rotating Rectangles Code
/*****************************************************************************/

POINT2D box1[] = {
  {1,1}, {2,1}, {1,2}, {2,2}
};
#define BOX_1_POINTS (sizeof(box1) / sizeof(POINT2D))

POINT2D box2[] = {
  {0,0}, {1,0}, {2,0}, {3,0},
  {3,1}, {3,2}, {3,3},
  {2,3}, {1,3}, {0,3},
  {0,2}, {0,1}
};
#define BOX_2_POINTS (sizeof(box2) / sizeof(POINT2D))

POINT2D rect1[] = {
  {1,0}, {2,1}, {3,2},
  {2,3}, {1,2}, {0,1}
};
#define RECT_1_POINTS (sizeof(rect1) / sizeof(POINT2D))

POINT2D rect2[] = {
  {2,0}, {3,1}, {2,2},
  {1,3}, {0,2}, {1,1}
};
#define RECT_2_POINTS (sizeof(rect2) / sizeof(POINT2D))

void drawBox1(byte z, RGB color) {
  for (byte i = 0; i < BOX_1_POINTS; i++) {
    set3DLEDRGB(box1[i].x, box1[i].y, z, color);
  }
}

void drawBox2(byte z, RGB color) {
  for (byte i = 0; i < BOX_2_POINTS; i++) {
    set3DLEDRGB(box2[i].x, box2[i].y, z, color);
  }
}

void drawRect1(byte z, RGB color) {
  for (byte i = 0; i < RECT_1_POINTS; i++) {
    set3DLEDRGB(rect1[i].x, rect1[i].y, z, color);
  }
}

void drawRect2(byte z, RGB color) {
  for (byte i = 0; i < RECT_2_POINTS; i++) {
    set3DLEDRGB(rect2[i].x, rect2[i].y, z, color);
  }
}

#define RR_ON_TIME  50
#define RR_NUMBER_OF_COLORS 12

void rotatingRectanglesCode() {
  
  RGB color;
  byte colorIndex = 0;
  byte loops = 40;  
  while (loops--) {
    createHSVColor(RR_NUMBER_OF_COLORS, colorIndex++, &color);
    drawBox1(3, color);
    delay(RR_ON_TIME);
    
    createHSVColor(RR_NUMBER_OF_COLORS, colorIndex++, &color);
    drawRect1(2, color);
    delay(RR_ON_TIME);
    
    createHSVColor(RR_NUMBER_OF_COLORS, colorIndex++, &color);
    drawRect2(1, color);
    delay(RR_ON_TIME);
    
    createHSVColor(RR_NUMBER_OF_COLORS, colorIndex++, &color);
    drawBox2(0, color);
    delay(RR_ON_TIME);
    
    colorIndex %= RR_NUMBER_OF_COLORS;
  }
  
  clearLEDData();
  
  loops = 40;  
  while (loops--) {
    createHSVColor(RR_NUMBER_OF_COLORS, colorIndex++, &color);
    drawBox1(0, color);
    delay(RR_ON_TIME);
    
    createHSVColor(RR_NUMBER_OF_COLORS, colorIndex++, &color);
    drawRect1(1, color);
    delay(RR_ON_TIME);
    
    createHSVColor(RR_NUMBER_OF_COLORS, colorIndex++, &color);
    drawRect2(2, color);
    delay(RR_ON_TIME);
    
    createHSVColor(RR_NUMBER_OF_COLORS, colorIndex++, &color);
    drawBox2(3, color);
    delay(RR_ON_TIME);
    
    colorIndex %= RR_NUMBER_OF_COLORS;
  }
}

/*****************************************************************************/
// Program entry point
/*****************************************************************************/

// Array of pointers to pattern display functions
pt2Function patternFunctions [] = {  
  blueGreenSweepCode,
  chaserCode,
//  colorTestCode,
  cubePaletteCode, 
  cubePalette2Code, 
  flashCode,
  matrixCode, 
  movingBoxesCode,
  movingBoxes2Code,
  movingBoxes3Code,
  rainbowCode,
  rainbow2Code,
  rainbow2Code,
  randomCode,
  randomLinesCode,
  rotatingRectanglesCode,
  rotatingSlicesCode,  
  slicesCode,
  slices2Code, 
  threeDSinCode,
  twoDSinCodeXZ, 
  twoDSinCodeYZ,
  twoDSinCodeXY,
  wallsCode,
  walls2Code
};

#define NUMBER_OF_PATTERNS (sizeof(patternFunctions) / sizeof(pt2Function))

void loop() {
  
  // Fade out cube before each pattern
  fadeToBlack();
  
  // Read the function switch every loop
  boolean on = readFunctionSwitch();

#ifdef DEBUG  
  printFreeMemory('0');
#endif

  if (on) {
    // Run the color organ
    colorOrganCode();
  
  }  else {

    // Wait until sound is detected
    while (! detectSound()) {
      ;
    }    

    // Select a pattern function randomly
    byte selectionIndex = random(NUMBER_OF_PATTERNS);
    
#ifdef DEBUG  
    p("%d of %d\n", selectionIndex, NUMBER_OF_PATTERNS);
#endif

    // Execute the pattern
    (*patternFunctions[selectionIndex])();
  }
}


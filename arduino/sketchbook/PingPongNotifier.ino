#include <Adafruit_NeoPixel.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define PIN 6
#define PIXEL_COUNT 8
#define BUFFER_SIZE PIXEL_COUNT*4

// Modified from an NeoPixel example sketch

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

byte incomingData[BUFFER_SIZE];

// Variables used in the UART ISR
static unsigned char *ptr;
static unsigned int pos = 0;

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Initialise the incoming data buffer
  for (int i = 0; i < PIXEL_COUNT; i++) {
    incomingData[i * 4] = 0xFF;
    incomingData[(i * 4) + 1] = 0xFF;
    incomingData[(i * 4) + 2] = 0xFF;
    incomingData[(i * 4) + 3] = 0x00;
  }

  // Set the USART interrupt variables to initial states
  ptr = incomingData;
  pos = BUFFER_SIZE;
  
  // Configure the USART at 57600
  serial_init();
 
  // Enable global interrupts
  sei();



}

int incomingByteNumber = 0;

int flash_counter = 0;

volatile boolean has_read_data = false;

unsigned long last_read_data = millis();

void loop() {
  
  // If we get over 100 ms since we last read data
  //  in, reset the pointer and input index
  if (has_read_data == true) {
    last_read_data = millis();
    has_read_data = false;
  }
  if (millis() - last_read_data > 100) {
     // reset
     ptr = incomingData;
     pos = BUFFER_SIZE;
  }

  float ratio = 0.0;
  ratio = float(flash_counter) / 50.0;
  if (flash_counter > 50) {
    ratio = float(100 - flash_counter) / 50.0;
  }

  for (int i = 0; i < PIXEL_COUNT; i++) {
    byte r = incomingData[(i * 4) + 0];
    byte g = incomingData[(i * 4) + 1];
    byte b = incomingData[(i * 4) + 2];
    byte f = incomingData[(i * 4) + 3];
    
    if (f != 0) {
      r = ratio * float(r);
      g = ratio * float(g);
      b = ratio * float(b);
    }
    strip.setPixelColor(i, strip.Color(r, g, b));

  }

  strip.show();
  delay(10);

  flash_counter++;
  if (flash_counter == 100) {
    flash_counter = 0;
  }

}

#define BAUD 9600
#define MYUBRR ( ( F_CPU / ( BAUD * 8UL ) ) - 1 )
void serial_init() {
	UBRR0H = ( MYUBRR >> 8 );
	UBRR0L = MYUBRR;
	UCSR0A |= ( 1 << U2X0 );
	UCSR0B |= (1 << RXEN0 ) | (1 << TXEN0 ) | ( 1 << RXCIE0 );
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);

}

// UART interrupt for receiving data from a controlling computer.
ISR(USART_RX_vect) 
{
  // Read the data out of the UART RX memory location
  char b = UDR0;
//  UDR0=b;
  *ptr=b;
  pos--;
  ptr++;
  if (pos == 0) {
    ptr = incomingData;
    pos = BUFFER_SIZE;
  }
  has_read_data = true;
  
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256 * 1; j++) { // 5 cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}



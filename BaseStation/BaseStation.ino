/**
 * Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 */

/**
 * To use, hook up the following to the Arduino Duemilanove:
 * Digital I/O 2: Gamecube controller serial line
 * Digital I/O 8: N64 serial line
 * All appropriate grounding and power lines
 * A 1K resistor to bridge digital I/O 2 and the 3.3V supply
 *
 * The pin-out for the N64 and Gamecube wires can be found here:
 * http://svn.navi.cx/misc/trunk/wasabi/devices/cube64/hardware/cube64-basic.pdf
 * Note: that diagram is not for this project, but for a similar project which
 * uses a PIC microcontroller. However, the diagram does describe the pinouts
 * of the gamecube and N64 wires.
 *
 * Also note: the N64 supplies a 3.3 volt line, but I don't plug that into
 * anything.  The arduino can't run off of that many volts, it needs more, so
 * it's powered externally. Additionally, the arduino has its own 3.3 volt
 * supply that I use to power the Gamecube controller. Therefore, only two lines
 * from the N64 are used.
 */

/*
 Copyright (c) 2009 Andrew Brown

 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following
 conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 */

#include "pins_arduino.h"
#include "SoftwareSerial.h"

#define DEBUG false

#define GC_PIN 2
#define GC_PIN_DIR DDRD
// these two macros set arduino pin 2 to input or output, which with an
// external 1K pull-up resistor to the 3.3V rail, is like pulling it high or
// low.  These operations translate to 1 op code, which takes 2 cycles
#define GC_HIGH DDRD &= ~0x04
#define GC_LOW DDRD |= 0x04
#define GC_QUERY (PIND & 0x04)

// 8 bytes of data that we get from the controller
struct {
    // bits: 0, 0, 0, start, y, x, b, a
    unsigned char data1;
    // bits: 1, L, R, Z, Dup, Ddown, Dright, Dleft
    unsigned char data2;
    unsigned char stick_x;
    unsigned char stick_y;
    unsigned char cstick_x;
    unsigned char cstick_y;
    unsigned char left;
    unsigned char right;
} gc_status;
char gc_raw_dump[65]; // 1 received bit per byte

// Zero points for the GC controller stick
unsigned char zero_x;
unsigned char zero_y;

int heartBeat;

void gc_send(unsigned char *buffer, char length);
void gc_get();
void print_gc_status();
void translate_raw_data();
void gc_to_64();
void get_n64_command();

unsigned char crc_repeating_table[] = {
        	0xFF, // 0x00
		0x14, // 0x01
		0xAC, // 0x02
		0x47, // 0x03
		0x59, // 0x04
		0xB2, // 0x05
		0x0A, // 0x06
		0xE1, // 0x07
		0x36, // 0x08
		0xDD, // 0x09
		0x65, // 0x0A
		0x8E, // 0x0B
		0x90, // 0x0C
		0x7B, // 0x0D
		0xC3, // 0x0E
		0x28, // 0x0F
		0xE8, // 0x10
		0x03, // 0x11
		0xBB, // 0x12
		0x50, // 0x13
		0x4E, // 0x14
		0xA5, // 0x15
		0x1D, // 0x16
		0xF6, // 0x17
		0x21, // 0x18
		0xCA, // 0x19
		0x72, // 0x1A
		0x99, // 0x1B
		0x87, // 0x1C
		0x6C, // 0x1D
		0xD4, // 0x1E
		0x3F, // 0x1F
		0xD1, // 0x20
		0x3A, // 0x21
		0x82, // 0x22
		0x69, // 0x23
		0x77, // 0x24
		0x9C, // 0x25
		0x24, // 0x26
		0xCF, // 0x27
		0x18, // 0x28
		0xF3, // 0x29
		0x4B, // 0x2A
		0xA0, // 0x2B
		0xBE, // 0x2C
		0x55, // 0x2D
		0xED, // 0x2E
		0x06, // 0x2F
		0xC6, // 0x30
		0x2D, // 0x31
		0x95, // 0x32
		0x7E, // 0x33
		0x60, // 0x34
		0x8B, // 0x35
		0x33, // 0x36
		0xD8, // 0x37
		0x0F, // 0x38
		0xE4, // 0x39
		0x5C, // 0x3A
		0xB7, // 0x3B
		0xA9, // 0x3C
		0x42, // 0x3D
		0xFA, // 0x3E
		0x11, // 0x3F
		0xA3, // 0x40
		0x48, // 0x41
		0xF0, // 0x42
		0x1B, // 0x43
		0x05, // 0x44
		0xEE, // 0x45
		0x56, // 0x46
		0xBD, // 0x47
		0x6A, // 0x48
		0x81, // 0x49
		0x39, // 0x4A
		0xD2, // 0x4B
		0xCC, // 0x4C
		0x27, // 0x4D
		0x9F, // 0x4E
		0x74, // 0x4F
		0xB4, // 0x50
		0x5F, // 0x51
		0xE7, // 0x52
		0x0C, // 0x53
		0x12, // 0x54
		0xF9, // 0x55
		0x41, // 0x56
		0xAA, // 0x57
		0x7D, // 0x58
		0x96, // 0x59
		0x2E, // 0x5A
		0xC5, // 0x5B
		0xDB, // 0x5C
		0x30, // 0x5D
		0x88, // 0x5E
		0x63, // 0x5F
		0x8D, // 0x60
		0x66, // 0x61
		0xDE, // 0x62
		0x35, // 0x63
		0x2B, // 0x64
		0xC0, // 0x65
		0x78, // 0x66
		0x93, // 0x67
		0x44, // 0x68
		0xAF, // 0x69
		0x17, // 0x6A
		0xFC, // 0x6B
		0xE2, // 0x6C
		0x09, // 0x6D
		0xB1, // 0x6E
		0x5A, // 0x6F
		0x9A, // 0x70
		0x71, // 0x71
		0xC9, // 0x72
		0x22, // 0x73
		0x3C, // 0x74
		0xD7, // 0x75
		0x6F, // 0x76
		0x84, // 0x77
		0x53, // 0x78
		0xB8, // 0x79
		0x00, // 0x7A
		0xEB, // 0x7B
		0xF5, // 0x7C
		0x1E, // 0x7D
		0xA6, // 0x7E
		0x4D, // 0x7F
		0x47, // 0x80
		0xAC, // 0x81
		0x14, // 0x82
		0xFF, // 0x83
		0xE1, // 0x84
		0x0A, // 0x85
		0xB2, // 0x86
		0x59, // 0x87
		0x8E, // 0x88
		0x65, // 0x89
		0xDD, // 0x8A
		0x36, // 0x8B
		0x28, // 0x8C
		0xC3, // 0x8D
		0x7B, // 0x8E
		0x90, // 0x8F
		0x50, // 0x90
		0xBB, // 0x91
		0x03, // 0x92
		0xE8, // 0x93
		0xF6, // 0x94
		0x1D, // 0x95
		0xA5, // 0x96
		0x4E, // 0x97
		0x99, // 0x98
		0x72, // 0x99
		0xCA, // 0x9A
		0x21, // 0x9B
		0x3F, // 0x9C
		0xD4, // 0x9D
		0x6C, // 0x9E
		0x87, // 0x9F
		0x69, // 0xA0
		0x82, // 0xA1
		0x3A, // 0xA2
		0xD1, // 0xA3
		0xCF, // 0xA4
		0x24, // 0xA5
		0x9C, // 0xA6
		0x77, // 0xA7
		0xA0, // 0xA8
		0x4B, // 0xA9
		0xF3, // 0xAA
		0x18, // 0xAB
		0x06, // 0xAC
		0xED, // 0xAD
		0x55, // 0xAE
		0xBE, // 0xAF
		0x7E, // 0xB0
		0x95, // 0xB1
		0x2D, // 0xB2
		0xC6, // 0xB3
		0xD8, // 0xB4
		0x33, // 0xB5
		0x8B, // 0xB6
		0x60, // 0xB7
		0xB7, // 0xB8
		0x5C, // 0xB9
		0xE4, // 0xBA
		0x0F, // 0xBB
		0x11, // 0xBC
		0xFA, // 0xBD
		0x42, // 0xBE
		0xA9, // 0xBF
		0x1B, // 0xC0
		0xF0, // 0xC1
		0x48, // 0xC2
		0xA3, // 0xC3
		0xBD, // 0xC4
		0x56, // 0xC5
		0xEE, // 0xC6
		0x05, // 0xC7
		0xD2, // 0xC8
		0x39, // 0xC9
		0x81, // 0xCA
		0x6A, // 0xCB
		0x74, // 0xCC
		0x9F, // 0xCD
		0x27, // 0xCE
		0xCC, // 0xCF
		0x0C, // 0xD0
		0xE7, // 0xD1
		0x5F, // 0xD2
		0xB4, // 0xD3
		0xAA, // 0xD4
		0x41, // 0xD5
		0xF9, // 0xD6
		0x12, // 0xD7
		0xC5, // 0xD8
		0x2E, // 0xD9
		0x96, // 0xDA
		0x7D, // 0xDB
		0x63, // 0xDC
		0x88, // 0xDD
		0x30, // 0xDE
		0xDB, // 0xDF
		0x35, // 0xE0
		0xDE, // 0xE1
		0x66, // 0xE2
		0x8D, // 0xE3
		0x93, // 0xE4
		0x78, // 0xE5
		0xC0, // 0xE6
		0x2B, // 0xE7
		0xFC, // 0xE8
		0x17, // 0xE9
		0xAF, // 0xEA
		0x44, // 0xEB
		0x5A, // 0xEC
		0xB1, // 0xED
		0x09, // 0xEE
		0xE2, // 0xEF
		0x22, // 0xF0
		0xC9, // 0xF1
		0x71, // 0xF2
		0x9A, // 0xF3
		0x84, // 0xF4
		0x6F, // 0xF5
		0xD7, // 0xF6
		0x3C, // 0xF7
		0xEB, // 0xF8
		0x00, // 0xF9
		0xB8, // 0xFA
		0x53, // 0xFB
		0x4D, // 0xFC
		0xA6, // 0xFD
		0x1E, // 0xFE
		0xF5 // 0xFF
};

void setup()
{
   Serial.begin(9600);

  
    
  heartBeat = 0;
  // Communication with gamecube controller on this pin
  // Don't remove these lines, we don't want to push +5V to the controller
  digitalWrite(GC_PIN, LOW);  
  pinMode(GC_PIN, INPUT);

  // Initialize the gamecube controller by sending it a null byte.
  // This is unnecessary for a standard controller, but is required for the
  // Wavebird.
  unsigned char initialize = 0x00;
  noInterrupts();
  gc_send(&initialize, 1);

  // Stupid routine to wait for the gamecube controller to stop
  // sending its response. We don't care what it is, but we
  // can't start asking for status if it's still responding
  int x;
  for (x=0; x<64; x++) {
      // make sure the line is idle for 64 iterations, should
      // be plenty.
      if (!GC_QUERY)
          x = 0;
  }

  // Query for the gamecube controller's status. We do this
  // to get the 0 point for the control stick.
  unsigned char command[] = {0x40, 0x03, 0x00};
  gc_send(command, 3);
  // read in data and dump it to gc_raw_dump
  gc_get();
  interrupts();
  translate_raw_data();
  zero_x = gc_status.stick_x;
  zero_y = gc_status.stick_y;
  
}

void translate_raw_data()
{
    // The get_gc_status function sloppily dumps its data 1 bit per byte
    // into the get_status_extended char array. It's our job to go through
    // that and put each piece neatly into the struct gc_status
    int i;
    memset(&gc_status, 0, sizeof(gc_status));
    // line 1
    // bits: 0, 0, 0, start, y, x, b a
    for (i=0; i<8; i++) {
        gc_status.data1 |= gc_raw_dump[i] ? (0x80 >> i) : 0;
    }
    // line 2
    // bits: 1, l, r, z, dup, ddown, dright, dleft
    for (i=0; i<8; i++) {
        gc_status.data2 |= gc_raw_dump[8+i] ? (0x80 >> i) : 0;
    }
    // line 3
    // bits: joystick x value
    // These are 8 bit values centered at 0x80 (128)
    for (i=0; i<8; i++) {
        gc_status.stick_x |= gc_raw_dump[16+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.stick_y |= gc_raw_dump[24+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.cstick_x |= gc_raw_dump[32+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.cstick_y |= gc_raw_dump[40+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.left |= gc_raw_dump[48+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        gc_status.right |= gc_raw_dump[56+i] ? (0x80 >> i) : 0;
    }
}

/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 * Oh, it destroys the buffer passed in as it writes it
 */
void gc_send(unsigned char *buffer, char length)
{
    // Send these bytes
    char bits;
    
    bool bit;

    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could throw the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    //
    // I use manually constructed for-loops out of gotos so I have more control
    // over the outputted assembly. I can insert nops where it was impossible
    // with a for loop
    
    asm volatile (";Starting outer for loop");
outer_loop:
    {
        asm volatile (";Starting inner for loop");
        bits=8;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile (";Setting line to low");
            GC_LOW; // 1 op, 2 cycles

            asm volatile (";branching");
            if (*buffer >> 7) {
                asm volatile (";Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // nop block 1
                asm volatile ("nop\nnop\nnop\nnop\nnop\n");
                
                asm volatile (";Setting line to high");
                GC_HIGH;

                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              );

            } else {
                asm volatile (";Bit is a 0");
                // 0 bit
                // remain low for 3us, then go high for 1us
                // nop block 3
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\n");

                asm volatile (";Setting line to high");
                GC_HIGH;

                // wait for 1us
                asm volatile ("; end of conditional branch, need to wait 1us more before next bit");
                
            }
            // end of the if, the line is high and needs to remain
            // high for exactly 16 more cycles, regardless of the previous
            // branch path

            asm volatile (";finishing inner loop body");
            --bits;
            if (bits != 0) {
                // nop block 4
                // this block is why a for loop was impossible
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\n");
                // rotate bits
                asm volatile (";rotating out bits");
                *buffer <<= 1;

                goto inner_loop;
            } // fall out of inner loop
        }
        asm volatile (";continuing outer loop");
        // In this case: the inner loop exits and the outer loop iterates,
        // there are /exactly/ 16 cycles taken up by the necessary operations.
        // So no nops are needed here (that was lucky!)
        --length;
        if (length != 0) {
            ++buffer;
            goto outer_loop;
        } // fall out of outer loop
    }

    // send a single stop (1) bit
    // nop block 5
    asm volatile ("nop\nnop\nnop\nnop\n");
    GC_LOW;
    // wait 1 us, 16 cycles, then raise the line 
    // 16-2=14
    // nop block 6
    asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\n");
    GC_HIGH;

}

void gc_get()
{
    // listen for the expected 8 bytes of data back from the controller and
    // blast it out to the gc_raw_dump array, one bit per byte for extra speed.
    // Afterwards, call translate_raw_data() to interpret the raw data and pack
    // it into the gc_status struct.
    asm volatile (";Starting to listen");
    unsigned char timeout;
    char bitcount = 64;
    char *bitbin = gc_raw_dump;

    // Again, using gotos here to make the assembly more predictable and
    // optimization easier (please don't kill me)
read_loop:
    timeout = 0x3f;
    // wait for line to go low
    while (GC_QUERY) {
        if (!--timeout)
            return;
    }
    // wait approx 2us and poll the line
    asm volatile (
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
            );
    *bitbin = GC_QUERY;
    ++bitbin;
    --bitcount;
    if (bitcount == 0)
        return;

    // wait for line to go high again
    // it may already be high, so this should just drop through
    timeout = 0x3f;
    while (!GC_QUERY) {
        if (!--timeout)
            return;
    }
    goto read_loop;

}

void print_gc_status()
{
    int i;
    Serial.println();
    Serial.print("Start: ");
    Serial.print(gc_status.data1 & 0x10 ? 1:0);

    Serial.print(" Y:     ");
    Serial.print(gc_status.data1 & 0x08 ? 1:0);

    Serial.print(" X:     ");
    Serial.print(gc_status.data1 & 0x04 ? 1:0);

    Serial.print(" B:     ");
    Serial.print(gc_status.data1 & 0x02 ? 1:0);

    Serial.print(" A:     ");
    Serial.print(gc_status.data1 & 0x01 ? 1:0);

    Serial.print(" L:     ");
    Serial.print(gc_status.data2 & 0x40 ? 1:0);
    Serial.print(" R:     ");
    Serial.print(gc_status.data2 & 0x20 ? 1:0);
    Serial.print(" Z:     ");
    Serial.print(gc_status.data2 & 0x10 ? 1:0);

    Serial.print(" Dup:   ");
    Serial.print(gc_status.data2 & 0x08 ? 1:0);
    Serial.print(" Ddown: ");
    Serial.print(gc_status.data2 & 0x04 ? 1:0);
    Serial.print(" Dright:");
    Serial.print(gc_status.data2 & 0x02 ? 1:0);
    Serial.print(" Dleft: ");
    Serial.print(gc_status.data2 & 0x01 ? 1:0);

    Serial.print(" Stick X:");
    Serial.print(gc_status.stick_x, DEC);
    Serial.print(" Stick Y:");
    Serial.print(gc_status.stick_y, DEC);

    Serial.print(" cStick X:");
    Serial.print(gc_status.cstick_x, DEC);
    Serial.print(" cStick Y:");
    Serial.print(gc_status.cstick_y, DEC);

    Serial.print(" L:     ");
    Serial.print(gc_status.left, DEC);
    Serial.print(" R:     ");
    Serial.print(gc_status.right, DEC);
}

int process_stickX_data(int data){
   //Stick X  MIN:24 MAX:222
   float val = data - 24;
   val = val * 1.287878;
   return (int) val;
}

int process_stickY_data(int data){
   //Stick Y  MIN:26 MAX:228
   float val = data - 25;
   val = val * 1.26237;
   return (int) val;
}

int process_cstickX_data(int data){
   //cStick X Min:34 Max:218
   float val = data - 34;
   val = val * 1.38586;
   return (int) val;
}

int process_cstickY_data(int data){
   //Stick Y  MIN:37 MAX:221
   float val = data - 37;
   val = val * 1.41;
   return (int) val;
}

bool rumble = false;
void loop()
{
    int i;
    unsigned char data, addr;

    // clear out incomming raw data buffer
    // this should be unnecessary
    //memset(gc_raw_dump, 0, sizeof(gc_raw_dump));
    //memset(n64_raw_dump, 0, sizeof(n64_raw_dump));

    // Command to send to the gamecube
    // The last bit is rumble, flip it to rumble
    // yes this does need to be inside the loop, the
    // array gets mutilated when it goes through gc_send
    unsigned char command[] = {0x40, 0x03, 0x00};
    if (rumble) {
        command[2] = 0x01;
    }

    // don't want interrupts getting in the way
    noInterrupts();
    // send those 3 bytes
    gc_send(command, 3);
    // read in data and dump it to gc_raw_dump
    gc_get();
    // end of time sensitive code
    interrupts();

    // translate the data in gc_raw_dump to something useful
    translate_raw_data();

  

    interrupts();

    // DEBUG: print it
    if (DEBUG){
      print_gc_status();
    };
    
    if (heartBeat == 4){
      Serial.println("*");
      heartBeat = 0;
    }
    else{
      Serial.print("@ ");
      Serial.print(process_stickY_data(gc_status.stick_y));
      Serial.print(" ");
      Serial.println(process_cstickY_data(gc_status.cstick_y));
      heartBeat++;
    }
    
    //Delay so buffer in arduino recieving xbee data doesn't overflow
    delay(40);
}


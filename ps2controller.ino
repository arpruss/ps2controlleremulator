#include <Arduino.h>

#define LED PB12 //uPC13

// PS2 Signal Pin Definitions
#define PIN_ATT  PA4  // PS2 pin 6: Input: Attention / Chip Select (Active LOW) 
#define PIN_CLK  PB10  // PS2 pin 7: Input: Clock (Active LOW pulse)
#define PIN_DAT  PA6  // PS2 pin 1: Output: Data (Open-Drain output)
#define PIN_CMD  PA7  // PS2 pin 2: Input: Command                 
#define PIN_ACK  PB0  // PS2 pin 9: Output: Acknowledge (Open-Drain output)
// GND: PS2 pin 4

// Pin numbering right to left on console!

// PS2 Protocol Constants
// Standard Digital Mode PS2 Response Header: 0xFF, 0x41, 0x5A
// Bit 0 = PRESSED (0 = pressed, 1 = released in PS2 protocol)
// Byte 1: [SLCT, L3, R3, STRT, UP, RIGHT, DOWN, LEFT]
// Byte 2: [L2, R2, L1, R1, TRIANGLE, CIRCLE, CROSS, SQUARE]
uint8_t ps2_data_byte1 = 0xFF; // D-Pad & System buttons
uint8_t ps2_data_byte2 = 0xFF; // Action & Shoulder buttons

void setup() {
    // 1. Initialize USB Serial interface
    Serial.begin(115200);

    pinMode(LED, OUTPUT);
    digitalWrite(LED,1);

    // 2. Configure PS2 hardware pins
    pinMode(PIN_ATT, INPUT_PULLUP);
    pinMode(PIN_CLK, INPUT);
    pinMode(PIN_CMD, INPUT_PULLUP);

    // Configure DAT and ACK as Open-Drain outputs
    pinMode(PIN_DAT, OUTPUT_OPEN_DRAIN);
    pinMode(PIN_ACK, OUTPUT_OPEN_DRAIN);

    // Default lines HIGH (idle state)
    digitalWrite(PIN_DAT, HIGH);
    digitalWrite(PIN_ACK, HIGH);
}

uint8_t ps2_transfer_byte_fast(uint8_t byte_to_send) {
    uint8_t received_byte = 0;

    for (uint8_t bit = 0; bit < 8; bit++) {
        uint32_t timeout = 10000;

        // 1. Wait for Clock (PB10) to go LOW driven by PS2 Host
        while ((GPIOB_BASE->IDR & (1 << 10)) != 0) {
            // Abort if ATT (PA4) is released or line times out
            if ((GPIOA_BASE->IDR & (1 << 4)) != 0 || --timeout == 0) return 0;
        }

        // 2. Drive DAT bit on PA6 (LSB first)
        if (byte_to_send & (1 << bit)) {
            GPIOA_BASE->BSRR = (1 << 6);        // PA6 HIGH (release line)
        } else {
            GPIOA_BASE->BSRR = (1 << (6 + 16)); // PA6 LOW (ground line)
        }

        // 3. Read CMD bit on PA7
        if ((GPIOA_BASE->IDR & (1 << 7)) != 0) {
            received_byte |= (1 << bit);
        }

        timeout = 10000;
        // 4. Wait for Clock (PB10) to return HIGH
        while ((GPIOB_BASE->IDR & (1 << 10)) == 0) {
            if ((GPIOA_BASE->IDR & (1 << 4)) != 0 || --timeout == 0) return 0;
        }
    }

    // 5. Pulse ACK on PB0 (Active LOW for ~2 microseconds)
    delayMicroseconds(1);
    GPIOB_BASE->BSRR = (1 << (0 + 16)); // Drive PB0 LOW
    delayMicroseconds(2);
    GPIOB_BASE->BSRR = (1 << 0);        // Release PB0 HIGH

    return received_byte;
}
// Low-level bit-bang transfer helper
uint8_t ps2_transfer_byte(uint8_t byte_to_send) {
    uint8_t received_byte = 0;

    for (uint8_t bit = 0; bit < 8; bit++) {
        // Wait for Clock to go LOW (PS2 Host drives CLK)
        while (digitalRead(PIN_CLK) == HIGH) {
            if (digitalRead(PIN_ATT) == HIGH) return 0; // Abort if ATT released
        }

        // Set DAT pin bit state (LSB First)
        if (byte_to_send & (1 << bit)) {
            digitalWrite(PIN_DAT, HIGH); // Released
        } else {
            digitalWrite(PIN_DAT, LOW);  // Driven LOW
        }

        // Read CMD pin bit state
        if (digitalRead(PIN_CMD) == HIGH) {
            received_byte |= (1 << bit);
        }

        // Wait for Clock to return HIGH
        while (digitalRead(PIN_CLK) == LOW) {
            if (digitalRead(PIN_ATT) == HIGH) return 0;
        }
    }

    // Small delay before pulsing ACK
    delayMicroseconds(2);
    digitalWrite(PIN_ACK, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ACK, HIGH);

    return received_byte;
}

// Process incoming USB Serial inputs to update PS2 button bitmask
void process_serial_input() {
    while (Serial.available() > 0) {
        char key = Serial.read();

        switch (key) {
            // --- Action Buttons (Byte 2 / ps2_data_byte2) ---
            case 'z': ps2_data_byte2 &= ~(1 << 6); break; // Cross (X) - Bit 6
            case 'Z': ps2_data_byte2 |=  (1 << 6); break;
            
            case 'x': ps2_data_byte2 &= ~(1 << 5); break; // Circle (O) - Bit 5
            case 'X': ps2_data_byte2 |=  (1 << 5); break;
            
            case 's': ps2_data_byte2 &= ~(1 << 4); break; // Triangle (/\) - Bit 4
            case 'S': ps2_data_byte2 |=  (1 << 4); break;
            
            case 'a': ps2_data_byte2 &= ~(1 << 7); break; // Square ([]) - Bit 7
            case 'A': ps2_data_byte2 |=  (1 << 7); break;

            // --- Shoulder Buttons (Byte 2 / ps2_data_byte2) ---
            case 'q': ps2_data_byte2 &= ~(1 << 2); break; // L1 - Bit 2
            case 'Q': ps2_data_byte2 |=  (1 << 2); break;
            case 'w': ps2_data_byte2 &= ~(1 << 3); break; // R1 - Bit 3
            case 'W': ps2_data_byte2 |=  (1 << 3); break;
            case 'e': ps2_data_byte2 &= ~(1 << 0); break; // L2 - Bit 0
            case 'E': ps2_data_byte2 |=  (1 << 0); break;
            case 'r': ps2_data_byte2 &= ~(1 << 1); break; // R2 - Bit 1
            case 'R': ps2_data_byte2 |=  (1 << 1); break;

            // --- D-Pad (Byte 1 / ps2_data_byte1 - Vim bindings) ---
            case 'k': ps2_data_byte1 &= ~(1 << 4); break; // Up - Bit 4
            case 'K': ps2_data_byte1 |=  (1 << 4); break;
            case 'l': ps2_data_byte1 &= ~(1 << 5); break; // Right - Bit 5
            case 'L': ps2_data_byte1 |=  (1 << 5); break;
            case 'j': ps2_data_byte1 &= ~(1 << 6); break; // Down - Bit 6
            case 'J': ps2_data_byte1 |=  (1 << 6); break;
            case 'h': ps2_data_byte1 &= ~(1 << 7); break; // Left - Bit 7
            case 'H': ps2_data_byte1 |=  (1 << 7); break;

            // --- System Buttons (Byte 1 / ps2_data_byte1) ---
            case '\n':
            case 'm': ps2_data_byte1 &= ~(1 << 3); break; // START - Bit 3
            case 'M': ps2_data_byte1 |=  (1 << 3); break;
            case 'n': ps2_data_byte1 &= ~(1 << 0); break; // SELECT - Bit 0
            case 'N': ps2_data_byte1 |=  (1 << 0); break;
        }
    }
}
void loop() {
    // 1. Keep reading serial stream for key state updates
    process_serial_input();

    // 2. Check if PS2 Host initiates communication (ATT line pulled LOW)
    if ((GPIOA_BASE->IDR & (1 << 4)) == 0 /*digitalRead(PIN_ATT) == LOW*/) {
        // Byte 1: Host sends 0x01 (Header start)
        uint8_t cmd1 = ps2_transfer_byte_fast(0xFF); 

        // Byte 2: Host sends 0x42 (Poll command) -> Responder returns 0x41 (Digital Mode ID)
        if (cmd1 == 0x01) {
            uint8_t cmd2 = ps2_transfer_byte_fast(0x41);

            // Byte 3: Host sends 0x00 -> Responder returns 0x5A (Ready signal)
            if (cmd2 == 0x42) {
                digitalWrite(LED,0);
                ps2_transfer_byte_fast(0x5A);

                // Byte 4: Send D-Pad state
                ps2_transfer_byte_fast(ps2_data_byte1);

                // Byte 5: Send Action buttons state
                ps2_transfer_byte_fast(ps2_data_byte2);
                digitalWrite(LED,1);
            }
        }

        // Wait until Host releases ATT line
        while (digitalRead(PIN_ATT) == LOW);
        digitalWrite(PIN_DAT, HIGH); // Release DAT line to high impedance
    }
}

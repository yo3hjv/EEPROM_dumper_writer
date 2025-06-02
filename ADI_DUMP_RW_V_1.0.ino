/* --------------------------------------
// I2C 24Cxx EEPROM R/W dumper
// by Adrian YO3HJV
// Key Features of ADI_DUMP_RW-v7               v_1.0 - release
////EEPROM Operations
Reads/writes 24Cxx EEPROM chips (24C01-24C512)
Handles multiple EEPROM sizes (128B-65KB)
////Data Storage
SD card integration with multiple output formats
Human-readable, Intel HEX, and raw binary formats
/////Hardware Interface
I2C for EEPROM communication
SPI for SD card (CS-D4, MOSI-D11, CLK-D13, MISO-D12)
////Button triggers for operations
READ/DUMP EEPROM
WRITE/PROGRAM EEPROM
////Status Indicators
LED system for operation status (RED, BLUE, GREEN, YELLOW)
115200 baud serial interface
////Safety & Diagnostics
I2C bus scanning
Safe SD card handling


////Pin Connections
// Component | Pin | Function
// ----------|-----|---------------------------
// RED LED   | D8  | SD card activity indicator
// GREEN LED | D3  | System status indicator
// BLUE LED  | D5  | EEPROM activity indicator
// YELLOW LED| D6  | Success/error indicator
// DUMP SW   | D2  | Trigger EEPROM dump (input)
// WRITE SW  | D10 | Trigger EEPROM write (input)
// SD CS     | D4  | SD card chip select
// I2C SDA   | A4  | EEPROM data line
// I2C SCL   | A5  | EEPROM clock line

////Status and error messages via LEDs
// LED      | Status      |     Meaning
// ---------|-------------|------------------------------------------
// RED      | ON          | SD card in use - DO NOT REMOVE!
// RED      | OFF         | SD card safe to remove
// RED      | Blinking    | SD card error. 
// GRN      | ON          | System ready (SD+EEPROM OK)
// GRN      | OFF         | System error
// BLU      | ON          | EEPROM reading
// BLU      | Slow Flash  | Initialization
// BLU      | Fast Flash  | EEPROM error
// YLW + BLU| Flash       | Operation success
// YLW +BLU | Alternating | Write error

Safety Note: Never remove SD card when RED LED is ON

*/
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

// EEPROM Configuration
// Default I2C address for most 24Cxx EEPROMs
#define EEPROM_I2C_ADDRESS 0x50

// EEPROM Type Definitions
#define EEPROM_24C01 0   // 1Kbit = 128 bytes
#define EEPROM_24C02 1   // 2Kbit = 256 bytes (default)
#define EEPROM_24C04 2   // 4Kbit = 512 bytes
#define EEPROM_24C08 3   // 8Kbit = 1024 bytes
#define EEPROM_24C16 4   // 16Kbit = 2048 bytes
#define EEPROM_24C32 5   // 32Kbit = 4096 bytes
#define EEPROM_24C64 6   // 64Kbit = 8192 bytes
#define EEPROM_24C128 7  // 128Kbit = 16384 bytes
#define EEPROM_24C256 8  // 256Kbit = 32768 bytes
#define EEPROM_24C512 9  // 512Kbit = 65536 bytes

#define BYTES_PER_PAGE 32  // Number of bytes to display per line in the dump
#define READ_DELAY 5       // Delay in ms between page reads to avoid overwhelming the EEPROM

// Global EEPROM configuration variables (will be set based on selected type)
uint8_t currentEEPROMType = EEPROM_24C02;  // Default to 24C02
uint32_t EEPROM_SIZE_BYTES = 256;          // Size in bytes (256 bytes for 24C02)
float EEPROM_SIZE_KB = 0.25;               // Size in kilobytes (0.25 KB for 24C02)
uint8_t ADDRESS_BYTES = 1;                 // Number of address bytes (1 for <=16KB, 2 for >16KB)

// Serial configuration
#define SERIAL_BAUD_RATE 115200

// SD Card configuration
#define SD_CS_PIN 4       // CS pin connected to D4
#define DUMP_SWITCH_PIN 2  // Switch pin to trigger EEPROM dump
#define WRITE_SWITCH_PIN 10  // Switch pin to trigger EEPROM write (D10)
#define MAX_FILENAME 13   // Maximum filename length (8.3 format + null terminator)
#define FILE_NUMBER_DIGITS 3  // Number of digits for file numbering

// LED indicators
#define RED_LED_PIN 8     // Red LED on D8 for SD card activity
#define GREEN_LED_PIN 3   // Green LED on D3 for system status (SD card + EEPROM OK)
#define BLUE_LED_PIN 5    // Blue LED on D6 for EEPROM read activity
#define YELLOW_LED_PIN 6  // Yellow LED on D7 for

// Switch debounce settings
#define DEBOUNCE_DELAY 50  // Debounce time in milliseconds



// Output format types
#define FORMAT_READABLE 1  // Human-readable format with address, hex, and ASCII
#define FORMAT_HEX_EDITOR 2  // Intel HEX format compatible with hex editors
#define FORMAT_BINARY 3     // Raw binary format

// Define header/footer options
#define HEADER_NONE 0      // No header or footer
#define HEADER_SIMPLE 1    // Simple header and footer

// Function prototypes
void setEEPROMType(uint8_t eepromType);
void printEEPROMTypeName(uint8_t eepromType);
void readEEPROMPage(uint16_t startAddr, uint8_t *buffer, uint8_t length);
void printHexByte(uint8_t byte);
void scanI2CBus();
bool initSD();
void dismountSD();
int findNextFileNumber();
void createInfoFile(int fileNumber);
void saveEEPROMToSD(int fileNumber);
void saveHumanReadable(File& file);
void saveIntelHex(File& file);
void saveRawBinary(File& file);
void processSerialInput();
bool isSafeToRemoveSD();
void setSDActivityLED(bool state);
void updateSystemStatusLED();
bool checkEEPROMConnection();
void setEEPROMActivityLED(bool state);
void checkDumpSwitch();
void checkWriteSwitch();
void writeToEEPROM(uint16_t address, uint8_t data);


// Global variables
int currentFileNumber = 1;  // Starting file number
bool sdCardReady = false;   // Flag to track SD card status
bool sdCardBusy = false;    // Flag to indicate active SD card operations
bool eepromReady = false;   // Flag to track EEPROM status

// Error flags
bool sdCardError = false;   // Flag to indicate SD card error
bool eepromError = false;   // Flag to indicate EEPROM error

// Blue LED flashing states
#define BLUE_LED_SLOW_FLASH 1  // Slow flashing during initialization
#define BLUE_LED_FAST_FLASH 2  // Fast flashing for error
#define BLUE_LED_OFF 0         // LED off
int blueLedFlashMode = BLUE_LED_SLOW_FLASH;  // Start with slow flashing
unsigned long lastBlueLedToggleTime = 0;     // Last time Blue LED was toggled
bool blueLedState = false;                  // Current state of Blue LED

// Switch state variables
bool lastSwitchState = HIGH;  // Last read switch state (HIGH = not pressed with pullup)
bool lastWriteSwitchState = HIGH;  // Last read write switch state (HIGH = not pressed with pullup)
unsigned long lastDebounceTime = 0;  // Last time the switch was toggled
unsigned long lastWriteDebounceTime = 0;  // Last time the write switch was toggled



void setup() {
              Serial.begin(SERIAL_BAUD_RATE);
              
              while (!Serial); // Wait for serial connection (needed for Leonardo/Micro)
              
              // Initialize LED indicators
              pinMode(RED_LED_PIN, OUTPUT);
              pinMode(GREEN_LED_PIN, OUTPUT);
              pinMode(BLUE_LED_PIN, OUTPUT);
              pinMode(YELLOW_LED_PIN, OUTPUT);
              digitalWrite(RED_LED_PIN, LOW);   // Start with red LED off
              digitalWrite(GREEN_LED_PIN, LOW); // Start with green LED off
              digitalWrite(BLUE_LED_PIN, LOW);  // Start with blue LED off

              
              // Initialize dump switch with internal pullup
              pinMode(DUMP_SWITCH_PIN, INPUT_PULLUP);
              
              // Initialize write switch with internal pullup
              pinMode(WRITE_SWITCH_PIN, INPUT_PULLUP);

              // Test LEDs briefly
              digitalWrite(RED_LED_PIN, HIGH);
              digitalWrite(YELLOW_LED_PIN, HIGH);
              digitalWrite(GREEN_LED_PIN, HIGH);
              delay(500);  // Brief LED test
              digitalWrite(RED_LED_PIN, LOW);
              digitalWrite(YELLOW_LED_PIN, LOW);
              digitalWrite(GREEN_LED_PIN, LOW);
              
              // Start Blue LED in slow flash mode
              blueLedFlashMode = BLUE_LED_SLOW_FLASH;
              lastBlueLedToggleTime = millis();
              Serial.println(F("\n===== EEPROM R/W Dumper ====="));
              
              // Initialize SD card first
              delay(500);
              
              // Slow flash Red LED to indicate SD card initialization in progress
              Serial.println(F("Initializing SD card..."));
              for (int i = 0; i < 3; i++) {
                digitalWrite(RED_LED_PIN, HIGH);
                delay(500);  // Slow flash - 500ms on
                digitalWrite(RED_LED_PIN, LOW);
                delay(500);  // 500ms off
              }
              
              // Keep Red LED on during SD card initialization
              digitalWrite(RED_LED_PIN, HIGH);
              
              sdCardReady = initSD();
                            // Turn off Red LED after SD card initialization completes
              digitalWrite(RED_LED_PIN, LOW);
              
              if (!sdCardReady) {
                Serial.println(F("ERROR: SD card not detected or not properly formatted"));
                Serial.println(F("Please insert a properly formatted SD card and reset the Arduino"));
                return; // Don't proceed with I2C checks if SD card fails
              }
              

              
              // Find the next available file number
              currentFileNumber = findNextFileNumber();

              
              // Initialize I2C and scan for devices
              
              // Blue LED will be handled by non-blocking code
              
              // Keep Blue LED on during I2C scan
              digitalWrite(BLUE_LED_PIN, HIGH);
              
              Wire.begin();
              scanI2CBus();
              
              // Check if EEPROM is responding
              eepromReady = checkEEPROMConnection();
              if (!eepromReady) {
                Serial.print(F("Setup: SD OK but "));
                Serial.println(F("I2C EEPROM not found. Check connections and RESET."));
                // Leave eepromError flag set so Blue LED will flash in handleErrorLEDs()
              } else {
                // Turn off Blue LED if EEPROM is detected successfully
                digitalWrite(BLUE_LED_PIN, LOW);
              }
              
              // Update system status LED
              updateSystemStatusLED();
              
              // Display EEPROM configuration

              
              // Print compact status message
              if (sdCardReady && eepromReady) {
                Serial.println(F("SD and I2C EEPROM OK"));
                Serial.print(F("EEPROM at 0x"));
                if (EEPROM_I2C_ADDRESS < 16) Serial.print("0");
                Serial.println(EEPROM_I2C_ADDRESS, HEX);
                
                Serial.print(F("EEPROM size: "));
                Serial.print(EEPROM_SIZE_KB);
                Serial.println(F(" KB"));
                
                Serial.print(F("Next file nr: "));
                Serial.println(currentFileNumber);
                
                Serial.println(F("Enter 'd' to dump EEPROM or a 3-digit number to set file number"));
              } else {
                //Serial.println(F("System not fully ready - check warnings above"));
                if (!sdCardReady) Serial.println(F("SD card issue detected"));
                if (!eepromReady) Serial.println(F("EEPROM issue detected"));
              }
}

// Handle Blue LED flashing in a non-blocking way
void handleBlueLED() {
  unsigned long currentTime = millis();
  
  switch (blueLedFlashMode) {
    case BLUE_LED_SLOW_FLASH:
      // Slow flash - 500ms on, 500ms off
      if (currentTime - lastBlueLedToggleTime >= 500) {
        blueLedState = !blueLedState;
        digitalWrite(BLUE_LED_PIN, blueLedState ? HIGH : LOW);
        lastBlueLedToggleTime = currentTime;
      }
      break;
      
    case BLUE_LED_FAST_FLASH:
      // Faster flash - 75ms on, 75ms off
      if (currentTime - lastBlueLedToggleTime >= 75) {
        blueLedState = !blueLedState;
        digitalWrite(BLUE_LED_PIN, blueLedState ? HIGH : LOW);
        lastBlueLedToggleTime = currentTime;
      }
      break;
      
    case BLUE_LED_OFF:
      // Ensure LED is off
      digitalWrite(BLUE_LED_PIN, LOW);
      break;
  }
}

void loop() {
  // Handle Blue LED flashing (non-blocking)
  handleBlueLED();
  
  // Check for serial input
  if (Serial.available() > 0) {
    processSerialInput();
  }
  
  // Check if dump switch is pressed
  checkDumpSwitch();
  
  // Check if write switch is pressed
  checkWriteSwitch();
  
  // Handle any error LED blinking
  handleErrorLEDs();
  
  // Other periodic tasks can be added here
  delay(10);
}

// Process serial input
void processSerialInput() {
  char input = Serial.read();
  

  
  if (!sdCardReady) {
    Serial.println(F("ERROR: SD card not ready"));
    Serial.println(F("Please insert SD card and reset Arduino"));
    // Clear input buffer
    while (Serial.available()) Serial.read();
    return;
  }
  
  // Check if I2C is working by trying to communicate with the EEPROM
  if (!eepromReady && !checkEEPROMConnection()) {
    Serial.println(F("ERROR: Cannot communicate with EEPROM"));
    Serial.println(F("Check connections and I2C address"));
    // Clear input buffer
    while (Serial.available()) Serial.read();
    return;
  }
  
  if (input == 'C' || input == 'c') {
    // EEPROM type selection command
    // Format: C01, C02, C04, C08, C16, C32, C64, C128, C256, C512
    if (Serial.available() >= 2) {
      char type1 = Serial.read();
      char type2 = Serial.read();
      
      // Check for valid EEPROM type codes
      if (type1 >= '0' && type1 <= '9' && type2 >= '0' && type2 <= '9') {
        int typeNum = (type1 - '0') * 10 + (type2 - '0');
        uint8_t eepromType = 255; // Invalid type
        
        // Map input to EEPROM type
        switch (typeNum) {
          case 1: eepromType = EEPROM_24C01; break;
          case 2: eepromType = EEPROM_24C02; break;
          case 4: eepromType = EEPROM_24C04; break;
          case 8: eepromType = EEPROM_24C08; break;
          case 16: eepromType = EEPROM_24C16; break;
          case 32: eepromType = EEPROM_24C32; break;
          case 64: eepromType = EEPROM_24C64; break;
          case 128: eepromType = EEPROM_24C128; break;
          case 256: eepromType = EEPROM_24C256; break;
          case 512: eepromType = EEPROM_24C512; break;
        }
        
        if (eepromType != 255) {
          setEEPROMType(eepromType);
          Serial.print(F("EEPROM type set to 24C"));
          printEEPROMTypeName(eepromType);
          Serial.println();
        } else {
          Serial.println(F("Invalid EEPROM type. Use C01, C02, C04, C08, C16, C32, C64, C128, C256, or C512"));
        }
      } else {
        Serial.println(F("Invalid EEPROM type format. Use C01, C02, C04, C08, C16, C32, C64, C128, C256, or C512"));
      }
    } else {
      Serial.println(F("EEPROM type command requires 2 digits (e.g., C02 for 24C02)"));
    }
  }
  else if (input == 'd' || input == 'D') {
    // Start EEPROM dump to SD card
    Serial.print(F("Dumping EEPROM to files with number: "));
    Serial.println(currentFileNumber);
    saveEEPROMToSD(currentFileNumber);
    
    // Increment file number for next time
    currentFileNumber++;
    Serial.print(F("Next file number: "));
    Serial.println(currentFileNumber);
    
    // The dismountSD function will print the safe-to-remove message
  }
  else if (input >= '0' && input <= '9') {
    // Read a 3-digit number
    int num = input - '0';
    
    // Wait for more digits (up to 3 total)
    int digitCount = 1;
    unsigned long startTime = millis();
    
    while (digitCount < FILE_NUMBER_DIGITS && (millis() - startTime < 3000)) {
      if (Serial.available()) {
        char digit = Serial.read();
        if (digit >= '0' && digit <= '9') {
          num = num * 10 + (digit - '0');
          digitCount++;
        } else {
          break; // Non-digit character
        }
      }
    }
    
    // Set the new file number
    if (digitCount == FILE_NUMBER_DIGITS) {
      currentFileNumber = num;
      Serial.print(F("File number set to: "));
      Serial.println(currentFileNumber);
      Serial.println(F("Enter 'd' to dump EEPROM or a 3-digit number to set file number"));
    } else {
      Serial.println(F("Please enter a 3-digit number"));
    }
  }
  
  // Clear any remaining characters in the buffer
  while (Serial.available()) Serial.read();
}

// Initialize SD card
bool initSD() {
  // Turn on SD activity LED during initialization
  setSDActivityLED(true);
  
  // Initialize the SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD card initialization failed!"));
    setSDActivityLED(false);  // Turn off LED if SD init fails
    sdCardError = true;  // Set SD card error flag
    return false;
  }
  sdCardError = false;  // Clear SD card error flag
  setSDActivityLED(false);  // Turn off LED after initialization
  return true;
}

// Set the SD activity LED state
void setSDActivityLED(bool state) {
  // Update the busy flag
  sdCardBusy = state;
  
  // Update the LED state based on flags
  updateSDLED();
}

// Update RED LED based on SD card status flags
void updateSDLED() {
  // Priority 1: SD Card Error - Blink LED (handled in handleErrorLEDs)
  // Priority 2: SD Card Busy - Solid ON
  // Priority 3: No error, not busy - OFF
  
  if (!sdCardError) { // Only control directly if not in error state
    digitalWrite(RED_LED_PIN, sdCardBusy ? HIGH : LOW);
  }
}

// Find the next available file number by scanning existing files
int findNextFileNumber() {
  // Turn on SD activity LED while scanning files
  setSDActivityLED(true);
  int maxNumber = 0;
  
  // Check for existing dump files
  for (int i = 1; i < 1000; i++) {
    char filename[MAX_FILENAME];
    
    // Check for .hex file
    sprintf(filename, "dump_%03d.hex", i);
    if (SD.exists(filename)) {
      maxNumber = i;
      continue;
    }
    
    // Check for .hum(an) file
    sprintf(filename, "dump_%03d.hum", i);
    if (SD.exists(filename)) {
      maxNumber = i;
      continue;
    }
    
    // Check for .raw file
    sprintf(filename, "dump_%03d.raw", i);
    if (SD.exists(filename)) {
      maxNumber = i;
      continue;
    }
  }
  
  // Turn off SD activity LED when done scanning
  setSDActivityLED(false);
  
  // Return the next number after the highest found
  return maxNumber + 1;
}

// Create info file with EEPROM details
void createInfoFile(int fileNumber) {
  // This function is now integrated into saveEEPROMToSD for better error handling
  // and to ensure consistent file numbering
}

// Safely dismount the SD card
void dismountSD() {
  // Ensure all files are closed (already done in saveEEPROMToSD)
  // Set the SD card as not ready
  sdCardReady = false;
  
  // End the SPI transaction with the SD card
  SPI.end();
  
  // Force LED off directly without using the function to avoid any issues
  digitalWrite(RED_LED_PIN, LOW);
  sdCardBusy = false;
  
  // Update system status LED since SD card is no longer ready
  updateSystemStatusLED();
  
  // Print confirmation message
  Serial.println();
 // Serial.println(F("*** SD CARD SAFELY DISMOUNTED ***"));
  Serial.println(F("*** SAFE TO REMOVE SD CARD ***"));
  Serial.println(F("*** Reset Arduino after reinserting SD card ***"));
}

// Check if EEPROM is connected and responding
bool checkEEPROMConnection() {
  Wire.beginTransmission(EEPROM_I2C_ADDRESS);
  byte error = Wire.endTransmission();
  if (error == 0) {
    eepromError = false;  // Clear EEPROM error flag
    return true;
  } else {
    eepromError = true;   // Set EEPROM error flag
    return false;
  }
}

// Handle error LED blinking
void handleErrorLEDs() {
  // Only handle blinking if there are errors
  if (sdCardError || eepromError) {
    // Get current time once per call
    unsigned long currentTime = millis();
    unsigned long cycleTime = currentTime % 600; // 600ms total cycle (300ms ON + 300ms OFF)
    
    // Handle SD card error blinking (RED LED)
    if (sdCardError) {
      // Simple time-based pattern without state variables
      if (cycleTime < 300) {
        // ON phase - first 300ms of each 600ms cycle
        digitalWrite(RED_LED_PIN, HIGH); // ON
      } else {
        // OFF phase - remaining 300ms of cycle
        digitalWrite(RED_LED_PIN, LOW);
      }
    }
    
    // Handle EEPROM error blinking (BLUE LED)
    if (eepromError) {
      if (cycleTime < 300) {
        // ON phase - first 300ms of each 600ms cycle
        digitalWrite(BLUE_LED_PIN, HIGH);
      } else {
        // OFF phase - remaining 300ms of cycle
        digitalWrite(BLUE_LED_PIN, LOW);
      }
    }
    
    // Update GREEN LED based on system status (not flashing)
    // Green LED is ON when everything is OK, OFF when there's an error
    if (sdCardReady && eepromReady) {
      digitalWrite(GREEN_LED_PIN, HIGH);
    } else {
      digitalWrite(GREEN_LED_PIN, LOW);
    }
  } else {
    // If no errors, update LEDs based on activity flags
    updateSDLED(); // Update RED LED based on sdCardBusy flag
    
    // Only turn off BLUE LED if not being used for EEPROM activity
    if (!sdCardBusy) { // Assuming EEPROM is not active during idle times
      digitalWrite(BLUE_LED_PIN, LOW);
    }
    
    // Ensure GREEN LED is ON when everything is OK
    if (sdCardReady && eepromReady) {
      digitalWrite(GREEN_LED_PIN, HIGH);
    }
  }
}

// Update the system status LED (green)
void updateSystemStatusLED() {
  if (sdCardReady && eepromReady) {
    //Serial.println(F("SD and EEPROM OK"));
    // Green LED indicates ready
    digitalWrite(GREEN_LED_PIN, HIGH);
  } else {
    // Issue with either SD card or EEPROM - error will be handled by handleErrorLEDs
    // Initial state is OFF
    digitalWrite(GREEN_LED_PIN, LOW);
  }
}

// Check if it's safe to remove the SD card
bool isSafeToRemoveSD() {
  return !sdCardReady && !sdCardBusy;
}

// Check the dump switch and trigger a dump if pressed
void checkDumpSwitch() {
  // Read the current state of the switch (LOW when pressed with pullup)
  bool currentSwitchState = digitalRead(DUMP_SWITCH_PIN);
  
  // Check if the switch state has changed
  if (currentSwitchState != lastSwitchState) {
    // Reset the debounce timer
    lastDebounceTime = millis();
  }
  
  // Check if enough time has passed since the last state change (debounce)
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // If the switch is pressed (LOW with pullup) and system is ready
    if (currentSwitchState == LOW && sdCardReady && eepromReady && !sdCardBusy) {
      Serial.println(F("\n*** DUMP STARTED! ***"));
      saveEEPROMToSD(currentFileNumber);
      
      // Increment file number for next dump
      currentFileNumber++;
      Serial.print(F("Next file number: "));
      Serial.println(currentFileNumber);
      
      // Wait until button is released to prevent multiple triggers

      while (digitalRead(DUMP_SWITCH_PIN) == LOW) {
        delay(10); // Small delay while waiting for button release
      }

    }
  }
  
  // Save the current switch state for next comparison
  lastSwitchState = currentSwitchState;
}

// Save EEPROM to SD card in all formats
void saveEEPROMToSD(int fileNumber) {
  // Turn on SD activity LED to indicate SD card operations
  setSDActivityLED(true);
  // Directly control the RED LED to ensure it's on
  digitalWrite(RED_LED_PIN, HIGH);
  //Serial.println(F("SD LED ON - Dump in progress"));
  
  char filename[MAX_FILENAME];
  
  // Check if files already exist and remove them if they do
  sprintf(filename, "info_%03d.txt", fileNumber);
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  // Also check for simple info filename
  sprintf(filename, "info%d.txt", fileNumber);
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  sprintf(filename, "dump_%03d.hum", fileNumber);
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  sprintf(filename, "dump_%03d.hex", fileNumber);
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  sprintf(filename, "dump_%03d.raw", fileNumber);
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  // Create info file
  sprintf(filename, "info_%03d.txt", fileNumber);
  
  // Try opening with explicit FILE_WRITE mode
  File infoFile = SD.open(filename, FILE_WRITE);
  if (infoFile) {
    // Write data to the file
    infoFile.println(F("EEPROM DUMP INFORMATION"));
    infoFile.print(F("I2C Address: 0x"));
    if (EEPROM_I2C_ADDRESS < 16) infoFile.print("0");
    infoFile.println(EEPROM_I2C_ADDRESS, HEX);
    
    infoFile.print(F("EEPROM Size: "));
    infoFile.print(EEPROM_SIZE_KB);
    infoFile.print(F(" KB ("));
    infoFile.print(EEPROM_SIZE_BYTES);
    infoFile.println(F(" bytes)"));
    
    infoFile.print(F("Address Width: "));
    infoFile.print(ADDRESS_BYTES);
    infoFile.println(F(" byte(s)"));
    
    infoFile.print(F("Serial: "));
    infoFile.println(F(""));
    
    // Make sure to flush and close the file
    infoFile.flush();
    infoFile.close();
    Serial.print(F("On SD: "));
    Serial.println(filename);
  } else {
    Serial.print(F("Error creating info file: "));
    Serial.println(filename);
    
    // Try with a simpler filename as a fallback
    sprintf(filename, "info%d.txt", fileNumber);
    infoFile = SD.open(filename, FILE_WRITE);
    if (infoFile) {
      infoFile.println(F("EEPROM DUMP INFORMATION"));
      infoFile.println(F("See serial output for details"));
      infoFile.println(F("Serial: "));
      infoFile.flush();
      infoFile.close();
      Serial.print(F("Created simple info file: "));
      Serial.println(filename);
    }
  }
  
  // Save human-readable format
  sprintf(filename, "dump_%03d.hum", fileNumber);
  File humanFile = SD.open(filename, FILE_WRITE);
  if (humanFile) {
    saveHumanReadable(humanFile);
    humanFile.close();
    Serial.print(F("Write: "));
    Serial.println(filename);
  } else {
    Serial.print(F("Error creating file: "));
    Serial.println(filename);
  }
  
  // Save Intel HEX format
  sprintf(filename, "dump_%03d.hex", fileNumber);
  File hexFile = SD.open(filename, FILE_WRITE);
  if (hexFile) {
    saveIntelHex(hexFile);
    hexFile.close();
    Serial.print(F("Write: "));
    Serial.println(filename);
  } else {
    Serial.print(F("Error creating file: "));
    Serial.println(filename);
  }
  
  // Save raw binary format
  sprintf(filename, "dump_%03d.raw", fileNumber);
  File rawFile = SD.open(filename, FILE_WRITE);
  if (rawFile) {
    saveRawBinary(rawFile);
    rawFile.close();
    Serial.print(F("Write: "));
    Serial.println(filename);
  } else {
    Serial.print(F("Error creating file: "));
    Serial.println(filename);
  }
  
  // Clear the busy flag after all operations are complete
  sdCardBusy = false;
  
  // Turn off the SD activity LED
  Serial.println(F("Dump complete"));
  setSDActivityLED(false);
  
  // Dismount the SD card
  dismountSD();
}

// Save EEPROM in human-readable format
void saveHumanReadable(File& file) {
  uint8_t buffer[BYTES_PER_PAGE];
  char asciiBuffer[BYTES_PER_PAGE + 1];
  asciiBuffer[BYTES_PER_PAGE] = '\0';
  
  // Turn on blue LED for the entire EEPROM read process
  setEEPROMActivityLED(true);
  //Serial.println(F("Starting human-readable dump"));
  
  for (uint32_t addr = 0; addr < EEPROM_SIZE_BYTES; addr += BYTES_PER_PAGE) {
    // Read a page of data
    uint8_t bytesToRead = min(BYTES_PER_PAGE, EEPROM_SIZE_BYTES - addr);
    readEEPROMPage(addr, buffer, bytesToRead);
    
    // Format the address
    file.print("0x");
    if (addr < 0x1000) file.print("0");
    if (addr < 0x100) file.print("0");
    if (addr < 0x10) file.print("0");
    file.print(addr, HEX);
    file.print(": ");
    
    // Format the hex values
    for (uint8_t i = 0; i < bytesToRead; i++) {
      if (buffer[i] < 0x10) file.print("0");
      file.print(buffer[i], HEX);
      file.print(" ");
      
      // Convert to ASCII (replace non-printable characters with a dot)
      asciiBuffer[i] = (buffer[i] >= 32 && buffer[i] <= 126) ? (char)buffer[i] : '.';
    }
    
    // Fill remaining space if not a full line
    for (uint8_t i = bytesToRead; i < BYTES_PER_PAGE; i++) {
      file.print("   ");
      asciiBuffer[i] = ' ';
    }
    
    // Turn on RED LED before writing to SD card
    digitalWrite(RED_LED_PIN, HIGH);
    
    // Print ASCII representation
    file.print(" ");
    file.println(asciiBuffer);
    
    // Keep RED LED on for a moment to make it visible
    delay(5);
    
    // Turn off RED LED after writing to SD card
    digitalWrite(RED_LED_PIN, LOW);
    
    // Small delay to avoid overwhelming the SD card
    delay(READ_DELAY);
  }
  
  // Turn off blue LED after all EEPROM reading is complete
  setEEPROMActivityLED(false);
}

// Calculate checksum for Intel HEX format
byte calculateHexChecksum(byte count, uint16_t address, byte type, byte* data) {
  byte checksum = count;
  checksum += (address >> 8) & 0xFF;
  checksum += address & 0xFF;
  checksum += type;
  
  for (byte i = 0; i < count; i++) {
    checksum += data[i];
  }
  
  return (0x100 - checksum) & 0xFF;
}

// Save EEPROM in Intel HEX format
void saveIntelHex(File& file) {
  uint8_t buffer[BYTES_PER_PAGE];
  byte recordLength = 16; // Standard record length for Intel HEX
  
  // Turn on blue LED for the entire EEPROM read process
  setEEPROMActivityLED(true);
  //Serial.println(F("Starting Intel HEX dump"));
  
  for (uint32_t addr = 0; addr < EEPROM_SIZE_BYTES; addr += recordLength) {
    // Read data
    uint8_t bytesToRead = min(recordLength, EEPROM_SIZE_BYTES - addr);
    readEEPROMPage(addr, buffer, bytesToRead);
    
    // Calculate checksum
    byte checksum = calculateHexChecksum(bytesToRead, addr, 0, buffer);
    
    // Write record
    file.print(":");
    if (bytesToRead < 0x10) file.print("0");
    file.print(bytesToRead, HEX);
    
    if ((addr >> 8) < 0x10) file.print("0");
    file.print((addr >> 8) & 0xFF, HEX);
    
    if ((addr & 0xFF) < 0x10) file.print("0");
    file.print(addr & 0xFF, HEX);
    
    file.print("00"); // Record type: 00 (data)
    
    // Write data bytes
    for (uint8_t i = 0; i < bytesToRead; i++) {
      if (buffer[i] < 0x10) file.print("0");
      file.print(buffer[i], HEX);
    }
    
    // Turn on RED LED before writing to SD card
    digitalWrite(RED_LED_PIN, HIGH);
    
    // Write checksum
    if (checksum < 0x10) file.print("0");
    file.println(checksum, HEX);
    
    // Keep RED LED on for a moment to make it visible
    //delay(5);
    
    // Turn off RED LED after writing to SD card
    digitalWrite(RED_LED_PIN, LOW);
    
    // Small delay to avoid overwhelming the SD card
    delay(READ_DELAY);
  }
  
  // End of file record
  file.println(":00000001FF");
  
  // Turn off blue LED after all EEPROM reading is complete
  setEEPROMActivityLED(false);
}

// Save EEPROM in raw binary format
void saveRawBinary(File& file) {
  uint8_t buffer[BYTES_PER_PAGE];
  
  // Turn on blue LED for the entire EEPROM read process
  setEEPROMActivityLED(true);
  //Serial.println(F("Starting raw binary dump"));
  
  for (uint32_t addr = 0; addr < EEPROM_SIZE_BYTES; addr += BYTES_PER_PAGE) {
    // Read a page of data
    uint8_t bytesToRead = min(BYTES_PER_PAGE, EEPROM_SIZE_BYTES - addr);
    readEEPROMPage(addr, buffer, bytesToRead);
    
    // Turn on RED LED before writing to SD card
    digitalWrite(RED_LED_PIN, HIGH);

    // Write raw binary data
    file.write(buffer, bytesToRead);
    
    // Keep RED LED on for a moment to make it visible
    delay(5);
    
    // Turn off RED LED after writing to SD card
    digitalWrite(RED_LED_PIN, LOW);
    
    // Small delay to avoid overwhelming the SD card
    delay(READ_DELAY);
  }
  
  // Turn off blue LED after all EEPROM reading is complete
  setEEPROMActivityLED(false);
}

void scanI2CBus() {
  byte error, address;
  int deviceCount = 0;
  bool eepromFound = false;
  
  // Common EEPROM I2C addresses to try first
  const byte knownEEPROMAddresses[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};
  const byte numKnownAddresses = sizeof(knownEEPROMAddresses) / sizeof(knownEEPROMAddresses[0]);
  
  // Try each known EEPROM address multiple times for reliability
  for (byte i = 0; i < numKnownAddresses; i++) {
    byte successCount = 0;
    address = knownEEPROMAddresses[i];
    
    // Try each address 10 times to ensure reliable detection
    for (byte attempt = 0; attempt < 10; attempt++) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      
      if (error == 0) {
        successCount++;
      }
      delay(5); // Small delay between attempts
    }
    
    // If at least 8 out of 10 attempts succeeded, consider it a valid device
    if (successCount >= 8) {
      deviceCount++;
      
      // If this is our configured EEPROM address, mark it as found
      if (address == EEPROM_I2C_ADDRESS) {
        eepromFound = true;
      }
    }
  }
  
  // If we didn't find our configured EEPROM address in the known addresses,
  // scan the rest of the I2C address range
  if (!eepromFound) {
    for (address = 1; address < 127; address++) {
      // Skip addresses we already checked
      bool isKnownAddress = false;
      for (byte i = 0; i < numKnownAddresses; i++) {
        if (address == knownEEPROMAddresses[i]) {
          isKnownAddress = true;
          break;
        }
      }
      
      if (!isKnownAddress) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
          deviceCount++;
          
          // If this is our configured EEPROM address, mark it as found
          if (address == EEPROM_I2C_ADDRESS) {
            eepromFound = true;
          }
        }
      }
    }
  }
  
  // Final check for our configured EEPROM address
  if (!eepromFound) {
    eepromError = true;  // Set EEPROM error flag
    
    // Set Blue LED to fast flash mode to indicate error
    blueLedFlashMode = BLUE_LED_FAST_FLASH;
    
    // Turn off Green LED to indicate error
    digitalWrite(GREEN_LED_PIN, LOW);
  } else {
    eepromError = false;  // Clear EEPROM error flag
    // Set Blue LED to off mode since EEPROM was found
    blueLedFlashMode = BLUE_LED_OFF;
  }
}

// Print header with EEPROM information
void printHeader(byte format) {
  Serial.println(F("EEPROM DUMP INFORMATION"));
  Serial.print(F("I2C Address: 0x"));
  if (EEPROM_I2C_ADDRESS < 16) Serial.print("0");
  Serial.println(EEPROM_I2C_ADDRESS, HEX);

  Serial.print(F("EEPROM Size: "));
  Serial.print(EEPROM_SIZE_KB);
  Serial.print(F(" KB ("));
  Serial.print(EEPROM_SIZE_BYTES);
  Serial.println(F(" bytes)"));
  
  Serial.print(F("Address Width: "));
  Serial.print(ADDRESS_BYTES);
  Serial.println(F(" byte(s)"));
  
  Serial.print(F("Dump Date: "));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));
}

// Print footer
void printFooter(byte format) {
  Serial.println();
  Serial.println(F("Dump complete!"));
}

// Dump the entire contents of the EEPROM
void dumpEEPROMContents(byte format, byte headerOption) {
  uint8_t buffer[BYTES_PER_PAGE];
  char asciiBuffer[BYTES_PER_PAGE + 1];
  asciiBuffer[BYTES_PER_PAGE] = '\0';
  
  // Print header if requested
  if (headerOption == HEADER_SIMPLE) {
    printHeader(format);
  }
  
  if (format == FORMAT_BINARY) {
    // Simple hex format - just bytes in hex without addresses or ASCII
    for (uint32_t addr = 0; addr < EEPROM_SIZE_BYTES; addr += BYTES_PER_PAGE) {
      uint8_t bytesToRead = min(BYTES_PER_PAGE, EEPROM_SIZE_BYTES - addr);
      readEEPROMPage(addr, buffer, bytesToRead);
      
      // Output bytes as hex values without any formatting
      for (uint8_t i = 0; i < bytesToRead; i++) {
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
      }
      
      // Add a newline every 32 bytes for readability
      if ((addr + BYTES_PER_PAGE) % 32 == 0) {
        Serial.println();
      }
    }
    // Ensure we end with a newline
    Serial.println();
  }
  else if (format == FORMAT_READABLE) {
    // Human-readable format with address, hex, and ASCII
    for (uint32_t addr = 0; addr < EEPROM_SIZE_BYTES; addr += BYTES_PER_PAGE) {
      // Read a page of data
      readEEPROMPage(addr, buffer, BYTES_PER_PAGE);
      
      // Print address
      if (EEPROM_SIZE_BYTES > 0xFFFF) {
        // For large EEPROMs, use 6 hex digits for address
        if (addr < 0x10000) Serial.print("0");
        Serial.print(addr, HEX);
      } else {
        // For smaller EEPROMs, use 4 hex digits
        if (addr < 0x1000) Serial.print("0");
        if (addr < 0x100) Serial.print("0");
        if (addr < 0x10) Serial.print("0");
        Serial.print(addr, HEX);
      }
      Serial.print(": ");
      
      // Print hex values
      for (uint8_t i = 0; i < BYTES_PER_PAGE; i++) {
        printHexByte(buffer[i]);
        Serial.print(" ");
        
        // Prepare ASCII representation
        if (buffer[i] >= 32 && buffer[i] <= 126) {
          asciiBuffer[i] = (char)buffer[i];
        } else {
          asciiBuffer[i] = '.'; // Non-printable character
        }
      }
      
      // Print ASCII representation
      Serial.print(" |");
      Serial.print(asciiBuffer);
      Serial.println("|");
      
      // Small delay to avoid overwhelming the EEPROM
      delay(READ_DELAY);
    }
  } 
  else if (format == FORMAT_HEX_EDITOR) {
    // Intel HEX format for hex editors
    // Format: :BBAAAATT[DD...]CC
    // BB = Byte count (number of data bytes)
    // AAAA = Address (16-bit)
    // TT = Record type (00 = data, 01 = end of file)
    // DD = Data bytes
    // CC = Checksum
    
    const byte BYTES_PER_HEX_LINE = 16;  // Standard 16 bytes per line for Intel HEX
    
    // Output data records
    for (uint32_t addr = 0; addr < EEPROM_SIZE_BYTES; addr += BYTES_PER_HEX_LINE) {
      // Read a chunk of data
      byte bytesToRead = min(BYTES_PER_HEX_LINE, EEPROM_SIZE_BYTES - addr);
      readEEPROMPage(addr, buffer, bytesToRead);
      
      // Calculate checksum
      byte checksum = calculateHexChecksum(bytesToRead, addr & 0xFFFF, 0x00, buffer);
      
      // Start of record
      Serial.print(":");
      
      // Byte count
      printHexByte(bytesToRead);
      
      // Address (16-bit)
      printHexByte((addr >> 8) & 0xFF);  // High byte
      printHexByte(addr & 0xFF);         // Low byte
      
      // Record type (00 = data)
      Serial.print("00");
      
      // Data bytes
      for (byte i = 0; i < bytesToRead; i++) {
        printHexByte(buffer[i]);
      }
      
      // Checksum
      printHexByte(checksum);
      
      Serial.println();
      delay(READ_DELAY);
    }
    
    // End of file record
    Serial.println(":00000001FF");
  }
  
  // Print footer if requested
  if (headerOption == HEADER_SIMPLE) {
    printFooter(format);
  }
  
  // Always show command reminder after dump
  Serial.println();
  Serial.println(F("s/c/h/x/b/r - dump options"));
}

// Set the EEPROM activity LED state
void setEEPROMActivityLED(bool state) {
  digitalWrite(BLUE_LED_PIN, state ? HIGH : LOW);
}



// Function to set the EEPROM type and update configuration variables
void setEEPROMType(uint8_t eepromType) {
  // Constrain to valid range
  if (eepromType > EEPROM_24C512) {
    eepromType = EEPROM_24C02; // Default to 24C02 if invalid
  }
  
  currentEEPROMType = eepromType;
  
  // Calculate size based on type (more efficient than switch statement)
  // 24C01=128, 24C02=256, 24C04=512, 24C08=1024, 24C16=2048, etc.
  if (eepromType <= EEPROM_24C16) {
    // For smaller EEPROMs (24C01 to 24C16), use 1 address byte
    ADDRESS_BYTES = 1;
    // Calculate size: 128 bytes * 2^type (type 0=24C01, 1=24C02, etc.)
    EEPROM_SIZE_BYTES = 128 << eepromType;
  } else {
    // For larger EEPROMs (24C32 to 24C512), use 2 address bytes
    ADDRESS_BYTES = 2;
    // Calculate size: 4096 bytes * 2^(type-5) (type 5=24C32, 6=24C64, etc.)
    EEPROM_SIZE_BYTES = 4096 << (eepromType - 5);
  }
  
  // Calculate KB size
  EEPROM_SIZE_KB = EEPROM_SIZE_BYTES / 1024.0;
  

}

// Function to print the EEPROM type name
void printEEPROMTypeName(uint8_t eepromType) {
  // Use lookup table for common types to save space
  static const char* const typeNames[] = {
    "01", "02", "04", "08", "16", "32", "64"
  };
  
  if (eepromType <= EEPROM_24C64) {
    Serial.print(typeNames[eepromType]);
  } else if (eepromType == EEPROM_24C128) {
    Serial.print(F("128"));
  } else if (eepromType == EEPROM_24C256) {
    Serial.print(F("256"));
  } else if (eepromType == EEPROM_24C512) {
    Serial.print(F("512"));
  } else {
    Serial.print(F("??"));
  }
}

// Read a page of data from the EEPROM
void readEEPROMPage(uint16_t startAddr, uint8_t *buffer, uint8_t length) {
  // Blue LED is now controlled at the save function level, not here
  byte deviceAddress;
  
  if (ADDRESS_BYTES == 2) {
    // For EEPROMs larger than 16KB (24C32 and up)
    // These use a 2-byte address format
    deviceAddress = EEPROM_I2C_ADDRESS;
    Wire.beginTransmission(deviceAddress);
    Wire.write((startAddr >> 8) & 0xFF);  // MSB
    Wire.write(startAddr & 0xFF);         // LSB
    Wire.endTransmission();
  } else {
    // For EEPROMs 16KB or smaller (24C01 through 24C16)
    // These use the upper address bits as part of the device address
    deviceAddress = EEPROM_I2C_ADDRESS | ((startAddr >> 8) & 0x07);
    Wire.beginTransmission(deviceAddress);
    Wire.write(startAddr & 0xFF);  // Only LSB is sent
    Wire.endTransmission();
  }
  
  // Request data from the EEPROM
  Wire.requestFrom(deviceAddress, length);
  
  // Read the data into the buffer
  for (uint8_t i = 0; i < length && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}

// Helper function to print a hex byte with leading zero if needed
void printHexByte(uint8_t byte)
{
  if (byte < 16) Serial.print("0");
  Serial.print(byte, HEX);
}

// Calculate page size based on EEPROM type
uint8_t getPageSize() {
  if (currentEEPROMType <= EEPROM_24C02) return 8;       // 24C01, 24C02: 8 bytes
  else if (currentEEPROMType <= EEPROM_24C16) return 16; // 24C04, 24C08, 24C16: 16 bytes
  else return 32;                                        // 24C32 and larger: 32+ bytes
}

// Write data to EEPROM
void writeToEEPROM(uint16_t address, uint8_t* data, uint8_t length) {
  // Handle addressing based on EEPROM size (similar to readEEPROMPage)
  byte deviceAddr = EEPROM_I2C_ADDRESS;
  
  // For 24C04, 24C08, 24C16, modify device address
  if (ADDRESS_BYTES == 1 && currentEEPROMType >= EEPROM_24C04 && currentEEPROMType <= EEPROM_24C16) {
    byte addressBits = currentEEPROMType - EEPROM_24C02;
    byte highAddrBits = (address >> 8) & ((1 << addressBits) - 1);
    deviceAddr |= highAddrBits;
  }
  
  Wire.beginTransmission(deviceAddr);
  
  // Send address bytes (1 or 2 depending on EEPROM size)
  if (ADDRESS_BYTES == 2) {
    Wire.write((byte)(address >> 8));    // MSB
  }
  Wire.write((byte)(address & 0xFF));    // LSB
  
  // Write data bytes (respecting page boundaries)
  for (uint8_t i = 0; i < length; i++) {
    Wire.write(data[i]);
  }
  
  Wire.endTransmission();
  delay(10);  // Write cycle time (typically 5-10ms)
}

// Check if write would cross page boundary
bool wouldCrossPageBoundary(uint16_t address, uint8_t length) {
  uint8_t pageSize = getPageSize();
  uint16_t pageOffset = address % pageSize;
  return (pageOffset + length > pageSize);
}

// Flash both LEDs together for success indication
void flashSuccessLEDs(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(duration);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    delay(duration);
  }
}

// Flash LEDs alternately for size mismatch error
void flashSizeMismatchError() {
  while (true) {  // Until reset
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(BLUE_LED_PIN, LOW);
    delay(300);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(300);
  }
}

// Flash yellow LED continuously for file not found error
void flashFileNotFoundError() {
  digitalWrite(GREEN_LED_PIN, LOW);  // Turn off green LED
  while (true) {  // Until reset
    digitalWrite(YELLOW_LED_PIN, HIGH);
    delay(300);
    digitalWrite(YELLOW_LED_PIN, LOW);
    delay(300);
  }
}

// Process EEPROM write from SD card file
void processEEPROMWrite() {
  Serial.println(F("\n*** EEPROM WRITE STARTED! ***"));
  setSDActivityLED(true);  // Turn on SD activity LED
  
  // Check if origin.raw file exists
  if (!SD.exists("origin.raw")) {
    Serial.println(F("ERROR: origin.raw file not found on SD card"));
    setSDActivityLED(false);  // Turn off SD activity LED
    flashFileNotFoundError();  // This will loop until reset
    return;
  }
  
  // Open the origin.raw file
  File sourceFile = SD.open("origin.raw", FILE_READ);
  if (!sourceFile) {
    Serial.println(F("ERROR: Could not open origin.raw file"));
    setSDActivityLED(false);
    flashFileNotFoundError();
    return;
  }
  
  // Check file size against EEPROM size
  uint32_t fileSize = sourceFile.size();
  Serial.print(F("File size: "));
  Serial.print(fileSize);
  Serial.print(F(" bytes, EEPROM size: "));
  Serial.print(EEPROM_SIZE_BYTES);
  Serial.println(F(" bytes"));
  
  if (fileSize > EEPROM_SIZE_BYTES) {
    Serial.println(F("ERROR: File size exceeds EEPROM capacity"));
    sourceFile.close();
    setSDActivityLED(false);
    flashSizeMismatchError();  // This will loop until reset
    return;
  }
  
  // Write data from file to EEPROM
  uint16_t address = 0;
  uint8_t buffer[32];  // Max page size buffer
  uint8_t verifyBuffer[32];
  uint8_t pageSize = getPageSize();
  bool writeError = false;
  
  Serial.println(F("Writing to EEPROM..."));
  Serial.print(F("Page size: "));
  Serial.print(pageSize);
  Serial.println(F(" bytes"));
  
  while (address < fileSize && !writeError) {
    // Determine how many bytes to write (don't cross page boundary)
    uint8_t bytesToWrite = min(pageSize - (address % pageSize), 
                             min((uint32_t)sourceFile.available(), 
                                 (uint32_t)(EEPROM_SIZE_BYTES - address)));
    
    // Read data from file
    sourceFile.read(buffer, bytesToWrite);
    
    // Set EEPROM activity LED
    setEEPROMActivityLED(true);
    
    // Write to EEPROM
    writeToEEPROM(address, buffer, bytesToWrite);
    
    // Verify written data
    readEEPROMPage(address, verifyBuffer, bytesToWrite);
    if (memcmp(buffer, verifyBuffer, bytesToWrite) != 0) {
      // Verification failed
      Serial.print(F("ERROR: Verification failed at address 0x"));
      printHexByte((address >> 8) & 0xFF);
      printHexByte(address & 0xFF);
      Serial.println();
      writeError = true;
      break;
    }
    
    // Move to next address
    address += bytesToWrite;
    
    // Turn off EEPROM activity LED
    setEEPROMActivityLED(false);
    
    // Print progress
    if (address % 256 == 0 || address >= fileSize) {
      Serial.print(F("Progress: "));
      Serial.print(address);
      Serial.print(F("/"));
      Serial.print(fileSize);
      Serial.println(F(" bytes"));
    }
    
    // Small delay between writes
    delay(READ_DELAY);
  }
  
  // Close the file
  sourceFile.close();
  
  // Final verification of the entire EEPROM contents
  if (!writeError) {
    Serial.println(F("Performing final verification of entire EEPROM..."));
    
    // Reopen the source file for verification
    sourceFile = SD.open("origin.raw", FILE_READ);
    if (!sourceFile) {
      Serial.println(F("ERROR: Could not reopen source file for verification"));
      setSDActivityLED(false);
      flashSizeMismatchError();
      return;
    }
    
    // Verify the entire EEPROM contents
    uint16_t verifyAddress = 0;
    bool verifyError = false;
    
    while (verifyAddress < fileSize && !verifyError) {
      // Read a chunk from the file
      uint8_t bytesToVerify = min((uint32_t)32, (uint32_t)(fileSize - verifyAddress));
      uint8_t fileBuffer[32];
      uint8_t eepromBuffer[32];
      
      // Read from file
      sourceFile.read(fileBuffer, bytesToVerify);
      
      // Read from EEPROM
      setEEPROMActivityLED(true);
      readEEPROMPage(verifyAddress, eepromBuffer, bytesToVerify);
      setEEPROMActivityLED(false);
      
      // Compare
      if (memcmp(fileBuffer, eepromBuffer, bytesToVerify) != 0) {
        // Verification failed
        Serial.print(F("FINAL VERIFICATION ERROR at address 0x"));
        printHexByte((verifyAddress >> 8) & 0xFF);
        printHexByte(verifyAddress & 0xFF);
        Serial.println();
        verifyError = true;
        break;
      }
      
      // Move to next chunk
      verifyAddress += bytesToVerify;
      
      // Print progress periodically
      if (verifyAddress % 256 == 0 || verifyAddress >= fileSize) {
        Serial.print(F("Verification progress: "));
        Serial.print(verifyAddress);
        Serial.print(F("/"));
        Serial.print(fileSize);
        Serial.println(F(" bytes"));
      }
      
      delay(READ_DELAY);
    }
    
    // Close the file again
    sourceFile.close();
    
    if (verifyError) {
      writeError = true;
    } else {
      Serial.println(F("Final verification successful!"));
    }
  }
  
  setSDActivityLED(false);
  
  if (writeError) {
    Serial.println(F("EEPROM write failed!"));
    // Flash error pattern
    flashSizeMismatchError();  // Reusing this function for write errors
  } else {
    Serial.println(F("EEPROM write completed successfully!"));
    // Flash success pattern (both LEDs 5 times at 600ms)
    flashSuccessLEDs(5, 600);
  }
}

// Check the write switch and trigger EEPROM write if pressed
void checkWriteSwitch() {
  // Read the current state of the switch (LOW when pressed with pullup)
  bool currentWriteSwitchState = digitalRead(WRITE_SWITCH_PIN);
  
  // Check if the switch state has changed
  if (currentWriteSwitchState != lastWriteSwitchState) {
    // Reset the debounce timer
    lastWriteDebounceTime = millis();
  }
  
  // Check if enough time has passed since the last state change (debounce)
  if ((millis() - lastWriteDebounceTime) > DEBOUNCE_DELAY) {
    // If the switch is pressed (LOW with pullup) and system is ready
    if (currentWriteSwitchState == LOW && sdCardReady && eepromReady && !sdCardBusy) {
      Serial.println(F("\n*** WRITE BUTTON PRESSED! ***"));
      processEEPROMWrite();
      
      // Wait until button is released to prevent multiple triggers
      while (digitalRead(WRITE_SWITCH_PIN) == LOW) {
        delay(10); // Small delay while waiting for button release
      }
    }
  }
  
  // Save the current switch state for next comparison
  lastWriteSwitchState = currentWriteSwitchState;
}

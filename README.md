# EEPROM_dumper_writer
An arduino based, R/W for I2C EEPROM 24CXXX reader/writer
by Adrian, YO3HJV.

  I needed a simple tool to extract the binaries from EEPROM (Source) and to write them back
into another after some inspection and modifications.
  The project evolved into something ready to be used "in the field".
  Multiple readings of the same EEPROM can be done. Or multiple EEPROMS... 
  Default is 24C08 but other models can be selected via Serial Console.

  To write a bin (.raw) into a EEPROM without the need of a complex tool,
just save a file with the bin content as "origin.raw" and when pressing the 
"Write" button, the content will be written into the target EEPROM and two step 
verification will follow to verify the content.
  First verification is immediately after writing a page, a second verification will
be made for the entire content against the bin file. The result is observed by LED indicators.


// Key Features of ADI_DUMP_RW-v7               v_1.0 - release
////EEPROM Operations
    Reads/writes 24Cxx EEPROM chips (24C01-24C512), selectable from Serial Console
    Handles multiple EEPROM sizes (128B-65KB)
////Data Storage
    SD card integration with multiple output formats.
    Automatic numbering for files.
    3 type of files are saved: Human-readable, Intel HEX, and raw binary formats
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


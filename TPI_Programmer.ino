/**************************************************
 * TPI programmer for ATtiny4/5/9/10/20/40
 *
 * Make the connections as shown below.
 *
 * To use:
 ***** Buad rate must be set to 9600 ****
 *
 *  - Upload to arduino and power off
 *  - Connect ATtiny10 as shown
 *  - Power on and open the serial monitor
 *  - If things are working so far you should
 *    see "NVM enabled" and "ATtiny10/20/40 connected".
 *  - Input one-letter commands via serial monitor:
 *
 *    D = dump memory. Displays all current memory
 *        on the chip
 *
 *    E = erase chip. Erases all program memory
 *        automatically done at time of programming
 *
 *    P = write program. After sending this, paste
 *        the program from the hex file into the
 *        serial monitor.
 *
 *    S = set fuse. follow the instructions to set
 *        one of the three fuses.
 *
 *    C = clear fuse. follow the instructions to clear
 *        one of the three fuses.
 *
 *    H = Toggle High Voltage Programming  
 *
 *    T = Toggle +12v enabled by High, or Low
 *
 *    R/r = Quick reset
 *
 *  - Finally, power off the arduino and remove the
 *    Attiny10/20/40
 *
 *
 * Arduino                 ATtiny10               *
 * ----------+          +----------------         *
 * (SS#)  10 |--[R]-----| 6 (RESET#/PB3)          *
 *           |          |                         *
 * (MOSI) 11 |--[R]--+--| 1 (TPIDATA/PB0)         *
 *           |       |  |                         *
 * (MISO) 12 |--[R]--+  |                         *
 *           |          |                         *
 * (SCK)  13 |--[R]-----| 3 (TPICLK/PB1)          *
 * ----------+          +----------------         *
 *                                                *
 * ----------+          +----------------         *
 * (HVP)   9 |---       | 6 (RESET#/PB3)          *
 *           |          |                         *
 *                                                *
 *  -[R]-  =  a 220 - 1K Ohm resistor             *
 *                                                *
 *  this picture : 2011/12/08 by pcm1723          *
 *  modified :2015/02/27 by KD                    *
 *                                                *
 * thanks to pcm1723 for tpitest.pde upon which   *
 * this is based                                  *
 **************************************************
 Updates:
   Feb 12, 2024: petrov@email.su
       * Address from HEX file.
       * Small editing for convenience.
	   * Added checksum calculation.
   Feb 24, 2023: KW
       * Simplified HV programming
       * Added dumpConfig() to make the set/clear flag functions more intuitive
       * Added line breaks to dumpMemory() output for clarity
       * Made the main menu case-insensitive
       * Fixed bug that caused the "clear flag" to clear all flags, not just the selected one
 
   Dec 04, 2017: thejamestate@gmail.com
       * Added support for ATtiny102 and ATtiny104
   Jan 23, 2017: Ksdsksd@gmail.com
                * Thanks to InoueTaichi Fixed incorrect #define Tiny40 
                
   Mar 05, 2015: Ksdsksd@gamil.com
                * Added notifications to setting and clearing the system flags.
 
   Feb 23, 2015: Ksdsksd@gamil.com
                * Changed the programmer Diagram, This is the config I use, and get a sucessful programming of a tiny10 at 9600 baud.
                
   Mar 22, 2014: Ksdsksd@gmail.com
                * Added the quick reset to high before resetting the device.
                * Added code to stop the SPI and float the pins for testing the device while connected.

   Mar 20, 2014: Ksdsksd@gmail.com
                * Added a quick reset by sending 'r' or 'R' via the serial monitor.
                * Added a High voltage programming option from pin 9, toggled by 'H'
                * Added a High/low option for providing 12v to the reset pin, toggled by 'T'

   Mar 17, 2014: Ksdsksd@gmail.com
                * Had some trouble with the nibbles being swapped when programming on the 10 & 20,
                  added b1,b2 to hold the serial data before calling byteval()
                * Added Nat Blundell's patch to the code

   Apr 10, 2013: Ksdsksd@gmail.com
                * Applied Fix for setting and clearing flags

   Feb 7,  2013: Ksdsksd@gmail.com
                * Fixed programming timer, had intitial start at zero instead of current time.

   Dec 11, 2012: Ksdsksd@gmail.com
                * Added detect and programming for 4/5/9

   Dec 5-6, 2012: Ksdsksd@gmail.com
                * Incorperated read, and verify into program. Now have no program size limitation by using 328p.
                * Changed the outHex routines consolidated them into 1, number to be printed, and number of nibbles
                * Added a type check to distinguish between Tiny10/20/40
                * Added an auto word size check to ensure that there is the proper amount of words written for a 10/20/40
                * Removed Read program, Verify, and Finish from options
                * Changed baud rate to 19200 for delay from data written to the chip, to prevent serial buffer overrun.

   Oct 5, 2012: Ksdsksd@gmail.com
                *** Noticed that when programming, the verification fails
                    at times by last 1-2 bytes programmed, and the Tiny would act erratic.
                    Quick fix was adding 3 NOP's to the end the Tiny's code, and ignoring the errors, the Tiny then performed as expected.

   Oct 4, 2012: Ksdsksd@gmail.com
                * Moved all Serial printed strings to program space
                * Added code to detect Tiny20
*/

#include <SPI.h>
#include "pins_arduino.h"
char HVP = true; // is high voltage programming on by default?
char HVON = HIGH;    // what is the active level for high voltage programming?
const char SSN = LOW;    // what is the active level for SS (reset)?

// define the instruction set bytes
#define SLD    0x20
#define SLDp   0x24
#define SST    0x60
#define SSTp   0x64
#define SSTPRH 0x69
#define SSTPRL 0x68
// see functions below ////////////////////////////////
// SIN  0b0aa1aaaa replace a with 6 address bits
// SOUT 0b1aa1aaaa replace a with 6 address bits
// SLDCS  0b1000aaaa replace a with address bits
// SSTCS  0b1100aaaa replace a with address bits
///////////////////////////////////////////////////////
#define SKEY   0xE0
#define NVM_PROGRAM_ENABLE 0x1289AB45CDD888FFULL // the ULL means unsigned long long

#define NVMCMD 0x33
#define NVMCSR 0x32
#define NVM_NOP 0x00
#define NVM_CHIP_ERASE 0x10
#define NVM_SECTION_ERASE 0x14
#define NVM_WORD_WRITE 0x1D

#define HVReset 9

#define Tiny4_5 10
#define Tiny9 1
#define Tiny10 1
#define Tiny20 2
#define Tiny40 4
#define Tiny102 1
#define Tiny104 1

#define TimeOut 1
#define HexError 2
#define TooLarge 3
#define Checksum 4
// represents the current pointer register value
unsigned short adrs = 0x0000;

// used for storing a program file
uint8_t data[16]; //program data
unsigned int progSize = 0; //program size in bytes

// used for various purposes
long startTime;
int timeout;
uint8_t b, b1, b2, b3;
boolean idChecked;
//boolean correct;
char type; // type of chip connected 1 = Tiny10, 2 = Tiny20

int counti = 0;

void setup(){
  // set up serial
  Serial.begin(9600); // you cant increase this, it'll overrun the buffer

	
		digitalWrite(HVReset, !HVON);
		pinMode(HVReset, OUTPUT);	


		digitalWrite(SS, !SSN);
		pinMode(SS, OUTPUT);

  

delay(1000);

  start_tpi();


 // initialize memory pointer register
 setPointer(0x0000);

 timeout = 20000;
 idChecked = false;
} // end setup()


void hvserial()
{
    if(HVP)
        Serial.println(F("***High Voltage Programming Enabled***"));
    else
        Serial.println(F("High Voltage Programming Disabled"));

    Serial.print(F("Pin "));
	Serial.print(HVReset);
    Serial.print(HVON?F(" HIGH"):F(" LOW"));
    Serial.println(F(" supplies 12v"));

}


void hvReset(char highLow)
{
    if(HVP)
    {
        if(HVON) //if high enables 12v
            highLow = !highLow; // invert the typical reset
        digitalWrite(HVReset, highLow);
    }
    else
        if(SSN) //if high enables 12v
            highLow = !highLow; // invert the typical reset
        digitalWrite(SS, highLow);
}

void quickReset()
{
    Serial.println(F("Reset."));
    //digitalWrite(SS,HIGH);
    hvReset(HIGH);
    delay(1);
//    digitalWrite(SS,LOW);
    hvReset(LOW);
    delay(10);
//    digitalWrite(SS,HIGH);
    hvReset(HIGH);
}

void start_tpi() {
  Serial.println(F("Initializing..."));
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);

    // enter TPI programming mode
    hvReset(LOW);
//    digitalWrite(SS, LOW); // assert RESET on tiny
    delay(1); // t_RST min = 400 ns @ Vcc = 5 V

  SPI.transfer(0xff); // activate TPI by emitting
  SPI.transfer(0xff); // 16 or more pulses on TPICLK
  SPI.transfer(0xff); // while holding TPIDATA to "1"

  writeCSS(0x02, 0x04); // TPIPCR, guard time = 8bits (default=128)
  send_skey(NVM_PROGRAM_ENABLE); // enable NVM interface
  // wait for NVM to be enabled

  while((readCSS(0x00) & 0x02) < 1){
    // wait
  }
  Serial.println(F("NVM enabled"));
}

void loop(){
  if(!idChecked){
//    start_tpi();
    checkID();
    idChecked = true;
    finish();
  }
  // when ready, send ready signal '.' and wait
  Serial.println();
  Serial.println(F("'P' = program the ATtiny using the read program"));
  Serial.println(F("'D' = dump memory to serial monitor"));
  Serial.println(F("'E' = erase chip. erases current program memory.(done automatically by 'P')"));
  Serial.println(F("'S' = set fuse"));
  Serial.println(F("'C' = clear fuse"));
  Serial.println(F("'H' = Toggle High Voltage Programming"));
  Serial.println(F("'T' = Toggle +12v enabled by High, or Low"));
  Serial.print(F("\n>"));
  while(Serial.available() < 1){
    // wait
  }

  // the first byte is a command
  //** 'P' = program the ATtiny using the read program
  //** 'D' = dump memory to serial monitor
  //** 'E' = erase chip. erases current program memory.(done automatically by 'P')
  //** 'S' = set fuse
  //** 'C' = clear fuse

  char comnd = Sread();

  switch( comnd ){
    case 'r':
    case 'R':
        quickReset();
        break;

    case 'd':
    case 'D':
      start_tpi();
      dumpMemory();
      finish();
      break;

  case 'h':
  case 'H':
    HVP = !HVP;
    hvserial();
    break;

  case 't':
  case 'T':
    HVON = !HVON;
    hvserial();
    break;

  case 'p':
  case 'P':
      start_tpi();
      if(!writeProgram()){
        startTime = millis();
        while(millis()-startTime < 8000)
          Serial.read();// if exited due to error, disregard all other serial data
      }
      finish();
    break;

  case 'e':
  case 'E':
    start_tpi();
    eraseChip();
    finish();
    break;

  case 's':
  case 'S':
    start_tpi();
    setConfig(true);
    finish();
    break;

  case 'c':
  case 'C':
    start_tpi();
    setConfig(false);
    finish();
    break;

  case ' ':
    break;

  default:
    Serial.println(F("Received unknown command"));
  }




}
void ERROR_pgmSize(void)
{
  Serial.println(F("program size is 0??"));
}

void ERROR_data(char i)
{
  Serial.println(F("couldn't receive data:"));
  switch(i){
    case TimeOut:
      Serial.println(F("timed out"));
      break;
    case HexError:
      Serial.println(F("hex file format error"));
      break;
    case TooLarge:
      Serial.println(F("program is too large"));
      break;
    case Checksum: 
      Serial.println(F("Checksum error"));
      break;
    default:
      break;
  }

}

void dumpConfig(){
  uint8_t i;
  adrs = 0x3F40;
  setPointer(adrs);
  Serial.print(F("Current config state: "));
  tpi_send_byte(SLDp);
  b = tpi_receive_byte(); // get data byte
  Serial.print(F("0x"));
  outHex(b, 2);
   
  if (b & 0x04)
    Serial.print(F("\n  [ ] "));
  else
    Serial.print(F("\n  [√] "));
  Serial.print(F("Output clock to PB2"));
    
  if (b & 0x02)
    Serial.print(F("\n  [ ] "));
  else
    Serial.print(F("\n  [√] "));
  Serial.print(F("Watch dog timer always on"));
    
  if (b & 0x01)
    Serial.print(F("\n  [ ] "));
  else
    Serial.print(F("\n  [√] "));
  Serial.print(F("Disable external reset on PB3"));

  Serial.println(F(" "));
}

//  print the register, SRAM, config and signature memory
void dumpMemory(){
  unsigned int len;
  uint8_t i;
  // initialize memory pointer register
  setPointer(0x0000);

  Serial.println(F("Current memory state:"));
  if(type != Tiny4_5)
    len = 0x400 * type; //the memory length for a 10/20/40 is 1024/2048/4096
  else
    len = 0x200; //tiny 4/5 has 512 bytes
  len += 0x4000;

  while(adrs < len){
    // read the byte at the current pointer address
    // and increment address
    tpi_send_byte(SLDp);
    b = tpi_receive_byte(); // get data byte

    // read all the memory, but only print
    // the register, SRAM, config and signature memory
    if ((0x0000 <= adrs && adrs <= 0x005F) // register/SRAM
       |(0x3F00 <= adrs && adrs <= 0x3F01) // NVM lock bits
       |(0x3F40 <= adrs && adrs <= 0x3F41) // config
       |(0x3F80 <= adrs && adrs <= 0x3F81) // calibration
       |(0x3FC0 <= adrs && adrs <= 0x3FC3) // ID
       |(0x4000 <= adrs && adrs <= len-1) ) { // program
      // print +number along the top
      if ((0x00 == adrs)
          |(0x3f00 == adrs) // NVM lock bits
          |(0x3F40 == adrs) // config
          |(0x3F80 == adrs) // calibration
          |(0x3FC0 == adrs) // ID
          |(0x4000 == adrs) ) {
        if(adrs == 0x0000){ Serial.print(F("registers, SRAM")); }
        if(adrs == 0x3F00){ Serial.print(F("\n\nNVM lock")); }
        if(adrs == 0x3F40){ Serial.print(F("\n\nconfiguration")); }
        if(adrs == 0x3F80){ Serial.print(F("\n\ncalibration")); }
        if(adrs == 0x3FC0){ Serial.print(F("\n\ndevice ID")); }
        if(adrs == 0x4000){ Serial.print(F("\n\nprogram")); }
        Serial.println();
        for (i = 0; i < 5; i++)
          Serial.print(F(" "));
        for (i = 0; i < 16; i++) {
          Serial.print(F(" +"));
          Serial.print(i, HEX);
        }
      }
      // print number on the left
      if (0 == (0x000f & adrs)) {
        Serial.println();
        outHex(adrs, 4);
        Serial.print(F(": ")); // delimiter
      }
        outHex(b, 2);
        Serial.print(F(" "));
    }
    adrs++; // increment memory address
    if(adrs == 0x0060){
      // skip reserved memory
      setPointer(0x3F00);
    }
  }
  Serial.println(F(" "));
} // end dumpMemory()


// receive and translate the contents of a hex file, Program and verify on the fly
boolean writeProgram(){
  char temp[] = "00";
  char addr[] = "0000";
  uint8_t checkSum = 0;
  uint8_t someth = 0;
  unsigned int currentByte = 0;
  progSize = 0;
  uint8_t linelength = 0;
  boolean fileEnd = false;

  unsigned long pgmStartTime = millis();
  eraseChip(); // erase chip
  Serial.println(F("\nSend hex file contents now"));
  char words = (type!=Tiny4_5?type:1);

  // read in the data and
  while(!fileEnd){
    startTime = millis();
    while(Serial.available() < 1){
      if(millis()-startTime > timeout){
        ERROR_data(TimeOut);
        return false;
      }
      if(pgmStartTime == 0)
        pgmStartTime = millis();
    }
    if(Sread() != ':'){ // maybe it was a newline??
      if(Sread() != ':'){
        ERROR_data(HexError);
        return false;
      }
    }
    // read data length

    temp[0] = Sread();
    temp[1] = Sread();
    checkSum = linelength = asciiHexToUShort(temp);

    // read address. if "0000" currentByte = 0
    temp[0] = addr[0] = Sread();
    temp[1] = addr[1] = Sread();
    checkSum += asciiHexToUShort(temp);
    temp[0] = addr[2] = Sread();
    temp[1] = addr[3] = Sread();
    checkSum += asciiHexToUShort(temp);
    adrs = asciiHexToUShort(addr) + 0x4000;
    if(linelength != 0x00 && adrs == 0x4000)
      currentByte = 0;

    // read type thingy. "01" means end of file
    temp[0] = Sread();
    temp[1] = Sread();
    someth = asciiHexToUShort(temp);
    checkSum += someth;   
    if(someth == 1){
      fileEnd = true;
    }

    if(someth == 2){
      for (int i = 0; i<=linelength; i++){
      temp[0] = Sread();
      temp[1] = Sread();
      checkSum += asciiHexToUShort(temp);
      }

    }
    else{
      // read in the data
      for(int k=0; k<linelength; k++){
        while(Serial.available() < 1){
          if(millis()-startTime > timeout){
            ERROR_data(TimeOut);
            return false;
          }
        }
        temp[0]=Sread();
        temp[1]=Sread();
        data[currentByte] = asciiHexToUShort(temp);
        checkSum += data[currentByte];
        currentByte++;
        progSize++;
        if(progSize > (type!=Tiny4_5?type*1024:512)){
          ERROR_data(TooLarge);
          return 0;
        }


        if(fileEnd) //has the end of the file been reached?
          while(currentByte < 2 * words){// append zeros to align the word count to program
            data[currentByte] = 0;
            currentByte++;
          }



        if( currentByte == 2 * words ){// is the word/Dword/Qword here?

          currentByte = 0; // yes, reset counter
          setPointer(adrs); // point to the address to program
          writeIO(NVMCMD, NVM_WORD_WRITE);
          for(int i = 0; i<2 * words; i+=2){// loop for each word size depending on micro

            // now write a word to program memory
            tpi_send_byte(SSTp);
            tpi_send_byte(data[i]); // LSB first
            tpi_send_byte(SSTp);
            tpi_send_byte(data[i+1]); // then MSB
            SPI.transfer(0xff); //send idle between words
            SPI.transfer(0xff); //send idle between words
          }
          while((readIO(NVMCSR) & (1<<7)) != 0x00){} // wait for write to finish

          writeIO(NVMCMD, NVM_NOP);
          SPI.transfer(0xff);
          SPI.transfer(0xff);


          //verify written words
          setPointer(adrs);
          for (int c = 0; c < 2 * words; c++){
            tpi_send_byte(SLDp);
            b = tpi_receive_byte(); // get data byte

            if(b != data[c]){
              Serial.println(F("program error:"));
              Serial.print(F("byte "));
              outHex(adrs, 4);
              Serial.print(F(" expected "));
              outHex(data[c],2);
              Serial.print(F(" read "));
              outHex(b,2);
              Serial.println();
              return false;
            }
          }
          adrs += 2 * words;
      }
     }

    // read in the checksum.
    startTime = millis();
    while(Serial.available() == 0){
      if(millis()-startTime > timeout){
        ERROR_data(TimeOut);
        return false;
      }
    }
    temp[0] = Sread();
    temp[1] = Sread();
    
    asm volatile(
      "com  %[value]" "\n\t"
      "inc  %[value]" "\n\t"
      :[value] "=r" (checkSum)
      :"[value]" (checkSum)
    );

    someth = asciiHexToUShort(temp);

    if (checkSum != someth) {
      ERROR_data(Checksum);
      Serial.print(F("The received checksum - "));
      Serial.println(someth, HEX);
      Serial.print(F("The calculated checksum - "));
      Serial.println(checkSum, HEX);
      return false;
    }
  }
  }
  // the program was successfully written
  Serial.print(F("Successfully wrote program: "));
  Serial.print(progSize, DEC);
  Serial.print(F(" of "));
  if(type != Tiny4_5)
    Serial.print(1024 * type, DEC);
  else
    Serial.print(512, DEC);
  Serial.print(F(" bytes\n in "));
  Serial.print((millis()-pgmStartTime)/1000.0,DEC);
  Serial.print(F(" Seconds"));
//  digitalWrite(SS, HIGH); // release RESET

  return true;
}


void eraseChip(){
  // initialize memory pointer register
  setPointer(0x4001); // need the +1 for chip erase

  // erase the chip
  writeIO(NVMCMD, NVM_CHIP_ERASE);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  while((readIO(NVMCSR) & (1<<7)) != 0x00){
    // wait for erasing to finish
  }
  Serial.println(F("chip erased"));
}

void setConfig(boolean val){
  // get current config byte
  dumpConfig();
  setPointer(0x3F40);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();

  Serial.print(F("\nInput one of these letters to "));
  if (val)
    Serial.println(F("activate it"));
  else
    Serial.println(F("deactivate it"));    
  Serial.println(F(" c = system clock output"));
  Serial.println(F(" w = watchdog timer on"));
  Serial.println(F(" r = disable reset"));
  Serial.println(F(" x = cancel. don't change anything"));

  while(Serial.available() < 1){
    // wait
  }
  char comnd = Serial.read();
  setPointer(0x3F40);
  if ((!val) & ((comnd == 'c') | (comnd == 'w') | (comnd == 'r')))
  {
    writeIO(NVMCMD, NVM_SECTION_ERASE);
    tpi_send_byte(SSTp);
    tpi_send_byte(0xFF);
    tpi_send_byte(SSTp);
    tpi_send_byte(0xFF);
    while((readIO(NVMCSR) & (1<<7)) != 0x00)  // wait for write to finish
    writeIO(NVMCMD, NVM_NOP);
    SPI.transfer(0xff);
    SPI.transfer(0xff);
    setPointer(0x3F40);
  }
//  writeIO(NVMCMD, (val ? NVM_WORD_WRITE : NVM_SECTION_ERASE) );
writeIO(NVMCMD, NVM_WORD_WRITE);

  if(comnd == 'c'){
    tpi_send_byte(SSTp);
    if(val){
      tpi_send_byte(b & 0b11111011);
    }else{
      tpi_send_byte(b | 0x04);
    }
    tpi_send_byte(SSTp);
    tpi_send_byte(0xFF);
  }
  else if(comnd == 'w'){
    tpi_send_byte(SSTp);
    if(val){
      tpi_send_byte(b & 0b11111101);
    }else{
      tpi_send_byte(b | 0x02);
    }
    tpi_send_byte(SSTp);
    tpi_send_byte(0xFF);
  }
  else if(comnd == 'r'){
    tpi_send_byte(SSTp);
    if(val){
      tpi_send_byte(b & 0b11111110);
    }else{
      tpi_send_byte(b | 0x01);
    }
    tpi_send_byte(SSTp);
    tpi_send_byte(0xFF);
  }
  else if(comnd == 'x'){
    // do nothing
  }
  else{
    Serial.println(F("received unknown command. Cancelling"));
  }
  while((readIO(NVMCSR) & (1<<7)) != 0x00){
    // wait for write to finish
  }
  writeIO(NVMCMD, NVM_NOP);
  SPI.transfer(0xff);
  SPI.transfer(0xff);
  if((comnd == 'c') | (comnd == 'w') | (comnd == 'r')){
    
    Serial.print(F("\nSuccessfully "));
    if(val)
      Serial.print(F("Set "));
      else
      Serial.print(F("Cleared "));
      
    Serial.print(F("\""));
    if(comnd == 'w')
      Serial.print(F("Watchdog"));
    else if(comnd == 'c')
      Serial.print(F("Clock Output"));
    else if(comnd == 'r')
      Serial.print(F("Reset"));
      
    Serial.println(F("\" Flag"));
    dumpConfig();
  }
}

void finish(){
  writeCSS(0x00, 0x00);
  SPI.transfer(0xff);
  SPI.transfer(0xff);
    hvReset(HIGH);
//  digitalWrite(SS, HIGH); // release RESET
  delay(1); // t_RST min = 400 ns @ Vcc = 5 V
  SPI.end();
  DDRB &= 0b11000011; //tri-state spi so target can be tested
  PORTB &= 0b11000011;
}

void checkID(){
  // check the device ID
  uint8_t id1, id2, id3;
  setPointer(0x3FC0);

  tpi_send_byte(SLDp);
  id1 = tpi_receive_byte();
  tpi_send_byte(SLDp);
  id2 = tpi_receive_byte();
  tpi_send_byte(SLDp);
  id3 = tpi_receive_byte();
  if(id1==0x1E && id2==0x8F && id3==0x0A){
    Serial.print(F("ATtiny4"));
    type = Tiny4_5;
  }else if(id1==0x1E && id2==0x8F && id3==0x09){
    Serial.print(F("ATtiny5"));
    type = Tiny4_5;
  }else if(id1==0x1E && id2==0x90 && id3==0x08){
    Serial.print(F("ATtiny9"));
    type = Tiny9;
  }else if(id1==0x1E && id2==0x90 && id3==0x03){
    Serial.print(F("ATtiny10"));
    type = Tiny10;
  }else if(id1==0x1E && id2==0x91 && id3==0x0f){
    Serial.print(F("ATtiny20"));
    type = Tiny20;
  }else if(id1==0x1E && id2==0x92 && id3==0x0e){
    Serial.print(F("ATtiny40"));
    type = Tiny40;
  }else if(id1==0x1E && id2==0x90 && id3==0x0c){
    Serial.print(F("ATtiny102"));
    type = Tiny102;
  }else if(id1==0x1E && id2==0x90 && id3==0x0b){
    Serial.print(F("ATtiny104"));
    type = Tiny104;
  }else{
    Serial.print(F("Unknown chip"));
  }
  Serial.println(F(" connected"));
}

/*
* send a byte in one TPI frame (12 bits)
* (1 start + 8 data + 1 parity + 2 stop)
* using 2 SPI data bytes (2 x 8 = 16 clocks)
* (with 4 extra idle bits)
*/
void tpi_send_byte( uint8_t data ){
  // compute partiy bit
  uint8_t par = data;
  par ^= (par >> 4); // b[7:4] (+) b[3:0]
  par ^= (par >> 2); // b[3:2] (+) b[1:0]
  par ^= (par >> 1); // b[1] (+) b[0]

  // REMEMBER: this is in LSBfirst mode and idle is high
  // (2 idle) + (1 start bit) + (data[4:0])
  SPI.transfer(0x03 | (data << 3));
  // (data[7:5]) + (1 parity) + (2 stop bits) + (2 idle)
  SPI.transfer(0xf0 | (par << 3) | (data >> 5));
} // end tpi_send_byte()

/*
* receive TPI 12-bit format byte data
* via SPI 2 bytes (16 clocks) or 3 bytes (24 clocks)
*/
uint8_t tpi_receive_byte( void ){
  //uint8_t b1, b2, b3;
  // keep transmitting high(idle) while waiting for a start bit
  do {
    b1 = SPI.transfer(0xff);
  } while (0xff == b1);
  // get (partial) data bits
  b2 = SPI.transfer(0xff);
  // if the first byte(b1) contains less than 4 data bits
  // we need to get a third byte to get the parity and stop bits
  if (0x0f == (0x0f & b1)) {
    b3 = SPI.transfer(0xff);
  }

  // now shift the bits into the right positions
  // b1 should hold only idle and start bits = 0b01111111
  while (0x7f != b1) { // data not aligned
    b2 <<= 1; // shift left data bits
    if (0x80 & b1) { // carry from 1st byte
      b2 |= 1; // set bit
    }
    b1 <<= 1;
    b1 |= 0x01; // fill with idle bit (1)
  }
  // now the data byte is stored in b2
  return( b2 );
} // end tpi_receive_byte()

// send the 64 bit NVM key
void send_skey(uint64_t nvm_key){
  tpi_send_byte(SKEY);
  while(nvm_key){
    tpi_send_byte(nvm_key & 0xFF);
    nvm_key >>= 8;
  }
} // end send_skey()

// sets the pointer address
void setPointer(unsigned short address){
  adrs = address;
  tpi_send_byte(SSTPRL);
  tpi_send_byte(address & 0xff);
  tpi_send_byte(SSTPRH);
  tpi_send_byte((address>>8) & 0xff);
}

// writes using SOUT
void writeIO(uint8_t address, uint8_t value){
  //  SOUT 0b1aa1aaaa replace a with 6 address bits
  tpi_send_byte(0x90 | (address & 0x0F) | ((address & 0x30) << 1));
  tpi_send_byte(value);
}

// reads using SIN
uint8_t readIO(uint8_t address){
  //  SIN 0b0aa1aaaa replace a with 6 address bits
  tpi_send_byte(0x10 | (address & 0x0F) | ((address & 0x30) << 1));
  return tpi_receive_byte();
}

// writes to CSS
void writeCSS(uint8_t address, uint8_t value){
  tpi_send_byte(0xC0 | address);
  tpi_send_byte(value);
}

// reads from CSS
uint8_t readCSS(uint8_t address){
  tpi_send_byte(0x80 | address);
  return tpi_receive_byte();
}

char Sread(void){
  while(Serial.available()<1){}
  return Serial.read();
}


void outHex(unsigned int n, char l){ // call with the number to be printed, and # of nibbles expected.
        for(char count = l-1; count > 0;count--){ // quick and dirty to add zeros to the hex value
          if(((n >> (count*4)) & 0x0f) == 0) // if MSB is 0
            Serial.print(F("0"));  //prepend a 0
          else
            break;  //exit the for loop
        }
    Serial.print(n, HEX);
}

#define CURRENT_DIGIT (*str)

unsigned short asciiHexToUShort( char * str )
{
    int result = 0;

    do
    {
        if ( (CURRENT_DIGIT >= '0') && (CURRENT_DIGIT <= '9') )
        {
            CURRENT_DIGIT = CURRENT_DIGIT - '0' + 0x0;
        }
        else if ( (CURRENT_DIGIT >= 'A') && (CURRENT_DIGIT <= 'F') )
        {
            CURRENT_DIGIT = CURRENT_DIGIT - 'A' + 0xA;
        }
        
        result = (result<<4) + CURRENT_DIGIT;
    }while (*++str);

    return result;
}

// end of file

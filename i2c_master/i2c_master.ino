// -------------------------------------------------------------------------------------------
// Teensy3.0/3.1/LC I2C Master
// 08Mar13 Brian (nox771 at gmail.com)
// -------------------------------------------------------------------------------------------
//
// This creates an I2C master device which talks to the simple I2C slave device given in the
// i2c_t3/slave sketch.
//
// This code assumes the slave config has 256byte memory and I2C addr is 0x44.
// The various tests are started by pulling one of the control pins low.
//
// This example code is in the public domain.
//
// -------------------------------------------------------------------------------------------
// Slave protocol is as follows:
// -------------------------------------------------------------------------------------------
// WRITE - The I2C Master can write to the device by transmitting the WRITE command,
//         a memory address to store to, and a sequence of data to store.
//         The command sequence is:
//
// START|I2CADDR+W|WRITE|MEMADDR|DATA0|DATA1|DATA2|...|STOP
//
// where START     = I2C START sequence
//       I2CADDR+W = I2C Slave address + I2C write flag
//       WRITE     = WRITE command
//       MEMADDR   = memory address to store data to
//       DATAx     = data byte to store, multiple bytes are stored to increasing address
//       STOP      = I2C STOP sequence
// -------------------------------------------------------------------------------------------
// READ - The I2C Master can read data from the device by transmitting the READ command,
//        a memory address to read from, and then issuing a STOP/START or Repeated-START,
//        followed by reading the data.  The command sequence is:
//
// START|I2CADDR+W|READ|MEMADDR|REPSTART|I2CADDR+R|DATA0|DATA1|DATA2|...|STOP
//
// where START     = I2C START sequence
//       I2CADDR+W = I2C Slave address + I2C write flag
//       READ      = READ command
//       MEMADDR   = memory address to read data from
//       REPSTART  = I2C Repeated-START sequence (or STOP/START if single Master)
//       I2CADDR+R = I2C Slave address + I2C read flag
//       DATAx     = data byte read by Master, multiple bytes are read from increasing address
//       STOP      = I2C STOP sequence
// -------------------------------------------------------------------------------------------
// SETRATE - The I2C Master can adjust the Slave configured I2C rate with this command
//           The command sequence is:
//
// START|I2CADDR+W|SETRATE|RATE|STOP
//
// where START     = I2C START sequence
//       I2CADDR+W = I2C Slave address + I2C write flag
//       SETRATE   = SETRATE command
//       RATE      = I2C RATE to use (must be from i2c_rate enum list, eg. I2C_RATE_xxxx)
// -------------------------------------------------------------------------------------------

#include <i2c_t3.h>

// Command definitions
#define WRITE    0x10
#define READ     0x20
#define SETRATE  0x30

// Function prototypes
void print_i2c_setup(void);
void print_i2c_status(void);


#define N_MOT_SENSORS 5
uint8_t mot_sensor_update;
uint8_t mot_sensor_events[N_MOT_SENSORS];
uint8_t mot_sensor_pins[N_MOT_SENSORS] = {0, 3, 4, 5, 6};


// Memory
//#define MEM_LEN 256
uint8_t databuf[N_MOT_SENSORS];

void setup()
{
    pinMode(LED_BUILTIN,OUTPUT);    // LED
    digitalWrite(LED_BUILTIN,LOW);  // LED off
    pinMode(12,INPUT_PULLUP);       // Control for Test1
    pinMode(11,INPUT_PULLUP);       // Control for Test2
    pinMode(10,INPUT_PULLUP);       // Control for Test3
    pinMode(9,INPUT_PULLUP);        // Control for Test4
    pinMode(8,INPUT_PULLUP);        // Control for Test5

    Serial.begin(115200);

    // Setup for Master mode, pins 18/19, external pullups, 400kHz
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1200);
    
    mot_sensor_update = false;
    for (uint8_t i = 0; i < N_MOT_SENSORS; i++) {
      mot_sensor_events[i] = 0;
    }
}


void loop()
{
    size_t addr, len;
    uint8_t target = 0x44; // slave addr
    uint32_t count;

    //
    // A sequence of different read/write techniques.
    // Pull respective control pin low to initiate sequence.
    //
    // All tests will first write values to the slave, then read back the values.
    // The readback values should match.
    //
    // The LED is turned on during I2C operations.  If it gets stuck on then the
    // ISR likely had a problem.  This can happen with excessive baud rate.
    //
    // Various forms of the Wire calls (blocking/non-blocking/STOP/NOSTOP) are
    // used in the different tests.
    //
    
    mot_sensor_update = true;
    
    for (uint8_t i = 0; i < N_MOT_SENSORS; i++) {
      mot_sensor_events[i] = digitalRead(mot_sensor_pins[i]);
    }
        
    

    if(mot_sensor_update == true)
    {
        mot_sensor_update = false;
        digitalWrite(LED_BUILTIN,HIGH);         // LED on
        Serial.print("---------------------------------------------------------\n");
        Serial.print("Test4 : Using DMA NON-blocking commands:\n");
        Serial.print("        1) WRITE a 256 byte block to Slave.  While block is\n");
        Serial.print("           transferring, perform other commands.\n");
        Serial.print("        2) READ back the 256 byte block from Slave.  While\n");
        Serial.print("           block is transferring, perform other commands.\n");
        Serial.print("---------------------------------------------------------\n");

        // Set operating mode to DMA
        Serial.print("Trying to set DMA mode : ");
        Wire.setOpMode(I2C_OP_MODE_DMA);
        if(Wire.i2c->opMode == I2C_OP_MODE_DMA)
            Serial.printf("OK (Channel %d)\n",Wire.i2c->DMA->channel);
        else
            Serial.print("Failed, using ISR\n");

        // Writing to Slave --------------------------------------------------------
        addr = 0;
        for(uint8_t i = 0; i < N_MOT_SENSORS; i++)           // prepare data to send
            databuf[i] = mot_sensor_events[i];     // set data (equal to bit inverse of memory address)
      
        Serial.printf("I2C WRITE 256 bytes to Slave 0x%0X at MemAddr %d\n", target, addr);
        Serial.print("Writing: ");
        for(uint8_t i = 0; i < N_MOT_SENSORS; i++) { Serial.printf("%d ",databuf[i]); }
        Serial.print("\n");

        Wire.beginTransmission(target);         // slave addr
        Wire.write(WRITE);                      // WRITE command
        Wire.write(addr);                       // memory address
        for(uint8_t i = 0; i < N_MOT_SENSORS; i++)          // write data block
            Wire.write(databuf[i]);
        Wire.sendTransmission();                // NON-blocking write (when not specified I2C_STOP is implicit)

        Serial.print("...write sent, counting while waiting for Wire.done()...\n");
        count = 1;
        while(!Wire.done()) count++; // Since write is non-blocking, do some counting while waiting
        Serial.printf("Counted to: %d\n", count++);
        print_i2c_status();                     // print I2C final status

        // Reading from Slave ------------------------------------------------------
        Wire.beginTransmission(target);         // slave addr
        Wire.write(READ);                       // READ command
        Wire.write(addr);                       // memory address
        Wire.endTransmission(I2C_NOSTOP);       // blocking write (NOSTOP triggers RepSTART on next I2C command)
        Wire.sendRequest(target, N_MOT_SENSORS, I2C_STOP);  // NON-blocking read (request 256 bytes)

        // Since request is non-blocking, do some other things.
        Serial.print("...request sent, doing one thing then waiting for Wire.finish()...\n");

        // After doing something, use finish() to wait until I2C done
        Wire.finish();

        Serial.printf("I2C READ %d bytes from Slave 0x%0X at MemAddr %d\n", Wire.available(), target, addr);
        Serial.print("Received: ");             // print received bytes
        while(Wire.available()) { Serial.printf("%d ", Wire.readByte()); }
        Serial.print("\n");
        print_i2c_status();                     // print I2C final status

        digitalWrite(LED_BUILTIN,LOW);          // LED off
        delay(500);                             // delay to space out tests
    }
}


//
// print current setup
//
void print_i2c_setup()
{
    Serial.print("Mode:");
    switch(Wire.i2c->opMode)
    {
    case I2C_OP_MODE_IMM: Serial.print("IMM    "); break;
    case I2C_OP_MODE_ISR: Serial.print("ISR    "); break;
    case I2C_OP_MODE_DMA: Serial.printf("DMA[%d] ",Wire.i2c->DMA->channel); break;
    }
    Serial.print("Pins:");
    switch(Wire.i2c->currentPins)
    {
    case I2C_PINS_18_19: Serial.print("18/19 "); break;
    case I2C_PINS_16_17: Serial.print("16/17 "); break;
    case I2C_PINS_22_23: Serial.print("22/23 "); break;
    case I2C_PINS_29_30: Serial.print("29/30 "); break;
    case I2C_PINS_26_31: Serial.print("26/31 "); break;
    }
}


//
// print I2C status
//
void print_i2c_status(void)
{
    switch(Wire.status())
    {
    case I2C_WAITING:  Serial.print("I2C waiting, no errors\n"); break;
    case I2C_ADDR_NAK: Serial.print("Slave addr not acknowledged\n"); break;
    case I2C_DATA_NAK: Serial.print("Slave data not acknowledged\n"); break;
    case I2C_ARB_LOST: Serial.print("Bus Error: Arbitration Lost\n"); break;
    case I2C_TIMEOUT:  Serial.print("I2C timeout\n"); break;
    case I2C_BUF_OVF:  Serial.print("I2C buffer overflow\n"); break;
    default:           Serial.print("I2C busy\n"); break;
    }
}


#include <stdint.h>

#if 0
#include <SPI.h>
// SPI and misc pins for the ADNS
#define PIN_SCLK   SCK
#define PIN_MISO   MISO
#define PIN_MOSI   MOSI
#define PIN_NCS    3
#define PIN_MOTION 5

class PMW3320
{
    inline void start_comm()
    {
        digitalWrite( PIN_NCS, LOW );
    }
    inline void end_comm( int predelay, int totaldelay )
    {
        if ( predelay > 0 )
            delayMicroseconds( predelay );

        digitalWrite( PIN_NCS, HIGH );

        if ( totaldelay > predelay )
            delayMicroseconds( totaldelay - predelay );
    }

public:
    enum RegisterID : uint8_t
    {
        PROD_ID              = 0x00,
        REV_ID               = 0x01,
        MOTION               = 0x02,
        DELTA_X              = 0x03,
        DELTA_Y              = 0x04,
        SQUAL                = 0x05,
        SHUT_HI              = 0x06,
        SHUT_LO              = 0x07,
        PIX_MAX              = 0x08,
        PIX_ACCUM            = 0x09,
        PIX_MIN              = 0x0a,
        PIX_GRAB             = 0x0b,
        DELTA_XY             = 0x0c,
        RESOLUTION           = 0x0d,
        RUN_DOWNSHIFT        = 0x0e,
        REST1_PERIOD         = 0x0f,
        REST1_DOWNSHIFT      = 0x10,
        REST2_PERIOD         = 0x11,
        REST2_DOWNSHIFT      = 0x12,
        REST3_PERIOD         = 0x13,
        MIN_SQUAL_RUN        = 0x17,
        AXIS_CONTROL         = 0x1a,
        PERFORMANCE          = 0x22,
        LOW_MOT_JIT          = 0x23,
        SHUT_MAX_HI          = 0x36,
        SHUT_MAX_LO          = 0x37,
        FRAME_RATE           = 0x39,
        POWER_UP_RESET       = 0x3a,
        SHUTDOWN             = 0x3b,
        NOT_REV_ID           = 0x3f,
        LED_CONTROL          = 0x40,
        MOTION_CTRL          = 0x41,
        BURST_READ_FIRST     = 0x42,
        REST_MODE_STATUS     = 0x45,
        NOT_PROD_ID          = 0x4f,
        BURST_MOTION         = 0x63
    };

    // in usecs
    static constexpr int tSRAD = 35;
    static constexpr int tSCLK_NCS_Read  = 30;//1;
    static constexpr int tSCLK_NCS_Write = 30;//20;
    static constexpr int tSRW_tSRR = 120;//20;
    static constexpr int tSWW_tSWR = 120;
    static constexpr int s_clock = 250000;//1000000;

    void Initialize()
    {
        SPI.begin();
        SPI.setBitOrder( MSBFIRST );
        SPI.setDataMode( SPI_MODE3 );

        pinMode( PIN_MISO, INPUT );
        pinMode( PIN_NCS, OUTPUT );

        SPI.beginTransaction(SPISettings(s_clock, MSBFIRST, SPI_MODE3));
          Reboot();
        SPI.endTransaction();
        //SPI.begin();
        // set the details of the communication
        //SPI.setBitOrder( MSBFIRST );
        //SPI.setDataMode( SPI_MODE3 );
        //SPI.setClockDivider( SPI_CLOCK_DIV16 ); // DIV16 16MHz->1MHz
        //delay(10);
    }

    void Reboot()
    {
        end_comm( 0, 0 );
        start_comm();
        end_comm( 0, 0 );

        Write( SHUTDOWN, 0xb6 );
        delay( 300 );

        // bounce ncs to reset SPI
        start_comm();
        end_comm( 40, 80 );
        
        Write( POWER_UP_RESET, 0x5a );
        delay( 100 );

        //Write( MOUSE_CTRL, 0x20);
        Write( LED_CONTROL, 0x20 );
        Write( MOTION_CTRL, 0x00 ); //Clear Motion Control register
    }

    uint8_t Read( RegisterID reg )
    {
        start_comm();

        SPI.transfer( reg );
        delayMicroseconds( tSRAD );
        // read data
        uint8_t data = SPI.transfer( 0 );

        end_comm( tSCLK_NCS_Read, tSRW_tSRR );
        return data;
    }

    void Write( RegisterID reg, byte data )
    {
        start_comm();

        SPI.transfer( static_cast<uint8_t>(reg) | 0x80 );
        SPI.transfer(data);

        end_comm( tSCLK_NCS_Write, tSWW_tSWR );
    }

    int8_t GetX()
    {
        //Convert from 2's complement
        //if (b & 0x80)
        //    b = -1 * ((b ^ 0xff) + 1);
        return static_cast<int8_t>( Read( DELTA_X ) );
    }
    int8_t GetY()
    {
        return static_cast<int8_t>( Read( DELTA_Y ) );
    }

    void PrintReg( RegisterID reg )
    {
        Serial.print( reg, HEX );
        Serial.print( " 0x" );
        Serial.print( Read( reg ), HEX );
        Serial.print( " " );
    }
};

PMW3320 PMW;
unsigned long lastTS;

void setup()
{
    Serial.begin(115200);

    PMW.Initialize();

    lastTS = micros();
}

void loop()
{
    unsigned long elapsed = micros() - lastTS;
    if ( elapsed > 870 )
    {
        SPI.beginTransaction(SPISettings(PMW3320::s_clock, MSBFIRST, SPI_MODE3));
        PMW.PrintReg( PMW3320::PROD_ID );
        PMW.PrintReg( PMW3320::REV_ID );
        PMW.PrintReg( PMW3320::MOTION );
        PMW.PrintReg( PMW3320::DELTA_X );
        PMW.PrintReg( PMW3320::DELTA_Y );
        PMW.PrintReg( PMW3320::SQUAL );
        PMW.PrintReg( PMW3320::SHUT_HI );
        PMW.PrintReg( PMW3320::SHUT_LO );
        PMW.PrintReg( PMW3320::PIX_MAX );
        PMW.PrintReg( PMW3320::PIX_ACCUM );
        PMW.PrintReg( PMW3320::PIX_MIN );
        PMW.PrintReg( PMW3320::PIX_GRAB );
        //PMW.PrintReg( PMW3320::DELTA_XY );
        PMW.PrintReg( PMW3320::RESOLUTION );
        //PMW.PrintReg( PMW3320::RUN_DOWNSHIFT );
        //PMW.PrintReg( PMW3320::REST1_PERIOD );
        //PMW.PrintReg( PMW3320::REST1_DOWNSHIFT );
        //PMW.PrintReg( PMW3320::REST2_PERIOD );
        //PMW.PrintReg( PMW3320::REST2_DOWNSHIFT );
        //PMW.PrintReg( PMW3320::REST3_PERIOD );
        //PMW.PrintReg( PMW3320::MIN_SQUAL_RUN );
        PMW.PrintReg( PMW3320::AXIS_CONTROL );
        PMW.PrintReg( PMW3320::PERFORMANCE );
        PMW.PrintReg( PMW3320::LOW_MOT_JIT );
        PMW.PrintReg( PMW3320::SHUT_MAX_HI );
        PMW.PrintReg( PMW3320::SHUT_MAX_LO );
        PMW.PrintReg( PMW3320::FRAME_RATE );
        PMW.PrintReg( PMW3320::NOT_REV_ID );
        PMW.PrintReg( PMW3320::LED_CONTROL );
        PMW.PrintReg( PMW3320::MOTION_CTRL );
        //PMW.PrintReg( PMW3320::BURST_READ_FIRST );
        PMW.PrintReg( PMW3320::REST_MODE_STATUS );
        PMW.PrintReg( PMW3320::NOT_PROD_ID );

        //Serial.print("Prod ");
        //Serial.print( PMW.Read( PMW3320::PROD_ID ) );
        //Serial.print(" rev ");
        //Serial.print( PMW.Read( PMW3320::REV_ID ) );
        //Serial.print(" mot ");
        //Serial.print( PMW.Read( PMW3320::MOTION ) );
        //Serial.print(" squal ");
        //Serial.print( PMW.Read( PMW3320::SQUAL ) );
        SPI.endTransaction();
 
        Serial.print( "\n" );
        lastTS = micros();
    }
    //Serial.print(" x ");
    //Serial.print( PMW.GetX() );
    //Serial.print(" y ");
    //Serial.println( PMW.GetY() );
    //delay( 100 );
}
#endif

#define PIN_MYSCLK 5
#define PIN_MYSDIO 4
#define PIN_MYNCS 3

#define REG_PRODUCT_ID 0x00
#define REG_REVISION_ID 0x01
#define REG_MOTION 0x02
#define REG_DELTA_X 0x03
#define REG_DELTA_Y 0x04
#define REG_SQUAL 0x05

#define REG_LED_CONTROL 0x40
#define REG_SHUTDOWN 0x3b
#define REG_POWER_UP_RESET 0x3a

void select()
{
  digitalWrite(PIN_MYNCS, LOW);
}

void deselect()
{
  digitalWrite(PIN_MYNCS, HIGH);
}

void reset() {
  pinMode(PIN_MYSCLK, OUTPUT);
  pinMode(PIN_MYSDIO, INPUT);
  pinMode(PIN_MYNCS, OUTPUT);

  digitalWrite(PIN_MYSCLK, LOW);

  deselect();
  delayMicroseconds(1);
  select();
}

byte readRegister(byte address) {
  pinMode (PIN_MYSDIO, OUTPUT);

  // 128 64 32 16 8 4 2 1
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWrite (PIN_MYSCLK, LOW);
    digitalWrite (PIN_MYSDIO, (address & i) != 0 ? HIGH : LOW);
    digitalWrite (PIN_MYSCLK, HIGH);
  }

  pinMode (PIN_MYSDIO, INPUT);

  delayMicroseconds(100); // tHOLD = 100us min.

  byte res = 0;
  for (byte i=128; i >0 ; i >>= 1) {
    digitalWrite (PIN_MYSCLK, LOW);
    digitalWrite (PIN_MYSCLK, HIGH);
    if( digitalRead (PIN_MYSDIO) == HIGH )
      res |= i;
  }

  return res;
}

void writeRegister(byte address, byte data) {
  address |= 0x80; // MSB indicates write mode.
  pinMode (PIN_MYSDIO, OUTPUT);

  for (byte i = 128; i > 0 ; i >>= 1) {
    digitalWrite (PIN_MYSCLK, LOW);
    digitalWrite (PIN_MYSDIO, (address & i) != 0 ? HIGH : LOW);
    digitalWrite (PIN_MYSCLK, HIGH);
  }

  for (byte i = 128; i > 0 ; i >>= 1) {
    digitalWrite (PIN_MYSCLK, LOW);
    digitalWrite (PIN_MYSDIO, (data & i) != 0 ? HIGH : LOW);
    digitalWrite (PIN_MYSCLK, HIGH);
  }

  delayMicroseconds(100); // tSWW, tSWR = 100us min.
}

unsigned long lastTS;
byte isok;

void setup()
{
    Serial.begin(115200);

    reset();

    writeRegister( REG_SHUTDOWN, 0xb6 );
    delay( 300 );

    select();
    delayMicroseconds( 40 );
    deselect();
    delayMicroseconds( 40 );
    select();
        
    writeRegister( REG_POWER_UP_RESET, 0x5a );
    delay( 100 );
 
    byte productId = readRegister(REG_PRODUCT_ID);
    byte revisionId = readRegister(REG_REVISION_ID);
    Serial.print("Found productId ");
    Serial.print(productId, HEX);
    Serial.print(", rev. ");
    Serial.print(revisionId, HEX);
    isok = (productId == 0x3b && revisionId == 0x01) ? 1 : 0;
    Serial.println(isok ? " OK." : " Error !");

    byte config = readRegister(REG_LED_CONTROL);
    Serial.print("init led 0x");
    Serial.println(config, HEX);
    config |= B00000001; // Don't sleep (LED always powered on).
    writeRegister(REG_LED_CONTROL, config);

    lastTS = micros();
}

void dumpDelta()
{
    readRegister(REG_MOTION); // Freezes DX and DY until they are read or MOTION is read again.
    char dx = readRegister(REG_DELTA_X);
    char dy = readRegister(REG_DELTA_Y);

    Serial.print("DELTA: ");
    Serial.print(dx, DEC);
    Serial.print(" ");
    Serial.println(dy, DEC);  
}

void loop()
{
    unsigned long elapsed = micros() - lastTS;
    if ( elapsed > 87000 )
    {
        dumpDelta();
        lastTS = micros();
    }
}

#if 0
void dumpFrame() {
  byte config = readRegister(REG_CONFIG_BITS);
  config |= B00001000; // PixDump
  writeRegister(REG_CONFIG_BITS, config);

  int count = 0;
  do {
    byte data = readRegister(REG_DATA_OUT_LOWER);
    if( (data & 0x80) == 0 ) { // Data is valid
      frame[count++] = data;
    }
  } 
  while (count != dumpWidth);

  config = readRegister(REG_CONFIG_BITS);
  config &= B11110111;
  writeRegister(REG_CONFIG_BITS, config);

  for(int i = 0; i < dumpWidth; i++) {
    byte pix = frame[i];
    if( pix < 0x10 )
      Serial.print("0");
    Serial.print(pix, HEX);
  }
  Serial.println();
}
#endif

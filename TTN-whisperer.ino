/*******************************************************************************

 * TTN-whisperer
 *
 * This sketch is a low power optimized LoRaWan node for sending sensor data
 * to TTN (The Things Network) with a WhisperNode Lora from Wisen.co.au. The
 * data sent to TTN is the temperature read from a DS18B20 sensor, the humidity 
 * from an AM2320 and the battery voltage, measured with the on-board voltage 
 * divider. It can be used as a starting point for own applications based on the
 * WhisperNode Lora.
 * 
 * Power optimisations:
 * - no LED usage
 * - only three bytes of LoRa payload
 * - sleep for a long time
 * - send measurement only if difference in value
 * - memorize OneWire sensor adresses on startup
 * - deactivate voltage divider when not used
 * - use lower DS18B20 resolution
 * - sent raw ADC data, calculation on server side
 * 
 * Further power optimisations to be done in future:
 * - reduce serial speed in DEBUG mode
 * - power off OneWire bus when not used
 * - power off I2C sensor when not used
 * - change delay of OneWire library to a sleep
 * - put on-board Flash to powerdown mode
 * - set all unused pins to input/pullup
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 * 
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include "LowPower.h"

#include <OneWire.h>
#include <DallasTemperature.h>

#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

/* Constants and constant variables ****************************/

// if debug mode is active, use LED and serial output. Powered by USB/FTDI-adapter.
#define DEBUGMODE false

#if DEBUGMODE
// number of 8s sleepcycles between waking up
const byte SLEEPCYCLES = 38;      // 5 minutes
const byte MAX_NOSEND_CYCLES = 4; // Send at least every 20 minutes
#else
const byte SLEEPCYCLES = 214;      // 30 minutes is 225 cycles. Experimentally 214 seems to be closed (clock offset)
const byte MAX_NOSEND_CYCLES = 3;  // Wait maximum of 3 x 30 minutes, send at least every 2 hours
#endif

const byte PIN_LED = 6;
const byte PIN_ONE_WIRE = A1;
const byte PIN_ADC_ACTIVATE = A0;
#if DEBUGMODE
const byte PIN_VOLTAGE_ADC = A7;
#else
const byte PIN_VOLTAGE_ADC = A6;
#endif

const byte TEMPERATURE_PRECISION = 10; //10 bits equals 0.25°C resolution


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued APPEUI the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Pin mapping is hardware specific.
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 7,
    .dio = {2, A2, LMIC_UNUSED_PIN},//DIO0 and DIO1 connected
};


/* Variables ***************************************************/

bool joined = false;
bool sleeping = false;
byte nosend_cycles = 0;

float temperature_sent = 0.0;
float humidity_sent = 0.0;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(PIN_ONE_WIRE);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress sensor_address_air;

// I2C humidity sensor
Adafruit_AM2320 am2320 = Adafruit_AM2320();

/* Functions ***************************************************/


static void initfunc (osjob_t*);

// provide APPEUI (8 bytes, LSBF)
void os_getArtEui (u1_t* buf)
{
  memcpy(buf, APPEUI, 8);
}

// provide DEVEUI (8 bytes, LSBF)
void os_getDevEui (u1_t* buf)
{
  memcpy(buf, DEVEUI, 8);
}

// provide APPKEY key (16 bytes)
void os_getDevKey (u1_t* buf)
{
  memcpy(buf, APPKEY, 16);
}

static osjob_t sendjob;
static osjob_t initjob;


void onEvent (ev_t ev)
{
  int i,j;
  
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      #if DEBUGMODE
        Serial.println(F("EV_SCAN_TIMEOUT"));
      #endif
      break;
    case EV_BEACON_FOUND:
      #if DEBUGMODE
        Serial.println(F("EV_BEACON_FOUND"));
      #endif
      break;
    case EV_BEACON_MISSED:
      #if DEBUGMODE
        Serial.println(F("EV_BEACON_MISSED"));
      #endif
      break;
    case EV_BEACON_TRACKED:
      #if DEBUGMODE
        Serial.println(F("EV_BEACON_TRACKED"));
      #endif
      break;
    case EV_JOINING:
      #if DEBUGMODE
        Serial.println(F("EV_JOINING"));
      #endif
      break;
    case EV_JOINED:
      #if DEBUGMODE
        Serial.println(F("EV_JOINED"));
      #endif
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      digitalWrite(PIN_LED,LOW);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      #if DEBUGMODE
        Serial.println(F("EV_RFU1"));
      #endif
      break;
    case EV_JOIN_FAILED:
      #if DEBUGMODE
        Serial.println(F("EV_JOIN_FAILED"));
      #endif
      break;
    case EV_REJOIN_FAILED:
      #if DEBUGMODE
        Serial.println(F("EV_REJOIN_FAILED"));
      #endif
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
      #if DEBUGMODE
        if (LMIC.dataLen) {
          // data received in rx slot after tx
          // if any data received, a LED will blink
          // this number of times, with a maximum of 10
          Serial.print(F("Data Received: "));
          Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
          i=(LMIC.frame[LMIC.dataBeg]);
          // i (0..255) can be used as data for any other application
          // like controlling a relay, showing a display message etc.
          if (i>10)
          {
            i=10;     // maximum number of BLINKs
          }
          for(j=0;j<i;j++)
          {
            digitalWrite(PIN_LED,HIGH);
            delay(200);
            digitalWrite(PIN_LED,LOW);
            delay(400);
          }
        }
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        delay(60);  // delay to complete Serial Output before Sleeping
      #endif

      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      #if DEBUGMODE
        Serial.println(F("EV_LOST_TSYNC"));
      #endif
      break;
    case EV_RESET:
      #if DEBUGMODE
        Serial.println(F("EV_RESET"));
      #endif
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      #if DEBUGMODE
        Serial.println(F("EV_RXCOMPLETE"));
      #endif
      break;
    case EV_LINK_DEAD:
      #if DEBUGMODE
        Serial.println(F("EV_LINK_DEAD"));
      #endif
      break;
    case EV_LINK_ALIVE:
      #if DEBUGMODE
        Serial.println(F("EV_LINK_ALIVE"));
      #endif
      break;
    default:
      #if DEBUGMODE
        Serial.println(F("Unknown event"));
      #endif
      break;
  }
}

void do_send(osjob_t* j) {
  byte buffer[22];
  float temperature_float = 0.0;
  float humidity_float = 0.0;
  float voltage_float = 0.0;
  float temperature_difference = 0.0;
  float humidity_difference = 0.0;

  // measure values and calculate difference
  sensors.requestTemperatures(); // Send the command to get temperatures
  temperature_float = sensors.getTempC(sensor_address_air);
  temperature_difference = temperature_float - temperature_sent;
  humidity_float = am2320.readHumidity();
  humidity_difference = humidity_float - humidity_sent;
  
  // Send something only if values changed or maximum sleep time reached
  if ( ((abs(temperature_difference)) >= 0.5)
    || ((abs(humidity_difference)) >= 5.0)
    || (nosend_cycles >= MAX_NOSEND_CYCLES) )
  {
    temperature_sent = temperature_float;
    humidity_sent = humidity_float;
    nosend_cycles = 0;
    
    // scale temperature to 0.25deg C per bit
    temperature_float = temperature_float * 4 + 80;
    buffer[0] = (int)temperature_float;
  
    // don't scale humidity yet (basically 6 bits would be enough for a 2% resolution)
    buffer[2] = (int)humidity_float;

    // Read supply voltage
    pinMode(PIN_VOLTAGE_ADC, INPUT);
    analogReference(INTERNAL); // use internal 1.1V reference
    digitalWrite(PIN_ADC_ACTIVATE, HIGH);
    delay(5);
    buffer[1] = (int)(analogRead(PIN_VOLTAGE_ADC) >>2);
    digitalWrite(PIN_ADC_ACTIVATE, LOW);
    pinMode(PIN_VOLTAGE_ADC, OUTPUT); // has to be set back to output, if not LMIC crashes although according to schema there is no connection...
    
    #if DEBUGMODE
      Serial.print(F("Temperature:"));
      Serial.print(temperature_sent, DEC);
      Serial.print(F(" Humidity:"));
      Serial.println(humidity_float, DEC);
      voltage_float = (7.282 * buffer[1]) / 256;
      Serial.print(F("Supply voltage:"));
      Serial.println(voltage_float, DEC);
    #endif
    
    if (LMIC.opmode & OP_TXRXPEND)
    {
      #if DEBUGMODE
        Serial.println(F("OP_TXRXPEND, not sending"));
      #endif
    } else
    {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, (uint8_t*) buffer, 3 , 0);
      #if DEBUGMODE
        Serial.println(F("Sending: "));
      #endif
    }
  } else
  {
    //almost the same temperature and humidity, don't send
    nosend_cycles++;
    sleeping = true;
  }
}

// initial job
static void initfunc (osjob_t* j)
{
  // reset MAC state
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // start joining
  LMIC_startJoining();
  // init done - onEvent() callback will be invoked...
}

void setup()
{    
  delay(10000);
  #if DEBUGMODE
    Serial.begin(9600);
    Serial.println(F("Starting"));
  #endif
  delay(10000);
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  os_setCallback(&initjob, initfunc);
  LMIC_reset();

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ADC_ACTIVATE, OUTPUT);

  // Start up the OneWire library
  sensors.begin();
  if (!sensors.getAddress(sensor_address_air, 0))
  {
    #if DEBUGMODE
      Serial.println("Unable to find address for Device 0");
    #endif
  } else
  {
    sensors.setResolution(sensor_address_air, TEMPERATURE_PRECISION);
  }
  
  // Start up the AM2320 library
  am2320.begin();
  
}


void loop()
{
  // start OTAA JOIN
  if (joined==false)
  {
    os_runloop_once();
  }
  else
  {
    do_send(&sendjob);    // Send sensor values
    while(sleeping == false)
    {
      os_runloop_once();
    }
    sleeping = false;
    for (byte i = 0; i < SLEEPCYCLES; i++)
    {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds
    }
  }
  #if DEBUGMODE
    digitalWrite(PIN_LED,((millis()/100) % 2) && (joined==false)); // only blinking when joining and not sleeping
  #endif

}


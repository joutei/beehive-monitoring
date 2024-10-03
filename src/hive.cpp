#include "hive.h"

const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

//Old Setup : MPU6050
Adafruit_MPU6050 mpu;


Adafruit_SHT31 sht31 = Adafruit_SHT31();

//TinyGPSPlus gps;
//SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

//Limit switch (fin de course) 
//TO UPDATE : Reduce openCount, closeCount to uint8_t
uint16_t openCount = 0;
uint16_t closeCount = 0;
uint32_t lastOpen = 0;

bool updateOpen = true;
bool updateClose = true; 

uint8_t mydata[22];
osjob_t sendjob;
osjob_t readCriticalJob;
osjob_t updatejob;

uint8_t isReset = 1;






void onEvent (ev_t ev) {
    Serial.print("ticks : " + String(os_getTime()) + ", ms : " + String(osticks2ms(os_getTime())));
    Serial.print(": ");
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            // Schedule next transmission
            os_setTimedCallback(&updatejob, os_getTime()+ (sec2osticks(UPDATE_INTERVAL) - ms2osticks(UPDATE_DURATION)), do_update);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            //if (!os_queryTimeCriticalJobs(ms2osticks(CRIT_CHECK_INTERVAL))) { // Had to make sure there is nothing
              os_setTimedCallback(&readCriticalJob, os_getTime()+ms2osticks(CRIT_CHECK_INTERVAL - CRIT_CHECK_DURATION), do_read_critical); 
//              Serial.println(F("Nothing to worry about."));}
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}



//MPU6050
/*void updateMPU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    int16_t tempInt = temp.temperature * 100; // Convert temperature to an integer to avoid floating point in payload
    mydata[0] = tempInt >> 8;
    mydata[1] = tempInt & 0xFF;
}*/



//SHT31, read and pack temperature and humidity data (mydata : 0-3)
void readAndPackTempHum(){
  float temperature = sht31.readTemperature();
  float humidity = sht31.readHumidity();

  // Check if readings failed and if so, use a sentinel value
  if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from SHT31 sensor!");
      return;
  }
  
  // Convvert temperature and humidity to integers for transmission
  int16_t tempInt = temperature  * 100; // e.g., 23.45 degrees -> 2345
  int16_t humInt = humidity * 100;     // e.g., 45.67% -> 4567

  // Pack the temperature into first two bytes of mydata
  mydata[0] = tempInt >> 8;
  mydata[1] = tempInt & 0xFF;

  // Pack the humidity into next two bytes of mydata
  mydata[2] = humInt >> 8;
  mydata[3] = humInt & 0xFF;

  Serial.println("temp : " + String(temperature) +", humid : " + String (humidity));
}


// Old Setup : GNSS (NEO-6M), read and pack GNSS data (mydata : 13-16 for lat, 17-20 for long)
/*void readAndPackGNSS() {
  while (gpsSerial.available() > 0) {  // Assuming NEO-6M is connected to gpsSerial
    if (gps.encode(gpsSerial.read())) {  // Process GNSS data
      if (gps.location.isValid()) {
        // Latitude
        int32_t lat = gps.location.lat() * 100000; // Convert latitude to integer by scaling
        mydata[13] = (lat >> 24) & 0xFF;
        mydata[14] = (lat >> 16) & 0xFF;
        mydata[15] = (lat >> 8) & 0xFF;
        mydata[16] = lat & 0xFF;

        // Longitude
        int32_t lng = gps.location.lng() * 100000; // Convert longitude to integer by scaling
        mydata[17] = (lng >> 24) & 0xFF;
        mydata[18] = (lng >> 16) & 0xFF;
        mydata[19] = (lng >> 8) & 0xFF;
        mydata[20] = lng & 0xFF;

        Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
      } else {
        Serial.println("GNSS data not valid.");
      }
    }
  }

  // Handle case where no valid GNSS data is available
  if (!gps.location.isValid()) {
    // Indicate invalid data with a specific value or error handling
    Serial.println("Failed to get valid GNSS data.");
  }
}*/


//LimitSwitch, packs switch related data (mydata : 4-11) 
//TO UPDATE : packaging of openCount/closeCount to hold one slot only
void packSwitch() {
  if (updateOpen) {
    mydata[4] = openCount >> 8;
    mydata[5] = openCount & 0xFF;

    mydata[6] = lastOpen >> 24;
    mydata[7] = (lastOpen >> 16) & 0xFF;
    mydata[8] = (lastOpen >> 8) & 0xFF;
    mydata[9] = lastOpen & 0xFF;

    updateOpen = false;
  }
    
  if (updateClose) {
    mydata[10] = closeCount >> 8;
    mydata[11] = closeCount & 0xFF;

    updateClose = false;
  }
  Serial.println("openCount : " + String(openCount) + ", lastOpen : " + String(lastOpen) + ", closeCount : " + String(closeCount));
}



// Updates switch related data
void checkSwitch(s4_t currTime) {
  int switchState = digitalRead(limitSwitchPin);
  if (openCount == closeCount && switchState == HIGH) { // openCount == closeCount : means last time we checked it was closed
    openCount ++;
    lastOpen = currTime / 1000;
    updateOpen = true;
  }

  else if (openCount > closeCount && switchState == LOW) { // openCount > closeCount : means last time we checked it was open
    closeCount ++;
    updateClose = true;
  }
}


//reads all data off of the time critical sensors, eventually limitswitch and accelerometer
void checkTimeCriticalSensors() {
  s4_t currTime = osticks2ms(os_getTime()); // We dont use millis() cuz it's interrupt dependant and LMIC by default has interrupts disabled

  checkSwitch(currTime);
}





void updateData() { 
  readAndPackTempHum(); // 0-3
  packSwitch(); //4-11

  //Pack the reset flag
  mydata[12] = isReset; 
  Serial.println("reset : " + String(isReset));
}

/* Decoder for TTN :
//TO UPDATE : packaging of openCount/closeCount to hold one slot only
function Decoder(bytes, port) {
    var decoded = {};
  
    // Decode temperature
    var tempRaw = (bytes[0] << 8) | bytes[1];
    decoded.temperature = tempRaw / 100.0;
  
    // Decode humidity
    var humRaw = (bytes[2] << 8) | bytes[3];
    decoded.humidity = humRaw / 100.0;

    // Decode openCount
    decoded.openCnt = (bytes[4] << 8) | bytes[5];

    //Decode lastOpen
    decoded.lastOpen = (bytes[6] << 24) | (bytes[7] << 16) | (bytes[8] << 8) | bytes[9];
  
    //Decode closeCount
    decoded.closeCnt = (bytes[10] << 8) | bytes[11];

    //Decode resetFLag; 
    decoded.reset = bytes[12];


  
    return decoded;
}
*/




void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void do_update(osjob_t* j){
  Serial.println(F("Start update!"));
  updateData();

  //If in (UPDATE_INTERVAL) milliseconds no jobs are scheduled, schedule the next check in (UPDATE_INTERVAL - UPDATE_DURATION) milliseconds. 
  //It'll (take UPDATE_DURATION) milliseconds for the function to complete running.
  if (!os_queryTimeCriticalJobs(ms2osticks(UPDATE_INTERVAL))) 
    os_setTimedCallback(&updatejob, os_getTime()+ms2osticks(UPDATE_INTERVAL - UPDATE_DURATION), do_update);

  Serial.println(F("Updated!"));
}



void do_read_critical(osjob_t* j){ 
  Serial.println(F("Start critical!"));
  checkTimeCriticalSensors();

  //If in (CRIT_CHECK_INTERVAL) milliseconds no jobs are scheduled, schedule the next check in (CRIT_CHECK_INTERVAL - CRIT_CHECK_DURATION) milliseconds. 
  //It'll take (CRIT_CHECK_DURATION) milliseconds for the function to complete running.
  if (!os_queryTimeCriticalJobs(ms2osticks(CRIT_CHECK_INTERVAL))) {
    os_setTimedCallback(&readCriticalJob, os_getTime()+ms2osticks(CRIT_CHECK_INTERVAL - CRIT_CHECK_DURATION), do_read_critical);
  }

  Serial.println(F("Read critical!"));
}

void setupMpu() {
  if (!mpu.begin()) {
        Serial.println(F("Failed to find MPU6050 chip"));
        while (1) {
            delay(10);
        }
    }
  Serial.println(F("MPU6050 Found!"));

  //Setup measurement frequency for low power purposes
  mpu.enableSleep(false);
  mpu.enableCycle(false);
  mpu.setCycleRate(MPU6050_CYCLE_5_HZ); // 5 measurements per second
  mpu.enableCycle(true);

  //Setup the motion detection interrupt
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

void initSensors() {
  while (!Serial); // wait for Serial to be initialized
  Serial.begin(115200);
  delay(100);

  setupMpu();
 
  if (!sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println(F("Couldn't find SHT31"));
    while (1) delay(1);
  }
  Serial.println(F("SHT31 Found!"));


  // TO DO : ADD SOME READINGS FOR ALL THE SENSORS TO REMOVE THE FIRST JUNK DATA
  
  pinMode(limitSwitchPin, INPUT_PULLUP);
}





void initAndSetupOS() {
  Serial.println(F("Starting LMIC OS"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  #if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
  for (int channel = 0; channel < 9; channel++) {
      if (channel == 0) {
          LMIC_setupChannel(channel, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);
      } else {
          LMIC_disableChannel(channel);
      }
  }
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
  #else
  # error Region not supported
  #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7,14);
}



void goSleep() {
      //flush the serial buffer (like a toilet), and release any text left (any water)
      //If we don't, the mcu goes to sleep before printing what's there to print
      Serial.flush(); 

      // disable ADC
      ADCSRA = 0; 

      // turn off all the boards module
      power_all_disable ();

      // clear various "reset" flags
      MCUSR = 0;     
      // allow changes, disable reset
      WDTCSR = bit (WDCE) | bit (WDE);
      // set interrupt mode and an interval 
      WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
      wdt_reset();  // pat the dog
      
      set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
      noInterrupts ();           // timed sequence follows
      sleep_enable();

      // turn off brown-out enable in software
      MCUCR = bit (BODS) | bit (BODSE);
      MCUCR = bit (BODS); 
      interrupts ();             // guarantees next instruction executed
      sleep_cpu ();  
      
      // cancel sleep as a precaution
      sleep_disable();


      // Re-enable and start the needed modules
      power_timer0_enable(); // Needed by the libraries timer (micros())

      power_spi_enable(); // Needed by the Lora Transceiver
      //SPI.begin();

      power_twi_enable(); // I2C - Needed by the temp/hum sensor (SHT31) and accelerometer (MPU6050)
      //Wire.begin();

      power_usart0_enable();
      //Serial.begin(9600);
      
      // Correct the time (tick) counter specific to the library LMIC
      osTimeCorrection(SLEEP_AND_WAKE_UP_DURATION); // 200 is an adjustment post-testing
}

// watchdog interrupt
ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
}  // end of WDT_vect

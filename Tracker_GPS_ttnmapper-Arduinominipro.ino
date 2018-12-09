/*******************************************************************************
   GPS TTNMapper by Akirasan 2018
    Nodo LoRaWAN con GPS para realizar mapeo sobre TTNMapper

   twitter: @akirasan

   Parte del código es grácias al ejemplo del envío de un paquete "Hello, world!"
  por LoRaWAN utilizando autenticación ABP, realizado por:
     Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
*/

#define GPS_ON        // GPS conectado. Si no está definido se envían datos nulos. Para uso con TTNMAPPER App móvil
#define DEBUG_ON      // Info por Serial
#define MODO_TEST     // Usa solo canal 0 ---- SOLO TEST
#define BAUD_SERIAL 115200


#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>


#ifdef GPS_ON
#include <SoftwareSerial.h>
#include <MicroNMEA.h>

#define RX_GPS 4
#define TX_GPS 3
#define BAUD_GPS 9600

// Refer to serial devices by use
HardwareSerial& console = Serial;
SoftwareSerial gps(RX_GPS, TX_GPS);

char nmeaBuffer[500];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
#endif GPS_ON


//*************************************
//***** CONFIGURACION LORAWAN *********

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x95, 0x5A, 0xE4, 0x3E, 0x75, 0x9C, 0xD3, 0xAA, 0xDF, 0x72, 0x9D, 0x29, 0xDF, 0x29, 0xF1, 0x4F};

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x53, 0x61, 0x36, 0x77, 0xC8, 0x1A, 0x9D, 0x3F, 0xED, 0x9E, 0xC3, 0xC2, 0xCE, 0xE0, 0xF4, 0x66 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260117BA ; // <-- Change this address for every node!
//**************************************



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static uint8_t payload[13];
static osjob_t sendjob;

// Planificacion del envio de paquetes LoRa. El intervalo es en segundos
const unsigned TX_INTERVAL = 30;


// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};


//************ Gestion de enventos del módulo LoRa RFM95W
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), enviar_datos_GPS);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}



// ----------------------------
// Captura y envío de datos GPS  -----------------------------
// ----------------------------
void enviar_datos_GPS(osjob_t* j) {
  boolean datos_gps = false;
  char c_gps;

  // GPS --- lat/lon
  long latitude_long = 0;
  long longitude_long = 0;

  // GPS --- Altitud
  long alt = 0;
  word altitude = 0;

  // GPS --- HDOP (horizontal dilution of precision)
  byte hdop = 0;

  // GPS --- Numero de satélites
  byte sats = 0;
#ifdef DEBUG_ON
  Serial.println(F("JOB para el datos GPS"));
#endif DEBUG_ON

#ifdef GPS_ON
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND"));
    Serial.println(OP_TXRXPEND);
    Serial.println(LMIC.opmode);
    Serial.println("---------------");
  } else {
    // Prepare upstream data transmission at the next possible time.
    // Empty input buffer
    while (gps.available()) {
      gps.read();
    }
    delay(500);

    nmea.clear();

    Serial.println("LEEMOS DATOS NUEVOS GPS");
    while (!(nmea.isValid())) {
      while (gps.available()) {
        c_gps = gps.read();
#ifdef DEBUG_ON
        console.print(c_gps);
#endif DEBUG_ON
        nmea.process(c_gps);
      }
    }
#ifdef DEBUG_ON
    Serial.print("================================ NAV SYSTEM: ");
    Serial.println(nmea.getNavSystem());
#endif DEBUG_ON

    if ((nmea.isValid()) & (nmea.getNavSystem() != "\0")) {

      // ----- MicroNMEA devuelve la info sin decimales
      // por lo tanto nos ahorramos el siguiente paso de pasarlo a entero
      // multiplicando por 10000000
      //----------------------------
      //float latitude = nmea.getLatitude();
      //float longitude = nmea.getLongitude();
      //long latitude_long = latitude * 10000000;
      //long longitude_long = longitude * 10000000;

      // GPS --- Latitud + Longitud
      latitude_long = nmea.getLatitude();
      longitude_long = nmea.getLongitude();


      // GPS --- Altitud
      altitude = 0;
      if (nmea.getAltitude(alt)) {   // Altitud correcta??
        altitude = alt / 1000;
      }

      // GPS --- HDOP (horizontal dilution of precision)
      hdop = nmea.getHDOP();

      // GPS --- Numero de satélites
      sats = nmea.getNumSatellites();

      // -----
      // ----- Conversión de datos recogidos a PAYLOAD LORAWAN

      // [0..3] 4 bytes LATITUDE
      payload[0] = (byte) ((latitude_long & 0xFF000000) >> 24 );
      payload[1] = (byte) ((latitude_long & 0x00FF0000) >> 16 );
      payload[2] = (byte) ((latitude_long & 0x0000FF00) >> 8 );
      payload[3] = (byte) ((latitude_long & 0X000000FF));

      // [4..7] 4 bytes LONGITUDE
      payload[4] = (byte) ((longitude_long & 0xFF000000) >> 24 );
      payload[5] = (byte) ((longitude_long & 0x00FF0000) >> 16 );
      payload[6] = (byte) ((longitude_long & 0x0000FF00) >> 8 );
      payload[7] = (byte) ((longitude_long & 0X000000FF));

      // [8..9] 2 bytes ALTITUDE
      payload[8] = (byte) ((altitude & 0xFF00) >> 8);
      payload[9] = (byte) (altitude & 0x00FF);

      // [10] 1 byte HDOP
      payload[10] = (byte) hdop;  //HDOP

      // [11] 1 byte SATELLITES
      payload[11] = (byte) sats; //Satelites

#ifdef DEBUG_ON
      Serial.println(F("datos GPS recogidos ------"));
      Serial.print(F("latitude: ")); Serial.println(latitude_long);
      Serial.print(F("longitude: ")); Serial.println(longitude_long);
      Serial.print(F("altitud: ")); Serial.println(altitude);
      Serial.print(F("hdop: ")); Serial.println(hdop);
      Serial.print(F("sats: ")); Serial.println(sats);
      Serial.println(F("-----------------"));
#endif DEBUG_ON

      datos_gps = true;
    }
    else {
#ifdef DEBUG_ON
      Serial.println(F("NMEA no VALIDO"));
#endif DEBUG_ON
    }
    // FIN tratamiento datos GPS

    // --- Revisamos si hemos recogido datos nuevos del GPS
    if (datos_gps) {
#ifdef DEBUG_ON
      Serial.print(F("...envio datos del GPS LoRaWAN. Tamaño paquete: "));
      Serial.println(sizeof(payload) - 1);
#endif DEBUG_ON

      // Envío de datos por LoRaWAN
      LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);
    }
  }
  // Reprogramamos el siguiente envío
  os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), enviar_datos_GPS);
#endif GPS_ON

#ifndef GPS_ON
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND"));
    Serial.println(OP_TXRXPEND);
    Serial.println(LMIC.opmode);
    Serial.println("---------------");
  } else {
    // Prepare upstream data transmission at the next possible time.
#ifdef DEBUG_ON
    Serial.println(F("enviando paquete vacío para TTNMapper con aplicación móvil"));
#endif DEBUG_ON
    LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);
    // Reprogramamos el siguiente envío
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), enviar_datos_GPS);
  }
#endif GPS_ON

}

// ---CODE FROM
//https://www.thethingsnetwork.org/forum/t/can-lmic-1-6-be-set-up-to-use-a-single-channel-and-sf/5207/11
// Disables all channels, except for the one defined above, and sets the
// data rate (SF). This only affects uplinks; for downlinks the default
// channels or the configuration from the OTAA Join Accept are used.
//
// Not LoRaWAN compliant; FOR TESTING ONLY!
//
void forceTxSingleChannelDr(int channel, int dr) {
  for (int i = 0; i < 9; i++) { // For EU; for US use i<71
    if (i != channel) {
      LMIC_disableChannel(i);
    }
  }
  // Set data rate (SF) and transmit power for uplink
  LMIC_setDrTxpow(dr, 14);
}
// ---CODE




void inicializar_LoRa() {
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
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);


#ifdef MODO_TEST
  // --------------------
  // Desactivamos todos los canales menos el canal 0 --> SOLO PARA TESTING!!!
  // Define the single channel and data rate (SF) to use
  // channel = 0   /    DR_SF7
  forceTxSingleChannelDr(0, DR_SF7);
  // ------------------
#endif MODO_TEST

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Iniciamos job de envio datos GPS
  enviar_datos_GPS(&sendjob);
}


void setup() {
  Serial.begin(BAUD_SERIAL);
  Serial.println(F("Starting"));
#ifdef GPS_ON
  gps.begin(BAUD_GPS);        // gps
#endif GPS_ON

  inicializar_LoRa();
}



void loop() {
  os_runloop_once();
}

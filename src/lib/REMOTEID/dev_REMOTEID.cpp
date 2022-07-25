#include "device.h"
// #include "rxtx_common.h"

#if defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32)

#if defined(TARGET_UNIFIED_TX) || defined(TARGET_UNIFIED_RX)
#include <ArduinoJson.h>
#if defined(PLATFORM_ESP8266)
#include <FS.h>
#else
#include <SPIFFS.h>
#endif
#endif

#if defined(PLATFORM_ESP32)
#include <Update.h>
#include <WiFi.h>

#include <esp_wifi.h>

extern "C" {
  esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
}

#else
#include <ESP8266WiFi.h>

extern "C"
{
#include "user_interface.h"
    typedef void (*freedom_outside_cb_t)(uint8 status);
    int wifi_register_send_pkt_freedom_cb(freedom_outside_cb_t cb);
    void wifi_unregister_send_pkt_freedom_cb(void);
    int wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}
// ==================== //

#endif

enum RemoteID_OperationalStatus
{
    RID_OP_Undeclared = 0,
    RID_OP_Ground = 1,
    RID_OP_Airborne = 2,
    RID_OP_Emergency = 3
};
enum RemoteID_MessageType
{
    RID_MT_BasicID = 0,
    RID_MT_Location = 1,
    RID_MT_System = 4,
    RID_MT_MessagePack = 0xf
};
enum RemoteID_IDType
{
    RID_ID_None = 0,
    RID_ID_SerialNumber = 1,
    RID_ID_CAA_AssignedRegistrationID = 2,
    RID_ID_UTM_AssignedUUID = 3,
    RID_ID_SpecificSessionID = 4
};
enum RemoteID_UAType
{
    RID_UA_None = 0,
    RID_UA_Aeroplane = 1,
    RID_UA_Multirotor = 2,
    RID_UA_Gyroplane = 3,
    RID_UA_HybridLift = 4,
    RID_UA_FreeFall_Parachute = 11,
    RID_UA_Rocket = 12,
    RID_UA_Other = 15
};
enum RemoteID_HighType
{
    RID_HT_AboveTakeOff = 0,
    RID_HT_AGL = 1
};
enum RemoteID_DirectionSegment
{
    RID_DS_Less180 = 0,
    RID_DS_More180 = 1
};
enum RemoteID_SpeedMultiplier
{
    RID_SM_025 = 0,
    RID_SM_075 = 1
};
enum RemoteID_ClassificationType
{
    RID_CT_Undeclared = 0,
    RID_CT_EU = 1
};
enum RemoteID_OperatorLocation
{
    RID_OL_TakeOffLocation = 0,
    RID_OL_LiveGNSS = 1,
    RID_OL_Fixedlocation = 2
};
enum RemoteID_UAClassification
{
    RID_UAC_Undefined = 0,
    RID_UAC_Open = 1,
    RID_UAC_Specific = 2,
    RID_UAC_Certified = 3
};
enum RemoteID_UAClassificationClass
{
    RID_AUCC_Undefined = 0,
    RID_AUCC_0 = 1,
    RID_AUCC_1 = 2,
    RID_AUCC_2 = 3,
    RID_AUCC_3 = 4,
    RID_AUCC_4 = 5,
    RID_AUCC_5 = 6,
    RID_AUCC_6 = 7
};

#if defined(USE_MSP_WIFI) && defined(TARGET_RX) // MSP2WIFI in enabled only for RX only at the moment
#include "CRSF.h"
extern CRSF crsf;
#endif

#include <StreamString.h>
#include <set>

#include "FHSS.h"
#include "POWERMGNT.h"
#include "common.h"
#include "devVTXSPI.h"
#include "helpers.h"
#include "hwTimer.h"
#include "logging.h"
#include "options.h"

#include "config.h"

static bool rIDStarted = false;

// static wl_status_t laststatus = WL_IDLE_STATUS;
// volatile WiFiMode_t wifiMode = WIFI_OFF;
// static volatile WiFiMode_t changeMode = WIFI_OFF;
// static volatile unsigned long changeTime = 0;

// ===== Settings ===== //
const bool appendSpaces = false; // makes all SSIDs 32 characters long to improve performance

const uint8_t batch = 2; // number of packets in batch

const uint32_t period = 300; // every x m to run

const char ssid[] PROGMEM = "REMOTEID";

// run-time variables
char emptySSID[32];
uint8_t macAddr[6];
uint8_t wifi_channel = 6; // channel
uint32_t packetSize = 0;
uint32_t currentTime = 0;
uint32_t attackTime = 0;

uint8_t counter = 0;

// beacon frame definition
uint8_t beaconPacket[194] = {
    /*  0 - 3  */ 0x80, 0x00, 0x00, 0x00,             // Type/Subtype: managment beacon frame
    /*  4 - 9  */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // Destination: broadcast
    /* 10 - 15 */ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // Source
    /* 16 - 21 */ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // Source

    // Fixed parameters
    /* 22 - 23 */ 0x00, 0x00,                                     // Fragment & sequence number (will be done by the SDK)
    /* 24 - 31 */ 0x83, 0x51, 0xf7, 0x8f, 0x0f, 0x00, 0x00, 0x00, // Timestamp
    /* 32 - 33 */ 0xe8, 0x03,                                     // Interval: 0x64, 0x00 => every 100ms - 0xe8, 0x03 => every 1s
    /* 34 - 35 */ 0x31, 0x00,                                     // capabilities Tnformation

    // Tagged parameters

    // SSID parameters
    /* 36 - 37 */ 0x00, 0x20, // Tag: Set SSID length, Tag length: 32
    /* 38 - 69 */ 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x20, // SSID

    // Supported Rates
    /* 70 - 71 */ 0x01, 0x08, // Tag: Supported Rates, Tag length: 8
    /* 72 */ 0x82,            // 1(B)
    /* 73 */ 0x84,            // 2(B)
    /* 74 */ 0x8b,            // 5.5(B)
    /* 75 */ 0x96,            // 11(B)
    /* 76 */ 0x24,            // 18
    /* 77 */ 0x30,            // 24
    /* 78 */ 0x48,            // 36
    /* 79 */ 0x6c,            // 54

    // Current Channel
    /* 80 - 81 */ 0x03, 0x01, // Channel set, length
    /* 82 */ 0x01,            // Current Channel

    // Remote ID
    /*  83 (vendor specific) -  84 (size) */ 221, (uint8_t)(8 + 3*25),
    /*  85 - 87 (REMOTE ID OUI) */ 0xFA, 0x0B, 0xBC,
    /*  88 (REMOTE ID VENDOR TYPE) */ 0x0D,
    /*  89 (Message counter) */ 0x00,

    /*  90 (message pack + message version) */ (RID_MT_MessagePack << 4) | 0x02,
    /*  91 (single message size) */ 25,
    /*  92 (number of messages in a pack) */ 0x03,
    
    /*  93 (message header for basic ID) */ (RID_MT_BasicID << 4) | 0x02,
    /*  94 (serial number ID type; aeroplane type) */ (RID_ID_None << 4) | RID_UA_Multirotor,
    /*  95 - 115 (serial number) */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /*  116 - 118 (reserved) */ 0, 0, 0,

    /*  119 (message header for location/vector message) */ (RID_MT_Location << 4) | 0x02,
    /*  120 (10 - airborn; 0000 - above take off, <180, x0.25 speed multiplier) */ (RID_OP_Undeclared << 4) | (RID_HT_AboveTakeOff << 2) | (RID_DS_Less180 << 1) | RID_SM_025,
    /*  121 (heading direction North) */ 0,
    /*  122 (speed of 0m/s) */ 0,
    /*  123 (no vertical speed) */ 0,
    /*  124 - 127 (latitude in UA deg*10^7) */ 0, 0, 0, 0,
    /*  128 - 131 (longitude in UA deg*10^7) */ 0, 0, 0, 0,
    /*  132 - 133 (pressure altitude) */ 0, 0,
    /*  134 - 135 (geodetic altitude) */ 0, 0,
    /*  136 - 137 (height above takeoff) */ 0, 0,
    /*  138 (no vertical/horizontal acuracy) */ 0,
    /*  139 (no accuracy on baro altitude and speed) */ 0,
    /*  140 - 141 (time in 1/10s since last hour) */ 0, 0,
    /*  142 (time unknown) */ 0,
    /*  143 (reserved) */ 0,

    /*  144 (system message) */ (RID_MT_System << 4) | 0x02,
    /*  145 (operator location at takeoff) */ (RID_CT_Undeclared << 2) | (RID_OL_TakeOffLocation << 1),
    /*  146 - 149 (operator latitude in UA deg*10^7) */ 0, 0, 0, 0,
    /*  150 - 153 (operator longitude in UA deg*10^7) */ 0, 0, 0, 0,
    /*  154 - 155 (number of aircrafts in area for swarms (2 bytes) - just one) */ 1, 0,
    /*  156 (radius for formations in 10 * m) */ 1,
    /*  157 - 158 (area ceiling) */ 0, 0,
    /*  159 - 160 (area Floor) */ 0, 0,
    /*  161 (UA classification) */ (RID_UAC_Undefined << 4) | RID_AUCC_Undefined,
    /*  162 - 163 (operator geodetic Altitude) */ 0, 0,
    /*  164 - 167 (timestamp) */ 0, 0, 0, 0,
    /*  168 (reserved) */ 0,

    // RSN information
    /*  169 - 170 */ 0x30, 0x18,
    /*  171 - 172 */ 0x01, 0x00,
    /*  173 - 176 */ 0x00, 0x0f, 0xac, 0x02,
    /*  177 - 178 */ 0x02, 0x00,
    /*  179 - 186 */ 0x00, 0x0f, 0xac, 0x04, 0x00, 0x0f, 0xac, 0x04, /*Fix: changed 0x02(TKIP) to 0x04(CCMP) is default. WPA2 with TKIP not supported by many devices*/
    /*  187 - 188 */ 0x01, 0x00,
    /*  189 - 192 */ 0x00, 0x0f, 0xac, 0x02,
    /*  193 - 194 */ 0x00, 0x00,
    };

// generates random MAC
void randomMac()
{
    for (int i = 0; i < 6; i++)
        macAddr[i] = random(256);
}

// static void HandleWebUpdate()
// {
//   unsigned long now = millis();
//   wl_status_t status = WiFi.status();

//   if (status != laststatus && wifiMode == WIFI_STA) {
//     DBGLN("WiFi status %d", status);
//     switch(status) {
//       case WL_NO_SSID_AVAIL:
//       case WL_CONNECT_FAILED:
//       case WL_CONNECTION_LOST:
//         changeTime = now;
//         changeMode = WIFI_AP;
//         break;
//       case WL_DISCONNECTED: // try reconnection
//         changeTime = now;
//         break;
//       default:
//         break;
//     }
//     laststatus = status;
//   }
//   if (status != WL_CONNECTED && wifiMode == WIFI_STA && (now - changeTime) > 30000) {
//     changeTime = now;
//     changeMode = WIFI_AP;
//     DBGLN("Connection failed %d", status);
//   }
//   if (changeMode != wifiMode && changeMode != WIFI_OFF && (now - changeTime) > 500) {
//     switch(changeMode) {
//       case WIFI_AP:
//         DBGLN("Changing to AP mode");
//         WiFi.disconnect();
//         wifiMode = WIFI_AP;
//         WiFi.mode(wifiMode);
//         changeTime = now;
//         // WiFi.softAPConfig(ipAddress, ipAddress, netMsk);
//         // WiFi.softAP(wifi_ap_ssid, wifi_ap_password);
//         // startServices();
//         break;
//       case WIFI_STA:
//         DBGLN("Connecting to network '%s'", station_ssid);
//         wifiMode = WIFI_STA;
//         WiFi.mode(wifiMode);
//         WiFi.setHostname(wifi_hostname); // hostname must be set after the mode is set to STA
//         changeTime = now;
//         WiFi.begin(station_ssid, station_password);
//         // startServices();
//       default:
//         break;
//     }
//     changeMode = WIFI_OFF;
//   }

//   #if defined(PLATFORM_ESP8266)
//   // if (scanComplete)
//   // {
//   //   WiFi.mode(wifiMode);
//   //   scanComplete = false;
//   // }
//   #endif

//     // When in STA mode, a small delay reduces power use from 90mA to 30mA when idle
//     // In AP mode, it doesn't seem to make a measurable difference, but does not hurt
//     if (!Update.isRunning())
//       delay(1);
// }

static void wifiOff()
{
    rIDStarted = false;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
#if defined(PLATFORM_ESP8266)
    WiFi.forceSleepBegin();
#endif
}

static void startWiFi(unsigned long now)
{
    if (rIDStarted)
    {
        return;
    }

    DBGLN("Begin Remote ID");

    WiFi.persistent(false);
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
#if defined(PLATFORM_ESP8266)
    WiFi.setOutputPower(20.5);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
#elif defined(PLATFORM_ESP32)
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
#endif

    // create empty SSID
    for (int i = 0; i < 32; i++)
        emptySSID[i] = ' ';

    // for random generator
    randomSeed(
#if defined(PLATFORM_ESP8266)
        os_random()
#else
        esp_random()
#endif
        );

    // set packetSize
    packetSize = sizeof(beaconPacket);

    // generate random mac address
    randomMac();

    // start WiFi
    WiFi.mode(WIFI_OFF);
#if defined(PLATFORM_ESP8266)
    wifi_set_opmode(STATION_MODE);

    // set channel
    wifi_set_channel(wifi_channel);
#else
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	esp_wifi_init(&cfg);
	esp_wifi_set_storage(WIFI_STORAGE_RAM);

	// Init dummy AP to get WiFi hardware into a mode where we can send the actual
	// fake beacon frames.
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t ap_config = {
        .ap = {
            "esp32-remoteid",
            "dummypassword",
            0,
            wifi_channel,
            WIFI_AUTH_WPA2_PSK,
            1,
            4,
            60000
        }
    };
    

	esp_wifi_set_config(WIFI_IF_AP, &ap_config);
	esp_wifi_start();
	esp_wifi_set_ps(WIFI_PS_NONE);
#endif

    rIDStarted = true;
}

void HandleRemoteID()
{
    currentTime = millis();

    // send out REMOTE ID
    if (currentTime - attackTime > period)
    {
        attackTime = currentTime;

        // int timeout = 0;
        // while ((!(Radio.GetIrqStatus(SX1280_Radio_All) & SX1280_IRQ_TX_DONE) && !(Radio.GetIrqStatus(SX1280_Radio_All) & SX1280_IRQ_RX_DONE)) && timeout <= 10)
        // {
        //     delay(1);
        //     timeout += 1;
        // }
        // Radio.SetMode(SX1280_MODE_FS, SX1280_Radio_All);
        // Radio.TXnbISR();
        // delay(1);

        // #if defined(USE_MSP_WIFI) && defined(TARGET_RX)
        // // check is there is any data to write out
        // if (crsf.crsf2msp.FIFOout.peekSize() > 0)
        // {
        //   const uint16_t len = crsf.crsf2msp.FIFOout.popSize();
        //   uint8_t data[len];
        //   crsf.crsf2msp.FIFOout.popBytes(data, len);
        //   // wifi2tcp.write(data, len);
        // }

        // #endif

        // go to next channel
        // nextChannel();

        uint8_t ssidLen = strlen(ssid);

        // write MAC address into beacon frame
        memcpy(&beaconPacket[10], macAddr, 6);
        memcpy(&beaconPacket[16], macAddr, 6);

        // reset SSID
        memcpy(&beaconPacket[38], emptySSID, 32);

        // write new SSID into beacon frame
        memcpy_P(&beaconPacket[38], &ssid, ssidLen);

        // set channel for beacon frame
        beaconPacket[82] = wifi_channel;

        // set remote id packet counter
        beaconPacket[89] = counter;

        // send packet
        if (appendSpaces)
        {
            for (int k = 0; k < batch; k++)
            {
#if defined(PLATFORM_ESP8266)
                wifi_send_pkt_freedom(beaconPacket, packetSize, 0);
#else
                esp_wifi_80211_tx(WIFI_IF_AP, beaconPacket, packetSize, true);
#endif
                delay(1);
            }
        }

        // remove spaces
        else
        {

            uint16_t tmpPacketSize = (packetSize - 32) + ssidLen;                 // calc size
            uint8_t *tmpPacket = new uint8_t[tmpPacketSize];                      // create packet buffer
            memcpy(&tmpPacket[0], &beaconPacket[0], 38 + ssidLen);                // copy first half of packet into buffer
            tmpPacket[37] = ssidLen;                                              // update SSID length byte
            memcpy(&tmpPacket[38 + ssidLen], &beaconPacket[70], packetSize - 70); // copy second half of packet into buffer

            // send packet
            for (int k = 0; k < batch; k++)
            {
#if defined(PLATFORM_ESP8266)
                wifi_send_pkt_freedom(tmpPacket, tmpPacketSize, 0);
#else
                esp_wifi_80211_tx(WIFI_IF_AP, tmpPacket, tmpPacketSize, true);
#endif
                delay(1);
            }

            delete tmpPacket; // free memory of allocated buffer
        }

        counter += 1;
    }
    // check if there is any data to read in
    // const uint16_t bytesReady = wifi2tcp.bytesReady();
    // if (bytesReady > 0)
    // {
    //   uint8_t data[bytesReady];
    // wifi2tcp.read(data);
    //   crsf.msp2crsf.parse(data, bytesReady);
    // }

    // wifi2tcp.handle();
    // delay(100);
}

static int start()
{
    // ipAddress.fromString(wifi_ap_address);
    // return firmwareOptions.wifi_auto_on_interval;
    return DURATION_IMMEDIATELY;
}

static int event()
{
    if (!rIDStarted)
    {
        startWiFi(millis());
        return DURATION_IMMEDIATELY;
    }
    return DURATION_IGNORE;
}

static int timeout()
{
    if (rIDStarted)
    {
        // HandleWebUpdate();
        HandleRemoteID();
        return DURATION_IMMEDIATELY;
    }

    return DURATION_NEVER;
}

device_t REMOTEID_device = {
    .initialize = wifiOff,
    .start = start,
    .event = event,
    .timeout = timeout};

#endif

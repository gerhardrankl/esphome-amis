#include <AsyncTCP.h>
#include "esphome/core/log.h"

/*
  Smartmeter Emulator TCP für Fronius Gen24. Stark vereinfacht auf das tatsächliche Fronius-Polling.
  Es wird nur der Wert AC-Power korrekt behandelt, der Rest ist Fake!
  Reicht aber für korrekte dyn. Einspeisebegrenzung.
*/
/// uint16_t holdregs[]= {
///*00*/  // 0,
///*01*/   21365, 28243,   // 0x5375  0x6e53                      // "SunsS"
///*03*/   1,
///*04*/   65,                                                    // 65 Register
//// hex   46  72  6f  6E  69  75  73
///*05*/   70,114,111,110,105,117,115,0,0,0,0,0,0,0,0,0,          // Manufacturer "Fronius"
///*21*/   83,109,97,114,116,32,77,101,116,101,114,32,54,51,65,0, // Device Model "Smart Meter 63A"
///*37*/   0,0,0,0,0,0,0,0,                                       // Options N/A
///*45*/   0,0,0,0,0,0,0,0,                                       // SW-Vers. N/A
///*53*/   48,48,48,48,48,48,49,49,0,0,0,0,0,0,0,0,               // Serial Number: 00000001 (49,54,50,50,48,49,49,56   muss unique sein per Symo
///*69*/   240,                                                   // Modbus TCP Address: 240
///*70*/   213,                                                   // 3-ph + Nulleiter,
///*71*/   124,                                                   // Länge Datenregister : 40195 ist letztes R
///*72*/   0,0,0,0,0,0,0,0,0,0,     // AC Curr 1,2,3              //
///*82*/   0,0,0,0,0,0,0,0,0,0,     //
///*92*/   0,0,0,0,0,0,0,0,0,0,     // 96: Frequ 50.0 Hz 98:P ges, 100: P Phase 1
///*102*/  0,0,0,0,0,0,0,0,0,0,     // P Phase2 P Phase3
///*112*/  0,0,0,0,0,0,0,0,0,0,
///*122*/  0,0,0,0,0,0,0,0,0,0,     // 130:  Total Lieferung, 2.8.0
///*132*/  0,0,0,0,0,0,0,0,0,0,     // 138:  Total Bezug, 1.8.0
///*142*/  0,0,0,0,0,0,0,0,0,0,
///*152*/  0,0,0,0,0,0,0,0,0,0,
///*162*/  0,0,0,0,0,0,0,0,0,0,
///*172*/  0,0,0,0,0,0,0,0,0,0,
///*182*/  0,0,0,0,0,0,0,0,0,0,
///*192*/  0,0,0,0,
///*196*/  65535,                                             // End Mark,
///*197*/  0};                                                // nächster Block 0

namespace esphome
{
    namespace amis
    {

        // Auf Big-Endian konvertierte Register-Var
        const uint16_t PROGMEM BE_holdregs[] = {                                                   // Zählerkennung:
            /*00*/ 0x7553, 0x536e, 0x0100, 0x4100, 0x4100, 0x6d00, 0x6900, 0x7300, 0x2000, 0x5200, // SunSA m i s   R
            /*10*/ 0x6500, 0x6100, 0x6400, 0x6500, 0x7200, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // e a d e r
            /*20*/ 0x5300, 0x6d00, 0x6100, 0x7200, 0x7400, 0x2000, 0x4d00, 0x6500, 0x7400, 0x6500,
            /*30*/ 0x7200, 0x2000, 0x3600, 0x3300, 0x4100, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            /*40*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
            /*50*/ 0x0000, 0x0000, 0x3500, 0x3400, 0x3400, 0x3300, 0x3300, 0x3200, 0x3200, 0x3100, // Ser-Nr
            /*60*/ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xf000, 0xd500,
            /*70*/ 0x7c00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

        uint8_t mBuffer[182]; // für die Zähler-Daten
        uint8_t mHeader[12];  // Header Rx + Tx
        typedef union
        { // für Float BigEndian wandeln
            unsigned int intval;
            float value;
            uint8_t bytes[sizeof(float)];
        } UFB;
        UFB floatvar;
        bool buffered;
        AsyncServer *meter_server;

        static const char *TAG = "amis_meter";

        uint32_t energy_a_positive = 0;
        uint32_t energy_a_negative = 0;
        uint32_t instantaneous_power_a_positive = 0;
        uint32_t instantaneous_power_a_negative = 0;

        /* clients events */
        static void handleError(void *arg, AsyncClient *client, int8_t error)
        {
            ESP_LOGE(TAG, "[Fronius] connection error %s from client %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
        }

        static void handleDisconnect(void *arg, AsyncClient *client)
        {
            ESP_LOGI(TAG, "[Fronius] client %s disconnected \n", client->remoteIP().toString().c_str());
        }

        static void handleTimeOut(void *arg, AsyncClient *client, uint32_t time)
        {
            ESP_LOGW(TAG, "[Fronius] client ACK timeout ip: %s \n", client->remoteIP().toString().c_str());
        }

        static void handleData(void *arg, AsyncClient *client, void *data, size_t len);

        static void handleNewClient(void *arg, AsyncClient *client)
        {
            ESP_LOGI(TAG, "[Fronius] new client has been connected to server, ip: %s\n", client->remoteIP().toString().c_str());

            // register events
            client->onData(&handleData, NULL);
            client->onError(&handleError, NULL);
            client->onDisconnect(&handleDisconnect, NULL);
            client->onTimeout(&handleTimeOut, NULL);
        }

        static bool isDataAvailable()
        {
            return energy_a_positive > 0 || energy_a_negative > 0;
        }

        static void handleData(void *arg, AsyncClient *client, void *data, size_t len)
        {
            ESP_LOGD(TAG, "[Fronius] Poll IP:%s\n", client->remoteIP().toString().c_str());
            if (!isDataAvailable())
                return; // erst beantworten wenn Zählerdaten vorhanden
            memcpy(mHeader, data, len < sizeof(mHeader) ? len : sizeof(mHeader));
            uint16_t reg_idx = (mHeader[8] << 8) | mHeader[9];
            uint16_t reg_len = (mHeader[10] << 8) | mHeader[11];
            ESP_LOGD(TAG, "[Fronius] RegIdx:%d RegLen:%02d Dta:", reg_idx, reg_len);
            //	for (unsigned i=0; i< len;i++) eprintf("%02x ",mHeader[i]);	eprintf("\n");
            if ((reg_idx > 40197 || reg_idx < 40000) || mHeader[7] != 3)
            { // Anfrage außerhalb Register
                ESP_LOGE(TAG, "[Fronius] Err %u\n", reg_idx);
                mHeader[5] = 3;    // Länge
                mHeader[7] = 0x83; // FC MSB gesetzt
                mHeader[8] = 2;    // Fehlercode "nicht verfügbar"
                len = 9;
                buffered = false;
                if (client->space() > len && client->canSend())
                {
                    client->add((char *)mHeader, len);
                    client->send();
                }
                return;
            }
            else
            {                             // gültige Anfrage:
                mHeader[8] = reg_len * 2; // Header aufbereiten
                reg_idx -= 40000;
                len = reg_len * 2 + 3;
                mHeader[4] = len >> 8;   // Länge Antwort
                mHeader[5] = len & 0xff; // Länge Antwort bis Ende
                len = reg_len * 2 + 9;
                if (!buffered)
                {
                    memset(mBuffer, 0, sizeof(mBuffer));
                    mBuffer[48] = 0x42; // 40096: 50 Hz
                    mBuffer[49] = 0x48;

                    mBuffer[16] = 0x43; // 40080: 230 V
                    mBuffer[17] = 0x66;
                    mBuffer[20] = 0x43; // 40082: 230 V
                    mBuffer[21] = 0x66;
                    mBuffer[24] = 0x43; // 40084: 230 V
                    mBuffer[25] = 0x66;
                    mBuffer[28] = 0x43; // 40086: 230 V
                    mBuffer[29] = 0x66;
                    buffered = true;
                }

                switch (reg_idx)
                {        // diese Adressen werden von Symo Geräten abgefragt
                case 71: // Anfragen an andere Adressen liefern Müll!
                    // die Register 40072..40128 werden im sec-Takt gelesen, reg_len==58
                    signed int xsaldo;
                    signed int xsaldoPhaseAB;
                    signed int xsaldoPhaseC;
                    xsaldo = ((int32_t)instantaneous_power_a_positive - (int32_t)instantaneous_power_a_negative); // 1.7.0 - 2.7.0 = Power
                    floatvar.value = (float)(xsaldo);
                    mBuffer[52] = (floatvar.bytes[3]); // Power  Big Endian korrekt kopieren auf P gesamt 40098
                    mBuffer[53] = (floatvar.bytes[2]);
                    mBuffer[54] = (floatvar.bytes[1]);
                    mBuffer[55] = (floatvar.bytes[0]);

                    xsaldoPhaseAB = xsaldo / 3;
                    floatvar.value = (float)(xsaldoPhaseAB);
                    mBuffer[56] = (floatvar.bytes[3]); // Power Phase A  Big Endian korrekt kopieren auf P Phase A 40100
                    mBuffer[57] = (floatvar.bytes[2]);
                    mBuffer[58] = (floatvar.bytes[1]);
                    mBuffer[59] = (floatvar.bytes[0]);
                    *((int32_t*)&mBuffer[60]) = *((int32_t*)&mBuffer[56]); // Power Phase B 40102 equals Phase A
                    
                    xsaldoPhaseC = xsaldo - xsaldoPhaseAB - xsaldoPhaseAB;
                    floatvar.value = (float)(xsaldoPhaseC);
                    mBuffer[64] = (floatvar.bytes[3]); // Power Phase C  Big Endian korrekt kopieren auf P Phase C 40104
                    mBuffer[65] = (floatvar.bytes[2]);
                    mBuffer[66] = (floatvar.bytes[1]);
                    mBuffer[67] = (floatvar.bytes[0]);
                    break;

                // die Register 40130..40160 werden jede Minute gelesen, reg_len==32
                case 129:
                    floatvar.value = ((float)(energy_a_negative)); // 2.8.0
                    // eprintf("E %f %u %x %x %x %x\n",floatvar.value,a_result[1],floatvar.bytes[3],floatvar.bytes[2],floatvar.bytes[1],floatvar.bytes[0]);
                    mBuffer[116] = floatvar.bytes[3]; // Energy  Big Endian korrekt kopieren auf export 40130
                    mBuffer[117] = floatvar.bytes[2];
                    mBuffer[118] = floatvar.bytes[1];
                    mBuffer[119] = (floatvar.bytes[0]);

                    floatvar.value = ((float)(energy_a_positive)); // 1.8.0
                    mBuffer[132] = (floatvar.bytes[3]);            // Energy  Big Endian korrekt kopieren auf import  40138
                    mBuffer[133] = (floatvar.bytes[2]);
                    mBuffer[134] = (floatvar.bytes[1]);
                    mBuffer[135] = (floatvar.bytes[0]);
                    break;

                case 195: // end-block, Abfrage nur 1x bei Start, reg_len==2
                    mBuffer[0] = 0xff;
                    mBuffer[1] = 0xff;
                    reg_idx = 193; // so tun als ob...
                    buffered = false;
                }
            }

            // reply to client
            if (client->space() > len && client->canSend())
            {
                int buffIdx;
                client->add((char *)mHeader, 9); // MBAP Header
                if (reg_idx < 71)
                { // Allg. Daten aus ROM
                    memcpy_P(mBuffer, &BE_holdregs[reg_idx], reg_len * 2);
                    buffIdx = 0;
                    buffered = false;
                }
                else
                { // elektr. Zählerdaten
                    if (reg_idx == 193)
                    { // Meter-Flags, immer 0, Daten vom mBuffer-Anfang nehmen; reg_len==2
                        buffIdx = 0;
                    }
                    else
                        buffIdx = (reg_idx - 71) * 2; // Addr. gewünschter Daten
                }
                client->add((char *)(&mBuffer[buffIdx]), len - 9);
                client->send();
            }
        }

        void meter_init()
        {
            ESP_LOGD(TAG, "meter_init()");
            meter_server = new AsyncServer(502); // start listening on tcp port 502
            meter_server->onClient(&handleNewClient, meter_server);
            meter_server->begin();
        }

    }
}

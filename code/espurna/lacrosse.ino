/*

Lacrosse RFMxx MODULE

Copyright (C) 2018 by Sven Kopetzki
FHEM

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

//#if LACROSSE_SUPPORT
//#define RFM1_SS          15

//#include "libs/RFMxx.h"
//#include "libs/Logger.h"
//#include "libs/TypedQueue.h"
//#include "libs/LaCrosse.h"

// -----------------------------------------------------------------------------
// Locals
// -----------------------------------------------------------------------------

Logger logger;
RFMxx* rfms[5];
RFMxx rfm1(13, 12, 14, RFM1_SS);
byte TOGGLE_MODE_R1          = 3;        // <n>m       bits 1: 17.241 kbps, 2 : 9.579 kbps, 4 : 8.842 kbps, 8 : 20.000 kbps (for RFM #1)
uint16_t TOGGLE_INTERVAL_R1  = 0;        // <n>t       0=no toggle, else interval in seconds (for RFM #1)
unsigned long INITIAL_FREQ   = 868300;   // <n>f       initial frequency in kHz (5 kHz steps, 860480 ... 879515)
unsigned long DATA_RATE_R1   = 17241ul;  // <n>r       use one of the possible data rates (for RFM #1)
byte PASS_PAYLOAD            = 0;        // <n>p       transmitted the payload on the serial port 1: all, 2: only undecoded data
bool DEBUG                   = 0;        // <n>d       set to 1 to see debug messages
bool USE_SERIAL              = 1;        //            0=do not send sensor data on the serial

struct _lnode_t {
    unsigned long count = 0;
    unsigned char lastPacketID = 0;
};

_lnode_t _lacrosse_node_info[64];
unsigned char _lacrosse_node_count;
unsigned long _lacrosse_packet_count;

//Influxdb influx(INFLUXDB_HOST, INFLUXDB_PORT);
Influxdb influx(INFLUXDB_HOST);

void _lacrosseProcess(LaCrosse::Frame * data) {

    // Count seen nodes and packets
    if (_lacrosse_node_info[data->ID].count == 0) ++_lacrosse_node_count;
    ++_lacrosse_packet_count;

    _lacrosse_node_info[data->ID].count = _lacrosse_node_info[data->ID].count + 1;

    // Send info to websocket clients
    /*
    {
        char buffer[200];
        snprintf_P(
            buffer,
            sizeof(buffer) - 1,
            PSTR("{\"nodeCount\": %d, \"packetCount\": %lu, \"packet\": {\"senderID\": %u, \"targetID\": %u, \"packetID\": %u, \"key\": \"%s\", \"value\": \"%s\", \"rssi\": %d, \"duplicates\": %d, \"missing\": %d}}"),
            _lacrosse_node_count, _lacrosse_packet_count,
            data->ID, 0, 0, 0, data->Temperature, data->Humidity, 0, 0);
        wsSend(buffer);
    }
    */

    // Try to find a matching mapping
    for (unsigned int i=0; i<RFM69_MAX_TOPICS; i++) {
        unsigned char node = getSetting("node", i, 0).toInt();
        if (0 == node) break;
        if (node == data->ID) {
            char _topic[50];
            snprintf(_topic, sizeof(_topic), "%s/%s/Temp", getSetting("key", i, "").c_str(), getSetting("topic", i, "").c_str());
            mqttSendRaw((char *) _topic, (char *) String(data->Temperature).c_str());
            return;
        }
    }


    // Mapping not found, use default topic
    String topic = getSetting("rfm69Topic", RFM69_DEFAULT_TOPIC);
    if (topic.length() > 0) {
        topic.replace("{node}", String(data->ID));
        topic.replace("{key}", "Temp");
        mqttSendRaw((char *) topic.c_str(), (char *) String(data->Temperature).c_str());
    }
}


// -----------------------------------------------------------------------------
// Radio
// -----------------------------------------------------------------------------
void Dispatch(String data, String raw="") {
  //if(USE_SERIAL) {
    //Serial.println(data);
  //}

  if (raw.length() > 0) {
    raw = " [" + raw + "]";
  }

  if (data.startsWith("\n")) {
    data = data.substring(1);
  }
  //logger.logData(data + raw);
  DEBUG_MSG("[Lacrosse] %s\n", data.c_str());
}

void SetDataRate(RFMxx *rfm, unsigned long dataRate) {
  if(rfm->GetDataRate() != 20000 && dataRate == 20000) {
    rfm->InitializeEC3000();
    rfm->EnableReceiver(true);
  }
  else if(rfm->GetDataRate() == 20000 && dataRate != 20000) {
    rfm->InitializeLaCrosse();
    rfm->EnableReceiver(true);
  }


  rfm->SetDataRate(dataRate);
}

bool HandleReceivedData(RFMxx *rfm) {
  bool result = false;

  rfm->EnableReceiver(false);

  byte payload[PAYLOADSIZE];
  rfm->GetPayload(payload);

  rfm->EnableReceiver(true);

  if (PASS_PAYLOAD == 1) {
    for (int i = 0; i < PAYLOADSIZE; i++) {
      Serial.print(payload[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  else {
    if (DEBUG) {
      Serial.print("\r\nEnd receiving, HEX raw data: ");
      for (int i = 0; i < 16; i++) {
        Serial.print(payload[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      String raw = rfm->GetRadioName() + ": ";
      for (int i = 0; i < 5; i++) {
        String bt = String(payload[i], HEX);
        bt = bt.length() == 1 ? ("0" + bt) : bt;
        raw += bt;
        raw += " ";
      }
      raw.toUpperCase();
      logger.println(raw);
    }

    //String data = "";
    byte frameLength = 16;
    struct LaCrosse::Frame lcf;
    //void LaCrosse::Decode2LaCrosseFrame(byte *data, struct Frame *lcframe) {

    // Try LaCrosse like TX29DTH
    if (lcf.Text.length() == 0 && LaCrosse::IsValidDataRate(rfm->GetDataRate())) {
      //data = LaCrosse::GetTextDataString(payload);
      LaCrosse::Decode2LaCrosseFrame(payload, &lcf);
      frameLength = LaCrosse::FRAME_LENGTH;
    }

    if (lcf.Text.length() > 0) {
      result = true;

      String raw = "";
      for (int i = 0; i < frameLength; i++) {
        String bt = String(payload[i], HEX);
        bt = bt.length() == 1 ? ("0" + bt) : bt;
        raw += bt;
        raw += i+1 < frameLength ? " " : "";
      }
      raw.toUpperCase();

      _lacrosseProcess(&lcf);

      Dispatch(lcf.Text, raw);

      //if(lcf.ID == 12){
        //idbSend("Humidity",    lcf.ID,  String(lcf.Humidity).c_str());
        //idbSend("Temperature", lcf.ID,  String(lcf.Temperature).c_str());
        //idbSend("NewBat", lcf.ID,  lcf.NewBatteryFlag ? "1" : "0");
        //idbSend("WeakBat", lcf.ID,  lcf.WeakBatteryFlag ? "1" : "0");

        //bool idbSend(const char * topic, unsigned char id, const char * payload)//

        // Prepare Influx data sample
        InfluxData row("temperature");
        char _id[20];
        String _known = "0";
        snprintf(_id, sizeof(_id), "%d", lcf.ID);
        row.addValue("temp", lcf.Temperature);
        row.addValue("hum", lcf.Humidity);
        row.addValue("newbat", lcf.NewBatteryFlag);
        row.addValue("weakbat", lcf.WeakBatteryFlag);

        // Try to find a matching mapping
        // Replace tag "ID" from air packet ID by topic field.
        for (unsigned int i=0; i<RFM69_MAX_TOPICS; i++) {
            unsigned char node = getSetting("node", i, 0).toInt();
            if (0 == node) break;
            if (node == lcf.ID) {
                snprintf(_id, sizeof(_id), "%s", getSetting("topic", i, "").c_str());
                _known = "1";
            }
        }

        row.addTag("MATCH", _known);
        row.addTag("ID", _id);
        influx.write(row);

      //}
    }
  }



  return result;
}



void _lacrosseLoop() {
  //byte receivedPackets = 0;
  if (rfm1.IsConnected()) {
    rfm1.Receive();
    if (rfm1.PayloadIsReady()) {
      if (HandleReceivedData(&rfm1)) {
        //receivedPackets++;
      }
    }
  }
  //return receivedPackets;
}

void _lacrosseClear() {
    //for(unsigned int i=0; i<255; i++) {
    //    _rfm69_node_info[i].duplicates = 0;
    //    _rfm69_node_info[i].missing = 0;
    //}
    _lacrosse_node_count = 0;
    _lacrosse_packet_count = 0;
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void lacrosseSetup() {
  _lacrosseClear();

  //Influxdb
  // set the target database
  influx.setDb(INFLUXDB_DATABASE);

  // TODO: Cleanup
  rfms[0] = &rfm1;
  rfm1.Begin();
  rfm1.ToggleMode = TOGGLE_MODE_R1; rfm1.ToggleInterval = TOGGLE_INTERVAL_R1;

  logger.println("Searching RFMs and Sensors");

  if (rfm1.IsConnected()) {
    rfm1.InitializeLaCrosse();
    rfm1.SetFrequency(INITIAL_FREQ);
    SetDataRate(&rfm1, DATA_RATE_R1);
    rfm1.EnableReceiver(true);
    logger.print("Radio #1 found: ");
    logger.println(rfm1.GetRadioName());
  }

  /*
  #if WEB_SUPPORT
      wsOnSendRegister(_rfm69WebSocketOnSend);
      wsOnReceiveRegister(_rfm69WebSocketOnReceive);
      wsOnActionRegister(_rfm69WebSocketOnAction);
  #endif
  */

    // Register loop
    espurnaRegisterLoop(_lacrosseLoop);

}

//#endif // RFM69_SUPPORT

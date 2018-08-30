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

//Influxdb influx(INFLUXDB_HOST, INFLUXDB_PORT);
Influxdb influx(INFLUXDB_HOST);

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

      Dispatch(lcf.Text, raw);

      //if(lcf.ID == 12){
        //idbSend("Humidity",    lcf.ID,  String(lcf.Humidity).c_str());
        //idbSend("Temperature", lcf.ID,  String(lcf.Temperature).c_str());
        //idbSend("NewBat", lcf.ID,  lcf.NewBatteryFlag ? "1" : "0");
        //idbSend("WeakBat", lcf.ID,  lcf.WeakBatteryFlag ? "1" : "0");

        //bool idbSend(const char * topic, unsigned char id, const char * payload)//

        InfluxData row("temperature");
        char _id[4];
        snprintf(_id, sizeof(_id), "%d", lcf.ID);
        row.addTag("ID", _id);
        row.addValue("temp", lcf.Temperature);
        row.addValue("hum", lcf.Humidity);
        row.addValue("newbat", lcf.NewBatteryFlag);
        row.addValue("weakbat", lcf.WeakBatteryFlag);
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


// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void lacrosseSetup() {

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

    // Register loop
    espurnaRegisterLoop(_lacrosseLoop);

}

//#endif // RFM69_SUPPORT

#ifndef _LACROSSE_h
#define _LACROSSE_h

#include "Arduino.h"
#include "SensorBase.h"


class LaCrosse : public SensorBase {
public:
  struct Frame {
    byte  Header;
    byte  ID;
    bool  NewBatteryFlag;
    bool  Bit12;
    float Temperature;
    bool  WeakBatteryFlag;
    byte  Humidity;
    byte  CRC;
    bool  IsValid;
    String Text = "";
  };

  struct LaCrosseFrame{
    struct Frame;
    String Text;
  };

  static const byte FRAME_LENGTH = 5;
  static bool USE_OLD_ID_CALCULATION;
  static byte CalculateCRC(byte data[]);
  static void EncodeFrame(struct LaCrosse::Frame *frame, byte bytes[5]);
  static void DecodeFrame(byte *bytes, struct LaCrosse::Frame *frame);
  static String AnalyzeFrame(byte *data);
  static bool TryHandleData(byte *data);
  static String GetFhemDataString(byte *data);
  static bool IsValidDataRate(unsigned long dataRate);
  static String BuildFhemDataString(struct LaCrosse::Frame *frame);
  static String GetTextDataString(byte *data);
  static void Decode2LaCrosseFrame(byte *data, struct LaCrosse::Frame *lcframe);
  static String BuildTextDataString(struct LaCrosse::Frame *frame);

protected:


};

#endif

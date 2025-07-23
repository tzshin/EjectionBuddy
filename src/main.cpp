#include <Arduino.h>
#include <AlfredoCRSF.h>

constexpr int PIN_RXI_RX = 4;
constexpr int PIN_RXI_TX = 5;
constexpr int PIN_RXO_RX = 6;
constexpr int PIN_RXO_TX = 7;
constexpr int PIN_DD_SENSE = 10;
constexpr int PIN_LED = 42;
constexpr int PIN_CAM_1 = 1;
HardwareSerial serial_rxi(1);
HardwareSerial serial_rxo(2);

AlfredoCRSF crsf_rxi;
AlfredoCRSF crsf_rxo;

bool is_detatched = false;

enum class MissionPhase
{
  STANDBY,
  PRE_EJECTION,
  EJECTION,
  POST_EJECTION,
};

MissionPhase mission_phase = MissionPhase::PRE_EJECTION;
unsigned long ejection_start_ts = 0;
const unsigned long ejection_duration = 3000; // In milliseconds

void update_ejection_phase(bool is_preflight)
{
  switch (mission_phase)
  {
  case MissionPhase::STANDBY:
    if (is_preflight)
    {
      mission_phase = MissionPhase::PRE_EJECTION;
    }
    break;
  case MissionPhase::PRE_EJECTION:
    if (!is_preflight)
    {
      mission_phase = MissionPhase::STANDBY;
    }
    if (is_detatched)
    {
      mission_phase = MissionPhase::EJECTION;
      ejection_start_ts = millis();
    }
    break;
  case MissionPhase::EJECTION:
    if (millis() - ejection_start_ts >= ejection_duration)
    {
      mission_phase = MissionPhase::POST_EJECTION;
    }
    break;
  case MissionPhase::POST_EJECTION:
    break;
  }
}

void send_raw_channels()
{
  const crsf_channels_t *channels_raw = crsf_rxi.getChannelsPacked();

  crsf_channels_t channels = *channels_raw;

  crsf_rxo.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &channels, sizeof(channels));
}

void send_auto_channels(MissionPhase mission_phase)
{
  crsf_channels_t channels = {0};
  channels.ch0 = CRSF_CHANNEL_VALUE_MID;
  channels.ch1 = CRSF_CHANNEL_VALUE_MID;
  channels.ch2 = CRSF_CHANNEL_VALUE_MID;
  channels.ch3 = CRSF_CHANNEL_VALUE_MID;
  channels.ch4 = CRSF_CHANNEL_VALUE_2000;

  switch (mission_phase)
  {
  case MissionPhase::STANDBY:
    channels.ch5 = CRSF_CHANNEL_VALUE_1000;
    channels.ch6 = CRSF_CHANNEL_VALUE_2000;
    channels.ch7 = CRSF_CHANNEL_VALUE_1000;
    break;
  case MissionPhase::PRE_EJECTION:
    channels.ch5 = CRSF_CHANNEL_VALUE_1000;
    channels.ch6 = CRSF_CHANNEL_VALUE_2000;
    channels.ch7 = CRSF_CHANNEL_VALUE_2000;
    break;
  case MissionPhase::EJECTION:
    channels.ch5 = CRSF_CHANNEL_VALUE_2000;
    channels.ch6 = CRSF_CHANNEL_VALUE_1000;
    channels.ch7 = CRSF_CHANNEL_VALUE_2000;
    break;
  case MissionPhase::POST_EJECTION:
    channels.ch5 = CRSF_CHANNEL_VALUE_1000;
    channels.ch6 = CRSF_CHANNEL_VALUE_1000;
    channels.ch7 = CRSF_CHANNEL_VALUE_2000;
    break;
  }

  channels.ch8 = CRSF_CHANNEL_VALUE_1000;
  channels.ch9 = CRSF_CHANNEL_VALUE_1000;
  channels.ch10 = CRSF_CHANNEL_VALUE_1000;
  channels.ch11 = CRSF_CHANNEL_VALUE_1000;
  channels.ch12 = CRSF_CHANNEL_VALUE_1000;
  channels.ch13 = CRSF_CHANNEL_VALUE_1000;
  channels.ch14 = CRSF_CHANNEL_VALUE_1000;
  channels.ch15 = CRSF_CHANNEL_VALUE_1000;

  crsf_rxo.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &channels, sizeof(channels));
}

void setup()
{
  Serial.begin(115200);

  serial_rxi.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RXI_RX, PIN_RXI_TX);
  delay(100);
  serial_rxo.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RXO_RX, PIN_RXO_TX);
  delay(100);
  while (!serial_rxi)
    ;
  delay(100);
  while (!serial_rxo)
    ;
  delay(100);
  crsf_rxi.begin(serial_rxi);
  delay(100);
  crsf_rxo.begin(serial_rxo);

  pinMode(PIN_DD_SENSE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DD_SENSE), []()
                  { is_detatched = true; }, FALLING);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_CAM_1, OUTPUT);
  digitalWrite(PIN_CAM_1, LOW);
}

void loop()
{
  crsf_rxi.update();
  bool is_bypassed = crsf_rxi.getChannelsPacked()->ch8 > CRSF_CHANNEL_VALUE_MID ? true : false;
  bool is_preflight = crsf_rxi.getChannelsPacked()->ch10 > CRSF_CHANNEL_VALUE_MID ? true : false;

  update_ejection_phase(is_preflight);

  if (crsf_rxi.isLinkUp())
  {
    digitalWrite(PIN_LED, HIGH);
    is_bypassed ? send_raw_channels() : send_auto_channels(mission_phase);
  }
  else
  {
    digitalWrite(PIN_LED, LOW);
    send_auto_channels(mission_phase);
  }

  digitalWrite(PIN_CAM_1, mission_phase != MissionPhase::STANDBY ? HIGH : LOW);

  while (serial_rxo.available() > 0)
  {
    uint8_t b = serial_rxo.read();
    serial_rxi.write(b);
  }
}
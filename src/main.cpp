#include <Arduino.h>
#include <AlfredoCRSF.h>

constexpr int PIN_RXI_RX = 4;
constexpr int PIN_RXI_TX = 5;
constexpr int PIN_RXO_RX = 6;
constexpr int PIN_RXO_TX = 7;
constexpr int PIN_DD_SENSE = 10;
constexpr int PIN_LED = 42;
HardwareSerial serial_rxi(1);
HardwareSerial serial_rxo(2);

AlfredoCRSF crsf_rxi;
AlfredoCRSF crsf_rxo;

bool is_detatched = false;

unsigned long staging_start_ts = 0;
unsigned long staging_duration = 3000; // In milliseconds

enum class KillState
{
  FORCE_KILL,
  FORCE_UNKILL,
  PASSTHROUGH
};

void send_raw_channels(KillState kill_state)
{
  const crsf_channels_t *channels_raw = crsf_rxi.getChannelsPacked();

  crsf_channels_t channels = *channels_raw;

  switch (kill_state)
  {
  case KillState::FORCE_KILL:
    channels.ch6 = CRSF_CHANNEL_VALUE_2000;
    break;
  case KillState::FORCE_UNKILL:
    channels.ch6 = CRSF_CHANNEL_VALUE_1000;
    break;
  case KillState::PASSTHROUGH:
    break;
  }

  crsf_rxo.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, &channels, sizeof(channels));
}

void send_auto_channels(KillState kill_state)
{
  crsf_channels_t channels = {0};
  channels.ch0 = CRSF_CHANNEL_VALUE_MID;
  channels.ch1 = CRSF_CHANNEL_VALUE_MID;
  channels.ch2 = CRSF_CHANNEL_VALUE_MID;
  channels.ch3 = CRSF_CHANNEL_VALUE_MID;
  channels.ch4 = CRSF_CHANNEL_VALUE_2000;
  channels.ch5 = CRSF_CHANNEL_VALUE_1000;

  switch (kill_state)
  {
  case KillState::FORCE_KILL:
    channels.ch6 = CRSF_CHANNEL_VALUE_2000;
    break;
  case KillState::FORCE_UNKILL:
    channels.ch6 = CRSF_CHANNEL_VALUE_1000;
    break;
  case KillState::PASSTHROUGH:
    channels.ch6 = crsf_rxi.getChannelsPacked()->ch6;
    break;
  }

  channels.ch7 = CRSF_CHANNEL_VALUE_2000;
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
  serial_rxo.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RXO_RX, PIN_RXO_TX);
  delay(1000);
  crsf_rxi.begin(serial_rxi);
  crsf_rxo.begin(serial_rxo);

  pinMode(PIN_DD_SENSE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DD_SENSE), []()
                  { is_detatched = true; }, FALLING);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}

void loop()
{
  crsf_rxi.update();
  bool is_bypassed = crsf_rxi.getChannelsPacked()->ch8 > CRSF_CHANNEL_VALUE_MID ? true : false;

  if (crsf_rxi.isLinkUp())
  {
    digitalWrite(PIN_LED, HIGH);
    if (is_bypassed)
    {
      send_raw_channels(KillState::PASSTHROUGH);
    }
    else
    {
      if (is_detatched)
      {
        send_raw_channels(KillState::FORCE_UNKILL);
      }
      else
      {
        send_raw_channels(KillState::FORCE_KILL);
      }
    }
  }
  else
  {
    digitalWrite(PIN_LED, LOW);
    if (is_detatched)
    {
      send_auto_channels(KillState::FORCE_UNKILL);
    }
    else
    {
      send_auto_channels(KillState::FORCE_KILL);
    }
  }

  if (serial_rxo.available())
  {
    uint8_t buf[64];
    int len = serial_rxo.readBytes(buf, sizeof(buf));
    serial_rxi.write(buf, len);
  }
}
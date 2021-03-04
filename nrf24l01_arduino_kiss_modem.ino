#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

enum KissMarker {
  Fend = 0xc0,
  Fesc = 0xdb,
  Tfend = 0xdc,
  Tfesc = 0xdd
};

enum KissState {
  Void = 0,
  GetCmd,
  GetData,
  GetP,
  GetSlotTime,
  Escape
};

enum KissCmd {
  Data = 0x00,
  P = 0x02,
  SlotTime = 0x03,
  NoCmd = 0x80
};

#define RADIO_PIN_CE 10
#define RADIO_PIN_CS 11
#define RADIO_ADDRESS "00001"
#define RADIO_CHANNEL 0
#define RADIO_SPEED RF24_250KBPS
#define RADIO_CRC RF24_CRC_DISABLED
#define RADIO_PA RF24_PA_MAX
#define RADIO_DEFAULT_P 255
#define RADIO_DEFAULT_SLOT_TIME 0

#define LOOP_SLEEP_MS 10
#define KISS_BUFFER_SIZE 256

RF24 radio_(RADIO_PIN_CE, RADIO_PIN_CS);
const byte address_[6] = "00001";

KissState kissState_;
KissCmd kissCmd_;
uint8_t kissBuffer_[KISS_BUFFER_SIZE];
int kissBufferPosition_ = 0;

// not used
byte csmaP_ = RADIO_DEFAULT_P;
long csmaSlotTime_ = RADIO_DEFAULT_SLOT_TIME;
long csmaSlotTimePrev_ = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  radio_.begin();
  radio_.openWritingPipe(address_);
  radio_.setPALevel(RADIO_PA);
  radio_.setDataRate(RADIO_SPEED);
  radio_.setCRCLength(RF24_CRC_8);
  radio_.setChannel(RADIO_CHANNEL);
  radio_.startListening();
  
  radio_.openWritingPipe(address_);
  radio_.openReadingPipe(1, address_);
}

void loop() { 
  if (radio_.available()) {
    onRadioDataAvailable();
  }
  else {
    long currentTime = millis();
    if (currentTime > csmaSlotTimePrev_ + csmaSlotTime_ && random(0, 255) < csmaP_) {
      if (Serial.available()) {
        onSerialDataAvailable();
      }
      csmaSlotTimePrev_ = currentTime;
    }
  }
  delay(LOOP_SLEEP_MS);
}

void kissResetState()
{
  kissCmd_ = KissCmd::NoCmd;
  kissState_ = KissState::Void;
}

void onRadioDataAvailable() 
{
  Serial.write(KissMarker::Fend);
  Serial.write(KissCmd::Data);

  while (radio_.available()) {
    uint8_t rxByte = 0;
    radio_.read((void*)&rxByte, sizeof(rxByte));

    if (rxByte == KissMarker::Fend) {
      Serial.write(KissMarker::Fesc);
      Serial.write(KissMarker::Tfend);
    }
    else if (rxByte == KissMarker::Fesc) {
      Serial.write(KissMarker::Fesc);
      Serial.write(KissMarker::Tfesc);
    }
    else {
      Serial.write(rxByte);
    }
  }
  Serial.write(KissMarker::Fend);
}

void onSerialDataAvailable() 
{ 
  while (Serial.available()) {
    
    int rxResult = Serial.read();
    if (rxResult == -1) break;
    
    byte rxByte = (byte)rxResult;

    switch (kissState_) {
      case KissState::Void:
        if (rxByte == KissMarker::Fend) {
          kissCmd_ = KissCmd::NoCmd;
          kissState_ = KissState::GetCmd;
        }
        break;
      case KissState::GetCmd:
        if (rxByte != KissMarker::Fend) {
          if (rxByte == KissCmd::Data) {
            radio_.stopListening();
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetData;
          }
          else if (rxByte == KissCmd::P) {
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetP;
          }
          else if (rxByte == KissCmd::SlotTime) {
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetSlotTime;
          }
          else {
            kissResetState();
          }
        }
        break;
      case KissState::GetP:
        csmaP_ = rxByte;
        kissState_ = KissState::GetData;
        break;
      case KissState::GetSlotTime:
        csmaSlotTime_ = (long)rxByte * 10;
        kissState_ = KissState::GetData;
        break;
      case KissState::GetData:
        if (rxByte == KissMarker::Fesc) {
          kissState_ = KissState::Escape;
        }
        else if (rxByte == KissMarker::Fend) {
          if (kissCmd_ == KissCmd::Data) {
            radio_.write((void*)kissBuffer_, kissBufferPosition_);
            kissBufferPosition_ = 0;
            radio_.startListening();
          }
          kissResetState();
        }
        else if (kissCmd_ == KissCmd::Data) {
          kissBuffer_[kissBufferPosition_++] = rxByte;
        }
        break;
      case KissState::Escape:
        if (rxByte == KissMarker::Tfend) {
          kissBuffer_[kissBufferPosition_++] = KissMarker::Fend;
          kissState_ = KissState::GetData;
        }
        else if (rxByte == KissMarker::Tfesc) {
          kissBuffer_[kissBufferPosition_++] = KissMarker::Fesc;
          kissState_ = KissState::GetData;
        }
        else {
          kissResetState();
        }
        break;
      default:
        break;
    }
    if (kissBufferPosition_ >= KISS_BUFFER_SIZE) {
      kissBufferPosition_ = 0;
    }
  }
}

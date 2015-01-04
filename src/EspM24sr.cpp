/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <pontus@sweetpeas.se> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return - Pontus Oldberg
 * ----------------------------------------------------------------------------
 *
 * Note:
 *   This lib is loosely based on the M24SR library by ReNa
 *   (http://regnerischernachmittag.wordpress.com/) but reduced and
 *   optimized for systems with small amounts of ram.
 *
 *   RAM usage can be further reduced by using an alternative I2C
 *   implementation that doesn't use it own internal buffers.
 */

#include <Arduino.h>
#include <EspM24sr.h>
#include <Wire.h>

// Set this flag to 1 if you need to debug the library
#define DEBUG 0

#define DEVICE_ADDRESS        0x56

#define CMD_GETI2CSESSION     0x26
#define CMD_KILLRFSESSION     0x52
#define INS_SELECT_FILE       0xA4
#define INS_UPDATE_BINARY     0xD6
#define INS_READ_BINARY       0xB0
#define INS_VERIFY            0x20

const char AID_NDEF_TAG_APPLICATION2[] PROGMEM =
    "\xD2\x76\x00\x00\x85\x01\x01";
const char DEFAULT_PASSWORD[] PROGMEM =
    "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";
const char NDEF_FILE[] PROGMEM =
    "\x00\x01";
const char FILE_SYSTEM[] PROGMEM =
    "\xE1\x01";
const char CAPABILITY_CONTAINER[] PROGMEM =
    "\xE1\x03";

EspM24SR::EspM24SR(void)
{
  err = M24SR_OPERATION_OK;
  block_num = 0;
}

EspM24SR::~EspM24SR(void)
{

}

void EspM24SR::begin()
{
  Wire.begin();
}

void EspM24SR::begin(int pin)
{
#if DEBUG
  Serial.print(F("Initializing Arduino GPO input pin "));
  Serial.println (pin);
#endif

  pinMode(pin, INPUT);

#if DEBUG
  Serial.println (F("Initializing the I2C interface !"));
#endif
  begin();
}

void EspM24SR::updateCrc (uint8_t ch, uint16_t *lpwCrc)
{
  ch = (ch^(uint8_t)((*lpwCrc) & 0x00FF));
  ch = (ch^(ch<<4));
  *lpwCrc = (*lpwCrc >> 8) ^ ((uint16_t)ch << 8) ^
            ((uint16_t)ch<<3) ^ ((uint16_t)ch>>4);
}

uint16_t EspM24SR::computeCrc(uint8_t *buf, uint8_t length)
{
  uint8_t chBlock;
  uint16_t wCrc;

  wCrc = 0x6363; // ITU-V.41

  do {
    chBlock = *buf++;
    updateCrc(chBlock, &wCrc);
  } while (--length);

  return wCrc ;
}

void EspM24SR::sendCommand(uint16_t len, boolean setPCB)
{
  int chksum;

  if (setPCB) {
    if (block_num == 0) {
      m24buf[0] = 0x02;
      block_num = 1;
    } else {
      m24buf[0] = 0x03;
      block_num = 0;
    }
  }

  Wire.beginTransmission(DEVICE_ADDRESS);   // transmit to device 0x2D
  Wire.write(CMD_GETI2CSESSION);            // GetI2Csession
  err = Wire.endTransmission();             // stop transmitting
  if (err) goto exit;

#if DEBUG
  Serial.print(F("GetI2Csession: "));
  Serial.println(err, HEX);
#endif

  Wire.beginTransmission(DEVICE_ADDRESS);

  for (int i = 0; i < len; i++)
    Wire.write(m24buf[i]);

  // 5.5 CRC of the I2C and RF frame ISO/IEC 13239.
  chksum = computeCrc((uint8_t *)m24buf, (uint8_t)len);

  Wire.write(chksum & 0xff);
  Wire.write((chksum >> 8) & 0xff);

  err = Wire.endTransmission();

exit:
  if (err) {
    // Set the global error flag.
    // TODO: Butt ugly, change this later
    err = M24SR_I2C_OPERATION_FAILED;
  }
}

int EspM24SR::receiveResponse(uint16_t offset, uint16_t len)
{
  int index = 0;
  boolean WTX = false;
  boolean loop = false;

#if DEBUG
  Serial.print(F("receiveResponse, len="));
  Serial.println(len, DEC);
#endif
  // We need a a small delay here for the device to set up for
  // responding.
  delay(2);

  do {
    WTX = false;
    loop = false;

#if DEBUG
    Serial.print(F("Reading: "));
    Serial.println(len);
    Serial.println(Wire.requestFrom(DEVICE_ADDRESS, len));
#else
    Wire.requestFrom(DEVICE_ADDRESS, len);
#endif

    while ((Wire.available() && index < len && !WTX) ||
            (WTX && index < len - 1))
    {
      int c = Wire.read();

      if (c == 0xF2 && index == 0) {
        WTX = true;
      }

      if (index >= 1) {
        response[offset + index - 1] = c;
      }

      index++;
    }

    if (WTX) {
#if DEBUG
      Serial.print(F("WTX: "));
      Serial.println(response[0]);
#endif
      m24buf[0] = 0xF2; //WTX response
      m24buf[1] = response[0];
      sendCommand(2, false);
      loop = true;
      index = 0;
      delay(100 * response[0]);
    }
  } while(loop);

  return index;
}

void EspM24SR::sendApdu(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2,
    uint8_t Lc, uint8_t* Data)
{
  m24buf[1] = CLA;
  m24buf[2] = INS;
  m24buf[3] = P1;
  m24buf[4] = P2;
  m24buf[5] = Lc;
  memcpy(&m24buf[6], Data, Lc);
  sendCommand(1 + 5 + Lc, true);
}

//APDU case 2
void EspM24SR::sendApdu(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2,
    uint8_t Le)
{
  m24buf[1] = CLA;
  m24buf[2] = INS;
  m24buf[3] = P1;
  m24buf[4] = P2;
  m24buf[5] = Le;
  sendCommand(1 + 5, true);
}

void EspM24SR::sendApdu_P(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2,
    uint8_t Lc, const char* Data)
{
  m24buf[1] = CLA;
  m24buf[2] = INS;
  m24buf[3] = P1;
  m24buf[4] = P2;
  m24buf[5] = Lc;
  memcpy_P(&m24buf[6], Data, Lc);
  sendCommand(1 + 5 + Lc, true);
}

void EspM24SR::sendSBLOCK(byte sblock)
{
  m24buf[0] = sblock;
  sendCommand(1, false);
  receiveResponse(0, 0 + 3);
}

void EspM24SR::sendDESELECT()
{
#if DEBUG
  Serial.println(F("send DESELECT"));
#endif
  sendSBLOCK(0xC2);       //PCB field
}

void EspM24SR::selectFile_NDEF_App()
{
#if DEBUG
  Serial.println(F("selectFile_NDEF_App"));
#endif
  sendApdu_P(0x00, INS_SELECT_FILE, 0x04, 0x00, 0x07, AID_NDEF_TAG_APPLICATION2);
  receiveResponse(0, 2 + 3);
}

void EspM24SR::selectFile_NDEF_file() {
#if DEBUG
  Serial.println(F("selectFile_NDEF_file"));
#endif

  sendApdu_P(0x00, INS_SELECT_FILE, 0x00, 0x0C, 0x02, NDEF_FILE);
  receiveResponse(0, 2 + 3);
}

M24SR_return_code_t EspM24SR::verifyI2cPassword()
{
  M24SR_return_code_t ret = M24SR_OPERATION_OK;
#if DEBUG
  Serial.println(F("verifyI2cPassword"));
#endif
  selectFile_NDEF_App();
  sendApdu_P(0x00, INS_VERIFY, 0x00, 0x03, 0x10, DEFAULT_PASSWORD);
  receiveResponse(0, 2 + 3);

  if (response[0] != 0x90 || response[1] != 0) {
#if DEBUG
    Serial.println(F("ERR: Password missmatch !!!"));
#endif
    ret = M24SR_PASSWORD_MISSMATCH;
  }
  return ret;
}

void EspM24SR::writeGPO(uint8_t value)
{

#if DEBUG
  Serial.println(F("writeGPO"));
#endif

  if (!verifyI2cPassword() != M24SR_OPERATION_OK) {
    return;
  }

  sendApdu_P(0x00, INS_SELECT_FILE, 0x00, 0x0C, 0x02, FILE_SYSTEM);
  receiveResponse(0, 2 + 3);

  //write system file at offset 0x0004 GPO
  sendApdu(0x00, INS_UPDATE_BINARY, 0x00, 0x04, 0x01, &value);
  receiveResponse(0, 2 + 3);
  sendDESELECT();
}

struct cc_file_layout* EspM24SR::getCCFile()
{
    uint8_t len[2];

    selectFile_NDEF_App();

    sendApdu_P(0x00, INS_SELECT_FILE, 0x00, 0x0C, 0x02, CAPABILITY_CONTAINER);
    receiveResponse(0, 2 + 3);

    // Now read length of file
    sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, 0x02);
    receiveResponse(0, 2 + 2 + 3);

    // sendDESELECT destroys the response buffer so we need to store
    // the length for a while.
    len[0] = response[0];
    len[1]=  response[1];

    // Read the rest of the CC file
    // Note, we're only handling the low order byte of the length
    sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, response[1]);
    receiveResponse(0, response[1] + 2 + 3);

exit:
    sendDESELECT();

    // Restore length
    response[0] = len[1];
    response[1] = len[0];

    // All 16 bit words are big endian so we need to change them
    // to little endian for the Arduino. Not portable, but keeps the
    // size down.
    len[0] = response[3];
    response[3] = response[4];
    response[4] = len[0];

    len[0] = response[5];
    response[5] = response[6];
    response[6] = len[0];

    len[0] = response[9];
    response[9] = response[10];
    response[10] = len[0];

    len[0] = response[11];
    response[11] = response[12];
    response[12] = len[0];

    return (struct cc_file_layout *)&response[0];
}

struct system_file* EspM24SR::getSystemFile()
{
  uint8_t len[2];

  selectFile_NDEF_App();
  sendApdu_P(0x00, INS_SELECT_FILE, 0x00, 0x0C, 0x02, FILE_SYSTEM);
  receiveResponse(0, 2 + 3);

  // First read length of file
  sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, 0x02);
  receiveResponse(0, 2 + 2 + 3);

  // Read the rest of the file content.
  sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, response[1]);
  receiveResponse(0, response[1] + 2 + 3);

  // sendDESELECT destroys the response buffer so we need to store
  // the length for a while.
  len[0] = response[0];
  len[1]=  response[1];

  sendDESELECT();

  // Restore length
  response[0] = len[1];
  response[1] = len[0];

  // Reverse length to little endian
  len[0] = response[15];
  response[15] = response[16];
  response[16] = len[0];

  return (struct system_file *)&response[0];
}

struct m24srNdefMessage* EspM24SR::getNdefMessage(size_t *length)
{
  uint8_t len[2];

#if DEBUG
  Serial.println(F("getNdefMessage()"));
#endif

  memset(response, 0xaa, sizeof response);
  selectFile_NDEF_App();
  selectFile_NDEF_file();

  // Get length of NDEF record
  sendApdu(0x00, INS_READ_BINARY, 0x00, 0x00, 0x02);
  receiveResponse(0, 2 + 2 + 3);

  // +2 because we include the length in the response as well
  *length = response[1] + 2;

  // Arduino Wire library only supports receiving 32 bytes in one
  // transaction.
  int cnt = *length / (BUFFER_LENGTH - 7);
  int remCnt = *length - (cnt * (BUFFER_LENGTH - 7));
  int offset = 0;

  // Read all full 32 byte blocks
  while (cnt--) {
    sendApdu(0x00, INS_READ_BINARY, 0x00, (uint8_t)offset, BUFFER_LENGTH - 7);
    receiveResponse(offset, BUFFER_LENGTH);
    // TODO: Do some error checking here
    offset += (BUFFER_LENGTH - 7);
  }

  // Read the remaining data
  if (remCnt) {
    sendApdu(0x00, INS_READ_BINARY, 0x00, (uint8_t)offset, remCnt);
    receiveResponse(offset, remCnt + 7);
  }
  // TODO: Do some error checking here

  // Temporarily store the first bytes, sendDESELECT will destroy it
  len[0] = response[0];
  len[1]=  response[1];

  sendDESELECT();

  // Restore length in little endian order.
  response[0] = len[1];
  response[1] = len[0];

  return (struct m24srNdefMessage *)&response[0];
}

#if defined(ESPM24SR_ENABLE_WRITE)

void EspM24SR::updateBinaryLen(int len) {
#if DEBUG
  Serial.println(F("\r\nupdateBinaryLen"));
#endif
    uint8_t len_bytes[2];
    len_bytes[0] = (len >> 8) & 0xff;
    len_bytes[1] = (len & 0xff);
    sendApdu(0x00, INS_UPDATE_BINARY, 0x00, 0x00, 0x02, len_bytes);
    receiveResponse(0, 2 + 3);
}

void EspM24SR::updateBinary(uint8_t *buf, uint8_t len) {
#if DEBUG
  Serial.println(F("updateBinary"));
#endif

  // Arduino Wire library only supports transmitting 32 bytes in one
  // transaction. Here we calculate the number of chunks we need
  // to split the data into.
  int cnt = len / (BUFFER_LENGTH - 8);
  int remCnt = len - (cnt * (BUFFER_LENGTH - 8));
  int offset = 2;

#if DEBUG
  Serial.print(F("Cnt = "));
  Serial.print(cnt);
  Serial.print(F(", remCnt = "));
  Serial.println(remCnt);
#endif

  while(cnt--) {
    sendApdu(0x00, INS_UPDATE_BINARY, (offset >> 8) & 0xff, offset & 0xff,
        BUFFER_LENGTH, buf);
    receiveResponse(0, 2 + 3);
    offset += (BUFFER_LENGTH - 8);
    buf += (BUFFER_LENGTH - 8);
  }

#if DEBUG
  Serial.print(F("Offset = "));
  Serial.println(offset);
#endif

  if (remCnt) {
    sendApdu(0x00, INS_UPDATE_BINARY, (offset >> 8) & 0xff, offset & 0xff,
        remCnt, buf);
    receiveResponse(0, 2 + 3);
  }
}

int EspM24SR::writeNdefMessage(struct ndefMessage *msg, uint8_t len)
{
  if (!msg) {
    return M24SR_PARAMETER_ERROR;
  }

  selectFile_NDEF_App();
  selectFile_NDEF_file();

  updateBinary((uint8_t *)msg, len);
  updateBinaryLen(len - 2);

  sendDESELECT();

  return 0;
}

void EspM24SR::clearTag(void)
{
#if DEBUG
  Serial.print(F("clearTag"));
#endif
  uint8_t len0[] = "\x00\x03\xD0\x00\x00";

  selectFile_NDEF_App();
  selectFile_NDEF_file();

  sendApdu(0x00, INS_UPDATE_BINARY, 0x00, 0x00, 0x05, len0);
  receiveResponse(0, 2 + 3);

  sendDESELECT();
}
#endif

/* EOF */

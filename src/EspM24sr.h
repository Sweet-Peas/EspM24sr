/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <pontus@sweetpeas.se> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return - Pontus Oldberg
 * ----------------------------------------------------------------------------
 */

#ifndef ESP_M24SR_H_
#define ESP_M24SR_H_

enum M24SR_return_codes {
  M24SR_OPERATION_OK = 0,
  M24SR_I2C_OPERATION_FAILED,
  M24SR_PASSWORD_MISSMATCH,
  M24SR_PARAMETER_ERROR,
};
typedef enum M24SR_return_codes    M24SR_return_code_t;

struct ndef_fc_tlv {
    uint8_t   t;
    uint8_t   l;
    uint16_t  file_id;
    uint16_t  max_ndef_size;
    uint8_t   read_access;
    uint8_t   write_access;
};

struct cc_file_layout {
    uint16_t  cc_file_size;
    uint8_t   version;
    uint16_t  max_read_size;
    uint16_t  max_write_size;
    struct ndef_fc_tlv ndef_tlv;
};

struct system_file {
    uint16_t  system_file_size;
    uint8_t   i2c_protect;
    uint8_t   i2c_watchdog;
    uint8_t   gpo;
    uint8_t   reserved;
    uint8_t   rf_enabled;
    uint8_t   rfu;
    uint8_t   uid[7];
    uint16_t  mem_size;
    uint8_t   product_code;
};

struct ndefMessage {
    union {
        uint8_t tnf;
        uint8_t data[7];
    } d;
};

struct m24srNdefMessage {
    uint16_t length;
    struct ndefMessage msg;
};

#define NDEF_TNF_MB       0x80
#define NDEF_TNF_ME       0x40
#define NDEF_TNF_CF       0x20
#define NDEF_TNF_SR       0x10
#define NDEF_TNF_IL       0x08
#define NDEF_TNF_MASK     0x07

class EspM24SR
{
    public:
      EspM24SR();
      ~EspM24SR();

      void begin(int pin);
      struct cc_file_layout* getCCFile();
      struct system_file*  getSystemFile();
      M24SR_return_code_t verifyI2cPassword();
      void writeGPO(uint8_t value);

      struct m24srNdefMessage* getNdefMessage(size_t *length);
      int writeNdefMessage(struct m24srNdefMessage *msg, uint8_t len);

      char m24buf[16];
      uint8_t response[256];
      uint8_t err;
      uint8_t block_num;

    private:
      void sendSBLOCK(byte sblock);
      void sendDESELECT();
      void selectFile_CC_file();
      void selectFile_NDEF_App();
      void selectFile_NDEF_file();
      void updateBinaryLen(int len);
      void updateBinary_NdefMsgLen0();
      void updateBinary(char* Data, uint8_t len);
      void updateCrc(uint8_t ch, uint16_t *lpwCrc);
      uint16_t computeCrc(uint8_t *buf, uint8_t length);
      void sendCommand(uint16_t len, boolean setPCB);
      int receiveResponse(uint16_t offset, uint16_t len);
      void sendApdu(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2,
          uint8_t Lc, uint8_t* Data);
      void sendApdu(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2,
          uint8_t Le);
      void sendApdu_P(uint8_t CLA, uint8_t INS, uint8_t P1, uint8_t P2,
          uint8_t Lc, const char* Data);
};

#endif /* ESP_M24SR_H_ */

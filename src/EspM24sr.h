/*
 * ----------------------------------------------------------------------------
 *            _____ _           _                   _
 *           | ____| | ___  ___| |_ _ __ ___  _ __ (_) ___
 *           |  _| | |/ _ \/ __| __| '__/ _ \| '_ \| |/ __|
 *           | |___| |  __/ (__| |_| | | (_) | | | | | (__
 *           |_____|_|\___|\___|\__|_|  \___/|_| |_|_|\___|
 *            ____                   _   ____
 *           / ___|_      _____  ___| |_|  _ \ ___  __ _ ___
 *           \___ \ \ /\ / / _ \/ _ \ __| |_) / _ \/ _` / __|
 *            ___) \ V  V /  __/  __/ |_|  __/  __/ (_| \__ \
 *           |____/ \_/\_/ \___|\___|\__|_|   \___|\__,_|___/
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <pontus@sweetpeas.se> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return - Pontus Oldberg
 * ----------------------------------------------------------------------------
 */

#ifndef ESP_M24SR_H_
#define ESP_M24SR_H_

/**
 * To save on program flash the write parts of the library can be
 * disabled. Simply comment the next line.
 */
#define ESPM24SR_ENABLE_WRITE

enum M24SR_return_codes {
  M24SR_OPERATION_OK = 0,
  M24SR_I2C_OPERATION_FAILED,
  M24SR_PASSWORD_MISSMATCH,
  M24SR_PARAMETER_ERROR,
};
typedef enum M24SR_return_codes    M24SR_return_code_t;

/**
 * Defines the tlv section of the Capability Container (CC) file.
 */
struct ndef_fc_tlv {
    uint8_t   t;
    uint8_t   l;
    uint16_t  file_id;
    uint16_t  max_ndef_size;
    uint8_t   read_access;
    uint8_t   write_access;
};

/**
 * Defines the Capability Container file.
 */
struct cc_file_layout {
    uint16_t  cc_file_size;
    uint8_t   version;
    uint16_t  max_read_size;
    uint16_t  max_write_size;
    struct ndef_fc_tlv ndef_tlv;
};

/**
 * Defines the system file
 */
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

/**
 * Due to the dynamic nature of the NDEF header I choose to
 * create a very simple mapping of the fields. Here only the tnf
 * field has a symbolic entry, the rest is simply mapped as an array
 */
struct ndefMessage {
    union {
        uint8_t tnf;
        uint8_t data[7];
    } d;
};

/**
 * Constants that define the onle NDEF header fields that are
 * constant.
 */
#define NDEF_FLD_TYPE_LENGTH      1
#define NDEF_FLD_PAYLOAD_LENGTH   2

/**
 * Type that is returned from getNdefMessage(). This encloses the
 * entire NDEF message including the length specifier inside the
 * EEPROM.
 */
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

/**
 * The EspM24SR class includes the basic functionality needed to be
 * able to read and write to a M24SR dual port memory. It was designed
 * to be as small as possible but still have the required functionality
 * to be able to read and write NDEF records.
 *
 * It does not (yet) include any functionality to manage passwords
 * or any of the advanced security features that is found in the
 * device itself.
 *
 * Example:
 *\code{.cpp}
 * EspM24SR m24sr;
 *
 * struct m24srNdefMessage *msg;
 * size_t length;
 *
 * m24sr.begin();
 * msg = m24sr.getNdefMessage(&length);
 *
 *\endcode
 */
class EspM24SR
{
    public:
      /**
       * Constructor of the EspM24SR class.
       */
      EspM24SR();

      /**
       * Destructor of the EspM24SR class.
       */
      ~EspM24SR();

      /**
       * Initializes the resources required for this library.
       */
      void begin();

      /**
       * Initializes the resources required for this library.
       * This version allows you to specify which pin is connected
       * to the GPO output of the M24SR device.
       */
      void begin(int pin);

      /**
       * Returns a pointer to the retrieved Capability Container
       * file. The returned pointer points to the data in the
       * global M24SR communication buffer which means that the
       * caller need to copy the wanted data before continuing to
       * communicate with the device.
       */
      struct cc_file_layout* getCCFile();

      /**
       * Returns a pointer to the retrieved System File file.
       *  The returned pointer points to the data in the
       * global M24SR communication buffer which means that the
       * caller need to copy the wanted data before continuing to
       * communicate with the device.
       */
      struct system_file*  getSystemFile();

      /**
       * Sets the required operating mode of the GPO pin.
       * Make sure you consult the datasheet to get the operating
       * mode that you are after.
       */
      void writeGPO(uint8_t value);

      /**
       * Returns a pointer to the message in the NDEF file.
       * The returned pointer points to the data in the
       * global M24SR communication buffer which means that the
       * caller need to copy the wanted data before continuing to
       * communicate with the device.
       */
      struct m24srNdefMessage* getNdefMessage(size_t *length);

#if defined(ESPM24SR_ENABLE_WRITE)
      int writeNdefMessage(struct ndefMessage *msg, uint8_t len);
      void clearTag();
#endif

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
      void updateBinary(uint8_t *buf, uint8_t len);
      void updateCrc(uint8_t ch, uint16_t *lpwCrc);
      M24SR_return_code_t verifyI2cPassword();
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

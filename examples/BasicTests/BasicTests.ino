#include <Arduino.h>
#include <Wire.h>
#include <EspM24sr.h>

#define GPO_PIN    3
#define LED        13

EspM24SR m24sr;
char *msg = NULL;

void parseNdefTable(struct ndefMessage *msg)
{
  byte num_entries = 0;
  byte loopit;
  
  // First we parse the entire message to see how many entries we can find.
  Serial.print(F("First time TNF: "));
  Serial.println(msg->d.tnf, HEX);
  
  // First check if NDEF file is empty
  if ((msg->d.tnf & NDEF_TNF_MASK) == 0) {
    Serial.println(F("NDEF File is empty !"));
    return;
  }
  
  do {
    byte offset = 0;
    byte pl_length;
    byte data_start;
    
    loopit = msg->d.tnf;

    // Now look for next entry.
    // Calculate offset for fixed fields in the header
    offset += 3;                 // Includes offset for type length, payload length (SR) and record type
    // Calculate offset for given length descriptors
    offset += msg->d.data[NDEF_FLD_TYPE_LENGTH];       // Type length offset
    offset += msg->d.data[NDEF_FLD_PAYLOAD_LENGTH];    // Payload length (Only SR records supported)
    pl_length = msg->d.data[NDEF_FLD_PAYLOAD_LENGTH];

    num_entries++;

    // Now determine if the ID field is present
    if (msg->d.tnf & NDEF_TNF_IL) {
      Serial.println(F("ID field present !"));
      offset += 1; // For the ID length field
      offset += msg->d.data[3];  // Add in the ID field length descriptor
    }
    
    Serial.print(F("Offset is: "));
    Serial.println(offset);
    data_start = offset - pl_length;
    for (int i=0;i<pl_length;i++)
      Serial.write(msg->d.data[data_start++]);
    Serial.println();
    
    msg = (struct ndefMessage *)(offset + (char *)msg);
  } while (!(loopit & NDEF_TNF_ME));
  
  Serial.print(F("Number of entries: "));
  Serial.println(num_entries);
}

unsigned char newBuf[256];
void setup()
{
  uint16_t length;
  struct cc_file_layout *ccfl;
  struct system_file *sf;
  
  Serial.begin(115200);
  Serial.println("Hello World !");
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  /* Initialize the m24sr library */
  m24sr.begin(GPO_PIN);
  
  /* Set GPO mode */
  m24sr.writeGPO(1);
  
  Serial.println(F("CC file content:"));
  ccfl = m24sr.getCCFile();
  Serial.print(F("  Length of CC file: "));
  Serial.println(ccfl->cc_file_size);
  Serial.print(F("  Mapping Version: "));
  Serial.println(ccfl->version, HEX);
  Serial.print(F("  Maximum number of bytes that can read: "));
  Serial.println(ccfl->max_read_size);
  Serial.print(F("  Maximum number of bytes that can written: "));
  Serial.println(ccfl->max_write_size);

  Serial.print(F("  NDEF TLV, T field: "));
  Serial.println(ccfl->ndef_tlv.t);
  Serial.print(F("  NDEF TLV, L field: "));
  Serial.println(ccfl->ndef_tlv.l);
  Serial.print(F("  NDEF TLV, File ID: "));
  Serial.println(ccfl->ndef_tlv.file_id);
  Serial.print(F("  NDEF TLV, Max NDEF file size: "));
  Serial.println(ccfl->ndef_tlv.max_ndef_size);

  Serial.print(F("  NDEF TLV, Read access: "));
  Serial.println(ccfl->ndef_tlv.read_access);
  Serial.print(F("  NDEF TLV, write access: "));
  Serial.println(ccfl->ndef_tlv.write_access);

  Serial.println(F("System File:"));
  sf = m24sr.getSystemFile();
  Serial.print(F("  System file length: "));
  Serial.println(sf->system_file_size);
  Serial.print(F("  I2C Protect: "));
  Serial.println(sf->i2c_protect);
  Serial.print(F("  I2C Watchdog: "));
  Serial.println(sf->i2c_watchdog);
  Serial.print(F("  GPO: "));
  Serial.println(sf->gpo);
  Serial.print(F("  Reserved: "));
  Serial.println(sf->reserved);
  Serial.print(F("  RF Enable: "));
  Serial.println(sf->rf_enabled);
  Serial.print(F("  NDEF File number (RFU): "));
  Serial.println(sf->rfu);
  Serial.print(F("  UID: "));
  for (int i=0;i<7;i++) {
    if (sf->uid[i] < 0x10) Serial.write('0');
    Serial.print(sf->uid[i], HEX);
    if (i < 6) Serial.print('-');
  }
  Serial.println();
  Serial.print(F("  EEPROM Memory size: "));
  Serial.println(sf->mem_size);
  Serial.print(F("  Product Code: "));
  Serial.println(sf->product_code, HEX);
  
#if 1
//  m24sr.clearTag();
//  Serial.println(F("Tag Cleared!"));
  
  struct m24srNdefMessage* m24sr_ndef = m24sr.getNdefMessage(&length);
  msg = (char *)m24sr_ndef;
  Serial.print(F("NDEF message length: "));
  Serial.println(m24sr_ndef->length);

  Serial.print(F("MB: "));
  if (m24sr_ndef->msg.d.tnf & NDEF_TNF_MB)
    Serial.print("1");
  else
    Serial.print("0");
  
  Serial.print(F(", ME: "));
  if (m24sr_ndef->msg.d.tnf & NDEF_TNF_ME)
    Serial.print("1");
  else
    Serial.print("0");
  
  Serial.print(F(", CF: "));
  if (m24sr_ndef->msg.d.tnf & NDEF_TNF_CF)
    Serial.print("1");
  else
    Serial.print("0");

  Serial.print(F(", SR: "));
  if (m24sr_ndef->msg.d.tnf & NDEF_TNF_SR)
    Serial.print("1");
  else
    Serial.print("0");
    
  Serial.print(F(", IL: "));
  if (m24sr_ndef->msg.d.tnf & NDEF_TNF_IL)
    Serial.print("1");
  else
    Serial.print("0");
    
  Serial.print(F(", TNF: "));
  Serial.println(m24sr_ndef->msg.d.tnf & NDEF_TNF_MASK);

  Serial.print(F("NDEF message length: "));
  Serial.println(length);
  Serial.print(F("NDEF Message: "));
  for (byte i=0; i<length; i++) {
    byte val = *(msg + i);
    if (val < 0x10) Serial.print(F("0"));
    Serial.print(val, HEX);
    Serial.print(F(" "));
  }
  for (byte i=0; i<length; i++) {
    byte val = *(msg + i);
    if (isprint(val) )
      Serial.write(val);
    else
      Serial.write('?');
  }
  Serial.println();
  
  parseNdefTable(&m24sr_ndef->msg);
  
  // Write the message back
  // Since the current NDEF msg is in the M24SR library working buffer
  // We need to copy it to a new area.
  memcpy(newBuf, msg, length);
  struct m24srNdefMessage *newMsg = (struct m24srNdefMessage *)newBuf;
  // Then we can write the content back to the tag EEPROM.
  m24sr.writeNdefMessage(&(newMsg->msg), (uint8_t)length);

  // Get and parse again to see the newly written data.... 
  // which should be the same as the old
  m24sr_ndef = m24sr.getNdefMessage(&length);
  parseNdefTable(&m24sr_ndef->msg);
  
#endif
}

void loop() 
{
}

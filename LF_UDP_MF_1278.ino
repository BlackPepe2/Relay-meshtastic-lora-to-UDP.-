
#include <SPI.h>                                 //the SX127X device is SPI based so load the SPI library
#include "SX127XLT.h"                            //include the appropriate library   
#include "Settings.h"                            //include the setiings file, frequencies, LoRa settings etc   

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

//------------------------ http (to do)
WebServer server(80);

//------------------------ udp
WiFiUDP Udp;
unsigned int localUdpPort  = 4210;  //  port to listen on
unsigned int remoteUdpPort = 4210;  //  port to write to
// WiFi client and UDP
const char *ssid = "FSM";
const char *password = "0101010101";

String act_ip;

//------------------------ lora
#define LED 22
SX127XLT LT;                                     //create a library class instance called LT

uint8_t RXPacketL;                               //stores length of packet received
uint8_t RXBUFFER[RXBUFFER_SIZE];                 //create the buffer that received packets are copied into
uint8_t TXPacketL;
uint8_t TXBUFFER[RXBUFFER_SIZE];
int8_t  PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio of received packet

int udprxSize;
byte rxPacket[RXBUFFER_SIZE];  // buffer for incoming UDP packets
byte txPacket[RXBUFFER_SIZE];  // buffer for outgoing UDP packets

void init_UDP()
{
  Udp.begin(localUdpPort);
  Serial.printf("UDP port %d\n", localUdpPort);
}

IPAddress ClientIP;
IPAddress RemoteIP = "192.168.1.120";

int UDPrxnum;
int UDPtxnum;
int LORArxnum;
int LORAtxnum;

bool read_UDP()
{
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) 
  {
    ClientIP = Udp.remoteIP();
    Serial.print("UDP RX ");
    Serial.print(ClientIP);
    Serial.print(" Size ");
    Serial.println(packetSize);
    Udp.read(rxPacket, packetSize);  
    udprxSize = packetSize;
    UDPrxnum++;
    return true;
  }
  //Udp.flush();
  return false; 
}
void send_UDP(int txSize)
{
  Udp.beginPacket(RemoteIP, remoteUdpPort);
  Udp.write(txPacket,txSize);
  Udp.endPacket();
  UDPtxnum++;
}

void loop()
{
  server.handleClient();
  RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 1000, WAIT_RX); //wait for a packet 

  if (RXPacketL != 0) 
  {
    PacketRSSI = LT.readPacketRSSI(); //read the recived RSSI value
    PacketSNR = LT.readPacketSNR();   //read the received SNR value
    packet_is_OK();
    LORArxnum++;
    for (int i=0; i<RXPacketL; i++) txPacket[i] = RXBUFFER[i];
    send_UDP(RXPacketL);
  }

  if (read_UDP())
  {
    TXPacketL = udprxSize;
    for (int i=0; i<TXPacketL; i++) TXBUFFER[i] = rxPacket[i];
    if (LT.transmit(TXBUFFER, TXPacketL, 5000, TXpower, WAIT_TX)) 
      Serial.println("sent");
    LORAtxnum++;
  }

  while(Serial.available() > 0)
  {
    char ch = Serial.read();
    if(ch == 'm') delay(10);
  }
}


void packet_is_OK()
{
  uint16_t IRQStatus, localCRC;

  IRQStatus = LT.readIrqStatus();                  //read the LoRa device IRQ status register

  Serial.println();
  for (int i=0; i<RXPacketL; i++)
  {
    byte ch = RXBUFFER[i];
    if (ch < 0x10) Serial.print('0'); 
    Serial.print(ch,HEX);
  }
  Serial.println();
  Serial.print(F("RSSI "));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm, SNR "));
  Serial.print(PacketSNR);
  Serial.print(F("dB, Length "));
  Serial.println(RXPacketL);
}

void setup()
{  
  if (VCCPOWER >= 0)
  {
    pinMode(VCCPOWER, OUTPUT);                  //For controlling power to external devices
    digitalWrite(VCCPOWER, LOW);                //VCCOUT on. lora device on
  }

  Serial.begin(115200);
  Serial.println(F("4_LoRa_Receiver_ESP32 Starting"));
  Serial.println();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  { delay(500); Serial.print(".");  }
  Serial.println();
  act_ip = WiFi.localIP().toString().c_str();
  Serial.print("IP: ");
  Serial.println(act_ip);

  init_UDP();
  server.on("/", handleCmd);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  //SPI.begin();                                      //default setup can be used be used
  SPI.begin(SCK, MISO, MOSI);                         //alternative format for SPI3, VSPI; SPI.begin(SCK,MISO,MOSI,SS)

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
    Serial.println(F("LoRa Device found"));
  else
  {
    Serial.println(F("No device responding"));
    while (1)delay(1000);                                       //long fast speed LED flash indicates device error
  }

  //The function call list below shows the complete setup for the LoRa device using the information defined in the
  //Settings.h file.
  //The 'Setup LoRa device' list below can be replaced with a single function call;
  //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, Optimisation);

  //***************************************************************************************************
  //Setup LoRa device
  //***************************************************************************************************
  LT.setMode(MODE_STDBY_RC);                              //got to standby mode to configure device
  LT.setPacketType(PACKET_TYPE_LORA);                     //set for LoRa transmissions
  LT.setRfFrequency(Frequency, Offset);                   //set the operating frequency
  LT.calibrateImage(0);                                   //run calibration after setting frequency
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, LDRO_AUTO);  //set LoRa modem parameters
  LT.setBufferBaseAddress(0x00, 0x00);                    //where in the SX buffer packets start, TX and RX
  LT.setPacketParams(16, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);  //set packet parameters
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);              //syncword, LORA_MAC_PRIVATE_SYNCWORD = 0x12, or LORA_MAC_PUBLIC_SYNCWORD = 0x34
  LT.setHighSensitivity();                                //set for highest sensitivity at expense of slightly higher LNA current
  LT.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RX_DONE, 0, 0);   //set for IRQ on RX done
  //***************************************************************************************************

  Serial.println();

  Serial.print(F("Receiver ready - RXBUFFER_SIZE "));
  Serial.println(RXBUFFER_SIZE);
  Serial.println();
}

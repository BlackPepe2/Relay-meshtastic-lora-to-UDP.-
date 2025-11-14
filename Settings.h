
//*******  Setup hardware pin definitions here ! ***************

// Heltec V2

#define NSS 18                                  //select pin on LoRa device
#define SCK  5                                  //SCK on SPI3
#define MISO 19                                 //MISO on SPI3 
#define MOSI 27                                 //MOSI on SPI3 

#define NRESET 14                               //reset pin on LoRa device
#define RFBUSY 35 // DIO1                       //busy line

#define LED1 25                                 //on board LED, high for on
#define DIO0 26                                 //DIO0 pin on LoRa device, used for RX and TX done 
#define BUZZER -1                               //pin for buzzer, set to -1 if not used 
#define VCCPOWER 21                             //pin controls power to external devices

#define LORA_DEVICE DEVICE_SX1278               //we need to define the device we are using


//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 869525000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_250;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF11;      //Long-Fast
//const uint8_t SpreadingFactor = LORA_SF9;     //Medium-Fast
const uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

const int8_t TXpower = 20;                      //LoRa transmit power in dBm

const uint16_t packet_delay = 1000;             //mS delay between packets

#define RXBUFFER_SIZE 250                        //RX buffer size 

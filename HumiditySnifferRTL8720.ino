//must install the SoftWire library
#include <Software_HDC1080.h>
#include <wifi_conf.h>
//have to undefine these arduino definss for the RTL std library
#undef max
#undef min
#include <list>
#define SCAN_CHANNEL_NUM 2 //2.4GHz + 5GHz
u8 scan_channel_list2[SCAN_CHANNEL_NUM] = {6,48};

SoftWire sw = SoftWire(PA30, PA8);
char swTxBuffer[16];
char swRxBuffer[16];
Software_HDC1080 hdc1080;

float temperature = 0;
float humidity = 0;

char buffer[256];

struct WiFiSignal {
    unsigned char addr[6]; 
    signed char rssi;
};

std::list<WiFiSignal> _signals;

const uint32_t address = 0x00002001;
const uint32_t multi_address = 0x00002000;
uint8_t headerState = 0;
uint8_t dataState = 0;
uint32_t readLength = 0;
uint32_t length = 0;
uint32_t readIndex = 0;
uint8_t buff1[1500];
uint8_t buff2[1500];
uint8_t *readData = buff1;
uint8_t *executeData = buff2;
uint32_t executeLength = 0;

bool readPacket(int serialInput) {
    uint8_t executeState = 0;
    if(serialInput != -1)
    {
      //this is triggered by the state machine below it
      if(dataState == 10)
      {
        if(readIndex != length)
        {
          readData[readIndex] = serialInput;
          readIndex++;
        }
        
        if(length == readIndex && executeState == 0)
        {
          //swap buffers
          uint8_t *tmp = executeData;
          executeData = readData;
          readData = tmp;
          
          executeLength = length;
          length = 0;
          readIndex = 0;
          dataState = 0;//end data state machine
          executeState = 10;//execute on data;
        }
      }
      
      //this header state machine always runs in case the length is somehow boggus and has this thing stuck
      if(headerState == 0 && serialInput == '$')
      {
        headerState = 10;//header start signal
      }
      else if(headerState == 10 && serialInput == '$')
      {
        headerState = 20;//header start signal
      }
      else if(headerState == 20 && (serialInput == (address & 0xFF) || serialInput == (multi_address & 0xFF)))
      {
        headerState = 30;//address LSB
      }
      else if(headerState == 30 && (serialInput == ((address >> 8) & 0xFF) || serialInput == ((multi_address >> 8) & 0xFF)))
      {
        headerState = 40;//address
      }
      else if(headerState == 40 && (serialInput == ((address >> 16) & 0xFF) || serialInput == ((multi_address >> 16) & 0xFF)))
      {
        headerState = 50;//address
      }
      else if(headerState == 50 && (serialInput == ((address >> 24) & 0xFF) || serialInput == ((multi_address >> 24) & 0xFF)))
      {
        headerState = 60;//address MSB
      }
      else if(headerState == 60)
      {
        readLength = serialInput;//length LSB
        headerState = 70;
      }
      else if(headerState == 70)
      {
        readLength |= serialInput << 8;//length
        headerState = 80;//length
      }
      else if(headerState == 80)
      {
        readLength |= serialInput << 16;//length
        headerState = 90;//length
      }
      else if(headerState == 90)
      {
        readLength |= serialInput << 24;//length MSB
        headerState = 100;//length
      }
      else if(headerState == 100 && serialInput == '$')
      {
        headerState = 110;//header end signal
      }
      else if(headerState == 110 && serialInput == '$')
      {
        if(readLength <= 1500)
        {
          length = readLength;
          readIndex = 0;
          dataState = 10;
        }
        headerState = 0;//header end signal
      }
      else if(headerState == 100 && serialInput == 'R')
      {
        headerState = 110;//header end reset signal
      }
      else if(headerState == 110 && serialInput == 'R')
      {
        dataState = 0;
        executeState = 0;
        headerState = 0;//header end reset signal
      }
      else
      {
        headerState = 0;
      }
    }

    return executeState == 10;
}

void setup() {
    //Initialize serial and wait for port to open:
    pinMode(PA12, OUTPUT);
    digitalWrite(PA12, LOW);
    pinMode(PB3, OUTPUT);
    digitalWrite(PB3, HIGH);
    Serial.begin(1000000);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    wifi_on(RTW_MODE_PROMISC);
    wifi_enter_promisc_mode();
    wifi_set_promisc(RTW_PROMISC_ENABLE_2, promisc_callback, 0);

    sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
    sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
    sw.enablePullups(true);
    
    hdc1080.begin(0x40, sw);
}

void readTemperatureAndHumidty() {
    hdc1080.setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);

    HDC1080_Registers reg = hdc1080.readRegister();
    
    temperature = hdc1080.readTemperature();
    double temperatureF = (temperature * 9/5) + 32;
    humidity = hdc1080.readHumidity();
    
    delayMicroseconds(50);
    printf("T=%dF, RH=%d%%\r\n", (int)temperatureF, (int)humidity);
}

void scanChannels(u8 *channels, u8 numberOfChannels, u32 scanTimePerChannel){
    _signals.clear();
    for(u8 ch = 0; ch < numberOfChannels; ch ++) {
        wifi_set_channel(channels[ch]);
    
        delay(scanTimePerChannel);
    }
    printSignals();
}

void printSignals() {
    delayMicroseconds(50);
    std::list<WiFiSignal>::iterator next = _signals.begin();
    while(next != _signals.end())
    {
        printMac(next->addr);
        printf("   %d\r\n", next->rssi);
        next++;
    }
    printf("\r\n");
}

void loop() {
    if(readPacket(Serial.read()))
    {
      if(executeLength > 0) 
      {
        switch(executeData[0])
        {
          case 1://Scan Channels
          if(executeLength > 5) 
          {
            uint32_t scanTimePerChannel = 0;
            scanTimePerChannel |= executeData[1];
            scanTimePerChannel |= executeData[2] << 8;
            scanTimePerChannel |= executeData[3] << 16;
            scanTimePerChannel |= executeData[4] << 24;
            uint8_t numberOfChannels = executeData[5];
            if(executeLength > numberOfChannels + 5)
            {
              printf("Scanning Channels: ");
              for(uint8_t i = 0; i < numberOfChannels; i++)
                  printf("%d ", executeData[6 + i]);
              
              printf("\t\tFor %dms each\r\n", scanTimePerChannel);
              
              scanChannels(&executeData[6], numberOfChannels, scanTimePerChannel);
            }
            else
            {
              printf("Scan Channels Command Length Less Than number of Channels: %d-6 < %d\r\n", executeLength, numberOfChannels);
            }
          }
          else
          {
            printf("Scan Channels Command Length Less Than 6: &d\r\n", executeLength);
          }
          break;
          case 11://Send Signals
          if(executeLength > 4) 
          {
            digitalWrite(PA12, HIGH);
            Serial.write(0x24);//header start signal
            Serial.write(0x24);//header start signal
            Serial.write(executeData[1]); //address LSB
            Serial.write(executeData[2]); //address
            Serial.write(executeData[3]); //address
            Serial.write(executeData[4]); //address MSB
            uint32_t signals = _signals.size() * 7;
            Serial.write(signals & 0xFF); ////length LSB
            Serial.write((signals >> 8) & 0xFF); ////length
            Serial.write((signals >> 16) & 0xFF); ////length
            Serial.write((signals >> 24) & 0xFF); ////length MSB
            Serial.write(0x24);//header end signal
            Serial.write(0x24);//header end signal
            std::list<WiFiSignal>::iterator next = _signals.begin();
            while(next != _signals.end())
            {
                for(uint8_t i = 0; i < 6; i++)
                    Serial.write(next->addr[i]);
                Serial.write(next->rssi);
                next++;
            }
            delayMicroseconds(20);
            digitalWrite(PA12, LOW);
          }
          else
          {
            printf("Send Channels Command Length Less Than 5: &d\r\n", executeLength);
          }
          break;
          case 2://Capture Temp and Humidity
          readTemperatureAndHumidty();
          break;
          case 12://Send Temp and Humidity
          if(executeLength > 4) 
          {
            digitalWrite(PA12, HIGH);
            Serial.write(0x24);//header start signal
            Serial.write(0x24);//header start signal
            Serial.write(executeData[1]); //address LSB
            Serial.write(executeData[2]); //address
            Serial.write(executeData[3]); //address
            Serial.write(executeData[4]); //address MSB
            Serial.write((uint8_t)0x08); //length LSB
            Serial.write((uint8_t)0x00); //length
            Serial.write((uint8_t)0x00); //length
            Serial.write((uint8_t)0x00); //length MSB
            Serial.write(0x24);//header end signal
            Serial.write(0x24);//header end signal
            Serial.write(*(reinterpret_cast<uint8_t *>(&temperature))); //temperature LSB
            Serial.write(*(reinterpret_cast<uint8_t *>(&temperature) + 1)); //temperature
            Serial.write(*(reinterpret_cast<uint8_t *>(&temperature) + 2)); //temperature
            Serial.write(*(reinterpret_cast<uint8_t *>(&temperature) + 3)); //temperature MSB
            Serial.write(*(reinterpret_cast<uint8_t *>(&humidity))); //humidity MSB
            Serial.write(*(reinterpret_cast<uint8_t *>(&humidity) + 1)); //humidity 
            Serial.write(*(reinterpret_cast<uint8_t *>(&humidity) + 2)); //humidity 
            Serial.write(*(reinterpret_cast<uint8_t *>(&humidity) + 3)); //humidity MSB
            delayMicroseconds(20);
            digitalWrite(PA12, LOW);
          }
          else
          {
            printf("Send Temp and Humidity Command Length Less Than 5: &d\r\n", executeLength);
          }
          break;
          default:
          printf("Unrecognized Command: %d\r\n", executeData[0]);
          break;
        }
      }
      else
      {
        printf("Command Length = 0\r\n");
      }
    }
}

/*  Make callback simple to prevent latency to wlan rx when promiscuous mode */
static void promisc_callback(unsigned char *buf, unsigned int len, void* userdata)
{
    const ieee80211_frame_info_t *frameInfo = (ieee80211_frame_info_t *)userdata;
//    if(frameInfo->i_addr1[0] == 0xff && frameInfo->i_addr1[1] == 0xff && frameInfo->i_addr1[2] == 0xff && frameInfo->i_addr1[3] == 0xff && frameInfo->i_addr1[4] == 0xff && frameInfo->i_addr1[5] == 0xff)
//        return;
    if(frameInfo->rssi == 0)
        return;
    WiFiSignal wifisignal;
    wifisignal.rssi = frameInfo->rssi;
    memcpy(&wifisignal.addr, &frameInfo->i_addr2, 6);

    
    std::list<WiFiSignal>::iterator next = _signals.begin();
    while(next != _signals.end())
    {
        if(next->addr[0] == wifisignal.addr[0] &&
          next->addr[1] == wifisignal.addr[1] &&
          next->addr[2] == wifisignal.addr[2] &&
          next->addr[3] == wifisignal.addr[3] &&
          next->addr[4] == wifisignal.addr[4] &&
          next->addr[5] == wifisignal.addr[5]
          )
            _signals.erase(next);
        next++;
    }
    _signals.push_back(wifisignal);
}


void printMac(const unsigned char mac[6]) {
    for(u8 i = 0; i < 6; i ++){
        printf(" %02x", mac[i]);
    }
}

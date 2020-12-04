#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define RETURN_BUFFER 256

//network configuration info for remote device
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x7E, 0x3A}; //Set device mac address from hardware
IPAddress ip(192,168,11,2); //Set device IP address 
unsigned int localPort = 8888; //Set port to listen for commands on
unsigned int returnPort = 8888; //Set port udp will return on

//Set up send and receive buffers
char netInBuffer[UDP_TX_PACKET_MAX_SIZE];
char outputBuffer[RETURN_BUFFER];

EthernetUDP Udp; //udp handler
IPAddress returnIP(192,168,11,1); //updated each packet received, this is where udp packets of serial data will be sent
uint8_t writeOut = 0; //Flag to send packetBuffer
uint8_t i = 0; //buffer index counter

//run initial startup/configuration
void setup() 
{
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
  Serial.begin(115200);
  Udp.beginPacket(returnIP, returnPort);
  Udp.write("On...");
  Udp.endPacket();
}

void loop() 
{
  //check for packet and process it
  int packetSize = Udp.parsePacket();
  if(packetSize)
  {
    returnIP = Udp.remoteIP(); //get IP to return packet to
    Udp.read(netInBuffer, UDP_TX_PACKET_MAX_SIZE); //Store packet contents to buffer
    Serial.print(netInBuffer); //send command to scan tool
  }
  
  //Build response from incoming serial data, finishes when buffer is full or newline
  if(Serial.available())
  {
    char in = Serial.read();
    outputBuffer[i++] = in;
    if(in == '>' || i >= RETURN_BUFFER)
    {
      writeOut = 1; //Set flag to send udp packet on newline or max packet size
    }
  }
  
  //Build and send response packet
  if(writeOut)
  {
    //Send Packet
    Udp.beginPacket(returnIP, returnPort);
    Udp.write(outputBuffer);
    Udp.endPacket();

    //Clean up counters and buffer
    for(uint8_t x = 0; x <= i; x++)
    {
      outputBuffer[x] = 0x00; //clear buffer
    }
    writeOut = 0; //clear flag
    i = 0; //reset index
  }
  
}
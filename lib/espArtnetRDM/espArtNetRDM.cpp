
/*
  espArtNetRDM v1 (pre-release) library
  Copyright (c) 2016, Matthew Tong
  https://github.com/mtongnz/
  Modified from https://github.com/forkineye/E131/blob/master/E131.h
  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
  later version.
  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
  You should have received a copy of the GNU General Public License along with this program.
  If not, see http://www.gnu.org/licenses/
*/

#include "espArtNetRDM.h"

static void artClearDMXBuffer(uint8_t* buf) {
  memset(buf, 0, DMX_BUFFER_SIZE);
}

static bool isBroadcastIP(const IPAddress& ip) {
  // 1) Global broadcast
  if (ip == IPAddress(255, 255, 255, 255))
    return true;

  // 2) Art-Net directed broadcast: 2.x.x.255
  if (ip[0] == 2 && ip[3] == 255)
    return true;

  // 3) Subnet-directed broadcast
  IPAddress localIP = WiFi.localIP();
  IPAddress subnet  = WiFi.subnetMask();

  for (int i = 0; i < 4; i++) {
    if ((localIP[i] | ~subnet[i]) != ip[i])
      return false;
  }
  return true;
}

espArtNetRDM::espArtNetRDM() {
}

espArtNetRDM::~espArtNetRDM() {
  end();
}

void espArtNetRDM::end() {
  if (_art == nullptr)
    return;

  #if defined(ESP8266)
  eUDP.stopAll();
  #endif

#if defined(ESP32)
  for (uint8_t g = 0; g < _art->numGroups; g++) {
    for (uint8_t p = 0; p < 4; p++) {
      if (_art->group[g]->ports[p] == 0)
        continue;

      if (_art->group[g]->ports[p]->ownBuffer)
        free(_art->group[g]->ports[p]->dmxBuffer);

      free(_art->group[g]->ports[p]->ipBuffer);
      free(_art->group[g]->ports[p]);
    }
    free(_art->group[g]);
  }
#elif defined(ESP8266)
  for (uint8_t g = 0; g < _art->numGroups; g++) {
    for (uint8_t p = 0; p < 4; p++) {
      if (_art->group[g]->ports[p] == 0)
        continue;

      if (_art->group[g]->ports[p]->ownBuffer)
        os_free(_art->group[g]->ports[p]->dmxBuffer);

      os_free(_art->group[g]->ports[p]->ipBuffer);
      os_free(_art->group[g]->ports[p]);
    }
    os_free(_art->group[g]);
  }
#endif

  delete _art;
  _art = nullptr;
}

void espArtNetRDM::init(IPAddress ip, IPAddress subnet, bool dhcp, const char* shortname, const char* longname, uint16_t oem, uint16_t esta, uint8_t* mac) {
  if (_art != nullptr)
    end();

  // Allocate memory for our settings
  _art = new artnet_device();

  delay(1);
  
  // Store values
  _art->firmWareVersion = 0;
  _art->numGroups = 0;
  _art->nodeReportCounter = 0;
  _art->nodeReportCode = ARTNET_RC_POWER_OK;
  _art->nodeReportChanged = false;
  _art->lastPollWasBroadcast = false;
  _art->lastIPProg = 0;
  _art->lastFlags = 0x00;
  _art->deviceIP = ip;
  _art->subnet = subnet;
  _art->broadcastIP[0] = _art->deviceIP[0] | (~_art->subnet[0]);
  _art->broadcastIP[1] = _art->deviceIP[1] | (~_art->subnet[1]);
  _art->broadcastIP[2] = _art->deviceIP[2] | (~_art->subnet[2]);
  _art->broadcastIP[3] = _art->deviceIP[3] | (~_art->subnet[3]);
  _art->dhcp = dhcp;
  _art->oemLo = (uint8_t)oem;
  _art->oemHi = (uint8_t)(oem >> 8);
  _art->estaLo = (uint8_t)esta;
  _art->estaHi = (uint8_t)(esta >> 8);
  _art->syncIP = INADDR_NONE;
  _art->lastSync = 0;
  memcpy(_art->shortName, shortname, ARTNET_SHORT_NAME_LENGTH);
  memcpy(_art->longName, longname, ARTNET_LONG_NAME_LENGTH);
  memcpy(_art->deviceMAC, mac, 6);

  _art->dmxCallBack = 0;
  _art->syncCallBack = 0;
  _art->rdmCallBack = 0;
  _art->ipCallBack = 0;
  _art->addressCallBack = 0;
  _art->todRequestCallBack = 0;
  _art->todFlushCallBack = 0;
}

void espArtNetRDM::setFirmwareVersion(uint16_t fw) {
  if (_art == nullptr)
    return;

  _art->firmWareVersion = fw;
}

void espArtNetRDM::setDefaultIP() {
  if (_art == nullptr)
    return;

  _art->dhcp = false;
  _art->subnet = IPAddress(255, 255, 255, 0);
  _art->broadcastIP = IPAddress(2, 0, 0, 255);

  uint8_t b = _art->deviceMAC[3] + _art->oemLo + _art->oemHi;
  uint8_t c = _art->deviceMAC[4];
  uint8_t d = _art->deviceMAC[5];

  _art->deviceIP = IPAddress(2, b, c, d);
}

uint8_t espArtNetRDM::addGroup(uint8_t net, uint8_t subnet) {
  if (_art == nullptr)
    return 255;

  if (_art->numGroups >= ARTNET_GROUPS_MAX)
    return 255;

  uint8_t g = _art->numGroups;
  
#if defined(ESP32)
  _art->group[g] = (group_def*) malloc(sizeof(group_def));
#elif defined(ESP8266)
  _art->group[g] = (group_def*) os_malloc(sizeof(group_def));
#endif
  _art->group[g]->netSwitch = net & 0b01111111;
  _art->group[g]->subnet = subnet;
  _art->group[g]->numPorts = 0;
  _art->group[g]->cancelMergeIP = INADDR_NONE;
  _art->group[g]->cancelMerge = false;
  _art->group[g]->cancelMergeTime = 0;

  for (int x = 0; x < 4; x++)
    _art->group[g]->ports[x] = 0;
  
  _art->numGroups++;

  return g;
}

uint8_t espArtNetRDM::addPort(uint8_t g, uint8_t p, uint8_t universe, uint8_t t, bool htp, uint8_t* buf) {
  if (_art == nullptr)
    return 255;

  // Check for a valid universe, group and port number
  if (universe > 15 || p >= 4 || g >= _art->numGroups)
    return 255;

  if (t > DMX_IN)
    return 255;
  
  group_def* group = _art->group[g];
  
  // Check if port is already initialised, return its port number
  if (group->ports[p] != 0)
    return p;

  // Allocate space for our port
#if defined(ESP32)
  group->ports[p] = (port_def*) malloc(sizeof(port_def));
#elif defined(ESP8266)
  group->ports[p] = (port_def*) os_malloc(sizeof(port_def));
#endif
  
  delay(1);
  port_def* port = group->ports[p];
  
  // DMX output buffer allocation
  if (buf == 0) {
#if defined(ESP32)
    port->dmxBuffer = (uint8_t*) malloc(DMX_BUFFER_SIZE);
#elif defined(ESP8266)
    port->dmxBuffer = (uint8_t*) os_malloc(DMX_BUFFER_SIZE);
#endif
    port->ownBuffer = true;
  } else {
    port->dmxBuffer = buf;
    port->ownBuffer = false;
  }

  // Clear the buffer
  artClearDMXBuffer(port->dmxBuffer);
  
  // Store settings
  group->numPorts++;
  port->portType = t;
  port->mergeHTP = htp;
  port->portUni = universe;
  port->senderIP[0] = INADDR_NONE;
  port->senderIP[1] = INADDR_NONE;

  for (uint8_t x = 0; x < 5; x++)
    port->rdmSenderIP[x] = INADDR_NONE;

  port->ipBuffer = 0;
  port->ipChans[0] = 0;
  port->ipChans[1] = 0;
  port->dmxChans = 0;
  port->merging = 0;
  port->lastTodCommand = 0;
  port->uidTotal = 0;
  port->todAvailable = 0;
  
  return p;
}

bool espArtNetRDM::closePort(uint8_t g, uint8_t p) {
  if (_art == nullptr || g >= _art->numGroups)
    return false;
  
  group_def* group = _art->group[g];
  
  // Port already closed
  if (group->ports[p] == 0)
    return true;

  // Delete buffers
#if defined(ESP32)
  if (group->ports[p]->ownBuffer)
    free(group->ports[p]->dmxBuffer);
  if (group->ports[p]->ipBuffer != 0)
    free(group->ports[p]->ipBuffer);

  free(group->ports[p]);
#elif defined(ESP8266)
  if (group->ports[p]->ownBuffer)
    os_free(group->ports[p]->dmxBuffer);
  if (group->ports[p]->ipBuffer != 0)
    os_free(group->ports[p]->ipBuffer);
  
  os_free(group->ports[p]);
#endif

  // Mark port as empty
  group->ports[p] = 0;
  group->numPorts--;
  return true;
}

void espArtNetRDM::setArtDMXCallback(artDMXCallBack callback) {
  if (_art == nullptr)
    return;

  _art->dmxCallBack = callback;
}

void espArtNetRDM::setArtSyncCallback(artSyncCallBack callback) {
  if (_art == nullptr)
    return;

  _art->syncCallBack = callback;
}

void espArtNetRDM::setArtRDMCallback(artRDMCallBack callback) {
  if (_art == nullptr)
    return;

  _art->rdmCallBack = callback;
}

void espArtNetRDM::setArtIPCallback(artIPCallBack callback) {
  if (_art == nullptr)
    return;

  _art->ipCallBack = callback;
}

void espArtNetRDM::setArtAddressCallback(artAddressCallBack callback) {
  if (_art == nullptr)
    return;

  _art->addressCallBack = callback;
}

void espArtNetRDM::setTODRequestCallback(artTodRequestCallBack callback) {
  if (_art == nullptr)
    return;

  _art->todRequestCallBack = callback;
}

void espArtNetRDM::setTODFlushCallback(artTodFlushCallBack callback) {
  if (_art == nullptr)
    return;

  _art->todFlushCallBack = callback;
}

void espArtNetRDM::begin() {
  if (_art == nullptr)
    return;

  // Start listening for UDP packets
  eUDP.begin(ARTNET_PORT);
  eUDP.flush();
  fUDP.begin(E131_PORT);
  fUDP.flush();
  
  // Send ArtPollReply to tell everyone we're here
  artPollReply();
}
    
void espArtNetRDM::pause() {
  if (_art == nullptr)
    return;

  eUDP.flush();
  #if defined(ESP8266)
  eUDP.stopAll();
  #endif
}

void espArtNetRDM::handler() {
  if (_art == nullptr)
    return;

  // Artnet packet
  uint16_t packetSize = eUDP.parsePacket();

  if (packetSize > 0) {

    static unsigned char _artBuffer[ARTNET_BUFFER_MAX];

    // Read data into buffer
    eUDP.read(_artBuffer, packetSize);
  
    // Get the Op Code
    int opCode = _artOpCode(_artBuffer);

    switch (opCode) {

      case ARTNET_ARTPOLL:
        if (packetSize > 12) {
          _art->lastPollIP = eUDP.remoteIP();
          _art->lastPollWasBroadcast = isBroadcastIP(_art->lastPollIP);
          _art->lastFlags = _artBuffer[12];
          _artPoll();
        }
        break;

      case ARTNET_ARTDMX:
        if (packetSize > ARTNET_ADDRESS_OFFSET)
          _artDMX(_artBuffer);
        break;

      case ARTNET_IP_PROG:
        if (packetSize > 23)
          _artIPProg(_artBuffer);
        break;
  
      case ARTNET_ADDRESS:
        if (packetSize > 106)
          _artAddress(_artBuffer);
        break;
  
      case ARTNET_SYNC:
        _artSync(_artBuffer);
        break;
  
      case ARTNET_FIRMWARE_MASTER:
        _artFirmwareMaster(_artBuffer);
        break;
  
      case ARTNET_TOD_REQUEST:
        if (packetSize > 23)
          _artTODRequest(_artBuffer);
        break;
  
      case ARTNET_TOD_CONTROL:
        if (packetSize > 23)
          _artTODControl(_artBuffer);
        break;
  
      case ARTNET_RDM:
        if (packetSize > 25)
          _artRDM(_artBuffer, packetSize);
        break;
  
      case ARTNET_RDM_SUB:
        _artRDMSub(_artBuffer);
        break;
    }
  }

  // e131 packet
  packetSize = fUDP.parsePacket();

  if (packetSize > 0) {

    e131_packet_t _e131Buffer;

    // Read data into buffer
    fUDP.readBytes(_e131Buffer.raw, packetSize);

    _e131Receive(&_e131Buffer);
  }

  // Send artPollReply if node condition/report changed
  bool sendOnChange = _art->lastFlags & ARTNET_FLAG_SEND_ON_CHANGE;
  if (_art->nodeReportChanged && sendOnChange) {
    _artPoll();
    _art->nodeReportChanged = false;
  }

}

int espArtNetRDM::_artOpCode(unsigned char *_artBuffer) {
  if (memcmp(_artBuffer, "Art-Net\0", 8) == 0) {
    if (_artBuffer[11] >= 14)                         //protocol version [10] hi byte [11] lo byte
        return (_artBuffer[9] << 8) | _artBuffer[8];  //opcode lo byte first
  }
  
  return 0;
}


void espArtNetRDM::_artPoll() {
  static uint8_t _artReplyBuffer[ARTNET_REPLY_SIZE];
  _artReplyBuffer[0] = 'A';
  _artReplyBuffer[1] = 'r';
  _artReplyBuffer[2] = 't';
  _artReplyBuffer[3] = '-';
  _artReplyBuffer[4] = 'N';
  _artReplyBuffer[5] = 'e';
  _artReplyBuffer[6] = 't';
  _artReplyBuffer[7] = 0x00;
  _artReplyBuffer[8] = (uint8_t)(ARTNET_ARTPOLL_REPLY);      	// op code lo-hi
  _artReplyBuffer[9] = (uint8_t)(ARTNET_ARTPOLL_REPLY >> 8); 	// 0x2100 = artPollReply
  _artReplyBuffer[10] = _art->deviceIP[0];        	          // ip address
  _artReplyBuffer[11] = _art->deviceIP[1];
  _artReplyBuffer[12] = _art->deviceIP[2];
  _artReplyBuffer[13] = _art->deviceIP[3];
  _artReplyBuffer[14] = 0x36;               		              // port lo first always 0x1936
  _artReplyBuffer[15] = 0x19;
  _artReplyBuffer[16] = _art->firmWareVersion >> 8;           // firmware hi-lo
  _artReplyBuffer[17] = _art->firmWareVersion;
  _artReplyBuffer[20] = _art->oemHi;                          // oem hi-lo
  _artReplyBuffer[21] = _art->oemLo;
  _artReplyBuffer[22] = 0;              		                  // ubea

  // -------- Status1 (byte 23) --------
  uint8_t status1 = 0x00;
  status1 |= 0x02;     // RDM capable
  status1 |= 0xC0;     // Indicator state: Normal Mode
  status1 |= 0x20;     // Port-Address programming authority - programmed by web UI
  _artReplyBuffer[23] = status1;

  // -------- ESTA manufacturer code (bytes 24–25) --------
  _artReplyBuffer[24] = _art->estaLo;
  _artReplyBuffer[25] = _art->estaHi;

  //short name
  for (int x = 0; x < ARTNET_SHORT_NAME_LENGTH; x++)
    _artReplyBuffer[x + 26] = _art->shortName[x];
    
  //long name
  for (int x = 0; x < ARTNET_LONG_NAME_LENGTH; x++)
    _artReplyBuffer[x + 44] = _art->longName[x];

  // node report - send blank
  for (int x = 0; x < ARTNET_NODE_REPORT_LENGTH; x++) {
    _artReplyBuffer[x + 108] = 0;
  }

  // Set reply code
  char tmp[7];
  snprintf(tmp, sizeof(tmp), "%04x", _art->nodeReportCode);
  _artReplyBuffer[108] = '#';
  _artReplyBuffer[109] = tmp[0];
  _artReplyBuffer[110] = tmp[1];
  _artReplyBuffer[111] = tmp[2];
  _artReplyBuffer[112] = tmp[3];
  _artReplyBuffer[113] = '[';

  // Max 6 digits for counter - could be longer if wanted
  snprintf(tmp, sizeof(tmp), "%u", _art->nodeReportCounter++);
  if (_art->nodeReportCounter > 999999)
    _art->nodeReportCounter = 0;

  // Format counter and add to reply buffer
  uint8_t x = 0;
  for (x = 0; tmp[x] != '\0' && x < 6; x++)
    _artReplyBuffer[x + 114] = tmp[x];

  uint8_t rLen = ARTNET_NODE_REPORT_LENGTH - x - 2;
  x = x + 114;

  _artReplyBuffer[x++] = ']';
  _artReplyBuffer[x++] = ' ';

  // Append plain text report
  for (uint8_t y = 0; y < rLen && _art->nodeReport[y] != '\0'; y++)
    _artReplyBuffer[x++] = _art->nodeReport[y];

  _artReplyBuffer[172] = 0;             //number of ports Hi (always 0)
  _artReplyBuffer[194] = 0;             // these are not used
  _artReplyBuffer[195] = 0;
  _artReplyBuffer[196] = 0;
  _artReplyBuffer[197] = 0;
  _artReplyBuffer[198] = 0;
  _artReplyBuffer[199] = 0;
  _artReplyBuffer[200] = 0;             // Style - 0x00 = DMX to/from Artnet

  for (int x = 0; x < 6; x++)           // MAC Address
    _artReplyBuffer[201 + x] = _art->deviceMAC[x];

  _artReplyBuffer[207] = _art->deviceIP[0];        // bind ip
  _artReplyBuffer[208] = _art->deviceIP[1];
  _artReplyBuffer[209] = _art->deviceIP[2];
  _artReplyBuffer[210] = _art->deviceIP[3];

  // -------- Status2 (byte 212) --------
  uint8_t status2 = 0x00;
  // status2 |= 0x80;     // RDM support ArtAddress
  status2 |= 0x10;     // sACN capable
  status2 |= 0x08;     // 15-bit addressing, Art-Net 3+
  status2 |= 0x04;     // DHCP capable
  if (_art->dhcp) status2 |= 0x02; 
  status2 |= 0x01;     // Web config supported

  _artReplyBuffer[212] = status2;

  for (int x = 213; x < ARTNET_REPLY_SIZE; x++)
    _artReplyBuffer[x] = 0;             // Reserved for future - transmit 0


  // Set values for each group of ports and send artPollReply
  for (uint8_t groupNum = 0; groupNum < _art->numGroups; groupNum++) {
    group_def* group = _art->group[groupNum];
    
    if (group->numPorts == 0)
      continue;

    _artReplyBuffer[18] = group->netSwitch;       // net
    _artReplyBuffer[19] = group->subnet;          // subnet
    _artReplyBuffer[173] = group->numPorts;       // number of ports (Lo byte)

    _artReplyBuffer[211] = groupNum + 1;    	    // Bind Index

    // Port details
    for (int x = 0; x < 4; x++) {

      // Send blank values for empty ports
      _artReplyBuffer[174 + x] = 0;
      _artReplyBuffer[178 + x] = 0;
      _artReplyBuffer[182 + x] = 0;
      _artReplyBuffer[186 + x] = 0;
      _artReplyBuffer[190 + x] = 0;

      // This port isn't in use
      if (group->ports[x] == 0)
        continue;

      // Set port type
      uint8_t portType = 0;
      if (group->ports[x]->portType == DMX_OUT || group->ports[x]->portType == RDM_OUT)
        portType |= 0x80; // output capable
      if (group->ports[x]->portType == DMX_IN)
        portType |= 0x40; // input capable

      _artReplyBuffer[174 + x] = portType;

      // DMX or RDM out port
      if (group->ports[x]->portType != DMX_IN) {
        // Get values for Good Output field
        uint8_t go = 0;
        if (group->ports[x]->dmxChans != 0)
          go |= 0x80;						// data being transmitted
        if (group->ports[x]->merging)
          go |= 0x08;						// artnet data being merged
        if (group->ports[x]->merging && !group->ports[x]->mergeHTP)
          go |= 0x02;						// Merge mode LTP
        if (group->ports[x]->e131)
          go |= 0x01;						// sACN

        _artReplyBuffer[182 + x] = go;				                  // Good output
        _artReplyBuffer[190 + x] = group->ports[x]->portUni;  	// swOut - port address

      // DMX In port info
      } else if (group->ports[x]->portType == DMX_IN) {
        if (group->ports[x]->dmxChans != 0)
          _artReplyBuffer[178 + x] = 0x80;       		            // Good input received

        _artReplyBuffer[186 + x] = group->ports[x]->portUni;  	// swIn
      }
    }

    // Prepare packet
    uint16_t replyLen = ARTNET_POLL_REPLY_MIN_LEN;
    // Trim trailing zero bytes (optional but recommended)
    for (int i = ARTNET_REPLY_SIZE - 1; i >= ARTNET_POLL_REPLY_MIN_LEN; i--) {
      if (_artReplyBuffer[i] != 0) {
        replyLen = i + 1;
        break;
      }
    }

    // Send packet
    IPAddress destIP;
    bool unicastReply = _art->lastFlags & ARTNET_FLAG_UNICAST_REPLY;
    if (unicastReply && !_art->lastPollWasBroadcast && _art->lastPollIP)
      destIP = _art->lastPollIP;
    else
      destIP = _art->broadcastIP;

    if (destIP == WiFi.localIP())
      return;

    eUDP.beginPacket(destIP, ARTNET_PORT);
    eUDP.write((const uint8_t *)_artReplyBuffer, replyLen);
    eUDP.endPacket();

    delay(0);
  }
}


void espArtNetRDM::artPollReply() {
  if (_art == nullptr)
    return;

  _artPoll();
}

void espArtNetRDM::_artDMX(unsigned char *_artBuffer) {
  group_def* group = nullptr;

  IPAddress rIP = eUDP.remoteIP();

  uint8_t net = (_artBuffer[15] & 0x7F);
  uint8_t sub = (_artBuffer[14] >> 4);
  uint8_t uni = (_artBuffer[14] & 0x0F);

  // Number of channels hi uint8_t first
  uint16_t numberOfChannels = _artBuffer[17] + (_artBuffer[16] << 8);
  if (numberOfChannels == 0)
    return;
  if (numberOfChannels > 512)
    numberOfChannels = 512;

  uint16_t startChannel = 0;

  // Loop through all groups
  for (uint8_t g = 0; g < _art->numGroups; g++) {
    group = _art->group[g];

    if (group->netSwitch != net || group->subnet != sub)
      continue;

    // Loop through each port
    for (uint8_t p = 0; p < 4; p++) {
      port_def* port = group->ports[p];
      if (!port || port->portType == DMX_IN)
        continue;

      if (port->portUni != uni)
        continue;

      // If this port has the correct Net, Sub & Uni then save DMX to buffer
      _saveDMX(&_artBuffer[ARTNET_ADDRESS_OFFSET],
               numberOfChannels,
               g,
               p,
               rIP,
               startChannel);
    }
  }
}

void espArtNetRDM::_saveDMX(unsigned char *dmxData, uint16_t length, uint8_t groupNum, uint8_t portNum, IPAddress srcIP, uint16_t startChannel) {
  group_def* group = _art->group[groupNum];
  port_def* port   = group->ports[portNum];

  const uint32_t now = millis();
  const uint32_t TIMEOUT = 10000;

  // ---------- Sender slot cleanup ----------
  for (uint8_t i = 0; i < 2; i++) {
    if (port->senderIP[i] != INADDR_NONE &&
        (now - port->lastPacketTime[i]) > TIMEOUT)
    {
      port->senderIP[i] = INADDR_NONE;
      port->lastPacketTime[i] = 0;
    }
  }

  // ---------- Assign sender slot ----------
  int8_t sender = -1;

  for (uint8_t i = 0; i < 2; i++) {
    if (port->senderIP[i] == srcIP) {
      sender = i;
      break;
    }
  }

  if (sender == -1) {
    for (uint8_t i = 0; i < 2; i++) {
      if (port->senderIP[i] == INADDR_NONE) {
        sender = i;
        port->senderIP[i] = srcIP;
        break;
      }
    }
  }

  // Third sender → DROP
  if (sender == -1)
    return;

  port->lastPacketTime[sender] = now;

  uint8_t other = sender ^ 1;
  port->merging = (port->senderIP[other] != INADDR_NONE);

  // ---------- Cancel Merge handling ----------
  if (group->cancelMerge &&
      group->cancelMergeIP == srcIP &&
      (now - group->cancelMergeTime) < ARTNET_CANCEL_MERGE_TIMEOUT)
  {
    port->mergeHTP = false;
    port->merging  = false;
  }
  else if (group->cancelMerge &&
            (now - group->cancelMergeTime) < ARTNET_CANCEL_MERGE_TIMEOUT)
  {
    // Other sender during cancel-merge → DROP
    return;
  }
  else {
    group->cancelMerge = false;
    group->cancelMergeIP = INADDR_NONE;
  }

  // ---------- Channel bounds ----------
  if (length > DMX_BUFFER_SIZE)
    length = DMX_BUFFER_SIZE;

  if (length > port->dmxChans)
    port->dmxChans = length;

  // ---------- HTP merge ----------
  if (port->merging && port->mergeHTP) {

    if (!port->ipBuffer) {
#if defined(ESP32)
      port->ipBuffer = (uint8_t*)malloc(2 * DMX_BUFFER_SIZE);
#else
      port->ipBuffer = (uint8_t*)os_malloc(2 * DMX_BUFFER_SIZE);
#endif
      artClearDMXBuffer(port->ipBuffer);
      artClearDMXBuffer(port->ipBuffer + DMX_BUFFER_SIZE);
    }

    memcpy(
        port->ipBuffer + sender * DMX_BUFFER_SIZE + startChannel,
        dmxData,
        length
    );

    for (uint16_t i = 0; i < port->dmxChans; i++) {
      port->dmxBuffer[i] =
          max(port->ipBuffer[i],
              port->ipBuffer[i + DMX_BUFFER_SIZE]);
    }

    _art->dmxCallBack(groupNum, portNum, port->dmxChans, false);
    return;
  }

  // ---------- LTP / single sender ----------
  memcpy(port->dmxBuffer + startChannel, dmxData, length);

  // Free merge buffer if not used
  if (port->ipBuffer) {
#if defined(ESP32)
    free(port->ipBuffer);
#else
    os_free(port->ipBuffer);
#endif
    port->ipBuffer = nullptr;
  }

  bool sync = (_art->lastSync &&
              (now - _art->lastSync) < 4000 &&
              _art->syncIP == srcIP);

  _art->syncIP = srcIP;
  _art->dmxCallBack(groupNum, portNum, port->dmxChans, sync);
}

uint8_t* espArtNetRDM::getDMX(uint8_t g, uint8_t p) {
  if (_art == nullptr)
    return NULL;

  if (g < _art->numGroups) {
    if (_art->group[g]->ports[p] != 0)
      return _art->group[g]->ports[p]->dmxBuffer;
  }
  return NULL;
}

uint16_t espArtNetRDM::numChans(uint8_t g, uint8_t p) {
  if (_art == nullptr)
    return 0;

  if (g < _art->numGroups) {
    if (_art->group[g]->ports[p] != 0)
      return _art->group[g]->ports[p]->dmxChans;
  }
  return 0;
}

void espArtNetRDM::_artIPProg(unsigned char *_artBuffer) {
  // Don't do anything if it's the same command again
  if ((_art->lastIPProg + 20) > millis())
    return;
  _art->lastIPProg = millis();
  
  uint8_t command = _artBuffer[14];

  // Enable DHCP
  if ((command & 0b11000000) == 0b11000000) {
    _art->dhcp = true;

  // Disable DHCP
  }else if ((command & 0b11000000) == 0b10000000) {
    _art->dhcp = false;
    
    // Program IP
    if ((command & 0b10000100) == 0b10000100)
      _art->deviceIP = IPAddress(_artBuffer[16], _artBuffer[17], _artBuffer[18], _artBuffer[19]);
      
    // Program subnet
    if ((command & 0b10000010) == 0b10000010) {
      _art->subnet = IPAddress(_artBuffer[20], _artBuffer[21], _artBuffer[22], _artBuffer[23]);
      _art->broadcastIP = IPAddress((uint32_t)_art->deviceIP | ~((uint32_t)_art->subnet));
    }

    // Use default address
    if ((command & 0b10001000) == 0b10001000)
      setDefaultIP();
  }
  
  // Run callback - must be before reply for correct dhcp setting
  if (_art->ipCallBack != 0)
    _art->ipCallBack();
  
  // Send reply
  _artIPProgReply();

  // Send artPollReply
  artPollReply();
}

void espArtNetRDM::_artIPProgReply() {
  // Initialise our reply
  static char ipProgReply[ARTNET_IP_PROG_REPLY_SIZE];
  
  ipProgReply[0] = 'A';
  ipProgReply[1] = 'r';
  ipProgReply[2] = 't';
  ipProgReply[3] = '-';
  ipProgReply[4] = 'N';
  ipProgReply[5] = 'e';
  ipProgReply[6] = 't';
  ipProgReply[7] = 0;
  ipProgReply[8] = (uint8_t)(ARTNET_IP_PROG_REPLY);      // op code lo-hi
  ipProgReply[9] = (uint8_t)(ARTNET_IP_PROG_REPLY >> 8); // 0x2100 = artPollReply
  ipProgReply[10] = 0;
  ipProgReply[11] = 14;                 // artNet version (14)
  ipProgReply[12] = 0;
  ipProgReply[13] = 0;
  ipProgReply[14] = 0;
  ipProgReply[15] = 0;
  ipProgReply[16] = _art->deviceIP[0];  // ip address
  ipProgReply[17] = _art->deviceIP[1];
  ipProgReply[18] = _art->deviceIP[2];
  ipProgReply[19] = _art->deviceIP[3];
  ipProgReply[20] = _art->subnet[0];    // subnet address
  ipProgReply[21] = _art->subnet[1];
  ipProgReply[22] = _art->subnet[2];
  ipProgReply[23] = _art->subnet[3];
  ipProgReply[24] = 0;
  ipProgReply[25] = 0;
  ipProgReply[26] = (_art->dhcp) ? (1 << 6) : 0;  // DHCP enabled
  ipProgReply[27] = 0;
  ipProgReply[28] = 0;
  ipProgReply[29] = 0;
  ipProgReply[30] = 0;
  ipProgReply[31] = 0;
  ipProgReply[32] = 0;
  ipProgReply[33] = 0;

  // Send packet
  eUDP.beginPacket(eUDP.remoteIP(), ARTNET_PORT);
  eUDP.write((const uint8_t *)ipProgReply, ARTNET_IP_PROG_REPLY_SIZE);
  eUDP.endPacket();
}

void espArtNetRDM::_artAddress(unsigned char *_artBuffer) {
  // _artBuffer[13]    bindIndex
  uint8_t g = _artBuffer[13] - 1;
  if (g >= _art->numGroups)
    return;

  uint8_t cmd = _artBuffer[106];
  uint8_t op = cmd & ARTNET_AC_OP_MASK;
  uint8_t p = cmd & ARTNET_AC_PORT_MASK;

  group_def* group = _art->group[g];
  port_def* port = nullptr;

  // Set net switch
  if ((_artBuffer[12] & 0x80) == 0x80)
    group->netSwitch = _artBuffer[12] & 0x7F;
  
  // Set short name
  if (_artBuffer[14] != '\0') {
    for (int x = 0; x < ARTNET_SHORT_NAME_LENGTH; x++)
      _art->shortName[x] = _artBuffer[x + 14];
  }

  // Set long name
  if (_artBuffer[32] != '\0') {
    for (int x = 0; x < ARTNET_LONG_NAME_LENGTH; x++)
      _art->longName[x] = _artBuffer[x + 32];
  }

  // Set Port Address
  for (int x = 0; x < 4; x++) {
    if ((_artBuffer[100 + x] & 0xF0) == 0x80 && group->ports[x] != 0)
      group->ports[x]->portUni = _artBuffer[100 + x] & 0x0F;
  }

  // Set subnet
  if ((_artBuffer[104] & 0xF0) == 0x80) {
    group->subnet = _artBuffer[104] & 0x0F;
  }

  // ---------- Global commands ----------
  if (cmd == ARTNET_AC_CANCEL_MERGE) {
    group->cancelMerge      = true;
    group->cancelMergeIP    = eUDP.remoteIP();
    group->cancelMergeTime  = millis();

    for (uint8_t i = 0; i < 4; i++) {
      port_def* cancelMergePort = group->ports[i];
      if (!cancelMergePort) continue;

      cancelMergePort->merging  = false;
      cancelMergePort->mergeHTP = false;
      cancelMergePort->senderIP[0] = INADDR_NONE;
      cancelMergePort->senderIP[1] = INADDR_NONE;
      cancelMergePort->lastPacketTime[0] = 0;
      cancelMergePort->lastPacketTime[1] = 0;

      if (cancelMergePort->ipBuffer) {
#if defined(ESP32)
        free(cancelMergePort->ipBuffer);
#else
        os_free(cancelMergePort->ipBuffer);
#endif
        cancelMergePort->ipBuffer = nullptr;
      }
    }
    goto send_reply;
  }

  // ---------- Per-port commands ----------
  if (p >= 4 || !group->ports[p])
    return;

  port = group->ports[p];

  switch (op) {
    case ARTNET_AC_MERGE_LTP:
      if (port->ipBuffer) {
#if defined(ESP32)
        free(port->ipBuffer);
#elif defined(ESP8266)
        os_free(port->ipBuffer);
#endif
        port->ipBuffer = nullptr;
      }

      port->lastPacketTime[0] = 0;
      port->lastPacketTime[1] = 0;
      port->mergeHTP = false;
      port->merging = false;

      group->cancelMerge = false;
      group->cancelMergeIP = INADDR_NONE;
      break;

    case ARTNET_AC_MERGE_HTP:
      if (port->portType != DMX_IN) {
        // OUTPUT → HTP merge
        port->mergeHTP = true;
        port->merging = false;
      } else {
        // INPUT → sACN select
        setE131(g, p, true);
      }
      group->cancelMerge = false;
      group->cancelMergeIP = INADDR_NONE;
      break;

    case ARTNET_AC_CLEAR_OP:
      if (port->ipBuffer) {
#if defined(ESP32)
        free(port->ipBuffer);
#elif defined(ESP8266)
        os_free(port->ipBuffer);
#endif
        port->ipBuffer = nullptr;
      }

      artClearDMXBuffer(port->dmxBuffer);
      port->merging = false;
      port->mergeHTP  = true;
      setE131(g, p, false);
      break;

    case ARTNET_AC_ARTNET_SEL:
      setE131(g, p, false);
      break;
  }

  // Send reply
send_reply:
  artPollReply();
  
  // Run callback
  if (_art->addressCallBack != 0)
    _art->addressCallBack();
}

void espArtNetRDM::_artSync(unsigned char *_artBuffer) {
  // Update sync timer
  _art->lastSync = millis();
  
  // Run callback
  if (_art->syncCallBack != 0 && _art->syncIP == eUDP.remoteIP())
    _art->syncCallBack();
}

void espArtNetRDM::_artFirmwareMaster(unsigned char *_artBuffer) {
  //Serial.println("artFirmwareMaster");
}

void espArtNetRDM::_artTODRequest(unsigned char *_artBuffer) {
  uint8_t net = _artBuffer[21];
  group_def* group;

  uint8_t numAddress = _artBuffer[23];
  uint8_t addr = 24;

  // Handle artTodControl requests
  if (_artOpCode(_artBuffer) == ARTNET_TOD_CONTROL) {
    numAddress = 1;
    addr = 23;
  }
  
  for (int g = 0; g < _art->numGroups; g++) {
    group = _art->group[g];
    
    // Net matches so loop through the addresses
    if (group->netSwitch == net) {
      for (int y = 0; y < numAddress; y++) {
        
        // Subnet doesn't match, try the next address
        if (group->subnet != (_artBuffer[addr + y] >> 4))
          continue;

        // Subnet matches so loop through the 4 ports and check universe
        for (int p = 0; p < 4; p++) {
          
          if (group->ports[p] == 0)
            continue;
          
          port_def* port = group->ports[p];
          
          if (port->portUni != (_artBuffer[addr + y] & 0x0F))
            continue;

          port->lastTodCommand = millis();
          
          // Flush TOD
          if (_artBuffer[22] == 0x01)
            _art->todFlushCallBack(g, p);
          
          // TOD Request
          else
            _art->todRequestCallBack(g, p);
        }
      }
    }
  }
}

void espArtNetRDM::artTODData(uint8_t g, uint8_t p, uint16_t* uidMan, uint32_t* uidDev, uint16_t uidTotal, uint8_t state) {
  if (_art == nullptr)
    return;

  // Initialise our reply
  uint16_t len = ARTNET_TOD_DATA_SIZE + (6 * uidTotal);
#if defined(ESP32)
  uint8_t* artTodData = (uint8_t*)malloc(len);
#elif defined(ESP8266)
  uint8_t* artTodData = (uint8_t*)os_malloc(len);
#endif
  if (!artTodData)
    return;

  artTodData[0] = 'A';
  artTodData[1] = 'r';
  artTodData[2] = 't';
  artTodData[3] = '-';
  artTodData[4] = 'N';
  artTodData[5] = 'e';
  artTodData[6] = 't';
  artTodData[7] = 0;
  artTodData[8] = (uint8_t)(ARTNET_TOD_DATA & 0x00FF);      // op code lo-hi
  artTodData[9] = (uint8_t)(ARTNET_TOD_DATA >> 8);
  artTodData[10] = 0;
  artTodData[11] = 14;                 // artNet version (14)
  artTodData[12] = 0x01;               // rdm standard Ver 1.0
  artTodData[13] = p + 1;              // port number (1-4 not 0-3)
  artTodData[14] = 0;
  artTodData[15] = 0;
  artTodData[16] = 0;
  artTodData[17] = 0;
  artTodData[18] = 0;
  artTodData[19] = 0;
  artTodData[20] = g + 1;              // bind index
  artTodData[21] = _art->group[g]->netSwitch;

  if (state == RDM_TOD_READY)
    artTodData[22] = 0x00;             // TOD full
  else
    artTodData[22] = 0xFF;             // TOD not avail or incomplete

  artTodData[23] = (_art->group[g]->subnet << 4) | _art->group[g]->ports[p]->portUni;
  artTodData[24] = uidTotal >> 8;      // number of RDM devices found
  artTodData[25] = uidTotal;

  uint8_t blockCount = 0;

  while (1) {
    artTodData[26] = blockCount;
    artTodData[27] = (uidTotal > 200) ? 200 : uidTotal;
    
    uint8_t uidCount = 0;

    // Add RDM UIDs (48 bit each) - max 200 per packet
    for (uint16_t xx = 28; uidCount < 200 && uidTotal > 0; uidCount++) {
      uidTotal--;
      
      artTodData[xx++] = uidMan[uidTotal] >> 8;
      artTodData[xx++] = uidMan[uidTotal];
      artTodData[xx++] = uidDev[uidTotal] >> 24;
      artTodData[xx++] = uidDev[uidTotal] >> 16;
      artTodData[xx++] = uidDev[uidTotal] >> 8;
      artTodData[xx++] = uidDev[uidTotal];
    }

    // Send packet
    eUDP.beginPacket(_art->broadcastIP, ARTNET_PORT);
    eUDP.write((const uint8_t *)artTodData, len);
    eUDP.endPacket();

    if (uidTotal == 0)
      break;

    blockCount++;
  }

  free(artTodData);
}

void espArtNetRDM::_artTODControl(unsigned char *_artBuffer) {
  _artTODRequest(_artBuffer);
}

void espArtNetRDM::_artRDM(unsigned char *_artBuffer, uint16_t packetSize) {
  if (_art->rdmCallBack == 0)
    return;

  IPAddress remoteIp = eUDP.remoteIP();

  uint8_t net = _artBuffer[21] & 0x7F;  // NetSwitch is 7 bits
  uint8_t sub = _artBuffer[23] >> 4;
  uint8_t uni = _artBuffer[23] & 0x0F;

  // Get RDM data into out buffer ready to send
  rdm_data c;
  c.buffer[0] = 0xCC;
  memcpy (&c.buffer[1], &_artBuffer[24], _artBuffer[25] + 2);

  group_def* group = 0;
  unsigned long timeNow = millis();
  
  // Get the group number
  for (int x = 0; x < _art->numGroups; x++) {
    if (net == _art->group[x]->netSwitch && sub == _art->group[x]->subnet) {
      group = _art->group[x];

      // Get the port number
      for (int y = 0; y < 4; y++) {

        // If the port isn't in use
        if (group->ports[y] == 0 || group->ports[y]->portType != RDM_OUT)
          continue;

        // Run callback
        if (uni == group->ports[y]->portUni) {
          _art->rdmCallBack(x, y, &c);

          bool ipSet = false;

          for (int q = 0; q < 5; q++) {
            // Check when last packets where received.  Clear if over 200ms
            if (timeNow >= (group->ports[y]->rdmSenderTime[q] + 200))
              group->ports[y]->rdmSenderIP[q] = INADDR_NONE;
        
            // Save our IP
            if (!ipSet) {
              if (group->ports[y]->rdmSenderIP[q] == INADDR_NONE || group->ports[y]->rdmSenderIP[q] == remoteIp) {
                group->ports[y]->rdmSenderIP[q] = remoteIp;
                group->ports[y]->rdmSenderTime[q] = timeNow;
                ipSet = true;
              }
            }
          }
        }
      }
    }
  }
}

void espArtNetRDM::rdmResponse(rdm_data* c, uint8_t g, uint8_t p) {
  if (_art == nullptr)
    return;

  uint16_t len = ARTNET_RDM_REPLY_SIZE + c->packet.Length + 1;
  // Initialise our reply
#if defined(ESP32)
  uint8_t* rdmReply = (uint8_t*)malloc(len);
#elif defined(ESP8266)
  uint8_t* rdmReply = (uint8_t*)os_malloc(len);
#endif
  if (!rdmReply)
    return;
  
  rdmReply[0] = 'A';
  rdmReply[1] = 'r';
  rdmReply[2] = 't';
  rdmReply[3] = '-';
  rdmReply[4] = 'N';
  rdmReply[5] = 'e';
  rdmReply[6] = 't';
  rdmReply[7] = 0;
  rdmReply[8] = (uint8_t)(ARTNET_RDM & 0x00FF); // op code lo-hi
  rdmReply[9] = (uint8_t)(ARTNET_RDM >> 8);
  rdmReply[10] = 0;
  rdmReply[11] = 14;                 // artNet version (14)
  rdmReply[12] = 0x01;               // RDM version - RDM STANDARD V1.0

  for (uint8_t x = 13; x < 21; x++)
    rdmReply[x] = 0;

  rdmReply[21] = _art->group[g]->netSwitch;
  rdmReply[22] = 0x00;              // Command - 0x00 = Process RDM Packet
  rdmReply[23] = (_art->group[g]->subnet << 4) | _art->group[g]->ports[p]->portUni;

  // Copy everything except the 0xCC start code
  memcpy(&rdmReply[24], &c->buffer[1], c->packet.Length + 1);

  for (int x = 0; x < 5; x++) {
    if (_art->group[g]->ports[p]->rdmSenderIP[x] != INADDR_NONE) {
      // Send packet
      eUDP.beginPacket(_art->group[g]->ports[p]->rdmSenderIP[x], ARTNET_PORT);
      eUDP.write((const uint8_t *)rdmReply, len);
      eUDP.endPacket();
    }
  }

  free(rdmReply);
}

void espArtNetRDM::_artRDMSub(unsigned char *_artBuffer) {
  //Serial.println("artRDMSub");
}

IPAddress espArtNetRDM::getIP() {
  if (_art == nullptr)
    return INADDR_NONE;
  return _art->deviceIP;
}

IPAddress espArtNetRDM::getSubnetMask() {
  if (_art == nullptr)
    return INADDR_NONE;
  return _art->subnet;
}

bool espArtNetRDM::getDHCP() {
  if (_art == nullptr)
    return 0;
  return _art->dhcp;
}

void espArtNetRDM::setIP(IPAddress ip, IPAddress subnet) {
  if (_art == nullptr)
    return;
  _art->deviceIP = ip;
  
  if ( (uint32_t)subnet != 0 )
    _art->subnet = subnet;
  
  _art->broadcastIP = IPAddress((uint32_t)_art->deviceIP | ~((uint32_t)_art->subnet));
}

void espArtNetRDM::setDHCP(bool d) {
  if (_art == nullptr)
    return;
  _art->dhcp = d;
}

void espArtNetRDM::setNet(uint8_t g, uint8_t net) {
  if (_art == nullptr || g >= _art->numGroups)
    return;
  _art->group[g]->netSwitch = net;
}

uint8_t espArtNetRDM:: getNet(uint8_t g) {
  if (_art == nullptr || g >= _art->numGroups)
    return 0;
  return _art->group[g]->netSwitch;
}

void espArtNetRDM::setSubNet(uint8_t g, uint8_t sub) {
  if (_art == nullptr || g >= _art->numGroups)
    return;
  _art->group[g]->subnet = sub;
}

uint8_t espArtNetRDM::getSubNet(uint8_t g) {
  if (_art == nullptr || g >= _art->numGroups)
    return 0;
  return _art->group[g]->subnet;
}

void espArtNetRDM::setUni(uint8_t g, uint8_t p, uint8_t uni) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0)
    return;
  _art->group[g]->ports[p]->portUni = uni;
}

uint8_t espArtNetRDM::getUni(uint8_t g, uint8_t p) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0)
    return 0;
  return _art->group[g]->ports[p]->portUni;
}


void espArtNetRDM:: setPortType(uint8_t g, uint8_t p, uint8_t t) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0)
    return;

  _art->group[g]->ports[p]->portType = t;
}

void espArtNetRDM::setMerge(uint8_t g, uint8_t p, bool htp) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0)
    return;
  _art->group[g]->ports[p]->mergeHTP = htp;
}

bool espArtNetRDM::getMerge(uint8_t g, uint8_t p) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0)
    return 0;
  return _art->group[g]->ports[p]->mergeHTP;
}

void espArtNetRDM::setShortName(const char* name) {
  if (_art == nullptr)
    return;
  memcpy(_art->shortName, name, ARTNET_SHORT_NAME_LENGTH);
}

const char* espArtNetRDM::getShortName() {
  if (_art == nullptr)
    return NULL;
  return _art->shortName;
}

void espArtNetRDM::setLongName(const char* name) {
  if (_art == nullptr)
    return;
  memcpy(_art->longName, name, ARTNET_LONG_NAME_LENGTH);
}

const char* espArtNetRDM::getLongName() {
  if (_art == nullptr)
    return NULL;
  return _art->longName;
}

void espArtNetRDM::setNodeReport(const char* c, uint16_t code) {
  if (_art == nullptr)
    return;

  if (strncmp(_art->nodeReport, c, ARTNET_NODE_REPORT_LENGTH) != 0)
    _art->nodeReportChanged = true;

  strlcpy(_art->nodeReport, c, ARTNET_NODE_REPORT_LENGTH);
  _art->nodeReportCode = code;
}

void espArtNetRDM::sendDMX(uint8_t g, uint8_t p, IPAddress bcAddress, uint8_t* data, uint16_t length) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0)
    return;

  uint8_t net = _art->group[g]->netSwitch;
  uint8_t subnet = _art->group[g]->subnet;
  uint8_t uni = _art->group[g]->ports[p]->portUni;

  // length is always even and up to 512 channels
  if (length % 2)
    length += 1;
  if (length > 512)
    length = 512;

  _art->group[g]->ports[p]->dmxChans = length;

  static uint8_t _artDMX[ARTNET_BUFFER_MAX];
  _artDMX[0] = 'A';
  _artDMX[1] = 'r';
  _artDMX[2] = 't';
  _artDMX[3] = '-';
  _artDMX[4] = 'N';
  _artDMX[5] = 'e';
  _artDMX[6] = 't';
  _artDMX[7] = 0;
  _artDMX[8] = (uint8_t)(ARTNET_ARTDMX & 0x00FF);      	// op code lo-hi
  _artDMX[9] = (uint8_t)(ARTNET_ARTDMX >> 8);	
  _artDMX[10] = 0;  		   	// protocol version (14)
  _artDMX[11] = 14;
  _artDMX[12] = _dmxSeqID++;		// sequence ID
  _artDMX[13] = p;		   	// Port ID (not really necessary)
  _artDMX[14] = (subnet << 4) | uni;	// Subuni
  _artDMX[15] = (net & 0x7F);		// Netswitch
  _artDMX[16] = (length >> 8);		// DMX Data length
  _artDMX[17] = (length & 0xFF);

  for (uint16_t x = 0; x < length; x++)
    _artDMX[18 + x] = data[x];

  // Send packet
  eUDP.beginPacket(bcAddress, ARTNET_PORT);
  eUDP.write((const uint8_t *)_artDMX, (18 + length));
  eUDP.endPacket();
}

void espArtNetRDM::setE131(uint8_t g, uint8_t p, bool a) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0)
    return;

  // Increment or decrement our e131Count variable
  if (!_art->group[g]->ports[p]->e131 && a) {
    e131Count += 1;

    // Clear the DMX output buffer
    artClearDMXBuffer(_art->group[g]->ports[p]->dmxBuffer);

  } else if (_art->group[g]->ports[p]->e131 && !a && e131Count > 0) {
    e131Count -= 1;

    // Clear the DMX output buffer
    artClearDMXBuffer(_art->group[g]->ports[p]->dmxBuffer);
  }

  _art->group[g]->ports[p]->e131 = a;
}

bool espArtNetRDM::getE131(uint8_t g, uint8_t p) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0 || _art->group[g]->ports[p]->e131 == false)
    return false;

  return true;
}

void espArtNetRDM::setE131Uni(uint8_t g, uint8_t p, uint16_t u) {
  if (_art == nullptr || g >= _art->numGroups || _art->group[g]->ports[p] == 0)
    return;

  _art->group[g]->ports[p]->e131Uni = u;
  _art->group[g]->ports[p]->e131Sequence = 0;
  _art->group[g]->ports[p]->e131Priority = 0;
}

void espArtNetRDM::_e131Receive(e131_packet_t* e131Buffer) {
  if (_art == nullptr || _art->numGroups == 0 || e131Count == 0)
    return;

  // Check for sACN packet errors.  Error reporting not implemented -> just dump packet

  if (memcmp(e131Buffer->acn_id, ACN_ID, sizeof(e131Buffer->acn_id)))
    //return ERROR_ACN_ID;
    return;

  if (__builtin_bswap32(e131Buffer->root_vector) != VECTOR_ROOT)
    //return ERROR_VECTOR_ROOT;
    return;

  if (__builtin_bswap32(e131Buffer->frame_vector) != VECTOR_FRAME)
    //return ERROR_VECTOR_FRAME;
    return;

  if (e131Buffer->dmp_vector != VECTOR_DMP)
    //return ERROR_VECTOR_DMP;
    return;

  // No errors -> continue with sACN processing

  uint16_t uni = (e131Buffer->universe << 8) | ((e131Buffer->universe >> 8) & 0xFF);
  uint16_t numberOfChannels = ((e131Buffer->property_value_count << 8) | ((e131Buffer->property_value_count >> 8) & 0xFF)) - 1;
  uint16_t startChannel = (e131Buffer-> first_address << 8) | ((e131Buffer-> first_address >> 8) & 0xFF);
  uint16_t seq = e131Buffer->sequence_number;

  uint8_t _e131Count = 0;

  group_def* group = 0;

  IPAddress rIP = fUDP.remoteIP();

  // Loop through all groups
  for (int x = 0; x < _art->numGroups; x++) {
    group = _art->group[x];

    // Loop through each port
    for (int y = 0; y < 4; y++) {
      if (group->ports[y] == 0 || group->ports[y]->portType == DMX_IN || !group->ports[y]->e131)
        continue;
      
      // If this port has the correct Uni, is a later packet, and is of a valid priority -> save DMX to buffer
      if (uni == group->ports[y]->e131Uni && seq > group->ports[y]->e131Sequence && e131Buffer->priority >= group->ports[y]->e131Priority) {
        // Drop non-zero start packets
        if (e131Buffer->property_values[0] != 0)
          continue;

        // A higher priority will override previous data - this is handled in saveDMX but we need to clear the IPs & buffer
        if (e131Buffer->priority > group->ports[y]->e131Priority) {
          artClearDMXBuffer(group->ports[y]->dmxBuffer);
          group->ports[y]->senderIP[0] = INADDR_NONE;
          group->ports[y]->senderIP[1] = INADDR_NONE;
        }

        group->ports[y]->e131Priority = e131Buffer->priority;

        _saveDMX(&e131Buffer->property_values[1], numberOfChannels, x, y, rIP, startChannel);
      }

      // If all the e131 ports are checked, then return
      if (e131Count == ++_e131Count)
        return;
    }
  }
}

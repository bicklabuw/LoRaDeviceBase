#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"
#include "../ece-707-lora-common/PacketConfig.h"

// --- HARDWARE ---
HT_st7735 st7735;

// --- BFS VARIABLES ---
#define MAX_NODES 20
uint8_t searchQueue[MAX_NODES];
int queueHead = 0;
int queueTail = 0;
bool visited[255] = {false}; // Track IDs we have already processed
uint8_t discoveredNodes[MAX_NODES];
int discoveredCount = 0;

// --- STATE MACHINE ---
typedef enum { 
  STATE_IDLE, 
  STATE_DISCOVER_NEIGHBORS, 
  STATE_TESTING_LINK, 
  STATE_PROCESS_QUEUE, 
  STATE_WAIT_REMOTE_SEARCH 
} BaseState_t;

BaseState_t currentState = STATE_IDLE;

// --- SF TEST VARIABLES (From your code) ---
int currentTestSF = 12;
int testPacketsSent = 0;
int testPacketsAcked = 0;
#define PACKETS_PER_SF 5 // Reduced for speed, increase to 10 if needed
uint8_t currentTargetNode = 0;

// --- RX BUFFERS ---
uint8_t rxBuffer[255];
uint8_t rxSize;
int16_t rxRssi;
bool packetReceived = false;

// --- HELPER FUNCTIONS ---

void ScreenUpdate(const char* line1, const char* line2, uint16_t color) {
  st7735.st7735_fill_screen(ST7735_BLACK);
  st7735.st7735_write_str(0, 0, "BASE STATION", Font_7x10, ST7735_CYAN);
  st7735.st7735_write_str(0, 20, (char*)line1, Font_7x10, color);
  st7735.st7735_write_str(0, 40, (char*)line2, Font_7x10, color);
  
  // Draw Discovered List
  char buf[30];
  sprintf(buf, "Found: %d Nodes", discoveredCount);
  st7735.st7735_write_str(0, 60, buf, Font_7x10, ST7735_GREEN);
  
  int y = 80;
  for(int i=0; i<discoveredCount && i<5; i++) {
     sprintf(buf, "ID:%d", discoveredNodes[i]);
     st7735.st7735_write_str((i%3)*40, y + (i/3)*15, buf, Font_7x10, ST7735_WHITE);
  }
}

void SendPacket(uint8_t dest, uint8_t type, uint8_t d1, uint8_t d2) {
  uint8_t packet[10];
  packet[0] = dest;
  packet[1] = BASE_STATION_ID;
  packet[2] = type;
  packet[3] = d1;
  packet[4] = d2;
  
  Radio.Send(packet, 5);
}

void ConfigRadioSF(int sf) {
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 8, false, true, 0, 0, false, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0, 8, 0, false, 0, true, 0, 0, false, true);
}

// --- QUEUE MANAGEMENT ---
void enqueue(uint8_t id) {
  if (visited[id]) return; // Already seen
  visited[id] = true;
  
  // Add to BFS Queue
  if (queueTail < MAX_NODES) {
    searchQueue[queueTail++] = id;
  }
  
  // Add to Display list
  if (discoveredCount < MAX_NODES) {
    discoveredNodes[discoveredCount++] = id;
  }
}

// --- INTERRUPTS ---
void OnTxDone() { Radio.Rx(RX_TIMEOUT_VALUE); }
void OnTxTimeout() { Radio.Rx(RX_TIMEOUT_VALUE); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxBuffer, payload, size);
  rxSize = size;
  rxRssi = rssi;
  packetReceived = true;
}

void setup() {
  Serial.begin(115200);
  Mcu.begin();
  st7735.st7735_init();
  ScreenUpdate("Initializing...", "BFS Logic", ST7735_WHITE);
  
  // Radio Init
  RadioEvents_t RadioEvents;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  ConfigRadioSF(12); // Default Listener
  
  visited[BASE_STATION_ID] = true; // Mark self as visited
  delay(1000);
  
  // Start the Process
  currentState = STATE_DISCOVER_NEIGHBORS;
}

void loop() {
  // BFS LOGIC
  switch(currentState) {
    
    // 1. Base Station searches for immediate neighbors first
    case STATE_DISCOVER_NEIGHBORS:
      ScreenUpdate("Scanning...", "Sending Discovery", ST7735_YELLOW);
      ConfigRadioSF(12); // Always discover on SF12
      SendPacket(BROADCAST_ADDR, PKT_DISCOVERY_REQ, 0, 0);
      
      // Listen for responses for a set time
      {
        unsigned long startWait = millis();
        packetReceived = false;
        Radio.Rx(RX_TIMEOUT_VALUE);
        
        while(millis() - startWait < DISCOVERY_TIMEOUT) {
          Radio.IrqProcess();
          if (packetReceived) {
            packetReceived = false;
            // Check if it's a Discovery Response directed to me
            if (rxBuffer[0] == BASE_STATION_ID && rxBuffer[2] == PKT_DISCOVERY_RESP) {
              uint8_t foundID = rxBuffer[1];
              if (!visited[foundID]) {
                // We found a new direct neighbor.
                // We must test it before adding to queue?
                // Logic: Add to temporary "To Test" list or just test immediately.
                // For simplicity: Stop scanning, go test this specific node, then come back.
                currentTargetNode = foundID;
                currentState = STATE_TESTING_LINK;
                return; // Break loop to transition
              }
            }
            Radio.Rx(RX_TIMEOUT_VALUE); // Keep listening
          }
        }
        // If timeout and no new nodes found, move to processing the queue
        currentState = STATE_PROCESS_QUEUE;
      }
      break;

    // 2. Perform SF Test with a specific Node (Reuse your logic)
    case STATE_TESTING_LINK:
      {
        ScreenUpdate("Testing Link", "SF Loop...", ST7735_MAGENTA);
        // Logic: Send packets from SF12 down to SF7
        // Simple synchronous implementation for robustness
        int bestSF = 0;
        
        for (int sf = 12; sf >= 7; sf--) {
           ConfigRadioSF(sf);
           int ackCount = 0;
           for(int k=0; k<PACKETS_PER_SF; k++) {
              SendPacket(currentTargetNode, PKT_SF_TEST, sf, k);
              // Wait for ACK
              unsigned long waitAck = millis();
              bool acked = false;
              packetReceived = false;
              Radio.Rx(500); // Short timeout for ACK
              while(millis() - waitAck < 500) {
                 Radio.IrqProcess();
                 if(packetReceived) {
                    if(rxBuffer[1] == currentTargetNode && rxBuffer[2] == PKT_SF_ACK) {
                       acked = true;
                    }
                    packetReceived = false;
                 }
                 if(acked) break;
              }
              if(acked) ackCount++;
              delay(100);
           }
           if (ackCount > 0) bestSF = sf; // Update best SF if we got at least one packet through
        }
        
        // Test Done. Add to BFS Queue.
        enqueue(currentTargetNode);
        ConfigRadioSF(12); // Return to base config
        currentState = STATE_DISCOVER_NEIGHBORS; // Go back to scan for more direct neighbors
      }
      break;

    // 3. Process BFS Queue (Delegate Search)
    case STATE_PROCESS_QUEUE:
      if (queueHead < queueTail) {
         // Get next node to command
         uint8_t commanderID = searchQueue[queueHead++];
         
         // If it's the base station (unlikely in queue, but safety check), skip
         if(commanderID == BASE_STATION_ID) { 
           currentState = STATE_PROCESS_QUEUE; 
           return; 
         }

         ScreenUpdate("Remote Search", "Commanding Node...", ST7735_ORANGE);
         
         // Command the node to search
         ConfigRadioSF(12);
         SendPacket(commanderID, PKT_CMD_SEARCH, 0, 0);
         
         currentState = STATE_WAIT_REMOTE_SEARCH;
      } else {
         ScreenUpdate("Network Mapped", "Idle", ST7735_GREEN);
         // Done. Just listen or restart periodically.
         delay(5000);
      }
      break;

    // 4. Wait for Remote Node to report back
    case STATE_WAIT_REMOTE_SEARCH:
      Radio.Rx(RX_TIMEOUT_VALUE);
      unsigned long waitStart = millis();
      packetReceived = false;
      
      // Wait indefinitely? Or Timeout? 
      // Diagram says: "Forward nodes and link capacities".
      // We wait until we get a "SEARCH DONE" packet or timeout.
      
      while(millis() - waitStart < 15000) { // 15 sec timeout for remote search
         Radio.IrqProcess();
         if(packetReceived) {
            packetReceived = false;
            if (rxBuffer[0] == BASE_STATION_ID) {
               if (rxBuffer[2] == PKT_REPORT_NODE) {
                  uint8_t foundID = rxBuffer[3];
                  // If this is a new node, we need to add it to the queue!
                  if (!visited[foundID]) {
                     enqueue(foundID); // Add to back of line
                     Serial.printf("Remote Node %d found Node %d\n", rxBuffer[1], foundID);
                  }
               }
               else if (rxBuffer[2] == PKT_SEARCH_DONE) {
                  // Node finished searching
                  currentState = STATE_PROCESS_QUEUE; // Move to next in queue
                  return;
               }
            }
            Radio.Rx(RX_TIMEOUT_VALUE);
         }
      }
      // Timeout
      currentState = STATE_PROCESS_QUEUE;
      break;
  }
}
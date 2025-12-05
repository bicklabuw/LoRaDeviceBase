/*
 * BaseNode.ino - BFS Discovery + RELIABILITY Link Test (Batch Testing)
 * UPDATE: Sends 5 packets per SF to test for packet loss/stability.
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"
#include "../ece-707-lora-common/PacketConfig.h"

// --- CONFIGURATION ---
#define NUM_TEST_PACKETS 5   // How many packets to send per SF level
#define PASS_THRESHOLD   4   // Minimum ACKs required to consider this SF "Reliable"

// --- HARDWARE ---
HT_st7735 st7735;

// --- NETWORK VARIABLES ---
#define MAX_NODES 20
uint8_t searchQueue[MAX_NODES];
uint8_t discoveredNodes[MAX_NODES];
int queueHead = 0;
int queueTail = 0;
int discoveredCount = 0;
bool visited[255] = {false};

// --- LINK TEST VARIABLES ---
int testNodeIndex = 0;   
int currentTestSF = 12;  
int bestSF[MAX_NODES];   
int packetsSent = 0;     // Counter for current batch
int acksReceived = 0;    // Counter for successful replies

// --- RADIO VARIABLES ---
volatile bool txDone = false;
volatile bool packetReceived = false;
uint8_t rxBuffer[255];
uint8_t rxSize = 0;
int16_t lastRssi = 0;

// --- STATE MACHINE ---
typedef enum {
    S_INIT,
    // --- DISCOVERY ---
    S_START_LOCAL_SCAN,
    S_WAIT_DISCOVERY_RESP,
    S_PROCESS_QUEUE,
    S_SEND_REMOTE_SCAN,
    S_WAIT_REMOTE_REPORT,
    
    // --- LINK TEST (BATCH) ---
    S_START_LINK_TEST,
    S_TEST_NEXT_NODE,
    S_TEST_INIT_SF,       // Reset counters for new SF
    S_TEST_SEND_CMD,      // Send "Switch to SFx" using SF12
    S_TEST_SWITCH_RX,     // Switch local Radio to SFx
    S_TEST_WAIT_PONG,     // Wait for echo
    S_TEST_EVALUATE,      // Check if batch passed/failed
    
    S_COMPLETE,
    S_IDLE
} State_t;

State_t currentState = S_INIT;
unsigned long stateStartTime = 0;
uint8_t currentTargetNode = 0;

// --- HELPERS ---
bool isNodeKnown(uint8_t id) {
    if (id == 0) return true;
    for(int i=0; i<discoveredCount; i++) if (discoveredNodes[i] == id) return true;
    return false;
}

void addToQueue(uint8_t id) {
    if (!visited[id] && queueTail < MAX_NODES) {
        searchQueue[queueTail++] = id;
        visited[id] = true;
        discoveredNodes[discoveredCount] = id;
        bestSF[discoveredCount] = 0; 
        discoveredCount++;
        
        char buf[20];
        sprintf(buf, "ID:%d (RSSI:%d)", id, lastRssi);
        st7735.st7735_write_str(0, 60 + (discoveredCount*10), buf, Font_7x10, ST7735_GREEN);
    }
}

// Helper to set Radio Config
void ConfigRadio(int sf) {
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE,
                      LORA_PREAMBLE_LEN, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0, LORA_PREAMBLE_LEN,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Radio.Rx(0);
}

void SendPacket(uint8_t target, uint8_t type, uint8_t payload) {
    uint8_t txPacket[5] = {target, 0, type, payload, 0};
    txDone = false;
    Radio.Send(txPacket, 5);
}

// --- INTERRUPTS ---
void OnTxDone(void) { 
    txDone = true; 
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    if (size > sizeof(rxBuffer)) size = sizeof(rxBuffer);
    memcpy(rxBuffer, payload, size);
    rxSize = size;
    lastRssi = rssi;
    packetReceived = true;
}
void OnTxTimeout(void) { txDone = true; } 
void OnRxTimeout(void) { }
void OnRxError(void) { }

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, 0); 
    
    st7735.st7735_init();
    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, "BASE: DISCOVERY", Font_7x10, ST7735_CYAN);

    static RadioEvents_t RadioEvents;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    
    ConfigRadio(12); 
    currentState = S_START_LOCAL_SCAN;
}

void loop() {
    Radio.IrqProcess(); 

    switch (currentState) {
        // ================= DISCOVERY PHASE =================
        case S_START_LOCAL_SCAN:
            st7735.st7735_write_str(0, 20, "Scanning Local...", Font_7x10, ST7735_YELLOW);
            ConfigRadio(12); // Ensure SF12
            SendPacket(0xFF, PKT_DISCOVERY_REQ, 0);
            
            // Wait for TX
            { unsigned long t = millis(); while(!txDone && millis()-t < 1000) Radio.IrqProcess(); }
            Radio.Rx(0);
            
            stateStartTime = millis();
            currentState = S_WAIT_DISCOVERY_RESP;
            break;

        case S_WAIT_DISCOVERY_RESP:
            if (packetReceived) {
                packetReceived = false;
                if (rxBuffer[2] == PKT_DISCOVERY_RESP) {
                    addToQueue(rxBuffer[1]);
                }
            }
            if (millis() - stateStartTime > DISCOVERY_TIMEOUT) {
                currentState = S_PROCESS_QUEUE;
            }
            break;

        case S_PROCESS_QUEUE:
            if (queueHead < queueTail) {
                currentTargetNode = searchQueue[queueHead++];
                currentState = S_SEND_REMOTE_SCAN;
            } else {
                currentState = S_START_LINK_TEST;
            }
            break;

        case S_SEND_REMOTE_SCAN:
            {
                char msg[30];
                sprintf(msg, "Remote Scan: %d", currentTargetNode);
                st7735.st7735_write_str(0, 20, msg, Font_7x10, ST7735_MAGENTA);
                
                ConfigRadio(12);
                SendPacket(currentTargetNode, PKT_SCAN_CMD, 0);
                
                unsigned long t = millis(); while(!txDone && millis()-t < 1000) Radio.IrqProcess();
                Radio.Rx(0);

                stateStartTime = millis();
                currentState = S_WAIT_REMOTE_REPORT;
            }
            break;

        case S_WAIT_REMOTE_REPORT:
            if (packetReceived) {
                packetReceived = false;
                if (rxBuffer[1] == currentTargetNode) {
                    if (rxBuffer[2] == PKT_REPORT_NODE) addToQueue(rxBuffer[3]);
                    if (rxBuffer[2] == PKT_SCAN_DONE) currentState = S_PROCESS_QUEUE;
                }
            }
            if (millis() - stateStartTime > DISCOVERY_TIMEOUT * 2) {
                currentState = S_PROCESS_QUEUE; 
            }
            break;

        // ================= RELIABILITY TEST PHASE =================
        case S_START_LINK_TEST:
            st7735.st7735_fill_screen(ST7735_BLACK);
            st7735.st7735_write_str(0, 0, "BASE: LINK TEST", Font_7x10, ST7735_CYAN);
            testNodeIndex = 0;
            if (discoveredCount == 0) currentState = S_COMPLETE;
            else currentState = S_TEST_NEXT_NODE;
            break;

        case S_TEST_NEXT_NODE:
            if (testNodeIndex >= discoveredCount) {
                currentState = S_COMPLETE;
            } else {
                currentTestSF = 12; // Start testing at SF12
                currentTargetNode = discoveredNodes[testNodeIndex];
                char msg[30];
                sprintf(msg, "Node %d...", currentTargetNode);
                st7735.st7735_write_str(0, 20, msg, Font_7x10, ST7735_WHITE);
                currentState = S_TEST_INIT_SF;
            }
            break;

        case S_TEST_INIT_SF:
            // Prepare for a batch of 5 packets at this SF
            packetsSent = 0;
            acksReceived = 0;
            currentState = S_TEST_SEND_CMD;
            break;

        case S_TEST_SEND_CMD:
            if (packetsSent >= NUM_TEST_PACKETS) {
                currentState = S_TEST_EVALUATE;
                break;
            }

            // 1. Configure to SF12 (Control Channel)
            ConfigRadio(12);
            delay(10); 

            char buf[30];
            sprintf(buf, "SF%d Pkt:%d/%d OK:%d", currentTestSF, packetsSent+1, NUM_TEST_PACKETS, acksReceived);
            st7735.st7735_write_str(0, 40, buf, Font_7x10, ST7735_YELLOW);
            
            // 2. Send Command: "Please reply on currentTestSF"
            SendPacket(currentTargetNode, PKT_SF_TEST, currentTestSF);
            
            // 3. WAIT FOR TX
            { unsigned long t = millis(); while(!txDone && millis()-t < 1000) Radio.IrqProcess(); }
            
            currentState = S_TEST_SWITCH_RX;
            break;

        case S_TEST_SWITCH_RX:
            // 4. Switch to Test SF
            ConfigRadio(currentTestSF);
            stateStartTime = millis();
            currentState = S_TEST_WAIT_PONG;
            break;

        case S_TEST_WAIT_PONG:
            if (packetReceived) {
                packetReceived = false;
                if (rxBuffer[1] == currentTargetNode && rxBuffer[2] == PKT_SF_TEST) {
                    acksReceived++;
                    packetsSent++;
                    delay(100); // Small gap between packets
                    currentState = S_TEST_SEND_CMD; // Next packet
                    return;
                }
            }
            
            // Timeout
            if (millis() - stateStartTime > 1500) { 
                packetsSent++; // Count as lost
                currentState = S_TEST_SEND_CMD; // Try next packet
            }
            break;

        case S_TEST_EVALUATE:
            // Batch complete. Did we pass?
            if (acksReceived >= PASS_THRESHOLD) {
                // This SF is reliable!
                bestSF[testNodeIndex] = currentTestSF; 
                
                // Can we go faster?
                if (currentTestSF > 7) {
                    currentTestSF--;
                    currentState = S_TEST_INIT_SF; // Start batch for next SF
                } else {
                    // Reached max speed (SF7)
                    testNodeIndex++;
                    currentState = S_TEST_NEXT_NODE;
                }
            } else {
                // Failed reliability test at this SF.
                // The PREVIOUS SF (recorded in bestSF) was the last good one.
                testNodeIndex++;
                currentState = S_TEST_NEXT_NODE;
            }
            break;

        case S_COMPLETE:
            ConfigRadio(12);
            st7735.st7735_fill_screen(ST7735_BLACK);
            st7735.st7735_write_str(0, 0, "RELIABILITY RESULT:", Font_7x10, ST7735_GREEN);
            
            for(int i=0; i<discoveredCount; i++) {
                char buf[30];
                sprintf(buf, "ID:%d BestSF:%d", discoveredNodes[i], bestSF[i]);
                st7735.st7735_write_str(0, 20 + (i*15), buf, Font_7x10, ST7735_WHITE);
            }
            currentState = S_IDLE;
            break;

        case S_IDLE:
            break;
    }
}
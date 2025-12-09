/*
 * BaseNode.ino - Master Controller
 * Features: Dynamic Discovery Timing, Link Capacity Testing, Wi-Fi Upload
 */
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_st7735.h"
#include "../ece-707-lora-common/PacketConfig.h"

// --- WIFI CONFIG ---
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char *ssid = "IanPhone";
const char *password = "theopen20";
const char *serverUrl = "http://172.20.10.3:8000/optimize";

// --- TEST CONFIG ---
uint8_t cfg_testPackets = 5;
uint8_t cfg_threshold = 4;

HT_st7735 st7735;

// --- NETWORK DATA ---
#define MAX_NODES 50
uint8_t searchQueue[MAX_NODES];
uint8_t discoveredNodes[MAX_NODES];
int queueHead = 0;
int queueTail = 0;
int discoveredCount = 0;
bool visited[255] = {false};

struct LinkStats
{
    uint8_t src;
    uint8_t dst;
    int successCount[6];
    int bestSF;
    long capacity;
};
LinkStats links[MAX_NODES * 5];
int linkCount = 0;

// --- STATE ---
uint8_t sessionID = 0;
int discoveryRound = 0;
int testNodeIndex = 0;
int currentTestSF = 12;
int packetsSent = 0;
int acksReceived = 0;
int localTestResults[6];

// --- RADIO ---
volatile bool txDone = false;
volatile bool packetReceived = false;
uint8_t rxBuffer[255];
uint8_t currentTargetNode = 0;

typedef enum
{
    S_INIT,
    S_DISCOVERY_START,
    S_DISCOVERY_WAIT,
    S_DISCOVERY_NEXT_ROUND,
    S_TEST_START,
    S_TEST_NEXT_NODE,
    S_TEST_INIT_SF,
    S_TEST_SEND_CMD,
    S_TEST_SWITCH_RX,
    S_TEST_WAIT_PONG,
    S_TEST_EVALUATE_SF,
    S_TEST_FINALIZE_NODE,
    S_PROCESS_QUEUE,
    S_REMOTE_CMD,
    S_REMOTE_WAIT,
    S_COMPLETE,
    S_IDLE
} State_t;
State_t currentState = S_INIT;
unsigned long stateStartTime = 0;

// --- PROTOTYPES ---
void sendTopologyToPC();

// --- HELPERS ---
bool isNodeKnown(uint8_t id)
{
    if (id == 0)
        return true;
    for (int i = 0; i < discoveredCount; i++)
        if (discoveredNodes[i] == id)
            return true;
    return false;
}

void addLink(uint8_t src, uint8_t dst, int *successes)
{
    if (linkCount < 50)
    {
        links[linkCount].src = src;
        links[linkCount].dst = dst;
        int bestSF = 0;
        float successRatio = 0;
        for (int i = 0; i < 6; i++)
        {
            links[linkCount].successCount[i] = successes[i];
            if (successes[i] >= cfg_threshold && bestSF == 0)
            {
                bestSF = 7 + i;
                successRatio = (float)successes[i] / cfg_testPackets;
            }
        }
        links[linkCount].bestSF = bestSF;
        if (bestSF > 0)
            links[linkCount].capacity = (long)(DATA_RATES[bestSF - 7] * successRatio);
        else
            links[linkCount].capacity = 0;

        // Original Color: Cyan for Links
        char buf[30];
        sprintf(buf, "%d->%d Cap:%ld", src, dst, links[linkCount].capacity);
        st7735.st7735_write_str(0, 100, buf, Font_7x10, ST7735_CYAN);

        Serial.printf("LINK: %d->%d | BestSF:%d | Cap:%ld bps\n", src, dst, bestSF, links[linkCount].capacity);
        linkCount++;
    }
}

void addToQueue(uint8_t id)
{
    if (!visited[id] && id != 0)
    {
        visited[id] = true;
        if (!isNodeKnown(id))
        {
            discoveredNodes[discoveredCount++] = id;
            // Original Color: Yellow for found list
            char buf[20];
            sprintf(buf, "Found: %d", id);
            st7735.st7735_write_str(0, 20 + (discoveredCount * 10), buf, Font_7x10, ST7735_YELLOW);
        }
        if (queueTail < MAX_NODES)
            searchQueue[queueTail++] = id;
    }
}

void ConfigRadio(int sf)
{
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE,
                      LORA_PREAMBLE_LEN, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0, LORA_PREAMBLE_LEN,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Radio.Rx(0);
}

void SendDiscoveryReq()
{
    uint8_t size = 7 + discoveredCount;
    if (size > 250)
        size = 250;

    uint8_t tx[255];
    tx[0] = 0xFF;
    tx[1] = 0;
    tx[2] = PKT_DISCOVERY_REQ;
    tx[3] = sessionID;

    // INJECT DYNAMIC TIME
    uint16_t window = DISCOVERY_WINDOW_MS;
    tx[4] = (uint8_t)(window >> 8);
    tx[5] = (uint8_t)(window & 0xFF);

    tx[6] = discoveredCount;
    for (int i = 0; i < discoveredCount; i++)
        if ((7 + i) < 250)
            tx[7 + i] = discoveredNodes[i];

    Serial.printf("[TX] Discovery Req (Window: %d ms)\n", window);
    txDone = false;
    Radio.Send(tx, size);
}

void SendPacket(uint8_t target, uint8_t type, uint8_t payload, uint8_t extra = 0)
{
    uint8_t tx[6] = {target, 0, type, payload, extra, 0};
    txDone = false;
    Radio.Send(tx, 6);
}

void OnTxDone(void) { txDone = true; }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    if (size > sizeof(rxBuffer))
        size = sizeof(rxBuffer);
    memcpy(rxBuffer, payload, size);
    packetReceived = true;
}
void OnTxTimeout(void) { txDone = true; }
void OnRxTimeout(void) {}
void OnRxError(void) {}

void setup()
{
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, 0);
    st7735.st7735_init();
    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, "BASE STATION", Font_7x10, ST7735_GREEN);

    static RadioEvents_t RadioEvents;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

    randomSeed(analogRead(0));
    sessionID = (uint8_t)random(1, 255);
    ConfigRadio(12);
    currentState = S_DISCOVERY_START;
}

void loop()
{
    Radio.IrqProcess();
    switch (currentState)
    {
    case S_DISCOVERY_START:
    {
        st7735.st7735_fill_screen(ST7735_BLACK);
        st7735.st7735_write_str(0, 0, "DISCOVERY MODE", Font_7x10, ST7735_BLUE);
        char buf[20];
        sprintf(buf, "Scan (%d/%d)", discoveryRound + 1, DISCOVERY_ROUNDS);
        st7735.st7735_write_str(0, 15, buf, Font_7x10, ST7735_YELLOW); // Original Yellow

        SendDiscoveryReq();
        unsigned long t = millis();
        while (!txDone && millis() - t < 1000)
            Radio.IrqProcess();
        Radio.Rx(0);
        stateStartTime = millis();
        currentState = S_DISCOVERY_WAIT;
    }
    break;
    case S_DISCOVERY_WAIT:
        if (packetReceived)
        {
            packetReceived = false;
            if (rxBuffer[2] == PKT_DISCOVERY_RESP)
            {
                uint8_t id = rxBuffer[1];
                if (!isNodeKnown(id))
                {
                    Serial.printf("[DISCOVERY] New Reply from Node %d\n", id);
                    addToQueue(id); // Colors handled in addToQueue
                }
            }
        }
        if (millis() - stateStartTime > DISCOVERY_WINDOW_MS)
        {
            currentState = S_DISCOVERY_NEXT_ROUND;
        }
        break;
    case S_DISCOVERY_NEXT_ROUND:
        discoveryRound++;
        if (discoveryRound < DISCOVERY_ROUNDS)
            currentState = S_DISCOVERY_START;
        else
            currentState = S_TEST_START;
        break;
    case S_TEST_START:
        Serial.println("[STATE] Link Sweep");
        st7735.st7735_write_str(0, 0, "LINK ANALYSIS", Font_7x10, ST7735_MAGENTA); // Original Magenta
        testNodeIndex = 0;
        if (discoveredCount == 0)
            currentState = S_COMPLETE;
        else
            currentState = S_TEST_NEXT_NODE;
        break;
    case S_TEST_NEXT_NODE:
        if (testNodeIndex >= discoveredCount)
            currentState = S_PROCESS_QUEUE;
        else
        {
            currentTargetNode = discoveredNodes[testNodeIndex];
            char msg[30];
            sprintf(msg, "Testing ID:%d   ", currentTargetNode);
            st7735.st7735_write_str(0, 20, msg, Font_7x10, ST7735_WHITE);
            for (int i = 0; i < 6; i++)
                localTestResults[i] = 0;
            currentTestSF = 12;
            currentState = S_TEST_INIT_SF;
        }
        break;
    case S_TEST_INIT_SF:
        packetsSent = 0;
        acksReceived = 0;
        currentState = S_TEST_SEND_CMD;
        break;
    case S_TEST_SEND_CMD:
        if (packetsSent >= cfg_testPackets)
        {
            currentState = S_TEST_EVALUATE_SF;
            break;
        }
        ConfigRadio(12);
        delay(10);
        char buf[30];
        sprintf(buf, "SF%d: %d/%d", currentTestSF, packetsSent + 1, cfg_testPackets);
        st7735.st7735_write_str(0, 40, buf, Font_7x10, ST7735_YELLOW);

        SendPacket(currentTargetNode, PKT_SF_TEST, currentTestSF);
        {
            unsigned long t = millis();
            while (!txDone && millis() - t < 1000)
                Radio.IrqProcess();
        }
        currentState = S_TEST_SWITCH_RX;
        break;
    case S_TEST_SWITCH_RX:
        ConfigRadio(currentTestSF);
        stateStartTime = millis();
        currentState = S_TEST_WAIT_PONG;
        break;
    case S_TEST_WAIT_PONG:
        if (packetReceived)
        {
            packetReceived = false;
            if (rxBuffer[1] == currentTargetNode && rxBuffer[2] == PKT_SF_TEST)
            {
                acksReceived++;
                packetsSent++;
                delay(50);
                currentState = S_TEST_SEND_CMD;
                return;
            }
        }
        if (millis() - stateStartTime > 1500)
        {
            packetsSent++;
            currentState = S_TEST_SEND_CMD;
        }
        break;
    case S_TEST_EVALUATE_SF:
        localTestResults[currentTestSF - 7] = acksReceived;
        if (currentTestSF > 7)
        {
            currentTestSF--;
            currentState = S_TEST_INIT_SF;
        }
        else
        {
            currentState = S_TEST_FINALIZE_NODE;
        }
        break;
    case S_TEST_FINALIZE_NODE:
        addLink(0, currentTargetNode, localTestResults);
        addLink(currentTargetNode, 0, localTestResults); // hardcoding reverse link with bs
        testNodeIndex++;
        currentState = S_TEST_NEXT_NODE;
        break;
    case S_PROCESS_QUEUE:
        if (queueHead < queueTail)
        {
            currentTargetNode = searchQueue[queueHead++];
            currentState = S_REMOTE_CMD;
        }
        else
            currentState = S_COMPLETE;
        break;
    case S_REMOTE_CMD:
    {
        st7735.st7735_write_str(0, 60, "Remote Scan... ", Font_7x10, ST7735_CYAN); // Original Cyan
        char msg[30];
        sprintf(msg, "Job -> ID:%d    ", currentTargetNode);
        st7735.st7735_write_str(0, 75, msg, Font_7x10, ST7735_WHITE);
        ConfigRadio(12);
        SendPacket(currentTargetNode, PKT_SCAN_CMD, cfg_testPackets, cfg_threshold);
        {
            unsigned long t = millis();
            while (!txDone && millis() - t < 1000)
                Radio.IrqProcess();
        }
        Radio.Rx(0);
        stateStartTime = millis();
        currentState = S_REMOTE_WAIT;
    }
    break;
    case S_REMOTE_WAIT:
        if (packetReceived)
        {
            packetReceived = false;
            if (rxBuffer[1] == currentTargetNode)
            {
                if (rxBuffer[2] == PKT_REPORT_NODE)
                {
                    uint8_t foundID = rxBuffer[3];
                    int remoteStats[6];
                    for (int i = 0; i < 6; i++)
                        remoteStats[i] = rxBuffer[4 + i];
                    addLink(currentTargetNode, foundID, remoteStats);
                    addToQueue(foundID);
                    stateStartTime = millis();
                }
                if (rxBuffer[2] == PKT_SCAN_DONE)
                    currentState = S_PROCESS_QUEUE;
            }
        }
        if (millis() - stateStartTime > REMOTE_OP_TIMEOUT)
        {
            st7735.st7735_write_str(0, 60, "Node Timeout!    ", Font_7x10, ST7735_RED);
            delay(1000);
            currentState = S_PROCESS_QUEUE;
        }
        break;
    case S_COMPLETE:
        st7735.st7735_fill_screen(ST7735_BLACK);
        st7735.st7735_write_str(0, 0, "CAPACITY MAP", Font_7x10, ST7735_GREEN);
        if (linkCount == 0)
            st7735.st7735_write_str(0, 20, "No Links", Font_7x10, ST7735_RED);
        else
        {
            for (int i = 0; i < linkCount; i++)
            {
                Serial.printf("LINK: %d->%d | BestSF:%d | Cap:%ld bps\n",
                              links[i].src, links[i].dst, links[i].bestSF, links[i].capacity);
                if (i > 14)
                    continue;
                char buf[30];
                sprintf(buf, "%d:%d %ldbps", links[i].src, links[i].dst, links[i].capacity);
                st7735.st7735_write_str(0, 20 + (i * 10), buf, Font_7x10, ST7735_WHITE);
            }
        }
        sendTopologyToPC();
        currentState = S_IDLE;
        break;
    case S_IDLE:
        break;
    }
}

void sendTopologyToPC() {
    // 1. Check if we have data to send
    if (linkCount == 0 && discoveredCount == 0) return;
    
    // 2. Connect to Wi-Fi (Robust Method)
    st7735.st7735_write_str(0, 140, "Wifi...", Font_7x10, ST7735_WHITE);
    Serial.printf("\n[WIFI] Connecting to %s", ssid);
    
    WiFi.mode(WIFI_STA); // Force Station Mode
    WiFi.begin(ssid, password);
    
    int retries = 0;
    while(WiFi.status() != WL_CONNECTED && retries < 40) { 
        delay(500); 
        Serial.print("."); 
        retries++; 
    }
    
    if(WiFi.status() != WL_CONNECTED) {
        Serial.print("\n[WIFI] Fail. Err: ");
        Serial.println(WiFi.status());
        st7735.st7735_write_str(0, 140, "Wifi Fail ", Font_7x10, ST7735_RED);
        return;
    }
    Serial.println("\n[WIFI] Connected!");
    st7735.st7735_write_str(0, 140, "Sending...", Font_7x10, ST7735_YELLOW);

    HTTPClient http; 
    http.begin(serverUrl); 
    http.addHeader("Content-Type", "application/json");

    // 3. Build JSON
    JsonDocument doc; 
    
    // Config
    JsonObject config = doc["config"].to<JsonObject>();
    config["num_channels"] = 1; 
    config["frequency_hz"] = RF_FREQUENCY; 
    config["tx_power_dbm"] = TX_OUTPUT_POWER;
    
    // Nodes
    JsonArray nodesArr = doc["nodes"].to<JsonArray>();
    
    // Add Base Station
    JsonObject baseNode = nodesArr.add<JsonObject>();
    baseNode["id"] = 0; 
    baseNode["name"] = "BaseStation"; 
    baseNode["type"] = "BASE_STATION"; 
    baseNode["lat"] = 0.0; baseNode["lon"] = 0.0;
    
    // Add Discovered Nodes
    for(int i=0; i<discoveredCount; i++) {
        JsonObject n = nodesArr.add<JsonObject>();
        n["id"] = discoveredNodes[i];
        n["name"] = "Node_" + String(discoveredNodes[i]);
        // CHANGE: Label them GENERATOR so the solver assigns them bandwidth!
        n["type"] = "GENERATOR"; 
        n["lat"] = 0.0; n["lon"] = 0.0;
    }
    
    // Links
    JsonArray linksArr = doc["link_overrides"].to<JsonArray>();
    for(int i=0; i<linkCount; i++) {
        JsonObject l = linksArr.add<JsonObject>();
        l["source_id"] = links[i].src; 
        l["target_id"] = links[i].dst; 
        l["capacity_bps"]  = links[i].capacity; 
    }
    
    // 4. Upload
    String requestBody; 
    serializeJson(doc, requestBody);
    
    int httpResponseCode = http.POST(requestBody);
    
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("[WIFI] Optimization Result:");
        Serial.println(response); // Copy this output to see your schedule!
        st7735.st7735_write_str(0, 140, "Upload OK ", Font_7x10, ST7735_GREEN);
    } else {
        Serial.printf("[WIFI] Error: %d\n", httpResponseCode);
        st7735.st7735_write_str(0, 140, "Http Err  ", Font_7x10, ST7735_RED);
    }
    
    http.end(); 
    WiFi.disconnect();
}
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
bool visited[255] = {false};
uint8_t discoveredNodes[MAX_NODES];
int discoveredCount = 0;

// --- STATE MACHINE ---
typedef enum
{
    STATE_IDLE,
    STATE_DISCOVER_NEIGHBORS,
    STATE_TESTING_LINK,
    STATE_PROCESS_QUEUE,
    STATE_WAIT_REMOTE_SEARCH
} BaseState_t;

BaseState_t currentState = STATE_IDLE;

// --- SF TEST VARIABLES ---
int currentTestSF = 12;
#define PACKETS_PER_SF 5
uint8_t currentTargetNode = 0;

// --- RX/TX STATE ---
uint8_t rxBuffer[255];
uint8_t rxSize;
int16_t rxRssi;
volatile bool packetReceived = false;
volatile bool txDone = false;

// --- HELPER FUNCTIONS ---

void ScreenUpdate(const char *line1, const char *line2, uint16_t color)
{
    st7735.st7735_fill_screen(ST7735_BLACK);
    st7735.st7735_write_str(0, 0, "BASE STATION", Font_7x10, ST7735_CYAN);
    st7735.st7735_write_str(0, 20, (char *)line1, Font_7x10, color);
    st7735.st7735_write_str(0, 40, (char *)line2, Font_7x10, color);

    char buf[30];
    sprintf(buf, "Found: %d Nodes", discoveredCount);
    st7735.st7735_write_str(0, 60, buf, Font_7x10, ST7735_GREEN);

    int y = 80;
    for (int i = 0; i < discoveredCount && i < 5; i++)
    {
        sprintf(buf, "ID:%d", discoveredNodes[i]);
        st7735.st7735_write_str((i % 3) * 40, y + (i / 3) * 15, buf, Font_7x10, ST7735_WHITE);
    }
}

void SendPacket(uint8_t dest, uint8_t type, uint8_t d1, uint8_t d2)
{
    uint8_t packet[10];
    packet[0] = dest;
    packet[1] = BASE_STATION_ID;
    packet[2] = type;
    packet[3] = d1;
    packet[4] = d2;

    txDone = false;
    Radio.Send(packet, 5);

    // BLOCKING WAIT for TX to finish
    unsigned long start = millis();
    while (!txDone && millis() - start < 1000)
    {
        Radio.IrqProcess();
    }
}

void ConfigRadioSF(int sf)
{
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 8, false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, sf, LORA_CODINGRATE, 0, 8, 0, false, 0, true, 0, 0, false, true);
}

void enqueue(uint8_t id)
{
    if (visited[id])
        return;
    visited[id] = true;
    if (queueTail < MAX_NODES)
        searchQueue[queueTail++] = id;
    if (discoveredCount < MAX_NODES)
        discoveredNodes[discoveredCount++] = id;
}

// --- INTERRUPTS ---
void OnTxDone()
{
    txDone = true;
    Radio.Rx(RX_TIMEOUT_VALUE);
}
void OnTxTimeout()
{
    txDone = true;
    Radio.Rx(RX_TIMEOUT_VALUE);
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    memcpy(rxBuffer, payload, size);
    rxSize = size;
    rxRssi = rssi;
    packetReceived = true;
}
void OnRxTimeout()
{
    Radio.Rx(RX_TIMEOUT_VALUE);
}

void setup()
{
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    st7735.st7735_init();
    ScreenUpdate("Initializing...", "BFS Logic", ST7735_WHITE);

    RadioEvents_t RadioEvents;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    ConfigRadioSF(12);

    visited[BASE_STATION_ID] = true;
    delay(1000);
    currentState = STATE_DISCOVER_NEIGHBORS;
}

void loop()
{
    switch (currentState)
    {
    case STATE_DISCOVER_NEIGHBORS:
        ScreenUpdate("Scanning...", "Sending Discovery", ST7735_YELLOW);
        ConfigRadioSF(12);

        SendPacket(BROADCAST_ADDR, PKT_DISCOVERY_REQ, 0, 0);

        {
            unsigned long startWait = millis();
            packetReceived = false;

            while (millis() - startWait < DISCOVERY_TIMEOUT)
            {
                Radio.IrqProcess();
                if (packetReceived)
                {
                    packetReceived = false;
                    if (rxBuffer[0] == BASE_STATION_ID && rxBuffer[2] == PKT_DISCOVERY_RESP)
                    {
                        uint8_t foundID = rxBuffer[1];
                        if (!visited[foundID])
                        {
                            currentTargetNode = foundID;
                            currentState = STATE_TESTING_LINK;
                            return;
                        }
                    }
                    Radio.Rx(RX_TIMEOUT_VALUE);
                }
            }
            currentState = STATE_PROCESS_QUEUE;
        }
        break;

    case STATE_TESTING_LINK:
    {
        ScreenUpdate("Testing Link", "SF Loop...", ST7735_MAGENTA);
        int bestSF = 0;

        for (int sf = 12; sf >= 7; sf--)
        {
            ConfigRadioSF(sf);
            int ackCount = 0;
            for (int k = 0; k < PACKETS_PER_SF; k++)
            {
                SendPacket(currentTargetNode, PKT_SF_TEST, sf, k);

                unsigned long waitAck = millis();
                bool acked = false;
                packetReceived = false;

                while (millis() - waitAck < 800)
                {
                    Radio.IrqProcess();
                    if (packetReceived)
                    {
                        packetReceived = false;
                        if (rxBuffer[1] == currentTargetNode && rxBuffer[2] == PKT_SF_ACK)
                        {
                            acked = true;
                        }
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }
                    if (acked)
                        break;
                }
                if (acked)
                    ackCount++;
                delay(50);
            }
            if (ackCount > 0)
                bestSF = sf;
        }

        enqueue(currentTargetNode);
        ConfigRadioSF(12);
        currentState = STATE_DISCOVER_NEIGHBORS;
    }
    break;

    case STATE_PROCESS_QUEUE:
        if (queueHead < queueTail)
        {
            uint8_t commanderID = searchQueue[queueHead++];
            if (commanderID == BASE_STATION_ID)
            {
                currentState = STATE_PROCESS_QUEUE;
                return;
            }
            ScreenUpdate("Remote Search", "Commanding Node...", ST7735_YELLOW); // Replaced ORANGE
            ConfigRadioSF(12);
            SendPacket(commanderID, PKT_CMD_SEARCH, 0, 0);
            currentState = STATE_WAIT_REMOTE_SEARCH;
        }
        else
        {
            ScreenUpdate("Network Mapped", "Idle", ST7735_GREEN);
            delay(5000);
        }
        break;

    case STATE_WAIT_REMOTE_SEARCH:
        unsigned long waitStart = millis();
        packetReceived = false;

        while (millis() - waitStart < 15000)
        {
            Radio.IrqProcess();
            if (packetReceived)
            {
                packetReceived = false;
                if (rxBuffer[0] == BASE_STATION_ID)
                {
                    if (rxBuffer[2] == PKT_REPORT_NODE)
                    {
                        uint8_t foundID = rxBuffer[3];
                        if (!visited[foundID])
                        {
                            enqueue(foundID);
                            Serial.printf("Remote Node %d found Node %d\n", rxBuffer[1], foundID);
                        }
                    }
                    else if (rxBuffer[2] == PKT_SEARCH_DONE)
                    {
                        currentState = STATE_PROCESS_QUEUE;
                        return;
                    }
                }
                Radio.Rx(RX_TIMEOUT_VALUE);
            }
        }
        currentState = STATE_PROCESS_QUEUE;
        break;
    }
}
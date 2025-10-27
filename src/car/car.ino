#define USE_GUI 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#define WIFI_SSID "aaaa"
#define WIFI_PASSWORD "00000906"
#define TRIGGER_HOST "192.168.225.86"
#define TRIGGER_PORT 7777
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define RSSI_THRESHOLD -30
#define SCAN_TIME 1
#define RECONNECT_BLOCK_TIME 5000

const char* VEHICLE_ID = "123가1234";
uint8_t TAG_ID = 19;
const uint8_t GUI_MAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

#ifdef USE_GUI
#define RECEIVER_PORT 7777
#define WAYPOINT_PORT 8888
#endif

bool isEntryVehicle = true;
unsigned long lastDisconnectTime = 0;
bool canReconnect = true;
bool triggerSent = false;
unsigned long pcConnectionFailedTime = 0;
bool isWaitingAfterPCFail = false;
float smoothedRSSI = -100.0;
const float ALPHA = 0.8;

String receivedVehicleType = "regular";
bool receivedIsHandicapped = false;
String receivedSpotType = "normal";
uint8_t receivedDestination = 0;

BLEScan* pBLEScan;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
BLEAdvertisedDevice* myDevice = nullptr;
BLEClient* pClient = nullptr;
#ifdef USE_GUI
WiFiServer server(RECEIVER_PORT);
WiFiServer waypointServer(WAYPOINT_PORT);
#endif
bool doConnect = false;
bool isConnected = false;
bool isScanning = false;
String currentState = "BOOTING";
bool shouldSendVehicleInfo = false;

void connectToServer();
void printPacket(const char* direction, const uint8_t* data, size_t len);
void updateState(String newState);
void sendTriggerToPC();
#ifdef USE_GUI
void handleNewClient(WiFiClient client);
void handleWaypointClient(WiFiClient client);
#endif
void onDataReceived(String vehicleType, bool isHandicapped, String spotType, uint8_t destination);
void sendVehicleInfo();
void resetConnectionState();
void forceDisconnect();

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    printPacket("[RECV_BLE]", pData, length);
    if (length > 4 && pData[0] == 0x02) {
        uint8_t cmd = pData[1];
        
        if (cmd == 0x15) {
            Serial.println("   -> Vehicle info request (0x15) received. Preparing response.");
            shouldSendVehicleInfo = true;
        }
        else if (cmd == 0x13) {T
            Serial.println("   -> Authentication REJECTED (0x13) by barrier. Disconnecting.");
            if(pClient->isConnected()) {
                pClient->disconnect();
            }
        }
    }
}

class MyClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        isConnected = true;
        Serial.printf("   -> Connection established for: %s\n", isEntryVehicle ? "ENTRY" : "EXIT");
        updateState("CONNECTED_TO_BARRIER");
    }
    
    void onDisconnect(BLEClient* pclient) {
        isEntryVehicle = !isEntryVehicle;
        lastDisconnectTime = millis();
        canReconnect = false;
        triggerSent = false;
        smoothedRSSI = -100.0;
        resetConnectionState();
    }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (!canReconnect && (millis() - lastDisconnectTime < RECONNECT_BLOCK_TIME)) return;
        
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(BLEUUID(SERVICE_UUID))) {
            int currentRSSI = advertisedDevice.getRSSI();
            if (smoothedRSSI == -100.0) smoothedRSSI = currentRSSI;
            else smoothedRSSI = (ALPHA * currentRSSI) + (1.0 - ALPHA) * smoothedRSSI;
            Serial.printf("RSSI Raw: %d, Smoothed: %.2f\n", currentRSSI, smoothedRSSI);

            if (smoothedRSSI > RSSI_THRESHOLD) {
                Serial.printf("Barrier found! (Smoothed RSSI: %.2f)\n", smoothedRSSI);
                BLEDevice::getScan()->stop();
                isScanning = false;
                canReconnect = true;
                
                if (myDevice == nullptr) {
                    myDevice = new BLEAdvertisedDevice(advertisedDevice);
                    
                    if (isEntryVehicle) {
                        updateState("DEVICE_FOUND_FOR_ENTRY");
                        #ifdef USE_GUI
                        if (!triggerSent) {
                            sendTriggerToPC();
                            triggerSent = true;
                        }
                        if (!isWaitingAfterPCFail) {
                            updateState("WAITING_FOR_GUI_DATA");
                        }
                        #else
                        Serial.println("   -> [DEBUG_MODE] GUI disabled. Proceeding directly to connection.");
                        doConnect = true;
                        #endif
                    } else { 
                        updateState("DEVICE_FOUND_FOR_EXIT");
                        doConnect = true;
                    }
                }
            }
        }
    }
};

void setup() {
    Serial.begin(115200);
    updateState("INITIALIZING");
    
    #ifdef USE_GUI
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.print("   My IP address is: ");
    Serial.println(WiFi.localIP());
    
    server.begin();
    waypointServer.begin();
    Serial.printf("   TCP Server for data reception started on port %d\n", RECEIVER_PORT);
    #endif

    BLEDevice::init("");
    BLEDevice::setMTU(50);
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallbacks());
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    updateState("IDLE");
}

void loop() {
    if (isWaitingAfterPCFail) {
        if (millis() - pcConnectionFailedTime > 5000) {
            Serial.println("   -> 5 seconds passed. Restarting process...");
            isWaitingAfterPCFail = false;
            triggerSent = false;
            resetConnectionState();
        }
        delay(100);
        return;
    }

    if (currentState == "WAITING_FOR_GUI_DATA") {
        #ifdef USE_GUI
        WiFiClient client = server.available();
        if (client) {
            handleNewClient(client);
        }
        #endif
        delay(50);
        return;
    }

    #ifdef USE_GUI
    WiFiClient waypointClient = waypointServer.available();
    if (waypointClient) { handleWaypointClient(waypointClient); }
    #endif

    if (!canReconnect && (millis() - lastDisconnectTime >= RECONNECT_BLOCK_TIME)) {
        canReconnect = true;
        Serial.println("   -> Reconnection allowed after 7 seconds.");
    }

    if (!isConnected && myDevice == nullptr && !isScanning && canReconnect) {
        updateState("SCANNING_FOR_BARRIER");
        pBLEScan->start(SCAN_TIME, [](BLEScanResults results){ 
            isScanning = false; 
            if(!isConnected && myDevice == nullptr) updateState("IDLE");
        }, false);
        isScanning = true;
    }
    
    if (doConnect && myDevice != nullptr && !isConnected) {
        connectToServer();
        doConnect = false;
    }

    if (isConnected) {
        if (shouldSendVehicleInfo) {
            sendVehicleInfo();
            shouldSendVehicleInfo = false;
            forceDisconnect();
        }
    }
    delay(100);
}

void resetConnectionState() {
    isConnected = false;
    pRemoteCharacteristic = nullptr;
    if (myDevice != nullptr) {
        delete myDevice;
        myDevice = nullptr;
    }
    shouldSendVehicleInfo = false;
    doConnect = false;
    isScanning = false; 
    updateState("DISCONNECTED_AND_IDLE");
}

void forceDisconnect() {
    if (isConnected && pClient != nullptr) {
        Serial.println("   -> Force disconnecting from barrier...");
        pClient->disconnect();
    }
}

void onDataReceived(String vehicleType, bool isHandicapped, String spotType, uint8_t destination) {
    receivedVehicleType = vehicleType;
    receivedIsHandicapped = isHandicapped;
    receivedSpotType = spotType;
    receivedDestination = destination;

    Serial.println("========== Parking Choice Info (Updated) ==========");
    Serial.printf("   - Vehicle Type: %s\n", receivedVehicleType.c_str());
    Serial.printf("   - Is Disabled: %s\n", receivedIsHandicapped ? "Yes" : "No");
    Serial.printf("   - Preferred Spot: %s\n", receivedSpotType.c_str());
    Serial.printf("   - Destination: %d\n", receivedDestination);
    Serial.println("=================================================");

    doConnect = true; 
}

void connectToServer() {
    if (!pClient->connect(myDevice)) {
        resetConnectionState(); 
        return;
    }
    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (pRemoteService == nullptr) { pClient->disconnect(); return; }
    
    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
    if (pRemoteCharacteristic == nullptr) { pClient->disconnect(); return; }
    
    if(pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
    }
    
    if (pRemoteCharacteristic->canWrite()) {
        if (isEntryVehicle) {
            uint8_t readyPacket[] = {0x02, 0x01, 0x00, 0x01, 0x03};
            pRemoteCharacteristic->writeValue(readyPacket, sizeof(readyPacket), false);
            printPacket("[SEND_BLE]", readyPacket, sizeof(readyPacket));
            Serial.println("   -> Sent Ready signal (0x01) for entry.");
        } else {
            sendVehicleInfo();
            forceDisconnect();
        }
    }
}

void sendVehicleInfo() {
    if (pRemoteCharacteristic == nullptr || !pRemoteCharacteristic->canWrite()) return;
    
    uint8_t vehicleTypeByte = (receivedVehicleType == "electric") ? 0x01 : 0x00;
    uint8_t disabledTypeByte = receivedIsHandicapped ? 0x01 : 0x00;
    
    uint8_t preferredByte = 0;
    if (receivedSpotType == "disabled") preferredByte = 1;
    else if (receivedSpotType == "elec") preferredByte = 2;
    
    size_t vehicleIdLen = strlen(VEHICLE_ID);
    
    uint8_t responsePacket[128];
    int idx = 0;
    
    responsePacket[idx++] = 0x02;
    
    if (isEntryVehicle) {
        responsePacket[idx++] = 0x10;
    } else {
        responsePacket[idx++] = 0x16;
    }
    
    responsePacket[idx++] = 0x00;
    int dataStartIdx = idx;
    
    memcpy(&responsePacket[idx], VEHICLE_ID, vehicleIdLen);
    idx += vehicleIdLen; 
    responsePacket[idx++] = 0x00;
    
    responsePacket[idx++] = TAG_ID;
    
    if (isEntryVehicle) {
        responsePacket[idx++] = vehicleTypeByte;
        responsePacket[idx++] = disabledTypeByte;
        responsePacket[idx++] = preferredByte;
        responsePacket[idx++] = receivedDestination;
        memcpy(&responsePacket[idx], GUI_MAC, 6);
        idx += 6;
    }
    
    uint8_t dataLen = idx - dataStartIdx;
    responsePacket[2] = dataLen;
    
    uint8_t checksum = 0;
    for(int i = 1; i < idx; i++) checksum ^= responsePacket[i];
    responsePacket[idx++] = checksum;
    
    responsePacket[idx++] = 0x03;

    pRemoteCharacteristic->writeValue(responsePacket, idx, false);
    printPacket("[SEND_BLE]", responsePacket, idx);
    
    if (isEntryVehicle) {
        Serial.printf("   -> Sent ENTRY info (0x10) - TagID: %d, Destination: %d\n", TAG_ID, receivedDestination);
    } else {
        Serial.printf("   -> Sent EXIT info (0x16) - TagID: %d\n", TAG_ID);
    }
}

void printPacket(const char* direction, const uint8_t* data, size_t len) {
    Serial.printf("%s (Len: %d): ", direction, len);
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();
}

void updateState(String newState) {
    if (currentState != newState) {
        currentState = newState;
        Serial.printf("\n===== [STATE] %s =====\n", newState.c_str());
        Serial.printf("   Next Action will be: %s\n", isEntryVehicle ? "ENTRY" : "EXIT");
    }
}

void sendTriggerToPC() {
    #ifdef USE_GUI
    WiFiClient client;
    Serial.printf("\n[PC Trigger] Connecting to PC server: %s:%d\n", TRIGGER_HOST, TRIGGER_PORT);
    
    if (!client.connect(TRIGGER_HOST, TRIGGER_PORT)) {
        Serial.println("[PC Trigger] Connection failed. Will retry in 5 seconds.");
        isWaitingAfterPCFail = true;
        pcConnectionFailedTime = millis();
        updateState("PC_CONNECTION_FAILED");
        return;
    }
    
    Serial.println("[PC Trigger] Connected to server!");
    StaticJsonDocument<256> doc;
    doc["command"] = "start_simulation";
    doc["vehicle_id"] = VEHICLE_ID;
    doc["vehicle_ip"] = WiFi.localIP().toString(); 
    
    String output;
    serializeJson(doc, output);
    client.print(output);
    Serial.print("[PC Trigger] Sent: ");
    Serial.println(output);
    client.stop();
    Serial.println("[PC Trigger] Connection closed.");
    #endif
}

#ifdef USE_GUI
void handleNewClient(WiFiClient client) {
    Serial.println("\nPC GUI Client connected!");
    
    unsigned long startTime = millis();
    while(client.connected() && !client.available() && millis() - startTime < 2000) {
        delay(10);
    }

    if (client.available()) {
        String jsonData = client.readString();
        Serial.print("Received Data from GUI: ");
        Serial.println(jsonData);
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, jsonData);
        
        if (error) {
            Serial.print("JSON parsing failed: ");
            Serial.println(error.c_str());
            client.println("{\"status\": \"error\", \"message\": \"Invalid JSON\"}");
        } else {
            String elec_str = doc["elec"];
            String disabled_str = doc["disabled"];
            
            String vehicleType = (elec_str == "true") ? "electric" : "regular";
            bool isHandicapped = (disabled_str == "true");
            String spotType = doc["preferred"] | "normal";
            uint8_t destination = doc["destination"] | 0;
            
            onDataReceived(vehicleType, isHandicapped, spotType, destination);
            
            client.println("{\"status\": \"success\", \"message\": \"Data received by ESP32\"}");

            updateState("IDLE"); 
        }
    } else {
        Serial.println("Client connected but no data received.");
    }

    client.stop();
    Serial.println("PC GUI Client disconnected.");
}

void handleWaypointClient(WiFiClient client) {
     if (client.available()) {
        String jsonData = client.readString();
        Serial.print("Received Waypoints: ");
        Serial.println(jsonData);
     }
}
#endif

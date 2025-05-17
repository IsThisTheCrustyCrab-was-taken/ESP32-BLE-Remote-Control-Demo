#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <hardwareSerial.h>
#include <string>
#include <cctype>
#include <utility>

// UUIDs for the BLE service and characteristic

// https://community.platformio.org/t/esp32-s3-zero-does-not-work-on-platformio/40297/6 <= esp32s3zero fix hopefully
#define SERVICE_UUID    "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SPEED_CHAR_UUID "beb5483e-36e2-4688-b7f5-ea07361b26a8" // 36e1 vs 36e2

#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define DEBUG_RX

// Forward declarations
void sendSerialAsBle(const String& payload);
void handleSpeedCommand(const std::string& value);

// Setup UART
HardwareSerial uartSerial(1);
// String uartBuffer = "";
const int rxPin = 13; // RX pin for UART <= Blue wire
const int txPin = 12; // TX pin for UART <= Green wire

BLECharacteristic *pDataCharacteristic;
BLECharacteristic *pSpeedCharacteristic;

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

bool serialSanityCheck = false; // Flag to check if the serial is working
auto breakpointAry = std::array<int, 8>{0, 0, 0, 0, 0, 0, 0, 0}; // Array to store breakpoints
std::string breakpointAryToString() {
    std::string str = "";
    for (const auto& i : breakpointAry) {
        str += std::to_string(i) + ",";
    }
    str.pop_back(); // Remove the last comma
    return str;
}

typedef struct{
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;
    int16_t  boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

unsigned long iTimeSend = 0;
int iTest = 0;

class SpeedCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        std::string value = pCharacteristic->getValue();
        if (value.empty()) return;
        handleSpeedCommand(value);
    }
};

void maybeSendBt(std::string payload) {
    if (millis() - iTimeSend > 500) {
        iTimeSend = millis();
        pDataCharacteristic->setValue(std::move(payload));
        pDataCharacteristic->notify(); // Notify connected device of the new value
    }
}

void handleSpeedCommand(const std::string &value) {
    int speed = atoi(value.c_str());
    Command.start = (int16_t)START_FRAME;
    Command.speed = (int16_t) speed;
    Command.steer = (int16_t) 0; // Reset steer to 0 when speed is set
    Command.checksum = (int16_t) (Command.start ^ Command.steer ^ Command.speed);
    // uartSerial.write((uint8_t *)&Command, sizeof(Command)); // Send command to UART
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (uartSerial.available()) {
        incomingByte 	  = uartSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
        breakpointAry[0] = 1;
    }
    else {
        maybeSendBt(breakpointAryToString());
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
    Serial.print("Prev: 0x"); Serial.print(incomingBytePrev, HEX);
    Serial.print("  Curr: 0x"); Serial.print(incomingByte, HEX);
    Serial.print("  Frame: 0x"); Serial.println(bufStartFrame, HEX);
        // return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;
        breakpointAry[1] = 1;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte;
        idx++;
        breakpointAry[2] = 1;
    } else {
        breakpointAry[3] = 1;
    }

    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        breakpointAry[4] = 1;
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
            // Send data over BLE
            String payload = String(Feedback.cmd1) + "," + String(Feedback.cmd2) + "," +
                            String(Feedback.speedR_meas) + "," + String(Feedback.speedL_meas) + "," +
                            String(Feedback.batVoltage) + "," + String(Feedback.boardTemp) + "," +
                            String(Feedback.cmdLed);
                            String(Feedback.cmdLed);
        } else {
            breakpointAry[5] = 1;
        }
        idx = 0;    // Reset the index (it prevents to enter this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}


// BLE server callback class for handling BLE events
class MyServerCallbacks: public BLEServerCallbacks {
    // Called when a BLE device connects to the ESP32
    void onConnect(BLEServer* pServer) {
        Serial.println("Device Connected!");
        // resetValues(); // Resets LED and display values to their default state
    }

    // Called when a BLE device disconnects from the ESP32 - TODO: separate heartbeat function
    void onDisconnect(BLEServer* pServer) {
        pServer->startAdvertising(); // Restart advertising to allow new connections - STOP THE FUCKING MOTOR
        Serial.println("Device Disconnected!");
    }
};

// BLE characteristic callback class for handling data written to the characteristic (kinda useless rn)
class ESP32Callbacks: public BLECharacteristicCallbacks {
    // Called when data is written to the characteristic
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue(); // Get the written value
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("Somethings alive");
    uartSerial.begin(115200, SERIAL_8N1, rxPin, txPin); // Initialize UART with RX and TX pins

    // Initialize BLE device with a unique name
    BLEDevice::init("ESP32_Control");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks()); // Set server callbacks

    // Create BLE service and characteristic
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pDataCharacteristic = pService->createCharacteristic(
                         DATA_CHAR_UUID,
                         BLECharacteristic::PROPERTY_READ |
                         BLECharacteristic::PROPERTY_WRITE
                     );

    // Set characteristic callbacks and start the service
    pDataCharacteristic->setCallbacks(new ESP32Callbacks());
    pDataCharacteristic->setValue("Data from hoverboard");
    //same for speed
    pSpeedCharacteristic = pService->createCharacteristic(
                            SPEED_CHAR_UUID,
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_WRITE
                            );
    pSpeedCharacteristic->setCallbacks(new SpeedCallbacks());
    pSpeedCharacteristic->setValue("0");
    pService->start();

    // Setup BLE advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(0x1234); 
    pAdvertising->addServiceUUID(SERVICE_UUID); 
    pAdvertising->setScanResponse(true); 
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12); 
    pAdvertising->start(); 

    Serial.println("BLE Device Initialized and Ready");

    pDataCharacteristic->setValue("beepboop"); // Set the characteristic value
    pDataCharacteristic->notify(); // Notify connected device of the new value
}

// int frameCounter = 0; // Frame counter for debugging
void loop() {
    Receive();
    // pDataCharacteristic->setValue("figner");
    // pDataCharacteristic->notify(); // Notify connected device of the new value
    // delay(1000); // Sleep for 1000ms
}
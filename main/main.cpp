// #include <Arduino.h>
#include "LoraMesher.h"
#include "esp_log.h"
#include "EspHal.h"

#define BOARD_LED   37
#define LED_ON      LOW
#define LED_OFF     HIGH

LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
struct dataPacket {
    uint32_t counter = 0;
};

dataPacket* helloPacket = new dataPacket;

//Led flash
// void led_Flash(uint16_t flashes, uint16_t delaymS) {
//     uint16_t index;
//     for (index = 1; index <= flashes; index++) {
//         digitalWrite(BOARD_LED, LED_ON);
//         delay(delaymS);
//         digitalWrite(BOARD_LED, LED_OFF);
//         delay(delaymS);
//     }
// }

/**
 * @brief Print the counter of the packet
 *
 * @param data
 */
void printPacket(dataPacket data) {
    // Serial.printf("Hello Counter received nº %d\n", data.counter);
}

/**
 * @brief Iterate through the payload of the packet and print the counter of the packet
 *
 * @param packet
 */
void printDataPacket(AppPacket<dataPacket>* packet) {
    // Serial.printf("Packet arrived from %X with size %d\n", packet->src, packet->payloadSize);

    //Get the payload to iterate through it
    dataPacket* dPacket = packet->payload;
    size_t payloadLength = packet->getPayloadLength();

    for (size_t i = 0; i < payloadLength; i++) {
        //Print the packet
        printPacket(dPacket[i]);
    }
}

/**
 * @brief Function that process the received packets
 *
 */
void processReceivedPackets(void*) {
    for (;;) {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        // led_Flash(1, 100); //one quick LED flashes to indicate a packet has arrived
        ESP_LOGW("","ulTaskNotifyTake!") ;

        //Iterate through all the packets inside the Received User Packets Queue
        while (radio.getReceivedQueueSize() > 0) {
            // Serial.println("ReceivedUserData_TaskHandle notify received");
            // Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());

            //Get the first element inside the Received User Packets Queue
            AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();

            //Print the data packet
            printDataPacket(packet);

            //Delete the packet when used. It is very important to call this function to release the memory of the packet.
            radio.deletePacket(packet);
        }
    }
}

TaskHandle_t receiveLoRaMessage_Handle = NULL;

/**
 * @brief Create a Receive Messages Task and add it to the LoRaMesher
 *
 */
void createReceiveMessages() {
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void*) 1,
        2,
        &receiveLoRaMessage_Handle);
    if (res != pdPASS) {
        // Serial.printf("Error: Receive App Task creation gave error: %d\n", res);
    }
}


/**
 * @brief Initialize LoRaMesher
 *
 */
void setupLoraMesher() {
    //Get the configuration of the LoRaMesher
    LoraMesher::LoraMesherConfig config = LoraMesher::LoraMesherConfig();

    //Set the configuration of the LoRaMesher (TTGO T-BEAM v1.1)
    config.loraCs = 7;
    config.loraRst = 8;
    config.loraIrq = 9;
    config.loraIo1 = 36;
    config.freq = 2400;
    config.bw = 1625;
    config.sf = 6;
    config.cr = 5;
    config.power = 13;
    config.syncWord = RADIOLIB_SX128X_SYNC_WORD_PRIVATE ;
    config.preambleLength = 12;
    config.max_packet_size = 255;
    config.hal = new EspHal(5, 3, 6);

    config.module = LoraMesher::LoraModules::SX1280_MOD;

    //Init the loramesher with a configuration
    radio.begin(config);

    //Create the receive task and add it to the LoRaMesher
    createReceiveMessages();

    //Set the task handle to the LoRaMesher
    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);

    //Start LoRaMesher
    radio.start();

    // Serial.println("Lora initialized");
}


void setup() {
    // Serial.begin(115200);
    // delay(1500);


    // Serial.println("initBoard");
    // pinMode(BOARD_LED, OUTPUT); //setup pin as output for indicator LED
    // led_Flash(4, 100);          //two quick LED flashes to indicate program start
    setupLoraMesher();
    // Serial.println("init done");
}


extern "C" int app_main(void)
{
    setup() ;

    for (;;) {

        // led_Flash(2,500) ;
        
        // Serial.printf("Send packet %d\n", dataCounter);
        ESP_LOGI("","Send packet %lu", dataCounter);

        helloPacket->counter = dataCounter++;

        //Create packet and send it.
        radio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);

        //Wait 20 seconds to send the next packet
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
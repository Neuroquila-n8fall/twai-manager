#include <Arduino.h>
#include <TWAIManager.h>

// Pins used to connect to CAN bus transceiver:
#define TX_PIN GPIO_NUM_21
#define RX_PIN GPIO_NUM_22

// Pin used to blink the LED when a message is received:
#define LED_BUILTIN GPIO_NUM_27

// Define the transmit and polling rates in milliseconds
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 250

// Flag to indicate if the driver is installed
static bool driver_installed = false;

// Variable to store the last time a message was send
unsigned long previousMillis = 0; // will store last time a message was send

// Create an instance of the TWAIManager class
TWAIManager twaiManager;

// Variable to store the last time a message was received
ulong previousMessageReceived = 0;

// Function to handle error conditions
void handle_error(uint32_t errorCode)
{
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);
    
    // Check and report each error condition separately
    if (errorCode & TWAI_ALERT_ABOVE_ERR_WARN) {
        Serial.println("Alert: The error count has reached the warning limit.");
        Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
    }
    if (errorCode & TWAI_ALERT_BUS_ERROR) {
        Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
        Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
    }
    if (errorCode & TWAI_ALERT_TX_FAILED) {
        Serial.println("Alert: The Transmission failed.");
        Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
        Serial.printf("TX error: %lu\t", twaistatus.tx_error_counter);
        Serial.printf("TX failed: %lu\n", twaistatus.tx_failed_count);
    }
    if (errorCode & TWAI_ALERT_RX_QUEUE_FULL) {
        Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
        Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
        Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
        Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
    }
    if (errorCode & TWAI_ALERT_ERR_PASS) {
        Serial.println("Alert: TWAI controller has become error passive.");
    }
    if (errorCode & TWAI_ALERT_BUS_OFF) {
        Serial.println("Alert: The bus has entered the Bus Off state.");
    }
    if (errorCode & TWAI_ALERT_RX_FIFO_OVERRUN) {
        Serial.println("Alert: The RX FIFO buffer has been overrun.");
    }
    if (errorCode & TWAI_ALERT_ARB_LOST) {
        Serial.println("Alert: A message was lost because the arbitration process took too long.");
    }

}


/**
 * @brief Handles the received message.
 * 
 * This function is called when a message is received. It blinks the LED, prints the received message details,
 * and checks if the message is received from self.
 * 
 * @param message The received message.
 */
void handle_rx_message(const twai_message_t &message)
{
  // BLink the LED when a packet arrives
  digitalWrite(LED_BUILTIN, HIGH);
  // Store the time the message was received
  previousMessageReceived = millis();
  // Check if the message was received from self. This is an indicator that something is wrong with the bus termination.
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  if (message.identifier == mac[5])
  {
    Serial.println("Message received from self");
  }

  // Print the received message details
  Serial.printf("[RCV] EXT: %s ID: %lx Bytes:", message.extd == 1 ? "YES" : "NO", message.identifier);
  for (int i = 0; i < message.data_length_code; i++)
  {
    Serial.printf(" %d = %02x,", i, message.data[i]);
  }
  Serial.println("");
}

void setup()
{

  // Setup the led
  pinMode(LED_BUILTIN, OUTPUT);

  // Start Serial:
  Serial.begin(115200);

  // Initialize the TWAIManager
  bool success = twaiManager.begin(TX_PIN, RX_PIN, TWAI_TIMING_CONFIG_100KBITS(), POLLING_RATE_MS, TRANSMIT_RATE_MS);

  // Check if the driver was installed successfully
  if (success)
  {
    driver_installed = true;
    Serial.println("Driver installed");
  }
  else
  {
    Serial.println("Failed to install driver");
  }

  // Register callback for onError event
  twaiManager.onError([](uint32_t errorCode)
                      { handle_error(errorCode); });

  // Register callback for onMessageReceived event
  twaiManager.onMessageReceived([](const twai_message_t &message)
                                { handle_rx_message(message); });

  // Show the MAC address
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  Serial.printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**
 * @brief Sends a message using TWAIManager.
 * 
 * This function configures a message to transmit and queues it for transmission using TWAIManager.
 * The message identifier is set to the last byte of the device's MAC address.
 * The data length is set to 4.
 * The message uses a standard identifier and is not a remote frame.
 * The message data is filled with random values between 0 and 255.
 * 
 * @note This function assumes that the TWAIManager instance is already initialized.
 * 
 * @return void
 */
static void send_message() {
    // Configure message to transmit
    twai_message_t message;
    // Use the last byte of the devices mac address as the identifier
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    message.identifier = mac[5];  // Use the last byte of the devices mac address as the identifier
    message.data_length_code = 4; // Data length
    message.extd = false;        // Standard identifier
    message.rtr = false;         // Not a remote frame

    for (int i = 0; i < 4; i++) {
        message.data[i] = random(0, 255);     // Fill data
    }

    // Queue message for transmission using TWAIManager
    if (twaiManager.sendMessage(message)) {
        Serial.println("Message queued for transmission.");
    } else {
        Serial.println("Failed to queue message for transmission.");
    }
}


void loop()
{
  twaiManager.loop();

  // If a message was received, reset the led to off
  if (millis() - previousMessageReceived >= 100)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Send message
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS)
  {
    previousMillis = currentMillis;
    send_message();
  }
}
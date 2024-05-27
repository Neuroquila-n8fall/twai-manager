// TWAIManager.cpp
#include "TWAIManager.h"
#include "esp32/clk.h"

TWAIManager::TWAIManager() : driverInstalled(false) {}

TWAIManager::~TWAIManager() {
    end();
}


/**
 * @brief Initializes the TWAIManager and starts the TWAI driver.
 *
 * This function initializes the TWAIManager with the specified parameters and starts the TWAI driver.
 * It installs the TWAI driver, sets the necessary configurations, and enables necessary alerts.
 *
 * @param txPin The GPIO pin number for the transmit line.
 * @param rxPin The GPIO pin number for the receive line.
 * @param speed The timing configuration for the TWAI bus.
 * @param pollingRateMs The polling rate in milliseconds.
 * @param transmitRateMs The transmit rate in milliseconds.
 * @param twai_mode The TWAI mode (TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK, TWAI_MODE_LISTEN_ONLY).
 * @param filterConfig The filter configuration for the TWAI bus.
 * @return `true` if the TWAI driver is installed and started successfully, `false` otherwise.
 */
bool TWAIManager::begin(gpio_num_t txPin, gpio_num_t rxPin, twai_timing_config_t speed, uint32_t pollingRateMs, uint32_t transmitRateMs, twai_mode_t twai_mode, twai_filter_config_t filterConfig) {
    TWAIManager::pollingRateMs = pollingRateMs;
    TWAIManager::transmitRateMs = transmitRateMs;
    TWAIManager::filterConfig = filterConfig;
    TWAIManager::twai_mode = twai_mode;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, twai_mode);
    twai_timing_config_t t_config = speed;
    twai_filter_config_t f_config = filterConfig;

    // Install the TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    // Start the TWAI driver
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    driverInstalled = true;

    // Enable necessary alerts
    twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);
    Serial.println("[TWAI Manager] Driver installed successfully.");
    Serial.printf("[TWAI Manager] Mode: %d BRP: %d TSEG1: %d TSED2: %d SJW: %d", g_config.mode, speed.brp, speed.tseg_1, speed.tseg_2, speed.sjw);
    Serial.println();
    Serial.printf("[TWAI Manager] Effective Speed: %d kbit/s", (esp_clk_apb_freq() / (speed.brp * (speed.tseg_1 + speed.tseg_2 + 1))) / 1000);
    Serial.println();
    Serial.printf("[TWAI Manager] Acceptance code: %08x Acceptance mask: %08x", f_config.acceptance_code, f_config.acceptance_mask);
    Serial.println();
    return true;
}

/**
 * @brief Ends the TWAIManager instance and releases any allocated resources.
 * 
 * This function should be called when you are done using the TWAIManager instance.
 * It releases any allocated resources and cleans up the state of the TWAIManager.
 * After calling this function, the TWAIManager instance should not be used anymore.
 */
void TWAIManager::end()
{
    if (driverInstalled) {
        twai_stop();
        twai_driver_uninstall();
        driverInstalled = false;
    }
}

/**
 * Sends a CAN message with the specified ID and payload.
 *
 * @param id The ID of the CAN message.
 * @param payload A pointer to the payload data of the CAN message.
 * @return `true` if the message was sent successfully, `false` otherwise.
 */
bool TWAIManager::sendMessage(uint32_t id, uint8_t length, const uint8_t *payload)
{
    if (!driverInstalled) return false;

    twai_message_t message;
    message.identifier = id;
    message.extd = true;
    message.data_length_code = length;
    memcpy(message.data, payload, length);

    return twai_transmit(&message, pdMS_TO_TICKS(TWAIManager::transmitRateMs)) == ESP_OK;
}

/**
 * Sends a TWAI message.
 *
 * This function sends the provided TWAI message over the TWAI bus.
 *
 * @param message The TWAI message to be sent.
 * @return True if the message was sent successfully, false otherwise.
 */
bool TWAIManager::sendMessage(const twai_message_t& message) {
    if (!driverInstalled) return false;

    // Transmit the message
    esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(transmitRateMs));
    return result == ESP_OK;
}

/**
 * @brief Executes the main loop of the TWAIManager.
 *
 * This function is responsible for executing the main loop of the TWAIManager class.
 * It should be called repeatedly in the main program loop to handle TWAIManager tasks.
 */
void TWAIManager::loop()
{
    if (!driverInstalled) return;

    uint32_t alerts;
    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(TWAIManager::pollingRateMs)) == ESP_OK) {
        if (alerts & TWAI_ALERT_RX_DATA) {
            twai_message_t message;
            while (twai_receive(&message, 0) == ESP_OK) {
                if (messageReceivedCallback) {
                    messageReceivedCallback(message);
                }
            }
        }
        // Define a mask for all error-related alerts
        uint32_t errorMask = TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_TX_FAILED |
                             TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
                             TWAI_ALERT_RX_FIFO_OVERRUN | TWAI_ALERT_ARB_LOST;

        // If any error alerts are triggered, call the error callback function
        if (alerts & errorMask)
        {
            if (errorCallback)
            {
                errorCallback(alerts);
            }
        }
    }
}

/**
 * Registers a callback function to be called when a message is received.
 *
 * @param callback The callback function to be registered.
 */
void TWAIManager::onMessageReceived(MessageReceivedCallback callback) {
    messageReceivedCallback = callback;
}

/**
 * Sets the error callback function for the TWAIManager.
 *
 * @param callback The error callback function to be set.
 */
void TWAIManager::onError(ErrorCallback callback) {
    errorCallback = callback;
}

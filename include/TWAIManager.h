// TWAIManager.h

#ifndef TWAI_MANAGER_H
#define TWAI_MANAGER_H

#include <functional>
#include <driver/twai.h>
#include <Arduino.h>

class TWAIManager {
public:
    // Define callback types for events
    typedef std::function<void(const twai_message_t& message)> MessageReceivedCallback;
    typedef std::function<void(uint32_t errorCode)> ErrorCallback;

    TWAIManager();
    ~TWAIManager();
    bool begin(gpio_num_t txPin, gpio_num_t rxPin, twai_timing_config_t speed = TWAI_TIMING_CONFIG_100KBITS(), uint32_t pollingRateMs = 100, uint32_t transmitRateMs = 1000, twai_mode_t twai_mode = TWAI_MODE_NORMAL, twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL());
    void end();
    bool sendMessage(uint32_t id, uint8_t length, const uint8_t* payload);
    bool sendMessage(const twai_message_t& message);
    void loop();

    void onMessageReceived(MessageReceivedCallback callback);
    void onError(ErrorCallback callback);

private:
    bool driverInstalled;
    uint32_t pollingRateMs;
    uint32_t transmitRateMs;
    twai_filter_config_t filterConfig;
    twai_mode_t twai_mode;

    MessageReceivedCallback messageReceivedCallback;
    ErrorCallback errorCallback;
};


#endif // TWAI_MANAGER_H
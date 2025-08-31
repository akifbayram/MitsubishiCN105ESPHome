#include "cn105.h"

using namespace esphome;

CN105Climate::CN105Climate(uart::UARTComponent *uart) : UARTDevice(uart)
{
    // Configure traits for this climate device.
    this->traits_.set_supports_action(true);
    this->traits_.set_supports_current_temperature(true);
    this->traits_.set_supports_two_point_target_temperature(false);
    this->traits_.set_visual_min_temperature(ESPMHP_MIN_TEMPERATURE);
    this->traits_.set_visual_max_temperature(ESPMHP_MAX_TEMPERATURE);
    this->traits_.set_visual_temperature_step(ESPMHP_TEMPERATURE_STEP);

    // Initialize internal state variables.
    this->isUARTConnected_ = false;
    this->tempMode = false;
    this->wideVaneAdj = false;
    this->functions = heatpumpFunctions();
    this->autoUpdate = false;
    this->firstRun = true;
    this->externalUpdate = false;
    this->lastSend = 0;
    this->infoMode = 0;
    this->lastConnectRqTimeMs = 0;
    this->currentStatus.operating = false;
    this->currentStatus.compressorFrequency = NAN;
    this->currentStatus.inputPower = NAN;
    this->currentStatus.kWh = NAN;
    this->currentStatus.runtimeHours = NAN;
    this->tx_pin_ = -1;
    this->rx_pin_ = -1;

    // Sensors and components initially not assigned.
    this->horizontal_vane_select_ = nullptr;
    this->vertical_vane_select_ = nullptr;
    this->compressor_frequency_sensor_ = nullptr;
    this->input_power_sensor_ = nullptr;
    this->kwh_sensor_ = nullptr;
    this->runtime_hours_sensor_ = nullptr;

    // Power request not supported on all heat pumps (#112)
    this->powerRequestWithoutResponses = 0;

    // Set remote temperature timeout to maximum (i.e. disabled).
    this->remote_temp_timeout_ = 4294967295;
    this->generateExtraComponents();
    this->loopCycle.init();
    this->wantedSettings.resetSettings();
#ifndef USE_ESP32
    this->wantedSettingsMutex = false;
#endif
}

void CN105Climate::set_baud_rate(int baud)
{
    this->baud_ = baud;
    ESP_LOGI(TAG, "Baud rate updated to: %d", baud);
}

void CN105Climate::set_tx_rx_pins(uint8_t tx_pin, uint8_t rx_pin)
{
    this->tx_pin_ = tx_pin;
    this->rx_pin_ = rx_pin;
    ESP_LOGI(TAG, "Configuring UART pins -> TX: %d, RX: %d", tx_pin, rx_pin);
}

void CN105Climate::pingExternalTemperature()
{
    this->set_timeout(SHEDULER_REMOTE_TEMP_TIMEOUT, this->remote_temp_timeout_, [this]()
                      {
    ESP_LOGW(LOG_ACTION_EVT_TAG, "Warning: Remote temperature sensor timed out. Reverting to internal measurement.");
    this->set_remote_temperature(0); });
}

void CN105Climate::set_remote_temp_timeout(uint32_t timeout)
{
    this->remote_temp_timeout_ = timeout;
    if (timeout == 4294967295)
    {
        ESP_LOGI(LOG_ACTION_EVT_TAG, "Remote temperature timeout disabled (set to never).");
    }
    else
    {
        log_info_uint32(LOG_ACTION_EVT_TAG, "Remote temperature timeout set to: ", timeout);
        this->pingExternalTemperature();
    }
}

void CN105Climate::set_debounce_delay(uint32_t delay)
{
    this->debounce_delay_ = delay;
    log_info_uint32(LOG_ACTION_EVT_TAG, "Debounce delay set to: ", delay);
}

float CN105Climate::get_compressor_frequency()
{
    return currentStatus.compressorFrequency;
}

float CN105Climate::get_input_power()
{
    return currentStatus.inputPower;
}

float CN105Climate::get_kwh()
{
    return currentStatus.kWh;
}

float CN105Climate::get_runtime_hours()
{
    return currentStatus.runtimeHours;
}

bool CN105Climate::is_operating()
{
    return currentStatus.operating;
}

void CN105Climate::setupUART()
{
    log_info_uint32(TAG, "Initializing UART with baud rate: ", this->parent_->get_baud_rate());
    this->setHeatpumpConnected(false);
    this->isUARTConnected_ = false;

    // This flag may be toggled via YAML (or a physical button) to force a reconnect.
    this->uart_setup_switch = true;

    if (this->parent_->get_data_bits() == 8 &&
        this->parent_->get_parity() == uart::UART_CONFIG_PARITY_EVEN &&
        this->parent_->get_stop_bits() == 1)
    {
        ESP_LOGD(TAG, "UART correctly configured as SERIAL_8E1.");
        this->isUARTConnected_ = true;
        this->initBytePointer();
    }
    else
    {
        ESP_LOGW(TAG, "UART configuration error: Expected SERIAL_8E1. Please verify data bits, parity, and stop bits.");
    }
}

void CN105Climate::setHeatpumpConnected(bool state)
{
    this->isHeatpumpConnected_ = state;
    if (this->hp_uptime_connection_sensor_ != nullptr)
    {
        if (state)
        {
            this->hp_uptime_connection_sensor_->start();
            ESP_LOGD(TAG, "Heat pump connection established. Starting uptime sensor.");
        }
        else
        {
            this->hp_uptime_connection_sensor_->stop();
            ESP_LOGD(TAG, "Heat pump connection lost. Stopping uptime sensor.");
        }
    }
}

void CN105Climate::disconnectUART()
{
    ESP_LOGD(TAG, "disconnectUART() called. Disconnecting UART and resetting state.");
    this->uart_setup_switch = false;
    this->setHeatpumpConnected(false);
    // Reset state for next connection attempt.
    this->firstRun = true;
    this->publish_state();
}

void CN105Climate::reconnectUART()
{
    ESP_LOGD(TAG, "Attempting UART reconnection...");
    this->lastReconnectTimeMs = CUSTOM_MILLIS;
    this->disconnectUART();
    this->setupUART();
    this->sendFirstConnectionPacket();
}

void CN105Climate::reconnectIfConnectionLost()
{
    long reconnectTimeMs = CUSTOM_MILLIS - this->lastReconnectTimeMs;
    if (reconnectTimeMs < this->update_interval_)
    {
        return;
    }

    if (!this->isHeatpumpConnectionActive())
    {
        long connectTimeMs = CUSTOM_MILLIS - this->lastConnectRqTimeMs;
        if (connectTimeMs > this->update_interval_)
        {
            long elapsedResponseSec = (CUSTOM_MILLIS - this->lastResponseMs) / 1000;
            ESP_LOGW(TAG, "Warning: No response from heat pump for %ld seconds.", elapsedResponseSec);
            ESP_LOGI(TAG, "Heat pump disconnected. Attempting reconnection...");
            this->reconnectUART();
        }
    }
}

bool CN105Climate::isHeatpumpConnectionActive()
{
    long elapsedMs = CUSTOM_MILLIS - this->lastResponseMs;
    return (elapsedMs < MAX_DELAY_RESPONSE_FACTOR * this->update_interval_);
}

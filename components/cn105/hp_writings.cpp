#include "cn105.h"

using namespace esphome;

uint8_t CN105Climate::checkSum(uint8_t bytes[], int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum += bytes[i];
    }
    return (0xfc - sum) & 0xff;
}

void CN105Climate::sendFirstConnectionPacket()
{
    if (this->isUARTConnected_)
    {
        // Update the reconnect timestamp to prevent excessive reconnections.
        this->lastReconnectTimeMs = CUSTOM_MILLIS;
        this->setHeatpumpConnected(false);
        ESP_LOGD(TAG, "Sending initial connection packet...");
        uint8_t packet[CONNECT_LEN];
        memcpy(packet, CONNECT, CONNECT_LEN);

        // Send the packet; note that checkIsActive is false as this is the first packet.
        this->writePacket(packet, CONNECT_LEN, false);

        this->lastSend = CUSTOM_MILLIS;
        this->lastConnectRqTimeMs = CUSTOM_MILLIS;
        this->nbHeatpumpConnections_++;

        // Wait for a 10-second timeout to verify the heat pump has responded.
        this->set_timeout("checkFirstConnection", 10000, [this]()
                          {
      if (!this->isHeatpumpConnected_) {
        ESP_LOGE(TAG, "Heat pump did not respond: NOT CONNECTED");
        ESP_LOGI(TAG, "Retrying connection...");
        this->sendFirstConnectionPacket();
      } });
    }
    else
    {
        ESP_LOGE(TAG, "UART connection is not available...");
        this->setupUART();
        // Prevent logging flood by delaying reconnection attempts.
        CUSTOM_DELAY(750);
    }
}

void CN105Climate::prepareInfoPacket(uint8_t *packet, int length)
{
    ESP_LOGV(TAG, "Preparing info packet...");
    memset(packet, 0, length * sizeof(uint8_t));
    for (int i = 0; i < INFOHEADER_LEN && i < length; i++)
    {
        packet[i] = INFOHEADER[i];
    }
}

void CN105Climate::prepareSetPacket(uint8_t *packet, int length)
{
    ESP_LOGV(TAG, "Preparing set packet...");
    memset(packet, 0, length * sizeof(uint8_t));
    for (int i = 0; i < HEADER_LEN && i < length; i++)
    {
        packet[i] = HEADER[i];
    }
}

void CN105Climate::writePacket(uint8_t *packet, int length, bool checkIsActive)
{
    if (this->isUARTConnected_ &&
        (this->isHeatpumpConnectionActive() || (!checkIsActive)))
    {

        ESP_LOGD(TAG, "Writing packet to hardware...");
        this->hpPacketDebug(packet, length, "WRITE");

        for (int i = 0; i < length; i++)
        {
            this->get_hw_serial_()->write_byte(static_cast<uint8_t>(packet[i]));
        }
        // Prevent rapid re-transmissions.
        this->lastSend = CUSTOM_MILLIS;
    }
    else
    {
        ESP_LOGW(TAG, "UART not connected; cannot write packet.");
        this->reconnectUART();
        ESP_LOGW(TAG, "Deferring packet writing until reconnection is complete...");
        this->set_timeout("write", 4000, [this, packet, length]()
                          { this->writePacket(packet, length); });
    }
}

const char *CN105Climate::getModeSetting()
{
    return (this->wantedSettings.mode) ? this->wantedSettings.mode : this->currentSettings.mode;
}

const char *CN105Climate::getPowerSetting()
{
    return (this->wantedSettings.power) ? this->wantedSettings.power : this->currentSettings.power;
}

const char *CN105Climate::getVaneSetting()
{
    return (this->wantedSettings.vane) ? this->wantedSettings.vane : this->currentSettings.vane;
}

const char* CN105Climate::getWideVaneSetting() {
    if (this->wantedSettings.wideVane) {
        if (strcmp(this->wantedSettings.wideVane, lookupByteMapValue(WIDEVANE_MAP, WIDEVANE, 8, 0x80 & 0x0F)) == 0 && !this->currentSettings.iSee) {
            this->wantedSettings.wideVane = this->currentSettings.wideVane;
        }
        return this->wantedSettings.wideVane;
    } else {
        return this->currentSettings.wideVane;
    }
}

const char *CN105Climate::getFanSpeedSetting()
{
    return (this->wantedSettings.fan) ? this->wantedSettings.fan : this->currentSettings.fan;
}

float CN105Climate::getTemperatureSetting() {
    if (this->wantedSettings.temperature != -1.0) {
        return this->wantedSettings.temperature;
    } else {
        return this->currentSettings.temperature;
    }
}



void CN105Climate::createPacket(uint8_t *packet)
{
    prepareSetPacket(packet, PACKET_LEN);

    ESP_LOGD(TAG, "Building packet for transmission...");

    if (this->wantedSettings.power != nullptr)
    {
        ESP_LOGD(TAG, "Setting power: %s", getPowerSetting());
        packet[8] = POWER[lookupByteMapIndex(POWER_MAP, 2, getPowerSetting(), "power (write)")];
        packet[6] += CONTROL_PACKET_1[0];
    }

    if (this->wantedSettings.mode != nullptr)
    {
        ESP_LOGD(TAG, "Setting mode: %s", getModeSetting());
        packet[9] = MODE[lookupByteMapIndex(MODE_MAP, 5, getModeSetting(), "mode (write)")];
        packet[6] += CONTROL_PACKET_1[1];
    }

    if (wantedSettings.temperature != -1)
    {
        if (!tempMode)
        {
            ESP_LOGD(TAG, "Setting temperature (non-temp mode): %f", getTemperatureSetting());
            packet[10] = TEMP[lookupByteMapIndex(TEMP_MAP, 16, getTemperatureSetting(), "temperature (write)")];
            packet[6] += CONTROL_PACKET_1[2];
        }
        else
        {
            ESP_LOGD(TAG, "Setting temperature (temp mode enabled): %f", getTemperatureSetting());
            float temp = (getTemperatureSetting() * 2) + 128;
            packet[19] = static_cast<int>(temp);
            packet[6] += CONTROL_PACKET_1[2];
        }
    }

    if (this->wantedSettings.fan != nullptr)
    {
        ESP_LOGD(TAG, "Setting fan speed: %s", getFanSpeedSetting());
        packet[11] = FAN[lookupByteMapIndex(FAN_MAP, 6, getFanSpeedSetting(), "fan (write)")];
        packet[6] += CONTROL_PACKET_1[3];
    }

    if (this->wantedSettings.vane != nullptr)
    {
        ESP_LOGD(TAG, "Setting vane: %s", getVaneSetting());
        packet[12] = VANE[lookupByteMapIndex(VANE_MAP, 7, getVaneSetting(), "vane (write)")];
        packet[6] += CONTROL_PACKET_1[4];
    }

    if (this->wantedSettings.wideVane != nullptr) {
        ESP_LOGD(TAG, "heatpump widevane -> %s", getWideVaneSetting());
        packet[18] = WIDEVANE[lookupByteMapIndex(WIDEVANE_MAP, 7, getWideVaneSetting(), "wideVane (write)")] | (this->wideVaneAdj ? 0x80 : 0x00);
        packet[7] += CONTROL_PACKET_2[0];
    }

    // Append the checksum.
    uint8_t chkSum = checkSum(packet, 21);
    packet[21] = chkSum;
}

void CN105Climate::publishWantedSettingsStateToHA()
{
    // Validate and update mode and power settings if available.
    if ((this->wantedSettings.mode != nullptr) || (this->wantedSettings.power != nullptr))
    {
        checkPowerAndModeSettings(this->wantedSettings, false);
        this->updateAction();
    }

    if (this->wantedSettings.fan != nullptr)
    {
        checkFanSettings(this->wantedSettings, false);
    }

    if ((this->wantedSettings.vane != nullptr) || (this->wantedSettings.wideVane != nullptr))
    {
        // Prevent null pointer issues by ensuring both settings are valid.
        if (this->wantedSettings.vane == nullptr)
        {
            this->wantedSettings.vane = this->currentSettings.vane;
        }
        if (this->wantedSettings.wideVane == nullptr)
        {
            this->wantedSettings.wideVane = this->currentSettings.wideVane;
        }
        checkVaneSettings(this->wantedSettings, false);
    }

    // Update target temperature for Home Assistant.
    this->target_temperature = this->getTemperatureSetting();
    this->publish_state();
}

void CN105Climate::publishWantedRunStatesStateToHA() {
    if (this->wantedRunStates.airflow_control != nullptr) {
        if (this->wantedRunStates.airflow_control == nullptr) {
            this->wantedRunStates.airflow_control = this->currentRunStates.airflow_control;
        }
        if (this->hasChanged(this->airflow_control_select_->state.c_str(), this->wantedRunStates.airflow_control, "select airflow control")) {
            ESP_LOGI(TAG, "airflow control setting changed");
            this->airflow_control_select_->publish_state(wantedRunStates.airflow_control);
        }
    }
    if (this->wantedRunStates.air_purifier > -1) {
        if (this->wantedRunStates.air_purifier == -1) {
            this->wantedRunStates.air_purifier = this->currentRunStates.air_purifier;
        }
        if (this->air_purifier_switch_->state != this->wantedRunStates.air_purifier) {
            ESP_LOGI(TAG, "air purifier setting changed");
            this->air_purifier_switch_->publish_state(wantedRunStates.air_purifier);
        }
    }
    if (this->wantedRunStates.night_mode > -1) {
        if (this->wantedRunStates.night_mode == -1) {
            this->wantedRunStates.night_mode = this->currentRunStates.night_mode;
        }
        if (this->night_mode_switch_->state != this->wantedRunStates.night_mode) {
            ESP_LOGI(TAG, "night mode setting changed");
            this->night_mode_switch_->publish_state(wantedRunStates.night_mode);
        }
    }
    if (this->wantedRunStates.circulator > -1) {
        if (this->wantedRunStates.circulator == -1) {
            this->wantedRunStates.circulator = this->currentRunStates.circulator;
        }
        if (this->circulator_switch_->state != this->wantedRunStates.circulator) {
            ESP_LOGI(TAG, "circulator setting changed");
            this->circulator_switch_->publish_state(wantedRunStates.circulator);
        }
    }
}

void CN105Climate::sendWantedSettingsDelegate()
{
    this->wantedSettings.hasBeenSent = true;
    this->lastSend = CUSTOM_MILLIS;
    ESP_LOGI(TAG, "Transmitting updated settings...");
    this->debugSettings("wantedSettings", wantedSettings);

    uint8_t packet[PACKET_LEN] = {};
    this->createPacket(packet);
    this->writePacket(packet, PACKET_LEN);
    this->hpPacketDebug(packet, 22, "WRITE_SETTINGS");

    // Publish updated state to Home Assistant.
    this->publishWantedSettingsStateToHA();

    // Reset desired settings after transmission.
    this->wantedSettings.resetSettings();

    // Defer the next cycle to allow the heat pump time to process the current command.
    this->loopCycle.deferCycle();
}

/**
 * Builds and transmits the update packet with the desired settings.
 */
void CN105Climate::sendWantedSettings()
{
    if (this->isHeatpumpConnectionActive() && this->isUARTConnected_)
    {
        if (CUSTOM_MILLIS - this->lastSend > 300)
        {
#ifdef USE_ESP32
            std::lock_guard<std::mutex> guard(wantedSettingsMutex);
            this->sendWantedSettingsDelegate();
#else
            this->emulateMutex("WRITE_SETTINGS", std::bind(&CN105Climate::sendWantedSettingsDelegate, this));
#endif
        }
        else
        {
            ESP_LOGD(TAG, "Delaying settings transmission; previous packet sent too recently.");
        }
    }
    else
    {
        this->reconnectIfConnectionLost();
    }
}



void CN105Climate::buildAndSendRequestPacket(int packetType) {
    uint8_t packet[PACKET_LEN] = {};
    createInfoPacket(packet, packetType);
    this->writePacket(packet, PACKET_LEN);
}

void CN105Climate::buildAndSendRequestsInfoPackets()
{
    if (this->isHeatpumpConnected_)
    {
        ESP_LOGV(LOG_UPD_INT_TAG, "Triggering info packet due to update interval.");
        ESP_LOGV("CONTROL_WANTED_SETTINGS", "hasChanged is %s", wantedSettings.hasChanged ? "true" : "false");
        ESP_LOGD(TAG, "Sending request for settings packet (0x02)...");
        this->loopCycle.cycleStarted();
        this->nbCycles_++;
        ESP_LOGD(LOG_CYCLE_TAG, "2a: Sending settings request (0x02).");
        this->buildAndSendRequestPacket(RQST_PKT_SETTINGS);
    }
    else
    {
        this->reconnectIfConnectionLost();
    }
}

void CN105Climate::createInfoPacket(uint8_t *packet, uint8_t packetType)
{
    ESP_LOGD(TAG, "Creating info packet...");
    // Append header.
    for (int i = 0; i < INFOHEADER_LEN; i++)
    {
        packet[i] = INFOHEADER[i];
    }

    // Set info mode: either based on the provided packet type or current info mode.
    if (packetType != PACKET_TYPE_DEFAULT)
    {
        packet[5] = INFOMODE[packetType];
    }
    else
    {
        packet[5] = INFOMODE[infoMode];
        infoMode = (infoMode == (INFOMODE_LEN - 1)) ? 0 : infoMode + 1;
    }

    // Pad the remainder of the packet.
    for (int i = 0; i < 15; i++)
    {
        packet[i + 6] = 0x00;
    }

    // Append checksum.
    uint8_t chkSum = checkSum(packet, 21);
    packet[21] = chkSum;
}

void CN105Climate::sendRemoteTemperature()
{
    this->shouldSendExternalTemperature_ = false;

    uint8_t packet[PACKET_LEN] = {};
    prepareSetPacket(packet, PACKET_LEN);

    packet[5] = 0x07; // Packet identifier for remote temperature update.
    if (this->remoteTemperature_ > 0)
    {
        packet[6] = 0x01;
        float temp = round(this->remoteTemperature_ * 2);
        packet[7] = static_cast<uint8_t>(temp - 16);
        packet[8] = static_cast<uint8_t>(temp + 128);
    }
    else
    {
        packet[8] = 0x80; // For certain models, send 0x80 when temperature is not positive.
    }

    // Append checksum.
    uint8_t chkSum = checkSum(packet, 21);
    packet[21] = chkSum;

    ESP_LOGD(LOG_REMOTE_TEMP, "Sending remote temperature update packet (value: %f)", this->remoteTemperature_);
    writePacket(packet, PACKET_LEN);

    // Reset the remote temperature sensor timeout.
    this->pingExternalTemperature();
}

void CN105Climate::sendWantedRunStates() {
    uint8_t packet[PACKET_LEN] = {};
    
    prepareSetPacket(packet, PACKET_LEN);
    
    packet[5] = 0x08;
    if (this->wantedRunStates.airflow_control != nullptr) {
        ESP_LOGD(TAG, "airflow control -> %s", getAirflowControlSetting());
        packet[11] = AIRFLOW_CONTROL[lookupByteMapIndex(AIRFLOW_CONTROL_MAP, 3, getAirflowControlSetting(), "run state (write)")];
        packet[6] += RUN_STATE_PACKET_1[4];
    }
    if (this->wantedRunStates.air_purifier > -1) {
        if (getAirPurifierRunState() != currentRunStates.air_purifier) {
            ESP_LOGI(TAG, "air purifier switch state -> %s", getAirPurifierRunState() ? "ON" : "OFF");
            packet[17] = getAirPurifierRunState() ? 0x01 : 0x00;
            packet[7] += RUN_STATE_PACKET_2[1];
        }
    }
    if (this->wantedRunStates.night_mode > -1) {
        if (getNightModeRunState() != currentRunStates.night_mode) {
            ESP_LOGI(TAG, "night mode switch state -> %s", this->getNightModeRunState() ? "ON" : "OFF");
            packet[18] = getNightModeRunState() ? 0x01 : 0x00;
            packet[7] += RUN_STATE_PACKET_2[2];
        }
    }
    if (this->wantedRunStates.circulator > -1) {
        if (getCirculatorRunState() != currentRunStates.circulator) {
            ESP_LOGI(TAG, "circulator switch state -> %s", getCirculatorRunState() ? "ON" : "OFF");
            packet[19] = getCirculatorRunState() ? 0x01 : 0x00;
            packet[7] += RUN_STATE_PACKET_2[3];
        }
    }
    
    // Add the checksum
    uint8_t chkSum = checkSum(packet, 21);
    packet[21] = chkSum;
    ESP_LOGD(LOG_SET_RUN_STATE, "Sending set run state package (0x08)");
    writePacket(packet, PACKET_LEN);
    
    this->publishWantedRunStatesStateToHA();
    
    this->wantedRunStates.resetSettings();
    this->loopCycle.deferCycle();
}

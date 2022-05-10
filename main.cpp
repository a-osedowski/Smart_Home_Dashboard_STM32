/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "AnalogIn.h"
#include "GattCharacteristic.h"
#include "PinNames.h"
#include <cstdio>
#define __BLE_ENVIRONMENTAL_SERVICE_H__
 
#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "ble/services/HeartRateService.h"
#include "ble/services/EnvironmentalService.h"
#include "pretty_printer.h"
#include <Dht11.h>
 
const static char DEVICE_NAME[] = "Measurement Node";
 
static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);
Dht11 sensor(PC_5);
AnalogIn input(PC_3);
 
class MeasurementNode : ble::Gap::EventHandler {
public:
    typedef int16_t  TemperatureType_t;
    typedef uint16_t HumidityType_t;
    typedef uint32_t PressureType_t;
    typedef uint16_t ResistanceType_t;

    /**
     * @brief   EnvironmentalService constructor.
     * @param   ble Reference to BLE device.
     * @param   temperature_en Enable this characteristic.
     * @param   humidity_en Enable this characteristic.
     * @param   pressure_en Enable this characteristic.
     */
    MeasurementNode(BLE& _ble, events::EventQueue &event_queue) :
        _ble(_ble),
        _event_queue(event_queue),
        _led1(LED1, 1),
        _connected(false),
        // _temp_uuid(GattService::UUID_ENVIRONMENTAL_SERVICE),
        temperatureCharacteristic(GattCharacteristic::UUID_TEMPERATURE_CHAR, &temperature),
        humidityCharacteristic(GattCharacteristic::UUID_HUMIDITY_CHAR, &humidity),
        pressureCharacteristic(GattCharacteristic::UUID_PRESSURE_CHAR, &pressure),
        resistanceCharacteristic(0x2B03, &resistance),
        _adv_data_builder(_adv_buffer) { }


 
    void start() {
        _ble.gap().setEventHandler(this);
 
        _ble.init(this, &MeasurementNode::on_init_complete);

        printf("cos");

        
 
        _event_queue.call_every(500, this, &MeasurementNode::blink);
        // _event_queue.call_every(1000, this, &HeartrateDemo::update_sensor_value);

        // _event_queue.call_every(10s, this, &MeasurementNode::updateValue);

        _event_queue.call_every(2s, this, &MeasurementNode::updateTest);

        // _event_queue.call_every(10s, this, &MeasurementNode::lightTest);
        
        updateTemperature(28.6);
        // updatePressure(1020);
 
        _event_queue.dispatch_forever();
    }
 
private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }
 
        print_mac_address();
 
        start_advertising();
    }
 
    void start_advertising() {
        /* Create advertising parameters and payload */
 
        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );
        // updateTemperature(27.4);
        // updatePressure(0x3FC);

        static bool serviceAdded = false; /* We should only ever need to add the information service once. */
        if (serviceAdded) {
            return;
        }

        GattCharacteristic *charTable[] = { &humidityCharacteristic,
                                            &pressureCharacteristic,
                                            &temperatureCharacteristic,
                                            &resistanceCharacteristic };

        GattService environmentalService(GattService::UUID_ENVIRONMENTAL_SERVICE, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

        _ble.gattServer().addService(environmentalService);
        serviceAdded = true;
 
        _adv_data_builder.setFlags();
        // _adv_data_builder.setAppearance(ble::adv_data_appearance_t::GENERIC_THERMOMETER);
        // _adv_data_builder.setLocalServiceList(mbed::make_Span(&_temp_uuid, 1));
        _adv_data_builder.setName(DEVICE_NAME);
 
        /* Setup advertising */
 
        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );
 
        if (error) {
            printf("_ble.gap().setAdvertisingParameters() failed\r\n");
            return;
        }
 
        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );
 
        if (error) {
            printf("_ble.gap().setAdvertisingPayload() failed\r\n");
            return;
        }
 
        /* Start advertising */
 
        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
 
        if (error) {
            printf("_ble.gap().startAdvertising() failed\r\n");
            return;
        }
    }
 

     /**
     * @brief   Update humidity characteristic.
     * @param   newHumidityVal New humidity measurement.
     */
    void updateHumidity(HumidityType_t newHumidityVal)
    {
        humidity = (HumidityType_t) (newHumidityVal * 100);
        _ble.gattServer().write(humidityCharacteristic.getValueHandle(), (uint8_t *) &humidity, sizeof(HumidityType_t));
    }

    /**
     * @brief   Update pressure characteristic.
     * @param   newPressureVal New pressure measurement.
     */
    void updatePressure(float newPressureVal)
    {
        pressure = (PressureType_t) (newPressureVal);
        _ble.gattServer().write(pressureCharacteristic.getValueHandle(), (uint8_t *) &pressure, sizeof(PressureType_t));
    }

     void updateResistance(int newResistanceVal)
    {
        float voltage = newResistanceVal * (5.0/1023) * 1000;
        float resistanceVal = 10000 * ( voltage / ( 5000.0 - voltage) );
        int resistanceInt = static_cast<int>(resistanceVal);
        resistance = (ResistanceType_t) (resistanceInt);
        _ble.gattServer().write(resistanceCharacteristic.getValueHandle(), (uint8_t *) &resistance, sizeof(ResistanceType_t));
    }

    /**
     * @brief   Update temperature characteristic.
     * @param   newTemperatureVal New temperature measurement.
     */
    void updateTemperature(float newTemperatureVal)
    {
        temperature = (TemperatureType_t) (newTemperatureVal*100);
        _ble.gattServer().write(temperatureCharacteristic.getValueHandle(), (uint8_t *) &temperature, sizeof(TemperatureType_t));
    }

    // void updateLight(float newLightVal)
    // {
    //     light= (TemperatureType_t) (newTemperatureVal);
    //     _ble.gattServer().write(temperatureCharacteristic.getValueHandle(), (uint8_t *) &temperature, sizeof(TemperatureType_t));
    // }
 
    void blink(void) {
        _led1 = !_led1;
    }

    void updateValue(void){
        int ret = sensor.read();
        printf("Working?:%d", ret);
        int hum = sensor.getHumidity();
        printf("Hummildity:%d", hum);
        printf("Temp:%d", sensor.getCelsius());
        updateTemperature(sensor.getCelsius());
        updatePressure(sensor.getHumidity());
    }

      void updateTest(void){
        test_Counter += 0.7;
        updateTemperature(test_Counter);
        int light = input.read_u16();
        printf("Light:%d", light);
        updateResistance(light);
    }

    void lightTest(void){

        #define NUM_SAMPLES 1024
        uint16_t samples[NUM_SAMPLES];

        for (int i = 0; i < NUM_SAMPLES; i++) {
            samples[i] = input.read_u16();
            ThisThread::sleep_for(1);
        }

        printf("Results:\n");
        for (int i = 0; i < NUM_SAMPLES; i++) {
            printf("%d, 0x%04X\n", i, samples[i]);
        }
    }

 
private:
    /* Event handler */
 
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _connected = false;
    }
 
    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            _connected = true;
        }
    }
 
private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    DigitalOut _led1;
    TemperatureType_t temperature;
    HumidityType_t    humidity;
    PressureType_t    pressure;
    ResistanceType_t resistance;
    

    ReadOnlyGattCharacteristic<TemperatureType_t> temperatureCharacteristic;
    ReadOnlyGattCharacteristic<HumidityType_t>    humidityCharacteristic;
    ReadOnlyGattCharacteristic<PressureType_t>    pressureCharacteristic;
    ReadOnlyGattCharacteristic<ResistanceType_t> resistanceCharacteristic;
 
    bool _connected;
 
    // UUID _temp_uuid;
 
    // uint8_t _temp_data;
    // HeartRateService _temp_service;
    // EnvironmentalService _temp_service;
 
    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;

    float test_Counter;
};
 
/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}
 
int main()
{
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);
 
    MeasurementNode demo(ble, event_queue);
    demo.start();
 
    return 0;
}
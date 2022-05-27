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
#include <DHT.h>
 
const static char DEVICE_NAME[] = "Measurement Node";
 
static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);
DHT dht(PC_2, DHT::DHT11);
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
        temperatureCharacteristic(GattCharacteristic::UUID_TEMPERATURE_CHAR, &temperature),
        humidityCharacteristic(GattCharacteristic::UUID_HUMIDITY_CHAR, &humidity),
        resistanceCharacteristic(0x2a6d, &resistance),
        _adv_data_builder(_adv_buffer) { }


 
    void start() {
        _ble.gap().setEventHandler(this);
 
        _ble.init(this, &MeasurementNode::on_init_complete);

        printf("Charactristic temp %d", GattCharacteristic::UUID_TEMPERATURE_CHAR);
        test_Counter = 0;
        
        _event_queue.call_every(500, this, &MeasurementNode::blink);
        _event_queue.call_every(1min, this, &MeasurementNode::updateValues);
 
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
        static bool serviceAdded = false; /* We should only ever need to add the information service once. */
        if (serviceAdded) {
            return;
        }

        GattCharacteristic *charTable[] = { &humidityCharacteristic,
                                            &temperatureCharacteristic,
                                            &resistanceCharacteristic };

        GattService environmentalService(GattService::UUID_ENVIRONMENTAL_SERVICE, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));

        _ble.gattServer().addService(environmentalService);
        serviceAdded = true;
 
        _adv_data_builder.setFlags();
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
     * @brief   Update light intensity characteristic.
     * @param   newLightVal New sensor voltage for light intensity measurement.
     */
     void updateResistance(int newLightVal)
    {
        float voltage = newLightVal * (5.0/1023) * 1000;
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
        temperature = (TemperatureType_t) (newTemperatureVal*10);
        _ble.gattServer().write(temperatureCharacteristic.getValueHandle(), (uint8_t *) &temperature, sizeof(TemperatureType_t));
    }

 
    void blink(void) {
        _led1 = !_led1;
    }

      void updateValues(void){
        int hum;
        int temp;
        int light = input.read_u16();
        int err = dht.read();
        printf("%d", err);
        temp = dht.getTemperature();
        hum = dht.getHumidity();
    
        updateResistance(light);
        updateHumidity(hum);
        updateTemperature(temp);
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
    ReadOnlyGattCharacteristic<ResistanceType_t> resistanceCharacteristic;
 
    bool _connected;
 
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
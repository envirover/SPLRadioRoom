/*
 BLEConfig.cpp

 Iridium SBD telemetry for ArduPilot.
 
 (C) Copyright 2017 Envirover.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "BLEConfig.h"

BLEConfig::BLEConfig() : 
  ble_service("861c0e0a-ded3-11e6-bf01-fe55135034f3"),
  ble_high_latency_period("a4cf3be2-ded3-11e6-bf01-fe55135034f3", BLERead | BLEWrite)
{
}

void BLEConfig::init()
{
  // set the local name peripheral advertises
  ble_peripheral.setLocalName(BLE_SERVICE_NAME);
  
  // set the UUID for the service this peripheral advertises:
  ble_peripheral.setAdvertisedServiceUuid(ble_service.uuid());

  // add service and characteristics
  ble_peripheral.addAttribute(ble_service);
  ble_peripheral.addAttribute(ble_high_latency_period);

  // advertise the service
  ble_peripheral.begin();
}

unsigned long BLEConfig::getHighLatencyMsgPeriod()
{
  ble_peripheral.poll();
  
  return ble_high_latency_period.value();
}

void BLEConfig::setHighLatencyMsgPeriod(unsigned long period)
{
  ble_high_latency_period.setValue(period);
}


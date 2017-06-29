/*
 SPLConfig.h

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

#define EEPROM_HL_PERIOD_ADDRESS 0

#define DEFAULT_REPORT_PERIOD 300000L // 5 minutes

/**
 * SPL configuration service.
 */
class SPLConfig
{
  unsigned long reportPeriod;
  
 public:
   SPLConfig();

   void init();

   unsigned long getReportPeriod();
   void setReportPeriod(unsigned long period);
 };


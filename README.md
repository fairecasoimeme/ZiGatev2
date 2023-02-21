# ZiGatev2

⚠️ **For more security, please [clone your actual ZiGate+](https://zigate.fr/documentation/cloner-la-zigate-pizigate/) before update** 

## Changelog

### Version 3.A0

* Increase PDM capacity
* Add more complete devices List
   - Add new structure (to enhance devices list)
* Add new API
   - Add Binding Table API (0x0052 command)
   - Add Routing Table API (0x0053 command)
   - Add Get Network key API (0x0054 command)
* Force route and lqi request to be more responsive
* Fix datatype for reportable change
* Fix zone enroll bug
* Fix Extended debug
* Backtrack with previous functions (v3.22) (about API and automatic repair - no longer useful)
* Update new SDK 2.6.10
	- Improve PDM mechanism, stability and fix PDM errors
	- Improve OTA mechanism and support
	- Improve Zigbee compliant

### Version 3.22

* Add child table size function. Command 0x0052
* Add delete PDM address map table function. Command 0x0051
* Add automatic repair when 0x87 messages occurs. (This automatism delete adress map table and reset the ZiGate) 
* Enhance group capacity for ZiGate 5 to 16
* Fix inconsistent datas with 0x8002 messages due to bad default response with ZDP packet
* Fix 0x8b warning messages. increase the BroadcastTransactionTableSize from 9 to 18
* Update new SDK 2.6.5
	- New error messages due to security fails

### Version 3.21

* Add 0x0b04 decodage
* Add raw message when there are errors on write attribute command
* Fix uint48 zcl values to the client
* Fix for Xiaomi and Lumi devices which ask manufacture code (0x115f) whith node descriptor request
* Fix BindGroup command
* Fix manufacture code by default --> 0x1037


### Version 3.20

* Add updateReportableChange conversion function
* Add HeartBeat command (to maintain TCP connection)
* Fix conversion data type
* Fix IASZONE to be more flexible
* Fix Wiser endpoint
* Fix RAW_MODE bug
* Fix NwkSteering function
* Fix getApduUsed() function in status message
* Update new SDK 2.6.4



### Version 3.1f

* Enhance 0x8140 INDIVIDUAL_ATTRIBUTE_EXTENDED_RESPONSE
* Fix sqn issue (used by domoticz)
* Fix reactive flow control
* Fix overflow on 0x8032 command

### Version 3.1e

initial version

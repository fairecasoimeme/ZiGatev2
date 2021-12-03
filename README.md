# ZiGatev2

## Changelog

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
# TTN-whisperer
Lowpower optimized LoRaWAN node connecting to The Things Network, running on a Whisper Node LoRa board

This sketch is a low power optimized LoRaWan node for sending sensor data to TTN (The Things Network)
with a WhisperNode Lora from Wisen.co.au. The data sent to TTN is the temperature read from a DS18B20
sensor and the battery voltage, measured with the on-board voltage divider. It can be used as a starting
point for own applications based on the WhisperNode Lora.

Power optimisations:
- no LED usage
- only two bytes of LoRa payload
- sleep for a long tim
- send measurement only if difference in value
- memorize OneWire sensor adresses on startup
- deactivate voltage divider when not used
- use lower DS18B20 resolution
- sent raw ADC data, calculation on server side

Further power optimisations to be done in future:
- reduce serial speed in DEBUG mode
- power off OneWire bus when not used
- change delay of OneWire library to a sleep
- put on-board Flash to powerdown mode
- set all unused pins to input/pullup

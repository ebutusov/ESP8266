# ESP8266
Various ESP8266 sketches

## cmqtt.ino
This is a simple sketch which can be used to monitor temperatures and control a relay.
It connects to an MQTT broker (e.g. mosquitto, there is a free broker at cloudmqtt.com) to send temperature and relay status info, and subscribes to topic which allows to control the relay (message 0 - off, 1 - on).
It uses additional libraries: OneWire, DallasTemperature, PubSubClient (all of these can be found on github).
Messages can be monitored with mqqt-spy.

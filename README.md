# BLE-detect

This is a very low latency BLE based vehicle arrival and departure detection system reporting over MQTT. Originally designed to work with Home Assistant, it can also be used with any other automation system that supports MQTT.

## Principle

BLE-detect uses an inexpensive Raspberry Pi Zero W board to detect nearby BLE beacons using passive scanning. The scanning and detection is performed in hardware by the Bluetooth chip and offers very reliable and extremely low latency detection in the milliseconds range. A BLE beacon is placed in a car and the Pi Zero W is placed close to the driveway the car passes through. When the car drives by, its BLE beacon signal is received by the detector and MQTT messages are sent for the car entering and exiting the scanner range. Multiple cars can be differentiated by their beacon GUIDs.

![image](https://user-images.githubusercontent.com/60828821/159815790-ddfde66c-9e0d-44e3-a955-cbe94c7a1f95.png)

Home assistant processes the MQTT messages and exposes a sensor that can be used in further automations.

## Hardware and software setup

* A Raspberry Pi Zero W. Doing the scanning, it connects to the MQTT broker over either wifi or over wired Ethernet (requires an additional Ethernet module connected to the GPIO or USB). Wired is recommended for best long term stability.
* A BLE beacon in the car. iBeacon or compatible clones are supported. It is recommended to use a beacon with permanent USB connection to the car, which will be turned on with the cars ignition. The DSD Tech SH-A10 beacon was tested, but other clones should work too. Their GUIDs, tx power and signal repeat delays can be freely configured.
* An MQTT broker. We recommend Mosquitto.
* An automation system like Home Assistant.

## Prerequisites

Install Bluetooth dev libraries

`sudo apt install libbluetooth-dev`

Install paho MQTT client libraries

```
git clone https://github.com/eclipse/paho.mqtt.c
cd paho.mqtt.c
make
sudo make install
```

## Configuring 

Clone this repo, go to the src folder and edit `beacon-detect.cpp`. Under configuration, customize the appropriate defines for your system:

`DEVICEFILE` the absolute path pointing to the beacon identity file, which contains the authorized GUIDs of your beacons in your car(s). There is an example `known_devices.txt` supplied. Edit it for your beacons. You can add new devices here or revoke existing ones.

`LOGFILE` the absolute path where BLE-detect will put its log file.

`ADDRESS` the URL to your MQTT broker.

`MIN_DETECTION_RSSI` the minimum signal strength in dBm the BLE signal need for a successful detection. You can adjust the detection range with this.

Note: if the paths above are wrong, you will get a SEGFAULT. There is no error checking at this time.

## Building

When you have the configuration customized to your needs, compile the tool. Make sure you're in the `src` folder:

`g++ -o bdetect ./beacon_detect.cpp -lbluetooth -lpaho-mqtt3c -Wno-psabi`

Launch the binary to start the detector:

`sudo ./bdetect`

The user running the binary needs access permissions to the Bluetooth device.

## MQTT messages

When a beacon enters or exits the detection range, and the beacon was recognized as known / authorized, the tool will publish an MQTT message that looks like this:

`blescanner/MyBeacon { "state" : "on" }`

The MyBeacon name is replaced by the name of the recognized beacon, as it appears in the know_devices.txt file (see section above). The state will be `on` when the beacon is entering the range or `off` when it leaves the range.

For Home Assistant, an MQTT sensor can be configured like this:

```yaml
sensor:
  - platform: mqtt
    name: car_beacon
    state_topic: "blescanner/MyBeacon"
    value_template: "{{ value_json.state }}"
```

## A note on general BLE security

Do not use a BLE based arrival detection to perform safety critical operations like unlocking doors or opening garages. A BLE beacon signal can be trivial to clone by using readily available smartphone apps and it can be done without you noticing from a distance. You can however use such detection for non critical arrival automations, like turning on lights or announcing an arrival. You can also use it for departure automations, like locking doors or arming an alarm system when your vehicle leaves the driveway. Changing your GUIDs from time to time is probably a good idea too.

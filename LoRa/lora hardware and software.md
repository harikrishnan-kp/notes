# LoRa HARDWARE

Criteria need to be considered while selecting a LoRa module
- **Frequency Band**: Determine the frequency band that is appropriate for your region and regulatory requirements. Common frequency bands for LoRa communication include 433 MHz, 868 MHz, and 915 MHz.
- **Communication Range**: Evaluate the required communication range or your application. Consider factors such as terrain, obstacles, and desired coverage area. Modules with higher transmit power and
    sensitivity typically offer longer communication ranges.
- **Data Rate**: Determine the required data rate for your pplication. LoRa modules support adjustable data rates ranging from a few hundred bits per second (bps) to several hundred kilobits per second (kbps). Select a module that can meet the data rate requirements of your application while balancing power consumption and communication range.
- **Power Consumption**: Consider the power consumption of the LoRa module, especially if your application is battery-powered or requires low power operation. Look for modules with low power consumption in both active and sleep modes to maximize battery life.
- **Interface**: Evaluate the interface options provided by the LoRa module for communication with external microcontrollers or host systems. This interface is used to configure the module\'s settings,  initiate transmissions, and receive data. Most modules use a Serial Peripheral Interface (SPI) for configuration and data exchange.
- **Compatibility**: These modules must be compatible with popular development platforms and microcontrollers, making them easy to integrate into existing projects.
- **Antenna Options**: Consider the antenna options available with the LoRa module. Some modules come with onboard antennas, while others provide connectors for external antennas. Select the appropriate
    antenna type and configuration based on your application requirements and environmental conditions.
- **Features**: Evaluate the additional features offered by the LoRa module, such as built-in error detection and correction mechanisms,frequency hopping capability, support for multiple spreading
    factors, and encryption options. Choose a module that provides the necessary features to optimize communication performance and reliability.
- **Cost**: Consider the cost of the LoRa module and its overall value proposition for your project.

## LoRa transceiver modules
LoRa (Long Range) transceiver modules are widely used for long-range communication in IoT (Internet of Things) applications. Here are a few popular LoRa transceiver modules:

- **Semtech SX1276/SX1278**: These are among the most common LoRa transceiver chips used in modules. They operate in the 868 MHz and 915 MHz bands and offer excellent sensitivity and long-range communication capabilities.
- **HopeRF RFM95/RFM96**: These modules are based on the Semtech SX1276/SX1278 chips and offer similar features. They are widely used in DIY and commercial LoRa applications.
- **Microchip RN2483/RN2903**: These modules integrate LoRa technology with a microcontroller, making them easy to use for IoT applications. They come with firmware that simplifies LoRaWAN connectivity.
- **Pycom LoPy/LoPy4**: These are development boards that integrate LoRa, Wi-Fi, and Bluetooth connectivity. They are based on Microchip\'s RN2483/RN2903 modules and are suitable for rapid prototyping of IoT applications.
- **Arduino MKR WAN 1300/1310**: These are Arduino-compatible boards equipped with a Murata CMWX1ZZABZ LoRa module, offering LoRa connectivity along with the ease of Arduino programming.
- **Heltec ESP32 LoRa Development Board**: This board combines the ESP32 microcontroller with a SX1276 LoRa transceiver module, providing both Wi-Fi and LoRa connectivity in a single board.
- **Dragino LoRa/GPS HAT**: This is a LoRa transceiver HAT for Raspberry Pi, allowing Raspberry Pi users to add LoRa connectivity to their projects. It also includes a GPS module for location-based applications.

### Semtech SX1276/SX1278

The Semtech SX1276/SX1278 is a family of programmable lora transceiver IC specifically designed for long-range communication using the LoRa modulation technique.
- It's not a microcontroller; instead, it's a specialized chip that handles the modulation, demodulation, and communication protocols for LoRa-based systems.
- The SX1276/SX1278 chips utilize Semtech's patented LoRa modulation technology, which enables long-range communication with low power consumption. LoRa modulation uses chirp spread spectrum modulation to achieve robust communication over long distances, even in challenging RF environments.
- **Interface**: The SX1276/SX1278 chips communicate with external microcontrollers or host systems via a Serial Peripheral Interface (SPI) to configure its settings, initiate transmissions and handle received data.
- **Features**: these chips feature adjustable parameters such as spreading factor, bandwidth, and coding rate, allowing users to optimize communication performance based on factors such as data rate, range, and power consumption.
- **Power Consumption**: low ower consumption,this IC offer various power-saving modes, including sleep and standby modes, to minimize power consumption during idle periods.
- **Frequency Bands**: These chips are available in different frequency bands, including 433 MHz, 868 MHz, and 915 MHz, to comply with regional regulations and requirements.
- Datasheets: [SX1272/73](https://www.semtech.com/uploads/documents/sx1272.pdf), [SX1276-7-8-9](https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V5.pdf)
  
### HopeRF RFM95/RFM96

The HopeRF RFM95/RFM96 LoRa modules are based on Semtech's SX1276/SX1278 LoRa transceiver chips.
- **Integration**: The RFM95/RFM96 modules integrate Semtech's SX1276/SX1278 LoRa transceiver chip into a compact module format. This integration simplifies the design process and makes it easier for developers to incorporate LoRa functionality into their projects without needing extensive RF expertise.
- **Frequency Bands**: Similar to the Semtech chips, the RFM95/RFM96 modules are available in various frequency bands, including 433 MHz, 868 MHz, and 915 MHz.
- **Features**: The RFM95/RFM96 modules offer features such as adjustable output power, configurable data rates, and support for multiple spreading factors. These features enable users to optimize communication performance based on factors such as range, data rate and power consumption.
- **Interface**: Like the Semtech chips, the RFM95/RFM96 modules communicate with external microcontrollers or host systems via a Serial Peripheral Interface (SPI).
- **Antenna Options**: The RFM95/RFM96 modules typically come with an onboard antenna connector, allowing users to connect external antennas for improved range and performance. The choice of antenna depends on the specific application requirements and environmental conditions.
- **Power Consumption**: The RFM95/RFM96 modules are designed for low power consumption, making them suitable for battery-powered applications. They offer various power-saving modes, such as sleep and standby modes, to minimize energy usage during idle periods.
- **LoRaWAN Compatibility**: The HopeRF RFM95/RFM96 LoRa modules are not pre-certified for LoRaWAN compatibility,but we can implement the LoRaWAN protocol stack using software libraries or development kits provided by the LoRa Alliance or other third-party vendors. This typically involves integrating LoRaWAN firmware into an external microcontroller or host system that communicates with the RFM95/RFM96 module.

### Microchip RN2483/RN2903

The Microchip RN2483 and RN2903 are LoRaWAN modules designed to simplify the integration of LoRa technology into IoT devices.
- **LoRaWAN Compatibility**: The RN2483 and RN2903 modules are compliant with the LoRaWAN protocol, which is a standardized networking protocol designed for low-power, wide-area networks(LPWANs). LoRaWAN enables long-range communication between IoT devices and gateway nodes, allowing for connectivity over several kilometers in urban and rural environments.
- **Integrated Solution**: These modules integrate LoRa transceiver functionality with a microcontroller unit (MCU) and firmware stack specifically designed for LoRaWAN communication. This integration simplifies the hardware and software design process for developers,enabling rapid development of LoRaWAN-enabled IoT devices.
- **Frequency Bands**: The RN2483 and RN2903 modules are available in various frequency bands, including 433 MHz, 868 MHz, and 915 Mhz.
- **Interface**: These modules communicate with external host systems or microcontrollers via a UART (Universal Asynchronous Receiver-Transmitter) serial interface.
- **Firmware Stack**: The RN2483 and RN2903 modules come preloaded with firmware that implements the LoRaWAN protocol stack, including support for joining LoRaWAN networks, sending and receiving data packets, and managing device parameters. This firmware simplifies the development process by handling low-level communication tasks, allowing developers to focus on application-specific functionality.
- **Power Consumption**: designed for low power consumption, making them suitable for battery-powered IoT applications. They offer various power-saving modes to minimize energy usage.





# LoRa SOFTWARES

## Base libraries
The `IBM LMiC` (LoRa MAC in C) and `Semtech LoRaMac-node` are the two base LoRaWAN library implementations that served as reference for all other implementations. Both have been ported to different platforms

### [IBM LMiC](https://github.com/mcci-catena/ibm-lmic)
- The IBM LMIC (LoRaMAC-in-C) library is one of the earliest implementations of the LoRaWAN protocol stack written in C, Developed by IBM
- it is lightweight and widely adopted
- After depreciation the library been maintained and extended by the open-source community, including significant contributions from MCCI Corporation.
- Repo: official code of IBM LMIC is not available, refer the following repos which are the Continuation of IBM LMIC:
    - <https://github.com/mcci-catena/ibm-lmic> 
    - <https://github.com/lorabasics/basicmac>
    - <https://github.com/LacunaSpace/basicmac>
    - <https://github.com/mkuyper/basicmac>
- [Doc v1.5](https://github.com/matthijskooijman/arduino-lmic/blob/master/doc/LMiC-v1.5.pdf)
- `Depreciated`

### [Semtech LoRaMac-node](https://github.com/Lora-net/LoRaMac-node)
- This is the latest reference implementation of LoRaWAN end node by Semtech. As of now the library only supports the following platforms: NAMote72, NucleoLxx, SKiM880B, SKiM980A, SKiM881AXL and SAML21.
- [API documentation](http://stackforce.github.io/LoRaMac-doc/)
- [Porting guide](http://stackforce.github.io/LoRaMac-doc/_p_o_r_t_i_n_g__g_u_i_d_e.html) which explains howt to port the project to other hardware platforms
- `Depreciated`

## Arduino libraries
### [Arduino LMIC](https://github.com/matthijskooijman/arduino-lmic)
- The `first Arduino port` of IBM LMIC library, The library supports SX1272, SX1276 transceivers and compatible modules such as HopeRF RFM92/RFM95 modules.
- This library provides a fairly complete LoRaWAN Class A and Class B implementation, supporting the EU-868 and US-915 bands.
- `Depreciated`
### [MCCI - Arduino LMIC](https://github.com/mcci-catena/arduino-lmic)
- The `official` lorawan library used in arduino, which is maintained by MCCI Corperation.
- This is the fork of matthijskooijman [arduino-lmic](https://github.com/matthijskooijman/arduino-lmic) 
### [Other libraries]()
Some Arduino LoRa end node libraries can be found at: <https://www.arduinolibraries.info/libraries>


## Raspberry-Pi libraries
### peer to peer - Python (physical layer)
- <https://github.com/mayeranalytics/pySX127x>
- <https://github.com/rpsreal/pySX127x> fork of mayeranalytics [pySX127x](https://github.com/mayeranalytics/pySX127x)
- <https://pypi.org/project/pyLoRa/>
- <https://github.com/rpsreal/pySX127x>
- <https://github.com/tamberg/pi-lora>
- <https://github.com/RAKWireless/rak_common_for_gateway>
- <https://github.com/epeters13/pyLoraRFM9x>
- <https://github.com/jgromes/LoRaLib>
- <https://github.com/Inteform/PyLora> (based on sandeep mistry arduino lora library)
- <https://github.com/chandrawi/LoRaRF-Python> (more detailed)
- <https://github.com/ladecadence/pyRF95> (for RF95)
### lorawan - Python (mac layer)
- <https://github.com/jeroennijhof/LoRaWAN>
    - some good forks of this repo
        -   <https://github.com/computenodes/dragino>
        -   <https://github.com/btemperli/LoRaPy>
        -   <https://github.com/rubenleon/LoRaWAN>
        -   <https://github.com/ryanzav/LoRaWAN>
### [lorawan-library-for-pico](https://github.com/ArmDeveloperEcosystem/lorawan-library-for-pico)
- Enable LoRaWAN communications on [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico) or any RP2040 based board using a Semtech SX1276 radio module 
- Based on the Semtech [LoRaMac-node](https://github.com/Lora-net/LoRaMac-node)

## Reference
- <https://circuitdigest.com/microcontroller-projects/raspberry-pi-with-lora-peer-to-peer-communication-with-arduino>
- <https://sirinsoftware.com/blog/lorawan-mac-layer-definition-architecture-classes-and-more>
- [Semtech LoRa](http://www.semtech.com/wireless-rf/lora.html)
- IBM LoRaWAN IN C <http://www.research.ibm.com/labs/zurich/ics/lrsc/lmic.html>
- LoRa Alliance <https://www.lora-alliance.org/>
- Semtech LoRa Net lora_gateway <https://github.com/lora-net/lora_gateway>
- Semtech LoRa Net packet_forwarder <https://github.com/Lora-net/packet_forwarder>
- TTN poly_pkt_fwd <ttps://github.com/TheThingsNetwork/packet_forwarder>
- Brian Gladman. AES library <http://www.gladman.me.uk/>
- Lander Casado, Philippas Tsigas. CMAC library <http://www.cse.chalmers.se/research/group/dcs/masters/contikisec/>
- diabloneo timespec_diff gitst <https://gist.github.com/diabloneo/9619917>
- CCAN (json libary is from CCAN project) <https://ccodearchive.net/>
- new lora documents: <https://lora.readthedocs.io/en/latest/>
- <https://www.hackster.io/glovebox lorawan-for-raspberry-pi-with-worldwide-frequency-support-e327d2>
- <https://github.com/hallard/RPI-Lora-Gateway>
- kgabis. parson (JSON parser) <https://github.com/kgabis/parson>
- An Introduction to Spread Spectrum Techniques <https://www.ausairpower.net/OSR-0597.html>
- https://sirinsoftware.com/blog/lorawan-mac-layer-definition-architecture-classes-and-more
- semtech lora modulation basics:  http://wiki.lahoud.fr/lib/exe/fetch.php?media=an1200.22.pdf
- Theory of Spread-Spectrum Communications-A Tutorial <https://pdos.csail.mit.edu/archive/decouto/papers/pickholtz82.pdf>
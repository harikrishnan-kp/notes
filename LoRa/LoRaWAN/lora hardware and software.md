## LoRa HARDWARE

Criteria need to be considered while selecting a LoRa module

1.  **Frequency Band**: Determine the frequency band that is appropriate for your region and regulatory requirements. Common frequency bands for LoRa communication include 433 MHz, 868 MHz, and 915 MHz.
3.  **Communication Range**: Evaluate the required communication range or your application. Consider factors such as terrain, obstacles, and desired coverage area. Modules with higher transmit power and
    sensitivity typically offer longer communication ranges.
4.  **Data Rate**: Determine the required data rate for your pplication. LoRa modules support adjustable data rates ranging from a few hundred bits per second (bps) to several hundred kilobits per second (kbps). Select a module that can meet the data rate requirements of your application while balancing power consumption and communication range.
5.  **Power Consumption**: Consider the power consumption of the LoRa module, especially if your application is battery-powered or requires low power operation. Look for modules with low power consumption in both active and sleep modes to maximize battery life.
6.  **Interface**: Evaluate the interface options provided by the LoRa module for communication with external microcontrollers or host systems. This interface is used to configure the module\'s settings,  initiate transmissions, and receive data. Most modules use a Serial Peripheral Interface (SPI) for configuration and data exchange.
7.  **Compatibility**: These modules must be compatible with popular
    development platforms and microcontrollers, making them easy to
    integrate into existing projects.
8.  **Antenna Options**: Consider the antenna options available with the
    LoRa module. Some modules come with onboard antennas, while others
    provide connectors for external antennas. Select the appropriate
    antenna type and configuration based on your application
    requirements and environmental conditions.
9.  **Features**: Evaluate the additional features offered by the LoRa
    module, such as built-in error detection and correction mechanisms,
    frequency hopping capability, support for multiple spreading
    factors, and encryption options. Choose a module that provides the
    necessary features to optimize communication performance and
    reliability.
10. **Cost**: Consider the cost of the LoRa module and its overall value
    proposition for your project.

## Some LoRa transceiver modules
LoRa (Long Range) transceiver modules are widely used for long-range
communication in IoT (Internet of Things) applications. Here are a few
popular LoRa transceiver modules:

1.  **Semtech SX1276/SX1278**: These are among the most common LoRa
    transceiver chips used in modules. They operate in the 868 MHz and
    915 MHz bands and offer excellent sensitivity and long-range
    communication capabilities.
2.  **HopeRF RFM95/RFM96**: These modules are based on the Semtech
    SX1276/SX1278 chips and offer similar features. They are widely used
    in DIY and commercial LoRa applications.
3.  **Microchip RN2483/RN2903**: These modules integrate LoRa technology
    with a microcontroller, making them easy to use for IoT
    applications. They come with firmware that simplifies LoRaWAN
    connectivity.
4.  **Pycom LoPy/LoPy4**: These are development boards that integrate
    LoRa, Wi-Fi, and Bluetooth connectivity. They are based on
    Microchip\'s RN2483/RN2903 modules and are suitable for rapid
    prototyping of IoT applications.
5.  **Arduino MKR WAN 1300/1310**: These are Arduino-compatible boards
    equipped with a Murata CMWX1ZZABZ LoRa module, offering LoRa
    connectivity along with the ease of Arduino programming.
6.  **Heltec ESP32 LoRa Development Board**: This board combines the
    ESP32 microcontroller with a SX1276 LoRa transceiver module,
    providing both Wi-Fi and LoRa connectivity in a single board.
7.  **Dragino LoRa/GPS HAT**: This is a LoRa transceiver HAT for
    Raspberry Pi, allowing Raspberry Pi users to add LoRa connectivity
    to their projects. It also includes a GPS module for location-based
    applications.

## Semtech SX1276/SX1278

-   The Semtech SX1276/SX1278 is a family of
    programmable lora transceiver IC specifically
    designed for long-range communication using the LoRa modulation
    technique.
-   It\'s not a microcontroller; instead, it\'s a specialized chip
    that handles the modulation, demodulation, and communication
    protocols for LoRa-based systems.
-   The SX1276/SX1278 chips utilize Semtech\'s patented LoRa
    modulation technology, which enables long-range communication with
    low power consumption. LoRa modulation uses chirp spread spectrum
    modulation to achieve robust communication over long distances, even
    in challenging RF environments.
-   **Interface**: The SX1276/SX1278 chips communicate with
    external microcontrollers or host systems via a Serial Peripheral
    Interface (SPI) to configure its settings, initiate transmissions,
    and handle received data.
-   **Features**: these chips feature adjustable parameters such
    as spreading factor, bandwidth, and coding rate, allowing users to
    optimize communication performance based on factors such as data
    rate, range, and power consumption.
-   **Power Consumption**: low ower consumption,this IC offer various power-saving modes, including sleep and standby modes, to minimize power consumption during idle periods.
-   **Frequency Bands**: These chips are available in different
    frequency bands, including 433 MHz, 868 MHz, and 915 MHz, to comply
    with regional regulations and requirements.

## HopeRF RFM95/RFM96

The HopeRF RFM95/RFM96 LoRa modules are based on Semtech\'s
SX1276/SX1278 LoRa transceiver chips.

1.  **Integration**: The RFM95/RFM96 modules integrate Semtech\'s
    SX1276/SX1278 LoRa transceiver chip into a compact module format.
    This integration simplifies the design process and makes it easier
    for developers to incorporate LoRa functionality into their projects
    without needing extensive RF expertise.
2.  **Frequency Bands**: Similar to the Semtech chips, the RFM95/RFM96
    modules are available in various frequency bands, including 433 MHz,
    868 MHz, and 915 MHz.
3.  **Features**: The RFM95/RFM96 modules offer features such as
    adjustable output power, configurable data rates, and support for
    multiple spreading factors. These features enable users to optimize
    communication performance based on factors such as range, data rate,
    and power consumption.
4.  **Interface**: Like the Semtech chips, the RFM95/RFM96 modules
    communicate with external microcontrollers or host systems via a
    Serial Peripheral Interface (SPI).
5.  **Antenna Options**: The RFM95/RFM96 modules typically come with an
    onboard antenna connector, allowing users to connect external
    antennas for improved range and performance. The choice of antenna
    depends on the specific application requirements and environmental
    conditions.
6.  **Power Consumption**: The RFM95/RFM96 modules are designed for low
    power consumption, making them suitable for battery-powered
    applications. They offer various power-saving modes, such as sleep
    and standby modes, to minimize energy usage during idle periods.
7.  **LoRaWAN Compatibility**: The HopeRF RFM95/RFM96 LoRa modules are
    not pre-certified for LoRaWAN compatibility,but we can implement the
    LoRaWAN protocol stack using software libraries or development kits
    provided by the LoRa Alliance or other third-party vendors. This
    typically involves integrating LoRaWAN firmware into an external
    microcontroller or host system that communicates with the
    RFM95/RFM96 module.

## Microchip RN2483/RN2903
The Microchip RN2483 and RN2903 are LoRaWAN modules designed to
simplify the integration of LoRa technology into IoT devices.

1.  **LoRaWAN Compatibility**: The RN2483 and RN2903 modules are
    compliant with the LoRaWAN protocol, which is a standardized
    networking protocol designed for low-power, wide-area networks
    (LPWANs). LoRaWAN enables long-range communication between IoT
    devices and gateway nodes, allowing for connectivity over several
    kilometers in urban and rural environments.
2.  **Integrated Solution**: These modules integrate LoRa transceiver
    functionality with a microcontroller unit (MCU) and firmware stack
    specifically designed for LoRaWAN communication. This integration
    simplifies the hardware and software design process for developers,
    enabling rapid development of LoRaWAN-enabled IoT devices.
3.  **Frequency Bands**: The RN2483 and RN2903 modules are available in
    various frequency bands, including 433 MHz, 868 MHz, and 915 Mhz.
4.  **Interface**: These modules communicate with external host systems
    or microcontrollers via a UART (Universal Asynchronous
    Receiver-Transmitter) serial interface.
5.  **Firmware Stack**: The RN2483 and RN2903 modules come preloaded
    with firmware that implements the LoRaWAN protocol stack, including
    support for joining LoRaWAN networks, sending and receiving data
    packets, and managing device parameters. This firmware simplifies
    the development process by handling low-level communication tasks,
    allowing developers to focus on application-specific functionality.
6.  **Power Consumption**: designed for low power consumption, making
    them suitable for battery-powered IoT applications. They offer
    various power-saving modes to minimize energy usage.





# LoRa SOFTWARES

## Lorawan libraries

There exist at least two major LoRaWAN library implementations that can
be used for end nodes:

1.  IBM's original LMiC *(formerly '*LoRa MAC in C*')* implementation.
2.  Semtech's [LoRaMac-node
    949](https://github.com/Lora-net/LoRaMac-node) reference
    implementation.

I call LMiC and LoRaMac-node **base types**.

Both base types have been ported to different platforms (microcontroller
families) e.g. AVR, SAM, STM32, nRF5x, ESP32 and different development
frameworks e.g. Arduino, ARM Mbed etc.

IBM's original LMiC

The IBM LMIC (LoraMAC-in-C) library is a widely-used, lightweight
implementation of the LoRaWAN protocol stack in C. Originally developed
by IBM, it has since been maintained and extended by the open-source
community, including significant contributions from MCCI Corporation.
The library is suitable for resource-constrained devices, making it a
popular choice for IoT applications.

Link : <https://research.ibm.com/labs/zurich/ics/lrsc/lmic.html>

### Key Features

1.  **LoRaWAN Compliance**: Supports LoRaWAN protocol features,
    including:

    -   **Class A**: Low-power, bidirectional communication with
        scheduled receive windows.
    -   **OTAA (Over-the-Air Activation)**: Devices join the network by
        exchanging keys with the network server.
    -   **ABP (Activation by Personalization)**: Devices are
        pre-provisioned with keys for immediate network communication.
    -   **Adaptive Data Rate (ADR)**: Dynamically adjusts the data rate
        based on network conditions and signal quality.

2.  **Multi-Region Support**: Compatible with various regional frequency
    plans, including:

    -   EU868 (Europe)
    -   US915 (North America)
    -   AU915 (Australia)
    -   AS923 (Asia)
    -   IN865 (India)

3.  **Low Power**: Designed for low power consumption, making it
    suitable for battery-powered devices.

4.  **Event-Driven API**: Uses an event-driven architecture to handle
    asynchronous communication, simplifying the management of LoRaWAN
    events and state transitions.

```
Note: Many libraries have an official name that is different from the name of
their (GitHub) repository. The official library name is important for
development tools like PlatformIO which can automatically download
libraries if their official name is specified in the platformio.ini
project configuration file.
```

## SEMTECH LORAMAC-NODE LIBRARY

-   Semtech maintains a reference implementation of a LoRa end node.
-   The code can be found at this location:[https://github.com/Lora-net/LoRaMac-node](https://github.com/Lora-net/LoRaMac-node)
-   API documentation can be found at:[http://stackforce.github.io/LoRaMac-doc/](http://stackforce.github.io/LoRaMac-doc/)
-   This LoRa node library only supports the following platforms:
-   NAMote72, NucleoLxx, SKiM880B, SKiM980A, SKiM881AXL and SAML21.
-   A porting guide is available which provides guide lines on how to
    port the project to other platforms, see:
    [http://stackforce.github.io/LoRaMac-doc/\_p_o_r_t_i_n_g\_\_g_u_i_d_e.html](http://stackforce.github.io/LoRaMac-doc/_p_o_r_t_i_n_g__g_u_i_d_e.html)



## LoRa libraries for arduino

### Arduino LMIC

-   The IBM LMIC library is ported to Arduino platforms and is
    called Arduino LMIC. this was the first arduino lmic
    implimentation.
-   This library supports SX1272, SX1276 transceivers and compatible
    modules such as HopeRF RFM92/RFM95 modules.
-   The code can be found at this location:[https://github.com/matthijskooijman/arduino-lmic](https://github.com/matthijskooijman/arduino-lmic) and are maintained by Matthijs Kooijman, Thomas Telkamp, and others.
-   The Arduino LMIC library provides a fairly complete LoRaWAN Class
    A and Class B implementation, supporting the EU-868 and US-915
    bands.
-   The compiled size of this library is about 30kBytes
-   The first Arduino LMIC implementation was
    [LMIC-Arduino](https://github.com/matthijskooijman/arduino-lmic)
    which has long served as the reference implementation but it is
    no longer maintained.


### Arduino LMIC forks

The Arduino framework initially was available only for the AVR family
of microcontrollers, but nowadays there are Arduino framework
implementations (called Arduino Cores) for many microcontroller families
like AVR, SAMD, STM32, ESP32, ESP8266 and nRF5x.\
While the Arduino LMIC reference implementations can be used with
multiple microcontroller families, there also exist modified versions
that are specially adapted for a certain microcontroller. I call these
variations (or decendants) of Arduino LMIC and call Arduino
LMIC the parent of the decendant.

### MCCI LMIC

-   The Arduino LMIC library is forked many times but the fork made by
    MCCI Catena seems to be actively maintained.[https://github.com/mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic). Among others, the MCCI Arduino LMIC has added regional support for Australia (921MHz), Asia (923 MHz) and India (866 MHz).

### Arduino LMIC usage
-   The Arduino LMIC and the MCCI Arduino LMIC library are intended to
    be used with plain LoRa transceivers, connecting to them using SPI
    (Serial Peripheral Interface).
-   This library contains a full LoRaWAN stack and is intended to
    drive these transceivers directly.
-   The library has only been tested with LoRaWAN 1.0.2 networks.
-   The library can not be used with full-stack devices like the
    Microchip RN2483. These modules contains a transceiver and
    microcontroller that implements the LoRaWAN stack and exposes a
    high-level serial interface instead of a low-level SPI transceiver
    interface.

### Arduino LMIC documentation
- To understand how the Arduino LMIC library works:
- IBM LoRaWAN in C Technical Specification [https://github.com/matthijskooijman/arduino-lmic/blob/master/doc/LMiC-v1.5.pdf*](https://github.com/matthijskooijman/arduino-lmic/blob/master/doc/LMiC-v1.5.pdf)
- Semtech SX1272/73 Datasheet [https://www.semtech.com/uploads/documents/sx1272.pdf](https://www.semtech.com/uploads/documents/sx1272.pdf)
- Semtech SX1276-7-8-9 Datasheet[https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V5.pdf](https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V5.pdf)

### OTHER ARDUINO LORA END NODE LIBRARIES
Other Arduino LoRa end node libraries can be found at: [https://www.arduinolibraries.info/libraries](https://www.arduinolibraries.info/libraries)
```
note: There are libraries available to setup direct communication between
two LoRa radio's.These libraries have nothing to do with the LoRaWAN protocol.
```
## LoRa libraries for raspberry pi

### python peer to peer lora libraries (physical layer)
- <https://github.com/mayeranalytics/pySX127x>
    - forks of mayeranalytics[/](https://github.com/mayeranalytics/pySX127x)pySX127x: <https://github.com/rpsreal/pySX127x/network/members>
- <https://pypi.org/project/pyLoRa/>
- <https://github.com/rpsreal/pySX127x>
- <https://github.com/tamberg/pi-lora>
- <https://github.com/RAKWireless/rak_common_for_gateway>
- <https://github.com/epeters13/pyLoraRFM9x>
- [https://github.com/jgromes/LoRaLib](https://github.com/jgromes/LoRaLib)
- [https://github.com/Inteform/PyLora](https://github.com/Inteform/PyLora) (peer to peer,based on sandeep mistry arduino lora library)
- <https://github.com/chandrawi/LoRaRF-Python> (more detailed)
- <https://github.com/ladecadence/pyRF95> (for rf95)


### python lorawan implimentations libraries (maclayer)
-   [https://github.com/jeroennijhof/LoRaWAN](https://github.com/jeroennijhof/LoRaWAN)
    -   forks of this library
        [https://github.com/jeroennijhof/LoRaWAN/network/members](https://github.com/jeroennijhof/LoRaWAN/network/members)
    -   some good forks of this repo
        -   [https://github.com/computenodes/dragino](https://github.com/computenodes/dragino)
        -   [https://github.com/btemperli/LoRaPy](https://github.com/btemperli/LoRaPy)
        -   [https://github.com/rubenleon/LoRaWAN](https://github.com/rubenleon/LoRaWAN)
        -   [https://github.com/ryanzav/LoRaWAN](https://github.com/ryanzav/LoRaWAN)


### C lora libraries(for raspberry pi pico)
- **Pico-LoRaWAN**: This library is a C-based LMIC implementation that is
designed for Raspberry Pi Pico. With some modifications, it can work on
a Raspberry Pi.
- [https://github.com/ArmDeveloperEcosystem/lorawan-library-for-pico](https://github.com/ArmDeveloperEcosystem/lorawan-library-for-pico)
- Enable LoRaWAN communications on your Raspberry Pi
Pico (https://www.raspberrypi.org/products/raspberry-pi-pico/) or
any RP2040 based board using a Semtech SX1276 radio
module (https://www.semtech.com/apps/product.php?pn=SX1276).


## Reference

- [https://circuitdigest.com/microcontroller-projects/raspberry-pi-with-lora-peer-to-peer-communication-with-arduino](https://circuitdigest.com/microcontroller-projects/raspberry-pi-with-lora-peer-to-peer-communication-with-arduino)
- [https://sirinsoftware.com/blog/lorawan-mac-layer-definition-architecture-classes-and-more](https://sirinsoftware.com/blog/lorawan-mac-layer-definition-architecture-classes-and-more)
-   Semtech LoRa [http://www.semtech.com/wireless-rf/lora.html](http://www.semtech.com/wireless-rf/lora.html)
-   IBM LoRaWAN IN C <http://www.research.ibm.com/labs/zurich/ics/lrsc/lmic.html>
-   LoRa Alliance <https://www.lora-alliance.org/>
-   kgabis. parson (JSON parser) <https://github.com/kgabis/parson>
-   Semtech LoRa Net lora_gateway
    <https://github.com/lora-net/lora_gateway>
-   Semtech LoRa Net packet_forwarder
    <https://github.com/Lora-net/packet_forwarder>
-   TTN poly_pkt_fwd
    <https://github.com/TheThingsNetwork/packet_forwarder>
-   Brian Gladman. AES library <http://www.gladman.me.uk/>
-   Lander Casado, Philippas Tsigas. CMAC library
    <http://www.cse.chalmers.se/research/group/dcs/masters/contikisec/>
-   diabloneo timespec_diff gitst
    <https://gist.github.com/diabloneo/9619917>
-   CCAN (json libary is from CCAN project) <https://ccodearchive.net/>
-   new lora documents: <https://lora.readthedocs.io/en/latest/>
-   <https://www.hackster.io/glovebox lorawan-for-raspberry-pi-with-worldwide-frequency-support-e327d2>
-   <https://github.com/hallard/RPI-Lora-Gateway>


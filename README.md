# simple_iot_system
System for gathering temperature and humidity based on SAM D21, Arduino and nrf24l01. Also Raspberry Pi is involved:)

# System over view


Why haven't you just connected nrf24l01 module through SPI into Raspberry Pi? It's straight forward isn't it?
Providing additional layer of communication gives independence from operating system or platform - there is no problem with drivers installation for both of SPI and nrf24l01. UART works every where. One can even connect it through RS232 converter to PC (like RPI:)).

# Parts of the system
1. Board with MCU, temperature & humidity sensor and radio (nrf24l01)
    * Board is based on Microchip SAM D21 microcontroller
    * You can use Arduino framework to play with this board (this board is Arduino Zero compatible - see more for reference in this [repo](https://github.com/arekosinski/samd21_minimal_board))
    * Radio module slot prepared for nrf24l01
    * Sensor for temp & humidity: SHT31
    * schematic is available in `sensor_schematic` directory. Board design is also attached
    * This board should take around 8-10uA in sleep mode
    * Other components
        * MCP16252 as voltage controller (as boost up). Designed for 2xAA batteries. Input voltage shouldn't excess 3.4V). Board voltage is 3V.
        * MOSFET On Semi FDC6306P (two on board) is used as power controller for radio module and other devices (3 gates)
        * One gate of FDC6306P is used as polarization protection between power source and MCP16252.
2. `sensor_node` - Code responsibile for communication with RF-RPI proxy
3. `rf_rpi_proxy` - Code for the board to act as a UART proxy to Raspberry Pi
4. `rpi_uart_client` - Python UART client resposible for translating received messages

# System diagram

![Components connection](./components_connection.png?raw=true)

# Why?
Just for fun :)

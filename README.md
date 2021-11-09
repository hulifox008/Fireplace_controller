# Fireplace_controller

Thermostat for the fireplace using STM32F103C8T6. It is split into two parts:

## Thermostat
Read temperature from TM1637, and send on/off commands to remote actuator using L12S wireless UART transciver.

## Actuator

Receive commands from thermostat and turn on/off fireplace using relay.

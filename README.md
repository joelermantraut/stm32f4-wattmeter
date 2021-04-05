# Wattmeter

This repository contains source code of a wattmeter made with STM32F407VG Discovery Board.

## Description

Project uses two ADC channel of the board, to measure voltage and current. To voltage, and adapt stage
was added to connect it to 220V. There an transformer and an operational amplifier circuit was used.
For current, a Hall effect sensor was connected through the line of an outlet to connect any device.
Obviouly, another adapt circuit was incorporated.
An 20x4 LCD display shows power measurement. 220V input voltage is used to feed all circuit.

## About code

To control LCD [Tilen Majerle HD44780](http://stm32f4-discovery.net/2014/06/library-16-interfacing-hd44780-lcd-controller-with-stm32f429-discovery/)
was used.
Calcs where simplified using [arm_math.h](https://github.com/ARM-software/CMSIS/blob/master/CMSIS/Include/arm_math.h) library.

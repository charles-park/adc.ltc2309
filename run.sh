#!/bin/sh

sudo ./adc-ltc2309 -D /dev/i2c-0 -p con1
sudo ./adc-ltc2309 -D /dev/i2c-0 -p p3
sudo ./adc-ltc2309 -D /dev/i2c-0 -p p13


// One VL53L0X on its default address, ranging continuously.

import i2c
import vl53l0x show Vl53l0x I2C-ADDRESS

SDA ::= 4
SCL ::= 5

main:
  bus := i2c.Bus --sda=SDA --scl=SCL --frequency=100_000
  device := bus.device I2C-ADDRESS
  sensor := Vl53l0x device
  sensor.set-timeout --ms=80
  if not sensor.init:
    print "no VL53L0X on the bus"
    return
  sensor.start-continuous

  while true:
    range := sensor.read-range
    print (range ? "$range mm" : "no reading")
    sleep --ms=100

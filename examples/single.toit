// One VL53L0X on its default address, ranging continuously.

import gpio
import i2c
import vl53l0x show Vl53l0x

SDA ::= 4
SCL ::= 5

main:
  bus := i2c.Bus --sda=SDA --scl=SCL --frequency=100_000
  sensor := Vl53l0x bus
  sensor.set-timeout 80
  if not sensor.init:
    print "no VL53L0X on the bus"
    return
  sensor.start-continuous

  while true:
    range := sensor.read-range
    print (range ? "$range mm" : "no reading")
    sleep --ms=100

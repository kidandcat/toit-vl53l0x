// Several VL53L0X on one bus.
//
// Every sensor powers up on 0x29, so they are all held in reset through their
// XSHUT pins and brought out one at a time, each being moved to its own address
// before the next one appears on the bus.

import gpio
import i2c
import vl53l0x show Vl53l0x

SDA ::= 4
SCL ::= 5

XSHUT ::= [7, 8, 43, 9, 6]
ADDRESSES ::= [0x30, 0x31, 0x32, 0x33, 0x34]
NAMES ::= ["front", "left", "right", "down", "up"]

main:
  shutdowns := XSHUT.map: | number |
    pin := gpio.Pin.out number
    pin.set 0
    pin
  sleep --ms=80

  bus := i2c.Bus --sda=SDA --scl=SCL --frequency=100_000
  sensors := []
  XSHUT.size.repeat: | index |
    shutdowns[index].set 1
    sleep --ms=50
    sensor := Vl53l0x bus
    sensor.set-timeout 80
    ok := false
    catch --trace=false: ok = sensor.init
    if ok:
      // Do this before releasing the next sensor, or two devices answer on 0x29.
      sensor.set-address ADDRESSES[index]
      sensor.start-continuous
      sensors.add sensor
    else:
      sensors.add null
    print "$NAMES[index] $(ok ? "ok" : "fail")"

  while true:
    readings := sensors.map: | sensor |
      // Non-blocking: returns null when the sensor has nothing new.
      sensor ? (sensor.read-range-if-ready or "-") : "x"
    print readings
    sleep --ms=100

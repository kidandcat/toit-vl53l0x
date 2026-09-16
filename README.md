# vl53l0x

Toit driver for the **ST VL53L0X** time-of-flight distance sensor, including
multi-sensor buses via XSHUT and runtime address assignment.

Ported from the [Pololu Arduino library](https://github.com/pololu/vl53l0x-arduino),
which is itself based on ST's VL53L0X API (STSW-IMG005). Both are MIT licensed.

## Install

```shell
toit pkg install github.com/kidandcat/toit-vl53l0x
```

## One sensor

The driver takes a `serial.Device` (an `i2c.Device` implements that interface):

```toit
import i2c
import vl53l0x show Vl53l0x I2C-ADDRESS

main:
  bus := i2c.Bus --sda=4 --scl=5 --frequency=100_000
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
```

## Several sensors on one bus

Every VL53L0X powers up on address `0x29`, so they cannot simply share a bus.
Hold them all in reset through their XSHUT pins, bring them out one at a time,
and give each one its own address **before** releasing the next.

```toit
shutdowns := XSHUT.map: | number |
  pin := gpio.Pin.out number
  pin.set 0
  pin
sleep --ms=80

bus := i2c.Bus --sda=4 --scl=5 --frequency=100_000
XSHUT.size.repeat: | index |
  shutdowns[index].set 1
  sleep --ms=50
  device := bus.device I2C-ADDRESS
  sensor := Vl53l0x device
  sensor.set-timeout --ms=80
  if sensor.init:
    sensor.set-address ADDRESSES[index] --bus=bus   // before the next one appears
    sensor.start-continuous
```

`set-address --bus=` closes the old `i2c.Device` for you and opens a
replacement on the new address. That matters: Toit's `i2c.Bus` refuses two
devices on one address, so a stale registration on `0x29` would break the
next sensor.

See `examples/multiple.toit` for the complete version.

## API

| Member | Purpose |
| --- | --- |
| `Vl53l0x device` | Bind to a sensor through a `serial.Device`. |
| `I2C-ADDRESS` / `ADDRESS-DEFAULT` | Power-up address `0x29`. |
| `init --io-2v8=true -> bool` | DataInit, StaticInit and reference calibration. False if the sensor did not answer. |
| `set-address new/int --bus=` | Move the sensor to another address, for multi-sensor buses. |
| `set-timeout --ms=80` | I/O timeout. 0 disables it. Positional `set-timeout 80` still works. |
| `start-continuous --period-ms=0` | Continuous ranging. 0 means back to back. |
| `stop-continuous` | Back to idle. |
| `close` | Stop ranging; close the device only if this driver created it. |
| `read-range -> int?` | Millimetres, or null on timeout. Blocks until a sample is ready. |
| `read-range-if-ready -> int?` | Millimetres, or null if nothing new. Never blocks. |
| `timeout-occurred -> bool` | Whether a read timed out since the last call. Reading clears it. |
| `set-signal-rate-limit --limit-mcps=0.25` | Return signal rate limit, default 0.25 MCPS. |
| `get-measurement-timing-budget -> int` | Current budget in microseconds. |
| `set-measurement-timing-budget --budget-us=20000` | Longer budget, better accuracy. Minimum around 20000. |

`toit docs serve` shows the library and class toitdocs.

### Blocking or not

`read-range` spins on the interrupt status register until a sample arrives.
Toit tasks are cooperative, so inside a control loop that spin holds every other
task in the process for up to the whole timeout. Prefer `read-range-if-ready`
there: in continuous mode a sample is almost always ready anyway, and null just
means "keep the previous value".

### Differences from the Pololu library

- Timeouts return **null** rather than the magic value 65535.
- Single-shot ranging and the VCSEL pulse period setters are not ported.
- Reference SPAD management is skipped, exactly as Pololu does: ST performs it
  on the bare modules.

## Tested on

ESP32-S3 with five sensors on one 100 kHz bus, readdressed to 0x30-0x34, running
continuously inside a flight-control loop. Toit SDK v2.0.0-alpha.198.

## License

MIT. See `LICENSE`, which keeps Pololu's copyright alongside the port's.

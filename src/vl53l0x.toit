// Ported from the Pololu Arduino library (VL53L0X.cpp 1.3.1), which is itself
// based on ST's VL53L0X API (STSW-IMG005). Both are MIT licensed; see LICENSE.

import i2c
import serial.device as serial
import serial.registers as serial

/**
Driver for the ST VL53L0X time-of-flight distance sensor.

Covers init, runtime address change for multi-sensor buses (XSHUT),
  continuous ranging and the timeout bookkeeping. Single-shot ranging and
  the VCSEL pulse period setters are not ported.

The driver talks to a $serial.Device, typically an $i2c.Device:

```
import i2c
import vl53l0x show Vl53l0x I2C-ADDRESS

main:
  bus := i2c.Bus --sda=4 --scl=5 --frequency=100_000
  device := bus.device I2C-ADDRESS
  sensor := Vl53l0x device
  sensor.set-timeout --ms=80
  if sensor.init:
    sensor.start-continuous
    print sensor.read-range
```

Every VL53L0X powers up on $I2C-ADDRESS. On a shared bus, hold unused
  sensors in reset through XSHUT, init one at a time, and move each to
  its own address with $Vl53l0x.set-address before releasing the next.
*/

/** Default I2C address. Every VL53L0X powers up on this address. */
I2C-ADDRESS ::= 0x29

/**
Driver for the ST VL53L0X time-of-flight distance sensor.

Takes a $serial.Device (usually from $(i2c.Bus.device i2c-address)). For
  multi-sensor buses, $set-address writes a new address and re-registers
  the I2C device so the default address is free for the next sensor.
*/
class Vl53l0x:
  /** Same as $I2C-ADDRESS. */
  static ADDRESS-DEFAULT ::= I2C-ADDRESS

  // Register addresses, from ST's vl53l0x_device.h.
  static SYSRANGE-START_ ::= 0x00
  static SYSTEM-SEQUENCE-CONFIG_ ::= 0x01
  static SYSTEM-INTERMEASUREMENT-PERIOD_ ::= 0x04
  static SYSTEM-INTERRUPT-CONFIG-GPIO_ ::= 0x0A
  static SYSTEM-INTERRUPT-CLEAR_ ::= 0x0B
  static GPIO-HV-MUX-ACTIVE-HIGH_ ::= 0x84
  static RESULT-INTERRUPT-STATUS_ ::= 0x13
  static RESULT-RANGE-STATUS_ ::= 0x14
  static I2C-SLAVE-DEVICE-ADDRESS_ ::= 0x8A
  static MSRC-CONFIG-CONTROL_ ::= 0x60
  static FINAL-RANGE-CONFIG-MIN-COUNT-RATE-RTN-LIMIT_ ::= 0x44
  static PRE-RANGE-CONFIG-VCSEL-PERIOD_ ::= 0x50
  static PRE-RANGE-CONFIG-TIMEOUT-MACROP-HI_ ::= 0x51
  static FINAL-RANGE-CONFIG-VCSEL-PERIOD_ ::= 0x70
  static FINAL-RANGE-CONFIG-TIMEOUT-MACROP-HI_ ::= 0x71
  static MSRC-CONFIG-TIMEOUT-MACROP_ ::= 0x46
  static IDENTIFICATION-MODEL-ID_ ::= 0xC0
  static OSC-CALIBRATE-VAL_ ::= 0xF8
  static GLOBAL-CONFIG-SPAD-ENABLES-REF-0_ ::= 0xB0
  static GLOBAL-CONFIG-REF-EN-START-SELECT_ ::= 0xB6
  static DYNAMIC-SPAD-NUM-REQUESTED-REF-SPAD_ ::= 0x4E
  static DYNAMIC-SPAD-REF-EN-START-OFFSET_ ::= 0x4F
  static VHV-CONFIG-PAD-SCL-SDA-EXTSUP-HV_ ::= 0x89

  registers_/serial.Registers := ?
  device_/serial.Device := ?
  owns-device_/bool := false
  address_/int := ?
  io-timeout-ms_/int := 0
  did-timeout_/bool := false
  stop-variable_/int := 0
  measurement-timing-budget-us_/int := 0

  /**
  Constructs a driver for the sensor connected through $device.

  The caller owns $device and should close it. $set-address closes this
    device (Toit's I2C bus refuses two devices on one address) and opens a
    replacement on the new address.

  If $device is an $i2c.Device, $address is taken from it. Otherwise it
    defaults to $I2C-ADDRESS.
  */
  constructor device/serial.Device:
    device_ = device
    registers_ = device.registers
    address_ = (device is i2c.Device) ? (device as i2c.Device).address : I2C-ADDRESS

  /** Current I2C address of this sensor. */
  address -> int: return address_

  /**
  Moves the sensor to $new-address.

  Every sensor powers up on $I2C-ADDRESS, so on a bus with more than one
    they are brought out of reset one at a time and readdressed here.

  After writing the new address to the chip, the current $serial.Device
    is closed and replaced with one opened on $new-address. That matters:
    Toit's $i2c.Bus refuses two devices on one address, so the old
    registration has to go before the next sensor is released from XSHUT.

  $bus is used to close the old $i2c.Device and open a new one on
    $new-address.
  */
  set-address new-address/int --bus/i2c.Bus -> none:
    if new-address == address_: return
    write-reg_ I2C-SLAVE-DEVICE-ADDRESS_ (new-address & 0x7F)
    previous := device_
    address_ = new-address
    device_ = bus.device new-address
    registers_ = device_.registers
    close-device_ previous
    owns-device_ = true

  /**
  Sets the I/O timeout used by $init, $read-range and calibration.

  0 disables the timeout.
  */
  set-timeout --ms/int -> none:
    io-timeout-ms_ = ms

  /**
  Variant of $(set-timeout --ms) that takes a positional duration.
  */
  set-timeout ms/int -> none:
    io-timeout-ms_ = ms

  /**
  Whether a read timed out since the last call.

  Reading clears the flag, which is what the Pololu library does too.
  */
  timeout-occurred -> bool:
    result := did-timeout_
    did-timeout_ = false
    return result

  /**
  Stops ranging.

  Does not close the $serial.Device unless this driver created it
    ($set-address). The caller still owns a device they passed to the
    constructor.
  */
  close -> none:
    catch: stop-continuous
    if owns-device_:
      close-device_ device_
      owns-device_ = false

  close-device_ device/serial.Device -> none:
    if device is i2c.Device:
      (device as i2c.Device).close

  // --- Register access -------------------------------------------------------

  write-reg_ register/int value/int -> none:
    registers_.write-u8 register value

  write-reg-16_ register/int value/int -> none:
    registers_.write-u16-be register value

  // Registers has write-i32-be but no unsigned 32-bit big-endian writer.
  write-reg-32_ register/int value/int -> none:
    bytes := ByteArray 4
    bytes[0] = (value >> 24) & 0xFF
    bytes[1] = (value >> 16) & 0xFF
    bytes[2] = (value >> 8) & 0xFF
    bytes[3] = value & 0xFF
    registers_.write-bytes register bytes

  read-reg_ register/int -> int:
    return registers_.read-u8 register

  read-reg-16_ register/int -> int:
    return registers_.read-u16-be register

  read-multi_ register/int count/int -> ByteArray:
    return registers_.read-bytes register count

  write-multi_ register/int bytes/ByteArray -> none:
    registers_.write-bytes register bytes

  // --- Init ------------------------------------------------------------------

  /**
  Runs DataInit, StaticInit and PerformRefCalibration.

  Returns whether the sensor answered and calibrated. Reference SPAD management
    is skipped, exactly as in the Pololu library: ST performs it on the bare
    modules.

  $io-2v8 should be true when the I/O voltage is 2.8 V (typical breakout
    boards). Set it to false for 1.8 V I/O.
  */
  init --io-2v8/bool=true -> bool:
    if (read-reg_ IDENTIFICATION-MODEL-ID_) != 0xEE: return false

    if io-2v8:
      write-reg_ VHV-CONFIG-PAD-SCL-SDA-EXTSUP-HV_
          ((read-reg_ VHV-CONFIG-PAD-SCL-SDA-EXTSUP-HV_) | 0x01)

    // "Set I2C standard mode"
    write-reg_ 0x88 0x00
    write-reg_ 0x80 0x01
    write-reg_ 0xFF 0x01
    write-reg_ 0x00 0x00
    stop-variable_ = read-reg_ 0x91
    write-reg_ 0x00 0x01
    write-reg_ 0xFF 0x00
    write-reg_ 0x80 0x00

    // Disable SIGNAL-RATE-MSRC (bit 1) and SIGNAL-RATE-PRE-RANGE (bit 4) checks.
    write-reg_ MSRC-CONFIG-CONTROL_ ((read-reg_ MSRC-CONFIG-CONTROL_) | 0x12)
    set-signal-rate-limit --limit-mcps=0.25
    write-reg_ SYSTEM-SEQUENCE-CONFIG_ 0xFF

    spad-info := get-spad-info_
    if not spad-info: return false
    spad-count := spad-info[0]
    spad-type-is-aperture := spad-info[1]

    ref-spad-map := read-multi_ GLOBAL-CONFIG-SPAD-ENABLES-REF-0_ 6

    write-reg_ 0xFF 0x01
    write-reg_ DYNAMIC-SPAD-REF-EN-START-OFFSET_ 0x00
    write-reg_ DYNAMIC-SPAD-NUM-REQUESTED-REF-SPAD_ 0x2C
    write-reg_ 0xFF 0x00
    write-reg_ GLOBAL-CONFIG-REF-EN-START-SELECT_ 0xB4

    first-spad-to-enable := spad-type-is-aperture ? 12 : 0
    spads-enabled := 0
    48.repeat: | i |
      if i < first-spad-to-enable or spads-enabled == spad-count:
        ref-spad-map[i / 8] = ref-spad-map[i / 8] & (0xFF - (1 << (i % 8)))
      else if ((ref-spad-map[i / 8] >> (i % 8)) & 0x1) == 1:
        spads-enabled++

    write-multi_ GLOBAL-CONFIG-SPAD-ENABLES-REF-0_ ref-spad-map

    load-tuning-settings_

    // "Set interrupt config to new sample ready"
    write-reg_ SYSTEM-INTERRUPT-CONFIG-GPIO_ 0x04
    write-reg_ GPIO-HV-MUX-ACTIVE-HIGH_ ((read-reg_ GPIO-HV-MUX-ACTIVE-HIGH_) & 0xEF)
    write-reg_ SYSTEM-INTERRUPT-CLEAR_ 0x01

    measurement-timing-budget-us_ = get-measurement-timing-budget

    // "Disable MSRC and TCC by default"
    write-reg_ SYSTEM-SEQUENCE-CONFIG_ 0xE8
    set-measurement-timing-budget --budget-us=measurement-timing-budget-us_

    write-reg_ SYSTEM-SEQUENCE-CONFIG_ 0x01
    if not perform-single-ref-calibration_ 0x40: return false

    write-reg_ SYSTEM-SEQUENCE-CONFIG_ 0x02
    if not perform-single-ref-calibration_ 0x00: return false

    write-reg_ SYSTEM-SEQUENCE-CONFIG_ 0xE8
    return true

  /**
  Sets the return signal rate limit in mega-counts per second.

  Default is 0.25 MCPS. Valid range is 0.0 to 511.99.
  */
  set-signal-rate-limit --limit-mcps/float -> bool:
    if limit-mcps < 0.0 or limit-mcps > 511.99: return false
    // Q9.7 fixed point.
    write-reg-16_ FINAL-RANGE-CONFIG-MIN-COUNT-RATE-RTN-LIMIT_
        (limit-mcps * 128.0).to-int
    return true

  // --- Continuous ranging ----------------------------------------------------

  /**
  Starts continuous ranging.

  With $period-ms of 0 the sensor measures back to back. A non-zero
    period is the inter-measurement interval.

  # Examples
  ```
  sensor.start-continuous
  sensor.start-continuous --period-ms=10
  ```
  */
  start-continuous --period-ms/int=0 -> none:
    write-reg_ 0x80 0x01
    write-reg_ 0xFF 0x01
    write-reg_ 0x00 0x00
    write-reg_ 0x91 stop-variable_
    write-reg_ 0x00 0x01
    write-reg_ 0xFF 0x00
    write-reg_ 0x80 0x00

    if period-ms != 0:
      osc-calibrate-val := read-reg-16_ OSC-CALIBRATE-VAL_
      period := osc-calibrate-val != 0 ? period-ms * osc-calibrate-val : period-ms
      write-reg-32_ SYSTEM-INTERMEASUREMENT-PERIOD_ period
      write-reg_ SYSRANGE-START_ 0x04  // timed
    else:
      write-reg_ SYSRANGE-START_ 0x02  // back to back

  /** Stops continuous ranging and returns the sensor to idle. */
  stop-continuous -> none:
    write-reg_ SYSRANGE-START_ 0x01
    write-reg_ 0xFF 0x01
    write-reg_ 0x00 0x00
    write-reg_ 0x91 0x00
    write-reg_ 0x00 0x01
    write-reg_ 0xFF 0x00

  /**
  Reads the latest range in millimetres, or null if the read timed out.

  Blocks until a sample is ready or $set-timeout expires. The Pololu library
    returns the magic value 65535 on timeout. Null is the Toit way to say
    "no reading", and it cannot be mistaken for a distance.
  */
  read-range -> int?:
    deadline := start-timeout_
    while ((read-reg_ RESULT-INTERRUPT-STATUS_) & 0x07) == 0:
      if timeout-expired_ deadline:
        did-timeout_ = true
        return null
    range := read-reg-16_ (RESULT-RANGE-STATUS_ + 10)
    write-reg_ SYSTEM-INTERRUPT-CLEAR_ 0x01
    return range

  /**
  Reads the latest range only if the sensor has one ready, else returns null.

  The blocking $read-range spins on the interrupt status register. Toit tasks
    are cooperative, so that spin would hold the attitude loop hostage for up
    to the whole timeout. In continuous mode a sample is almost always there.
  */
  read-range-if-ready -> int?:
    if ((read-reg_ RESULT-INTERRUPT-STATUS_) & 0x07) == 0: return null
    range := read-reg-16_ (RESULT-RANGE-STATUS_ + 10)
    write-reg_ SYSTEM-INTERRUPT-CLEAR_ 0x01
    return range

  // --- Timing budget ---------------------------------------------------------

  /**
  Current measurement timing budget in microseconds.

  A longer budget improves accuracy. Typical values are around 20_000
    (default) to 200_000.
  */
  get-measurement-timing-budget -> int:
    start-overhead := 1910
    end-overhead := 960
    msrc-overhead := 660
    tcc-overhead := 590
    dss-overhead := 690
    pre-range-overhead := 660
    final-range-overhead := 550

    enables := sequence-step-enables_
    timeouts := sequence-step-timeouts_ enables

    budget := start-overhead + end-overhead
    if enables["tcc"]: budget += timeouts["msrc-dss-tcc-us"] + tcc-overhead
    if enables["dss"]:
      budget += 2 * (timeouts["msrc-dss-tcc-us"] + dss-overhead)
    else if enables["msrc"]:
      budget += timeouts["msrc-dss-tcc-us"] + msrc-overhead
    if enables["pre-range"]: budget += timeouts["pre-range-us"] + pre-range-overhead
    if enables["final-range"]: budget += timeouts["final-range-us"] + final-range-overhead

    measurement-timing-budget-us_ = budget
    return budget

  /**
  Sets the measurement timing budget to $budget-us microseconds.

  Longer budgets improve accuracy. The minimum is around 20_000.
  */
  set-measurement-timing-budget --budget-us/int -> bool:
    start-overhead := 1910
    end-overhead := 960
    msrc-overhead := 660
    tcc-overhead := 590
    dss-overhead := 690
    pre-range-overhead := 660
    final-range-overhead := 550

    enables := sequence-step-enables_
    timeouts := sequence-step-timeouts_ enables

    used := start-overhead + end-overhead
    if enables["tcc"]: used += timeouts["msrc-dss-tcc-us"] + tcc-overhead
    if enables["dss"]:
      used += 2 * (timeouts["msrc-dss-tcc-us"] + dss-overhead)
    else if enables["msrc"]:
      used += timeouts["msrc-dss-tcc-us"] + msrc-overhead
    if enables["pre-range"]: used += timeouts["pre-range-us"] + pre-range-overhead

    if not enables["final-range"]: return true

    used += final-range-overhead
    // "Requested timeout too big."
    if used > budget-us: return false

    final-range-timeout-us := budget-us - used
    final-range-timeout-mclks := timeout-us-to-mclks_
        final-range-timeout-us
        timeouts["final-range-vcsel-period-pclks"]
    if enables["pre-range"]:
      final-range-timeout-mclks += timeouts["pre-range-mclks"]

    write-reg-16_ FINAL-RANGE-CONFIG-TIMEOUT-MACROP-HI_
        encode-timeout_ final-range-timeout-mclks
    measurement-timing-budget-us_ = budget-us
    return true

  // --- Private ---------------------------------------------------------------

  get-spad-info_ -> List?:
    write-reg_ 0x80 0x01
    write-reg_ 0xFF 0x01
    write-reg_ 0x00 0x00
    write-reg_ 0xFF 0x06
    write-reg_ 0x83 ((read-reg_ 0x83) | 0x04)
    write-reg_ 0xFF 0x07
    write-reg_ 0x81 0x01
    write-reg_ 0x80 0x01
    write-reg_ 0x94 0x6B
    write-reg_ 0x83 0x00

    deadline := start-timeout_
    while (read-reg_ 0x83) == 0x00:
      if timeout-expired_ deadline: return null

    write-reg_ 0x83 0x01
    tmp := read-reg_ 0x92
    count := tmp & 0x7F
    type-is-aperture := ((tmp >> 7) & 0x01) == 1

    write-reg_ 0x81 0x00
    write-reg_ 0xFF 0x06
    write-reg_ 0x83 ((read-reg_ 0x83) & 0xFB)
    write-reg_ 0xFF 0x01
    write-reg_ 0x00 0x01
    write-reg_ 0xFF 0x00
    write-reg_ 0x80 0x00
    return [count, type-is-aperture]

  sequence-step-enables_ -> Map:
    config := read-reg_ SYSTEM-SEQUENCE-CONFIG_
    return {
      "tcc": ((config >> 4) & 0x1) == 1,
      "dss": ((config >> 3) & 0x1) == 1,
      "msrc": ((config >> 2) & 0x1) == 1,
      "pre-range": ((config >> 6) & 0x1) == 1,
      "final-range": ((config >> 7) & 0x1) == 1,
    }

  sequence-step-timeouts_ enables/Map -> Map:
    pre-range-vcsel := decode-vcsel-period_ (read-reg_ PRE-RANGE-CONFIG-VCSEL-PERIOD_)
    msrc-dss-tcc-mclks := (read-reg_ MSRC-CONFIG-TIMEOUT-MACROP_) + 1
    msrc-dss-tcc-us := timeout-mclks-to-us_ msrc-dss-tcc-mclks pre-range-vcsel
    pre-range-mclks := decode-timeout_ (read-reg-16_ PRE-RANGE-CONFIG-TIMEOUT-MACROP-HI_)
    pre-range-us := timeout-mclks-to-us_ pre-range-mclks pre-range-vcsel
    final-range-vcsel := decode-vcsel-period_ (read-reg_ FINAL-RANGE-CONFIG-VCSEL-PERIOD_)
    final-range-mclks := decode-timeout_ (read-reg-16_ FINAL-RANGE-CONFIG-TIMEOUT-MACROP-HI_)
    if enables["pre-range"]: final-range-mclks -= pre-range-mclks
    final-range-us := timeout-mclks-to-us_ final-range-mclks final-range-vcsel
    return {
      "pre-range-vcsel-period-pclks": pre-range-vcsel,
      "final-range-vcsel-period-pclks": final-range-vcsel,
      "msrc-dss-tcc-mclks": msrc-dss-tcc-mclks,
      "msrc-dss-tcc-us": msrc-dss-tcc-us,
      "pre-range-mclks": pre-range-mclks,
      "pre-range-us": pre-range-us,
      "final-range-mclks": final-range-mclks,
      "final-range-us": final-range-us,
    }

  perform-single-ref-calibration_ vhv-init-byte/int -> bool:
    write-reg_ SYSRANGE-START_ (0x01 | vhv-init-byte)
    deadline := start-timeout_
    while ((read-reg_ RESULT-INTERRUPT-STATUS_) & 0x07) == 0:
      if timeout-expired_ deadline: return false
    write-reg_ SYSTEM-INTERRUPT-CLEAR_ 0x01
    write-reg_ SYSRANGE-START_ 0x00
    return true

  static decode-vcsel-period_ reg-value/int -> int:
    return (reg-value + 1) << 1

  /** Macro period in nanoseconds. PLL period 1655 ps, macro period 2304 vclks. */
  static calc-macro-period_ vcsel-period-pclks/int -> int:
    return ((2304 * vcsel-period-pclks * 1655) + 500) / 1000

  /** Format: "(LSByte * 2^MSByte) + 1". */
  static decode-timeout_ reg-value/int -> int:
    return ((reg-value & 0x00FF) << ((reg-value & 0xFF00) >> 8)) + 1

  static encode-timeout_ timeout-mclks/int -> int:
    if timeout-mclks <= 0: return 0
    ls-byte := timeout-mclks - 1
    ms-byte := 0
    while (ls-byte & 0xFFFFFF00) > 0:
      ls-byte >>= 1
      ms-byte++
    return (ms-byte << 8) | (ls-byte & 0xFF)

  static timeout-mclks-to-us_ timeout-period-mclks/int vcsel-period-pclks/int -> int:
    macro-period-ns := calc-macro-period_ vcsel-period-pclks
    return ((timeout-period-mclks * macro-period-ns) + 500) / 1000

  static timeout-us-to-mclks_ timeout-period-us/int vcsel-period-pclks/int -> int:
    macro-period-ns := calc-macro-period_ vcsel-period-pclks
    return ((timeout-period-us * 1000) + (macro-period-ns / 2)) / macro-period-ns

  start-timeout_ -> int:
    return Time.monotonic-us + io-timeout-ms_ * 1000

  timeout-expired_ deadline/int -> bool:
    if io-timeout-ms_ <= 0: return false
    return Time.monotonic-us > deadline

  load-tuning-settings_ -> none:
    // DefaultTuningSettings from ST's vl53l0x_tuning.h, verbatim.
    write-reg_ 0xFF 0x01
    write-reg_ 0x00 0x00
    write-reg_ 0xFF 0x00
    write-reg_ 0x09 0x00
    write-reg_ 0x10 0x00
    write-reg_ 0x11 0x00
    write-reg_ 0x24 0x01
    write-reg_ 0x25 0xFF
    write-reg_ 0x75 0x00
    write-reg_ 0xFF 0x01
    write-reg_ 0x4E 0x2C
    write-reg_ 0x48 0x00
    write-reg_ 0x30 0x20
    write-reg_ 0xFF 0x00
    write-reg_ 0x30 0x09
    write-reg_ 0x54 0x00
    write-reg_ 0x31 0x04
    write-reg_ 0x32 0x03
    write-reg_ 0x40 0x83
    write-reg_ 0x46 0x25
    write-reg_ 0x60 0x00
    write-reg_ 0x27 0x00
    write-reg_ 0x50 0x06
    write-reg_ 0x51 0x00
    write-reg_ 0x52 0x96
    write-reg_ 0x56 0x08
    write-reg_ 0x57 0x30
    write-reg_ 0x61 0x00
    write-reg_ 0x62 0x00
    write-reg_ 0x64 0x00
    write-reg_ 0x65 0x00
    write-reg_ 0x66 0xA0
    write-reg_ 0xFF 0x01
    write-reg_ 0x22 0x32
    write-reg_ 0x47 0x14
    write-reg_ 0x49 0xFF
    write-reg_ 0x4A 0x00
    write-reg_ 0xFF 0x00
    write-reg_ 0x7A 0x0A
    write-reg_ 0x7B 0x00
    write-reg_ 0x78 0x21
    write-reg_ 0xFF 0x01
    write-reg_ 0x23 0x34
    write-reg_ 0x42 0x00
    write-reg_ 0x44 0xFF
    write-reg_ 0x45 0x26
    write-reg_ 0x46 0x05
    write-reg_ 0x40 0x40
    write-reg_ 0x0E 0x06
    write-reg_ 0x20 0x1A
    write-reg_ 0x43 0x40
    write-reg_ 0xFF 0x00
    write-reg_ 0x34 0x03
    write-reg_ 0x35 0x44
    write-reg_ 0xFF 0x01
    write-reg_ 0x31 0x04
    write-reg_ 0x4B 0x09
    write-reg_ 0x4C 0x05
    write-reg_ 0x4D 0x04
    write-reg_ 0xFF 0x00
    write-reg_ 0x44 0x00
    write-reg_ 0x45 0x20
    write-reg_ 0x47 0x08
    write-reg_ 0x48 0x28
    write-reg_ 0x67 0x00
    write-reg_ 0x70 0x04
    write-reg_ 0x71 0x01
    write-reg_ 0x72 0xFE
    write-reg_ 0x76 0x00
    write-reg_ 0x77 0x00
    write-reg_ 0xFF 0x01
    write-reg_ 0x0D 0x01
    write-reg_ 0xFF 0x00
    write-reg_ 0x80 0x01
    write-reg_ 0x01 0xF8
    write-reg_ 0xFF 0x01
    write-reg_ 0x8E 0x01
    write-reg_ 0x00 0x01
    write-reg_ 0xFF 0x00
    write-reg_ 0x80 0x00

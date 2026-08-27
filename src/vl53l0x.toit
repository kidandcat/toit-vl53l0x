// Driver for the ST VL53L0X time-of-flight distance sensor.
//
// Ported from the Pololu Arduino library (VL53L0X.cpp 1.3.1), which is itself
// based on ST's VL53L0X API (STSW-IMG005). Both are MIT licensed; see LICENSE.
//
// Covers init, runtime address change for multi-sensor buses, continuous
// ranging and the timeout bookkeeping. Single-shot ranging and the VCSEL pulse
// period setters are not ported.

import i2c

class Vl53l0x:
  static ADDRESS-DEFAULT ::= 0x29

  // Register addresses, from ST's vl53l0x_device.h.
  static SYSRANGE-START ::= 0x00
  static SYSTEM-SEQUENCE-CONFIG ::= 0x01
  static SYSTEM-INTERMEASUREMENT-PERIOD ::= 0x04
  static SYSTEM-INTERRUPT-CONFIG-GPIO ::= 0x0A
  static SYSTEM-INTERRUPT-CLEAR ::= 0x0B
  static GPIO-HV-MUX-ACTIVE-HIGH ::= 0x84
  static RESULT-INTERRUPT-STATUS ::= 0x13
  static RESULT-RANGE-STATUS ::= 0x14
  static I2C-SLAVE-DEVICE-ADDRESS ::= 0x8A
  static MSRC-CONFIG-CONTROL ::= 0x60
  static FINAL-RANGE-CONFIG-MIN-COUNT-RATE-RTN-LIMIT ::= 0x44
  static PRE-RANGE-CONFIG-VCSEL-PERIOD ::= 0x50
  static PRE-RANGE-CONFIG-TIMEOUT-MACROP-HI ::= 0x51
  static FINAL-RANGE-CONFIG-VCSEL-PERIOD ::= 0x70
  static FINAL-RANGE-CONFIG-TIMEOUT-MACROP-HI ::= 0x71
  static MSRC-CONFIG-TIMEOUT-MACROP ::= 0x46
  static IDENTIFICATION-MODEL-ID ::= 0xC0
  static OSC-CALIBRATE-VAL ::= 0xF8
  static GLOBAL-CONFIG-SPAD-ENABLES-REF-0 ::= 0xB0
  static GLOBAL-CONFIG-REF-EN-START-SELECT ::= 0xB6
  static DYNAMIC-SPAD-NUM-REQUESTED-REF-SPAD ::= 0x4E
  static DYNAMIC-SPAD-REF-EN-START-OFFSET ::= 0x4F
  static VHV-CONFIG-PAD-SCL-SDA-EXTSUP-HV ::= 0x89

  bus_/i2c.Bus
  device_/i2c.Device := ?
  address_/int := ?
  io-timeout-ms_/int := 0
  did-timeout_/bool := false
  stop-variable_/int := 0
  measurement-timing-budget-us_/int := 0

  constructor .bus_ --address/int=ADDRESS-DEFAULT:
    address_ = address
    device_ = bus_.device address

  address -> int: return address_

  /**
  Moves the sensor to $new-address.

  Every sensor powers up on $ADDRESS-DEFAULT, so on a bus with more than one
    they are brought out of reset one at a time and readdressed here.
  */
  set-address new-address/int -> none:
    if new-address == address_: return
    write-reg I2C-SLAVE-DEVICE-ADDRESS (new-address & 0x7F)
    // The bus refuses two devices on one address, so the old registration has
    // to go before the next sensor is brought out of reset on 0x29.
    previous := device_
    address_ = new-address
    device_ = bus_.device new-address
    previous.close

  set-timeout ms/int -> none:
    io-timeout-ms_ = ms

  /**
  Whether a read timed out since the last call. Reading clears the flag, which
    is what the Pololu library does too.
  */
  timeout-occurred -> bool:
    result := did-timeout_
    did-timeout_ = false
    return result

  // --- Register access -------------------------------------------------------

  write-reg reg/int value/int -> none:
    buffer := ByteArray 1
    buffer[0] = value & 0xFF
    device_.write-reg reg buffer

  write-reg-16 reg/int value/int -> none:
    buffer := ByteArray 2
    buffer[0] = (value >> 8) & 0xFF
    buffer[1] = value & 0xFF
    device_.write-reg reg buffer

  write-reg-32 reg/int value/int -> none:
    buffer := ByteArray 4
    buffer[0] = (value >> 24) & 0xFF
    buffer[1] = (value >> 16) & 0xFF
    buffer[2] = (value >> 8) & 0xFF
    buffer[3] = value & 0xFF
    device_.write-reg reg buffer

  read-reg reg/int -> int:
    return (device_.read-reg reg 1)[0]

  read-reg-16 reg/int -> int:
    bytes := device_.read-reg reg 2
    return (bytes[0] << 8) | bytes[1]

  read-multi reg/int count/int -> ByteArray:
    return device_.read-reg reg count

  write-multi reg/int bytes/ByteArray -> none:
    device_.write-reg reg bytes

  // --- Init ------------------------------------------------------------------

  /**
  Runs DataInit, StaticInit and PerformRefCalibration.

  Returns whether the sensor answered and calibrated. Reference SPAD management
    is skipped, exactly as in the Pololu library: ST performs it on the bare
    modules.
  */
  init --io-2v8/bool=true -> bool:
    if (read-reg IDENTIFICATION-MODEL-ID) != 0xEE: return false

    if io-2v8:
      write-reg VHV-CONFIG-PAD-SCL-SDA-EXTSUP-HV
          ((read-reg VHV-CONFIG-PAD-SCL-SDA-EXTSUP-HV) | 0x01)

    // "Set I2C standard mode"
    write-reg 0x88 0x00
    write-reg 0x80 0x01
    write-reg 0xFF 0x01
    write-reg 0x00 0x00
    stop-variable_ = read-reg 0x91
    write-reg 0x00 0x01
    write-reg 0xFF 0x00
    write-reg 0x80 0x00

    // Disable SIGNAL-RATE-MSRC (bit 1) and SIGNAL-RATE-PRE-RANGE (bit 4) checks.
    write-reg MSRC-CONFIG-CONTROL ((read-reg MSRC-CONFIG-CONTROL) | 0x12)
    set-signal-rate-limit 0.25
    write-reg SYSTEM-SEQUENCE-CONFIG 0xFF

    spad-info := get-spad-info_
    if not spad-info: return false
    spad-count := spad-info[0]
    spad-type-is-aperture := spad-info[1]

    ref-spad-map := read-multi GLOBAL-CONFIG-SPAD-ENABLES-REF-0 6

    write-reg 0xFF 0x01
    write-reg DYNAMIC-SPAD-REF-EN-START-OFFSET 0x00
    write-reg DYNAMIC-SPAD-NUM-REQUESTED-REF-SPAD 0x2C
    write-reg 0xFF 0x00
    write-reg GLOBAL-CONFIG-REF-EN-START-SELECT 0xB4

    first-spad-to-enable := spad-type-is-aperture ? 12 : 0
    spads-enabled := 0
    48.repeat: | i |
      if i < first-spad-to-enable or spads-enabled == spad-count:
        ref-spad-map[i / 8] = ref-spad-map[i / 8] & (0xFF - (1 << (i % 8)))
      else if ((ref-spad-map[i / 8] >> (i % 8)) & 0x1) == 1:
        spads-enabled++

    write-multi GLOBAL-CONFIG-SPAD-ENABLES-REF-0 ref-spad-map

    load-tuning-settings_

    // "Set interrupt config to new sample ready"
    write-reg SYSTEM-INTERRUPT-CONFIG-GPIO 0x04
    write-reg GPIO-HV-MUX-ACTIVE-HIGH ((read-reg GPIO-HV-MUX-ACTIVE-HIGH) & 0xEF)
    write-reg SYSTEM-INTERRUPT-CLEAR 0x01

    measurement-timing-budget-us_ = get-measurement-timing-budget

    // "Disable MSRC and TCC by default"
    write-reg SYSTEM-SEQUENCE-CONFIG 0xE8
    set-measurement-timing-budget measurement-timing-budget-us_

    write-reg SYSTEM-SEQUENCE-CONFIG 0x01
    if not perform-single-ref-calibration_ 0x40: return false

    write-reg SYSTEM-SEQUENCE-CONFIG 0x02
    if not perform-single-ref-calibration_ 0x00: return false

    write-reg SYSTEM-SEQUENCE-CONFIG 0xE8
    return true

  set-signal-rate-limit limit-mcps/float -> bool:
    if limit-mcps < 0.0 or limit-mcps > 511.99: return false
    // Q9.7 fixed point.
    write-reg-16 FINAL-RANGE-CONFIG-MIN-COUNT-RATE-RTN-LIMIT
        (limit-mcps * 128.0).to-int
    return true

  // --- Continuous ranging ----------------------------------------------------

  /**
  Starts continuous ranging.

  With $period-ms of 0 the sensor measures back to back, which is what the
    airframe wants.
  */
  start-continuous period-ms/int=0 -> none:
    write-reg 0x80 0x01
    write-reg 0xFF 0x01
    write-reg 0x00 0x00
    write-reg 0x91 stop-variable_
    write-reg 0x00 0x01
    write-reg 0xFF 0x00
    write-reg 0x80 0x00

    if period-ms != 0:
      osc-calibrate-val := read-reg-16 OSC-CALIBRATE-VAL
      period := osc-calibrate-val != 0 ? period-ms * osc-calibrate-val : period-ms
      write-reg-32 SYSTEM-INTERMEASUREMENT-PERIOD period
      write-reg SYSRANGE-START 0x04  // timed
    else:
      write-reg SYSRANGE-START 0x02  // back to back

  stop-continuous -> none:
    write-reg SYSRANGE-START 0x01
    write-reg 0xFF 0x01
    write-reg 0x00 0x00
    write-reg 0x91 0x00
    write-reg 0x00 0x01
    write-reg 0xFF 0x00

  /**
  Reads the latest range in millimetres, or null if the read timed out.

  The Pololu library returns the magic value 65535 on timeout. Null is the
    Toit way to say "no reading", and it cannot be mistaken for a distance.
  */
  read-range -> int?:
    deadline := start-timeout_
    while ((read-reg RESULT-INTERRUPT-STATUS) & 0x07) == 0:
      if timeout-expired_ deadline:
        did-timeout_ = true
        return null
    range := read-reg-16 (RESULT-RANGE-STATUS + 10)
    write-reg SYSTEM-INTERRUPT-CLEAR 0x01
    return range

  /**
  Reads the latest range only if the sensor has one ready, else returns null.

  The blocking $read-range spins on the interrupt status register. Toit tasks
    are cooperative, so that spin would hold the attitude loop hostage for up
    to the whole timeout. In continuous mode a sample is almost always there.
  */
  read-range-if-ready -> int?:
    if ((read-reg RESULT-INTERRUPT-STATUS) & 0x07) == 0: return null
    range := read-reg-16 (RESULT-RANGE-STATUS + 10)
    write-reg SYSTEM-INTERRUPT-CLEAR 0x01
    return range

  // --- Timing budget ---------------------------------------------------------

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

  set-measurement-timing-budget budget-us/int -> bool:
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

    write-reg-16 FINAL-RANGE-CONFIG-TIMEOUT-MACROP-HI
        encode-timeout_ final-range-timeout-mclks
    measurement-timing-budget-us_ = budget-us
    return true

  // --- Private ---------------------------------------------------------------

  get-spad-info_ -> List?:
    write-reg 0x80 0x01
    write-reg 0xFF 0x01
    write-reg 0x00 0x00
    write-reg 0xFF 0x06
    write-reg 0x83 ((read-reg 0x83) | 0x04)
    write-reg 0xFF 0x07
    write-reg 0x81 0x01
    write-reg 0x80 0x01
    write-reg 0x94 0x6B
    write-reg 0x83 0x00

    deadline := start-timeout_
    while (read-reg 0x83) == 0x00:
      if timeout-expired_ deadline: return null

    write-reg 0x83 0x01
    tmp := read-reg 0x92
    count := tmp & 0x7F
    type-is-aperture := ((tmp >> 7) & 0x01) == 1

    write-reg 0x81 0x00
    write-reg 0xFF 0x06
    write-reg 0x83 ((read-reg 0x83) & 0xFB)
    write-reg 0xFF 0x01
    write-reg 0x00 0x01
    write-reg 0xFF 0x00
    write-reg 0x80 0x00
    return [count, type-is-aperture]

  sequence-step-enables_ -> Map:
    config := read-reg SYSTEM-SEQUENCE-CONFIG
    return {
      "tcc": ((config >> 4) & 0x1) == 1,
      "dss": ((config >> 3) & 0x1) == 1,
      "msrc": ((config >> 2) & 0x1) == 1,
      "pre-range": ((config >> 6) & 0x1) == 1,
      "final-range": ((config >> 7) & 0x1) == 1,
    }

  sequence-step-timeouts_ enables/Map -> Map:
    pre-range-vcsel := decode-vcsel-period_ (read-reg PRE-RANGE-CONFIG-VCSEL-PERIOD)
    msrc-dss-tcc-mclks := (read-reg MSRC-CONFIG-TIMEOUT-MACROP) + 1
    msrc-dss-tcc-us := timeout-mclks-to-us_ msrc-dss-tcc-mclks pre-range-vcsel
    pre-range-mclks := decode-timeout_ (read-reg-16 PRE-RANGE-CONFIG-TIMEOUT-MACROP-HI)
    pre-range-us := timeout-mclks-to-us_ pre-range-mclks pre-range-vcsel
    final-range-vcsel := decode-vcsel-period_ (read-reg FINAL-RANGE-CONFIG-VCSEL-PERIOD)
    final-range-mclks := decode-timeout_ (read-reg-16 FINAL-RANGE-CONFIG-TIMEOUT-MACROP-HI)
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
    write-reg SYSRANGE-START (0x01 | vhv-init-byte)
    deadline := start-timeout_
    while ((read-reg RESULT-INTERRUPT-STATUS) & 0x07) == 0:
      if timeout-expired_ deadline: return false
    write-reg SYSTEM-INTERRUPT-CLEAR 0x01
    write-reg SYSRANGE-START 0x00
    return true

  static decode-vcsel-period_ reg-value/int -> int:
    return (reg-value + 1) << 1

  /// Macro period in nanoseconds. PLL period 1655 ps, macro period 2304 vclks.
  static calc-macro-period_ vcsel-period-pclks/int -> int:
    return ((2304 * vcsel-period-pclks * 1655) + 500) / 1000

  /// Format: "(LSByte * 2^MSByte) + 1".
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
    write-reg 0xFF 0x01
    write-reg 0x00 0x00
    write-reg 0xFF 0x00
    write-reg 0x09 0x00
    write-reg 0x10 0x00
    write-reg 0x11 0x00
    write-reg 0x24 0x01
    write-reg 0x25 0xFF
    write-reg 0x75 0x00
    write-reg 0xFF 0x01
    write-reg 0x4E 0x2C
    write-reg 0x48 0x00
    write-reg 0x30 0x20
    write-reg 0xFF 0x00
    write-reg 0x30 0x09
    write-reg 0x54 0x00
    write-reg 0x31 0x04
    write-reg 0x32 0x03
    write-reg 0x40 0x83
    write-reg 0x46 0x25
    write-reg 0x60 0x00
    write-reg 0x27 0x00
    write-reg 0x50 0x06
    write-reg 0x51 0x00
    write-reg 0x52 0x96
    write-reg 0x56 0x08
    write-reg 0x57 0x30
    write-reg 0x61 0x00
    write-reg 0x62 0x00
    write-reg 0x64 0x00
    write-reg 0x65 0x00
    write-reg 0x66 0xA0
    write-reg 0xFF 0x01
    write-reg 0x22 0x32
    write-reg 0x47 0x14
    write-reg 0x49 0xFF
    write-reg 0x4A 0x00
    write-reg 0xFF 0x00
    write-reg 0x7A 0x0A
    write-reg 0x7B 0x00
    write-reg 0x78 0x21
    write-reg 0xFF 0x01
    write-reg 0x23 0x34
    write-reg 0x42 0x00
    write-reg 0x44 0xFF
    write-reg 0x45 0x26
    write-reg 0x46 0x05
    write-reg 0x40 0x40
    write-reg 0x0E 0x06
    write-reg 0x20 0x1A
    write-reg 0x43 0x40
    write-reg 0xFF 0x00
    write-reg 0x34 0x03
    write-reg 0x35 0x44
    write-reg 0xFF 0x01
    write-reg 0x31 0x04
    write-reg 0x4B 0x09
    write-reg 0x4C 0x05
    write-reg 0x4D 0x04
    write-reg 0xFF 0x00
    write-reg 0x44 0x00
    write-reg 0x45 0x20
    write-reg 0x47 0x08
    write-reg 0x48 0x28
    write-reg 0x67 0x00
    write-reg 0x70 0x04
    write-reg 0x71 0x01
    write-reg 0x72 0xFE
    write-reg 0x76 0x00
    write-reg 0x77 0x00
    write-reg 0xFF 0x01
    write-reg 0x0D 0x01
    write-reg 0xFF 0x00
    write-reg 0x80 0x01
    write-reg 0x01 0xF8
    write-reg 0xFF 0x01
    write-reg 0x8E 0x01
    write-reg 0x00 0x01
    write-reg 0xFF 0x00
    write-reg 0x80 0x00

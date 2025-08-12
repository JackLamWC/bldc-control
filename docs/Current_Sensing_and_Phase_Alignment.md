## Current Sensing and Phase Alignment Guide

### Purpose
This guide explains how to:
- Verify BLDC phase sequence (A→B→C) using the built-in alignment mode
- Calibrate per-phase current sensor offsets
- Align current sensing channels with PWM phases using software mapping/signs (no rewiring)
- Validate that currents are correct in amps

### Prerequisites
- Firmware built with the latest changes (per-channel offsets, mapping, and signs)
- Serial shell connected (115200 baud)
- Motor secured; power applied; no mechanical load during calibration

---

## 1) Verify phase sequence (A→B→C)
The rotor should advance by 120/P mechanical degrees per alignment step, where P is `MOTOR_POLE_PAIRS` (e.g., 7 → ~17.14°).

Steps:
1. Run each alignment and read angle:
   ```
   align_start 7000 3000 5000
   align_status   # note angle θ0
   align_start 5000 7000 3000
   align_status   # note angle θ1
   align_start 3000 5000 7000
   align_status   # note angle θ2
   ```
2. Compute wrapped deltas Δ1 = wrap(θ1−θ0), Δ2 = wrap(θ2−θ1) to [−180°, +180°]
3. Expect ~+17.14° per step for 7 pole pairs (sign indicates direction). If sign is negative, the sequence is reversed (fix later in software if desired).

Stop alignment when done:
```
align_stop
```

---

## 2) Calibrate zero-current offsets (per-channel)
Offsets differ between phases; calibrate at idle so that Ia≈Ib≈Ic≈0 A.

1. Ensure no PWM and control disabled:
   ```
   align_stop
   angle_enable 0
   ```
2. Auto-calibrate offsets (averages samples):
   ```
   current_offset_auto
   ```
3. Check amps at idle (should be near zero; Sum≈0):
   ```
   current_amps
   ```

Formula used by firmware: I[A] = (V − offset) × 16.367, where offset is per-phase and 16.367 = 1/(Rshunt×gain).

Optional: manually tweak one phase offset if needed:
```
current_offset_set A 1.665
current_offsets
```

---

## 3) Align current sensing mapping and polarity (no hardware swap)
If the measured phases do not correspond to the PWM-driven phases under alignment, set a software mapping and/or sign flips.

Concepts:
- Mapping selects which measured channel (A,B,C) is used for true (Ia,Ib,Ic). Indices are 0=A,1=B,2=C.
- Signs flip polarity per phase if your analog front-end inverts a channel.

Commands:
```
current_map            # show mapping [iA,iB,iC]
current_map 1 2 0      # example: true (Ia,Ib,Ic) = measured (B,C,A)

current_signs          # show signs A,B,C
current_signs 1 1 1    # example: all positive
current_signs -1 1 1   # example: flip phase A
```

Procedure:
1. Set an initial mapping (identity) and signs positive:
   ```
   current_map 0 1 2
   current_signs 1 1 1
   ```
2. Recalibrate offsets at idle:
   ```
   align_stop
   angle_enable 0
   current_offset_auto
   ```
3. Run alignment steps and check signs with amps:
   ```
   align_start 9000 1000 5000   # A high, B low, C mid
   current_amps                 # expect Ia>0, Ib<0, Ic≈0, Sum≈0
   align_start 5000 9000 1000   # expect Ib>0, Ic<0, Ia≈0
   current_amps
   align_start 1000 5000 9000   # expect Ic>0, Ia<0, Ib≈0
   current_amps
   ```
4. If the “largest” phase is not the driven one, rotate mapping until it is. If a phase’s sign is opposite, flip that phase with `current_signs`.

Tips:
- Re-run `current_offset_auto` after changing mapping/signs.
- “Sum” may not be exactly zero under alignment due to sampling instant; prioritize correct rotation and signs.

Stop alignment:
```
align_stop
```

---

## 4) Set encoder electrical offset
With alignment active and the rotor settled, set the encoder offset the shell prints:
```
align_start 8500 1500 5000
align_status   # shows: Set offset now with: encoder_offset <deg>
encoder_offset <deg>
align_stop
```

---

## 5) Quick torque/closed-loop check
1. Enable controller and command small torque:
   ```
   angle_enable 1
   q_ref_set 0.1     # ~10% of max current (≈2.7A if FOC_MAX_CURRENT=27A)
   ```
2. Expect smooth hold/rotation. If direction is opposite, invert feedback polarity:
   ```
   motor_direction -1
   ```

Stop torque:
```
q_ref_set off
angle_enable 0
```

---

## 6) Troubleshooting
- Idle currents not near zero:
  - Ensure `align_stop` and `angle_enable 0` before `current_offset_auto`
  - Repeat `current_offset_auto` twice; or nudge with `current_offset_set`
- Wrong phase responds under alignment:
  - Adjust mapping with `current_map`, e.g., `current_map 1 2 0`
- Signs reversed for a phase:
  - Flip with `current_signs`, e.g., `current_signs -1 1 1`
- Encoder angle steps not ~120/P:
  - Re-check pole pairs and alignment duties; let rotor settle 1–2 s before reading

---

## Reference (shell commands)
```
# ADC/FOC status
status

# Alignment
align_start <A B C>
align_status
align_stop

# Angle control
angle_enable <0|1>
angle_set <deg> | angle_revs <rev> | angle_move <deg>
encoder_offset <deg>
motor_direction <1|-1>

# Current sensors
current_offset_auto
current_offsets | current_offsets <A_V> <B_V> <C_V>
current_offset_set <A|B|C> <V>
current_amps
current_signs | current_signs <A_sign> <B_sign> <C_sign>
current_map | current_map <iA> <iB> <iC>
```

---

## Notes
- Expected mechanical step per alignment: 120° / `MOTOR_POLE_PAIRS`
- Current conversion constants are defined in `src/foc/foc.h`:
  - Rshunt=0.005 Ω, gain=12.22, offset≈1.65 V → 1/(R×gain)=16.367
- Mapping/signs persist in RAM only; set them at boot or add persistence if needed.


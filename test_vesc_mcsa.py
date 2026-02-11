#!/usr/bin/env python3
"""
VESC Test Script — Standard Values + MCSA Phase Currents

Usage:
    python3 test_vesc_mcsa.py                  # Default: /dev/ttyACM0
    python3 test_vesc_mcsa.py /dev/ttyUSB0     # Custom port
    python3 test_vesc_mcsa.py /dev/ttyACM0 5   # Custom port + 5 seconds MCSA
"""

import sys
import time
import numpy as np
from pyvesc import encode, decode
from pyvesc.VESC import VESC
from pyvesc.VESC.messages.getters import GetValues, GetVersion, McsaStreamData
from pyvesc.VESC.messages.setters import SetMcsaStreamStart, SetMcsaStreamStop


def print_header(title):
    print(f"\n{'=' * 60}")
    print(f"  {title}")
    print(f"{'=' * 60}")


def print_values(m):
    """Print all standard VESC measurements."""
    print(f"  FET Temperature:       {m.temp_fet:.1f} °C")
    print(f"  Motor Temperature:     {m.temp_motor:.1f} °C")
    print(f"  Avg Motor Current:     {m.avg_motor_current:.2f} A")
    print(f"  Avg Input Current:     {m.avg_input_current:.2f} A")
    print(f"  Avg Id:                {m.avg_id:.2f} A")
    print(f"  Avg Iq:                {m.avg_iq:.2f} A")
    print(f"  Duty Cycle:            {m.duty_cycle_now:.1%}")
    print(f"  RPM:                   {m.rpm}")
    print(f"  Input Voltage:         {m.v_in:.2f} V")
    print(f"  Amp Hours:             {m.amp_hours:.4f} Ah")
    print(f"  Amp Hours Charged:     {m.amp_hours_charged:.4f} Ah")
    print(f"  Watt Hours:            {m.watt_hours:.4f} Wh")
    print(f"  Watt Hours Charged:    {m.watt_hours_charged:.4f} Wh")
    print(f"  Tachometer:            {m.tachometer}")
    print(f"  Tachometer Abs:        {m.tachometer_abs}")
    print(f"  Fault Code:            {ord(m.mc_fault_code)}")
    print(f"  PID Position:          {m.pid_pos_now:.4f}")
    print(f"  Controller ID:         {ord(m.app_controller_id)}")
    print(f"  Time:                  {m.time_ms} ms")


def collect_mcsa(vesc, duration_s=2.0):
    """Collect MCSA phase current data for the given duration."""
    all_ia = []
    all_ib = []
    all_ic = []
    sample_rate = 0.0
    total_samples = 0
    packets = 0
    gaps = 0
    expected_next = None

    # Start streaming
    vesc.mcsa_stream_start()
    t_start = time.monotonic()

    buf = bytearray()
    while (time.monotonic() - t_start) < duration_s:
        chunk = vesc.serial_port.read(max(1, vesc.serial_port.in_waiting))
        if not chunk:
            continue
        buf.extend(chunk)

        # Decode all complete packets in the buffer
        while True:
            msg, consumed = decode(buf)
            if msg is None:
                break
            buf = buf[consumed:]

            if not isinstance(msg, McsaStreamData):
                continue

            packets += 1
            sample_rate = msg.sample_rate
            total_samples += msg.num_samples

            # Check for gaps
            if expected_next is not None and msg.sample_cnt != expected_next:
                gap = msg.sample_cnt - expected_next
                gaps += 1
                print(f"  [WARNING] Gap detected: expected sample {expected_next}, "
                      f"got {msg.sample_cnt} (lost {gap} samples)")
            expected_next = msg.sample_cnt + msg.num_samples

            all_ia.extend(msg.ia)
            all_ib.extend(msg.ib)
            all_ic.extend(msg.ic)

    # Stop streaming
    vesc.mcsa_stream_stop()
    t_elapsed = time.monotonic() - t_start

    # Drain remaining data
    time.sleep(0.1)
    vesc.serial_port.read(vesc.serial_port.in_waiting)

    return {
        'ia': np.array(all_ia),
        'ib': np.array(all_ib),
        'ic': np.array(all_ic),
        'sample_rate': sample_rate,
        'total_samples': total_samples,
        'packets': packets,
        'gaps': gaps,
        'duration': t_elapsed,
    }


def print_mcsa_results(data):
    """Print MCSA collection statistics and current analysis."""
    ia, ib, ic = data['ia'], data['ib'], data['ic']

    print(f"  Sample Rate:           {data['sample_rate']:.0f} Hz")
    print(f"  Duration:              {data['duration']:.2f} s")
    print(f"  Packets Received:      {data['packets']}")
    print(f"  Total Samples:         {data['total_samples']}")
    print(f"  Gaps Detected:         {data['gaps']}")

    if len(ia) == 0:
        print("\n  [ERROR] No MCSA data received!")
        return

    effective_rate = len(ia) / data['duration']
    print(f"  Effective Sample Rate: {effective_rate:.0f} Hz")

    print(f"\n  Phase Currents (Ampere):")
    print(f"  {'Phase':<8} {'Min':>10} {'Max':>10} {'Mean':>10} {'RMS':>10} {'Std':>10}")
    print(f"  {'-'*58}")
    for name, arr in [('Ia', ia), ('Ib', ib), ('Ic', ic)]:
        rms = np.sqrt(np.mean(arr ** 2))
        print(f"  {name:<8} {arr.min():>10.4f} {arr.max():>10.4f} "
              f"{arr.mean():>10.4f} {rms:>10.4f} {arr.std():>10.4f}")

    # Total current magnitude
    i_mag = np.sqrt(ia**2 + ib**2 + ic**2)
    print(f"  {'|I|':<8} {i_mag.min():>10.4f} {i_mag.max():>10.4f} "
          f"{i_mag.mean():>10.4f} {np.sqrt(np.mean(i_mag**2)):>10.4f} {i_mag.std():>10.4f}")

    # Check current balance (sum should be ~0 for balanced 3-phase)
    i_sum = ia + ib + ic
    print(f"\n  Current Balance (Ia + Ib + Ic):")
    print(f"    Mean:  {i_sum.mean():.6f} A  (ideal: 0.0)")
    print(f"    Std:   {i_sum.std():.6f} A")

    # FFT preview of Ia
    if len(ia) >= 256 and data['sample_rate'] > 0:
        print(f"\n  FFT Preview (Phase A, top 5 frequencies):")
        N = len(ia)
        fft_vals = np.fft.rfft(ia * np.hanning(N))
        fft_mag = np.abs(fft_vals) * 2.0 / N
        freqs = np.fft.rfftfreq(N, d=1.0 / data['sample_rate'])
        # Skip DC (index 0)
        fft_mag[0] = 0
        top_idx = np.argsort(fft_mag)[-5:][::-1]
        print(f"  {'Freq (Hz)':>12} {'Magnitude (A)':>15}")
        print(f"  {'-'*30}")
        for idx in top_idx:
            if fft_mag[idx] > 1e-6:
                print(f"  {freqs[idx]:>12.1f} {fft_mag[idx]:>15.6f}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    mcsa_duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    print(f"Connecting to VESC on {port}...")

    with VESC(serial_port=port, start_heartbeat=True, baudrate=115200, timeout=0.1) as vesc:

        # ── Firmware Version ──
        print_header("Firmware Version")
        version = vesc.get_firmware_version()
        print(f"  Version: {version or 'unknown (no response)'}")

        # ── Standard Values ──
        print_header("Standard Values (COMM_GET_VALUES)")
        measurements = vesc.get_measurements()
        if measurements is not None:
            print_values(measurements)
        else:
            print("  [ERROR] No response from VESC!")

        # ── MCSA Phase Currents ──
        print_header(f"MCSA Phase Currents ({mcsa_duration:.1f}s)")
        print(f"  Starting synchronous current sampling...")
        data = collect_mcsa(vesc, duration_s=mcsa_duration)
        print_mcsa_results(data)

        # ── Second Values Read (to confirm VESC still responsive) ──
        print_header("Post-MCSA Check")
        measurements = vesc.get_measurements()
        if measurements is not None:
            print(f"  VESC responsive: OK")
            print(f"  Input Voltage:   {measurements.v_in:.2f} V")
            print(f"  RPM:             {measurements.rpm}")
            print(f"  Fault Code:      {ord(measurements.mc_fault_code)}")
        else:
            print("  [WARNING] No response after MCSA streaming!")

    print(f"\n{'=' * 60}")
    print(f"  Done.")
    print(f"{'=' * 60}\n")


if __name__ == '__main__':
    main()

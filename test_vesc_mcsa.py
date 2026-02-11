#!/usr/bin/env python3
"""
VESC MCSA Test — Motor start, phase current capture, and spectral analysis.

Usage:
    python3 test_vesc_mcsa.py                          # Defaults
    python3 test_vesc_mcsa.py --port /dev/ttyUSB0      # Custom port
    python3 test_vesc_mcsa.py --rpm 3000 --duration 5  # 3000 eRPM, 5s capture
    python3 test_vesc_mcsa.py --current 2.0             # Current mode instead of RPM
    python3 test_vesc_mcsa.py --no-motor                # Capture without starting motor
"""

import sys
import os
import time
import argparse
import signal
import numpy as np
from datetime import datetime
from pyvesc import encode, decode
from pyvesc.VESC import VESC
from pyvesc.VESC.messages.getters import GetValues, GetVersion, McsaStreamData
from pyvesc.VESC.messages.setters import (
    SetMcsaStreamStart, SetMcsaStreamStop, SetRPM, SetCurrent
)

# ── Globals for clean shutdown ───────────────────────────────────────────
_vesc = None
_motor_started = False


def signal_handler(sig, frame):
    """Handle Ctrl+C: stop motor and MCSA before exiting."""
    print("\n\n  [!] Ctrl+C detected — emergency stop...")
    if _vesc is not None:
        try:
            _vesc.mcsa_stream_stop()
        except Exception:
            pass
        if _motor_started:
            try:
                _vesc.set_current(0)
                _vesc.set_rpm(0)
            except Exception:
                pass
    sys.exit(1)


signal.signal(signal.SIGINT, signal_handler)


# ── Helpers ──────────────────────────────────────────────────────────────
def print_header(title):
    print(f"\n{'=' * 65}")
    print(f"  {title}")
    print(f"{'=' * 65}")


def print_values(m):
    print(f"  FET Temperature:       {m.temp_fet:.1f} C")
    print(f"  Motor Temperature:     {m.temp_motor:.1f} C")
    print(f"  Avg Motor Current:     {m.avg_motor_current:.2f} A")
    print(f"  Avg Input Current:     {m.avg_input_current:.2f} A")
    print(f"  Avg Id / Iq:           {m.avg_id:.2f} / {m.avg_iq:.2f} A")
    print(f"  Duty Cycle:            {m.duty_cycle_now:.1%}")
    print(f"  RPM:                   {m.rpm}")
    print(f"  Input Voltage:         {m.v_in:.2f} V")
    print(f"  Fault Code:            {ord(m.mc_fault_code)}")


def collect_mcsa(vesc, duration_s=2.0):
    """Collect MCSA data for the given duration. Returns dict with arrays."""
    all_ia, all_ib, all_ic = [], [], []
    sample_rate = 0.0
    packets = 0
    gaps = 0
    expected_next = None

    vesc.mcsa_stream_start()
    t_start = time.monotonic()
    buf = bytearray()

    while (time.monotonic() - t_start) < duration_s:
        chunk = vesc.serial_port.read(max(1, vesc.serial_port.in_waiting))
        if not chunk:
            continue
        buf.extend(chunk)
        while True:
            msg, consumed = decode(buf)
            if msg is None:
                break
            buf = buf[consumed:]
            if not isinstance(msg, McsaStreamData):
                continue
            packets += 1
            sample_rate = msg.sample_rate
            if expected_next is not None and msg.sample_cnt != expected_next:
                gaps += 1
            expected_next = msg.sample_cnt + msg.num_samples
            all_ia.extend(msg.ia)
            all_ib.extend(msg.ib)
            all_ic.extend(msg.ic)

    vesc.mcsa_stream_stop()
    t_elapsed = time.monotonic() - t_start
    time.sleep(0.05)
    vesc.serial_port.read(vesc.serial_port.in_waiting)

    return {
        'ia': np.array(all_ia), 'ib': np.array(all_ib), 'ic': np.array(all_ic),
        'sample_rate': sample_rate, 'packets': packets,
        'total_samples': len(all_ia), 'gaps': gaps, 'duration': t_elapsed,
    }


def print_stats(data):
    ia, ib, ic = data['ia'], data['ib'], data['ic']
    print(f"  Sample Rate:           {data['sample_rate']:.0f} Hz")
    print(f"  Duration:              {data['duration']:.2f} s")
    print(f"  Packets / Samples:     {data['packets']} / {data['total_samples']}")
    print(f"  Gaps:                  {data['gaps']}")
    if len(ia) == 0:
        print("  [ERROR] No data received!")
        return
    eff = len(ia) / data['duration']
    print(f"  Effective Rate:        {eff:.0f} Hz")
    print(f"\n  {'Phase':<6} {'Min':>9} {'Max':>9} {'Mean':>9} {'RMS':>9} {'Std':>9}")
    print(f"  {'-' * 52}")
    for name, arr in [('Ia', ia), ('Ib', ib), ('Ic', ic)]:
        rms = np.sqrt(np.mean(arr ** 2))
        print(f"  {name:<6} {arr.min():>9.4f} {arr.max():>9.4f} "
              f"{arr.mean():>9.4f} {rms:>9.4f} {arr.std():>9.4f}")
    i_sum = ia + ib + ic
    print(f"\n  Balance (Ia+Ib+Ic):    mean={i_sum.mean():.4f} A, std={i_sum.std():.4f} A")


# ── MCSA Spectral Analysis ──────────────────────────────────────────────
def mcsa_analysis(data, pole_pairs=7, rpm_measured=None):
    """Perform MCSA frequency analysis on the captured phase currents."""
    ia = data['ia']
    fs = data['sample_rate']
    N = len(ia)

    if N < 512 or fs <= 0:
        print("  [ERROR] Not enough data for FFT analysis.")
        return None

    # Frequency resolution
    df = fs / N
    print(f"\n  FFT Parameters:")
    print(f"    Samples:             {N}")
    print(f"    Sample Rate:         {fs:.0f} Hz")
    print(f"    Freq Resolution:     {df:.4f} Hz")
    print(f"    Max Frequency:       {fs/2:.0f} Hz")

    # Apply Hanning window and compute FFT for each phase
    window = np.hanning(N)
    freqs = np.fft.rfftfreq(N, d=1.0 / fs)

    results = {}
    for name, arr in [('Ia', data['ia']), ('Ib', data['ib']), ('Ic', data['ic'])]:
        fft_vals = np.fft.rfft(arr * window)
        # Amplitude spectrum (compensate for window energy loss)
        magnitude = np.abs(fft_vals) * 2.0 / (N * np.mean(window))
        magnitude[0] /= 2.0  # DC component
        results[name] = magnitude

    mag_ia = results['Ia']

    # ── Identify fundamental frequency ──
    # Electrical frequency from RPM: f_e = RPM * pole_pairs / 60
    f_electrical = None
    f_mechanical = None

    if rpm_measured is not None and rpm_measured > 0:
        f_mechanical = rpm_measured / 60.0
        f_electrical = f_mechanical * pole_pairs
        print(f"\n  Motor Parameters:")
        print(f"    Pole Pairs:          {pole_pairs}")
        print(f"    Measured RPM:        {rpm_measured:.0f}")
        print(f"    Mechanical Freq:     {f_mechanical:.2f} Hz")
        print(f"    Electrical Freq:     {f_electrical:.2f} Hz")

    # Find dominant frequency (skip DC, search above 5 Hz)
    min_bin = max(1, int(5.0 / df))
    max_bin = len(mag_ia) - 1
    peak_bin = min_bin + np.argmax(mag_ia[min_bin:max_bin])
    f_dominant = freqs[peak_bin]
    a_dominant = mag_ia[peak_bin]

    print(f"\n  Dominant Frequency:    {f_dominant:.2f} Hz  ({a_dominant:.6f} A)")
    if f_electrical:
        print(f"  Expected Electrical:   {f_electrical:.2f} Hz")
        print(f"  Deviation:             {abs(f_dominant - f_electrical):.2f} Hz")

    # ── Top spectral peaks ──
    print(f"\n  Top 10 Spectral Peaks (Phase A):")
    print(f"  {'#':<4} {'Freq (Hz)':>12} {'Ampl (A)':>12} {'Ampl (dB)':>12}  Note")
    print(f"  {'-' * 58}")

    # Find peaks (local maxima above noise floor)
    noise_floor = np.median(mag_ia[min_bin:]) * 3
    peak_indices = []
    for i in range(min_bin + 1, max_bin - 1):
        if mag_ia[i] > mag_ia[i-1] and mag_ia[i] > mag_ia[i+1] and mag_ia[i] > noise_floor:
            peak_indices.append(i)
    # Sort by amplitude
    peak_indices.sort(key=lambda i: mag_ia[i], reverse=True)

    ref_amplitude = a_dominant if a_dominant > 0 else 1.0
    for rank, idx in enumerate(peak_indices[:10], 1):
        f = freqs[idx]
        a = mag_ia[idx]
        db = 20 * np.log10(a / ref_amplitude) if a > 0 else -999
        note = _identify_frequency(f, f_electrical, f_mechanical, pole_pairs)
        print(f"  {rank:<4} {f:>12.2f} {a:>12.6f} {db:>12.1f}  {note}")

    # ── Characteristic MCSA fault frequencies ──
    if f_electrical and f_mechanical:
        print(f"\n  MCSA Fault Frequency Check:")
        print(f"  {'Fault Type':<25} {'Expected (Hz)':>14} {'Measured (A)':>14} {'dB':>8}")
        print(f"  {'-' * 65}")

        checks = [
            ("Fundamental (f_e)", f_electrical),
            ("2x Electrical", 2 * f_electrical),
            ("3x Electrical", 3 * f_electrical),
            ("5x Electrical", 5 * f_electrical),
            ("Mechanical (f_r)", f_mechanical),
            ("2x Mechanical", 2 * f_mechanical),
            ("Eccentricity (f_e-f_r)", f_electrical - f_mechanical),
            ("Eccentricity (f_e+f_r)", f_electrical + f_mechanical),
        ]

        for label, f_check in checks:
            if f_check <= 0 or f_check >= fs / 2:
                continue
            bin_idx = int(round(f_check / df))
            if 0 < bin_idx < len(mag_ia):
                # Take max in ±2 bins for spectral leakage
                lo = max(1, bin_idx - 2)
                hi = min(len(mag_ia) - 1, bin_idx + 3)
                local_max = np.max(mag_ia[lo:hi])
                db = 20 * np.log10(local_max / ref_amplitude) if local_max > 0 else -999
                print(f"  {label:<25} {f_check:>14.2f} {local_max:>14.6f} {db:>8.1f}")

    return {
        'freqs': freqs, 'mag_ia': mag_ia, 'mag_ib': results['Ib'],
        'mag_ic': results['Ic'], 'f_dominant': f_dominant,
    }


def _identify_frequency(f, f_e, f_m, pp):
    """Try to label a frequency peak."""
    if f_e is None or f_e <= 0:
        return ""
    tol = max(2.0, f_e * 0.02)  # 2% or 2 Hz tolerance
    # Check harmonics of electrical freq
    for n in range(1, 20):
        if abs(f - n * f_e) < tol:
            return f"{n}x f_e" if n > 1 else "f_e (fundamental)"
    # Check mechanical freq harmonics
    if f_m and f_m > 0:
        for n in range(1, 10):
            if abs(f - n * f_m) < tol:
                return f"{n}x f_mech" if n > 1 else "f_mech"
        # Eccentricity sidebands
        for n in range(1, 5):
            if abs(f - (n * f_e - f_m)) < tol:
                return f"{n}*f_e - f_mech"
            if abs(f - (n * f_e + f_m)) < tol:
                return f"{n}*f_e + f_mech"
    # Switching frequency
    if abs(f - 15000) < 500 or abs(f - 30000) < 500:
        return "PWM switching"
    return ""


def save_data(data, analysis, filename):
    """Save raw time-domain data and FFT results to CSV files."""
    # Time domain
    td_file = filename + "_time.csv"
    N = data['total_samples']
    t = np.arange(N) / data['sample_rate']
    header = "time_s,ia_A,ib_A,ic_A"
    np.savetxt(td_file, np.column_stack([t, data['ia'], data['ib'], data['ic']]),
               delimiter=',', header=header, comments='', fmt='%.8f')
    print(f"  Time data:  {td_file} ({N} samples)")

    # Frequency domain
    if analysis is not None:
        fd_file = filename + "_fft.csv"
        header = "freq_Hz,mag_ia_A,mag_ib_A,mag_ic_A"
        np.savetxt(fd_file, np.column_stack([
            analysis['freqs'], analysis['mag_ia'],
            analysis['mag_ib'], analysis['mag_ic']
        ]), delimiter=',', header=header, comments='', fmt='%.8f')
        print(f"  FFT data:   {fd_file} ({len(analysis['freqs'])} bins)")


def try_plot(data, analysis, pole_pairs, rpm):
    """Try to plot results with matplotlib (optional)."""
    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [INFO] matplotlib not available, skipping plots.")
        return

    ia, ib, ic = data['ia'], data['ib'], data['ic']
    fs = data['sample_rate']
    N = len(ia)
    t = np.arange(N) / fs

    fig, axes = plt.subplots(3, 1, figsize=(14, 10))

    # ── Plot 1: Time domain (first 500 samples) ──
    ax = axes[0]
    show_n = min(500, N)
    ax.plot(t[:show_n] * 1000, ia[:show_n], label='Ia', linewidth=0.8)
    ax.plot(t[:show_n] * 1000, ib[:show_n], label='Ib', linewidth=0.8)
    ax.plot(t[:show_n] * 1000, ic[:show_n], label='Ic', linewidth=0.8)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Current (A)')
    ax.set_title('Phase Currents (Time Domain)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    if analysis is not None:
        freqs = analysis['freqs']
        mag_ia = analysis['mag_ia']
        mag_ib = analysis['mag_ib']
        mag_ic = analysis['mag_ic']

        # ── Plot 2: FFT full spectrum ──
        ax = axes[1]
        ax.semilogy(freqs, mag_ia, label='Ia', linewidth=0.6, alpha=0.8)
        ax.semilogy(freqs, mag_ib, label='Ib', linewidth=0.6, alpha=0.8)
        ax.semilogy(freqs, mag_ic, label='Ic', linewidth=0.6, alpha=0.8)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Amplitude (A)')
        ax.set_title('MCSA Spectrum (Full Range)')
        ax.legend()
        ax.grid(True, alpha=0.3, which='both')

        # ── Plot 3: FFT zoomed around fundamental ──
        ax = axes[2]
        if rpm and rpm > 0:
            f_e = rpm / 60.0 * pole_pairs
            zoom_max = min(f_e * 8, fs / 2)
        else:
            zoom_max = min(2000, fs / 2)
        mask = freqs <= zoom_max
        ax.plot(freqs[mask], mag_ia[mask], label='Ia', linewidth=0.8)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('Amplitude (A)')
        ax.set_title(f'MCSA Spectrum (0 - {zoom_max:.0f} Hz)')
        # Mark electrical frequency
        if rpm and rpm > 0:
            for n in range(1, 8):
                f_h = n * f_e
                if f_h < zoom_max:
                    ax.axvline(f_h, color='red', alpha=0.4, linestyle='--', linewidth=0.8)
                    ax.text(f_h, ax.get_ylim()[1] * 0.9, f'{n}x f_e',
                            fontsize=7, ha='center', color='red')
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_file = f"mcsa_plot_{datetime.now().strftime('%H%M%S')}.png"
    plt.savefig(plot_file, dpi=150)
    print(f"  Plot saved: {plot_file}")
    plt.show(block=False)
    plt.pause(0.5)


# ── Main ─────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="VESC MCSA Test & Analysis")
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--rpm', type=int, default=2000, help='Target eRPM')
    parser.add_argument('--current', type=float, default=None,
                        help='Use current mode (A) instead of RPM')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='MCSA capture duration in seconds')
    parser.add_argument('--pole-pairs', type=int, default=7,
                        help='Motor pole pairs (default: 7)')
    parser.add_argument('--settle', type=float, default=2.0,
                        help='Seconds to wait for motor to stabilize')
    parser.add_argument('--no-motor', action='store_true',
                        help='Capture without starting the motor')
    parser.add_argument('--no-plot', action='store_true',
                        help='Skip matplotlib plots')
    parser.add_argument('--save', default=None,
                        help='Save CSV prefix (default: auto-generated)')
    args = parser.parse_args()

    global _vesc, _motor_started

    print(f"Connecting to VESC on {args.port}...")
    with VESC(serial_port=args.port, start_heartbeat=True,
              baudrate=115200, timeout=0.1) as vesc:
        _vesc = vesc

        # ── Firmware Version ──
        print_header("Firmware Version")
        version = vesc.get_firmware_version()
        print(f"  Version: {version or 'unknown'}")

        # ── Initial Values ──
        print_header("Initial State")
        m = vesc.get_measurements()
        if m:
            print_values(m)
        else:
            print("  [ERROR] No response!")
            return

        # ── Baseline capture (motor off) ──
        print_header("Baseline Capture (Motor OFF, 1s)")
        baseline = collect_mcsa(vesc, duration_s=1.0)
        print_stats(baseline)

        # ── Start motor ──
        rpm_measured = 0
        if not args.no_motor:
            if args.current is not None:
                print_header(f"Starting Motor (Current Mode: {args.current:.1f} A)")
                vesc.set_current(args.current)
                mode_str = f"{args.current:.1f}A"
            else:
                print_header(f"Starting Motor (RPM Mode: {args.rpm} eRPM)")
                vesc.set_rpm(args.rpm)
                mode_str = f"{args.rpm} eRPM"
            _motor_started = True

            print(f"  Waiting {args.settle:.1f}s for motor to stabilize...")
            time.sleep(args.settle)

            m = vesc.get_measurements()
            if m:
                rpm_measured = m.rpm
                print(f"  Actual RPM:     {m.rpm}")
                print(f"  Motor Current:  {m.avg_motor_current:.2f} A")
                print(f"  Duty Cycle:     {m.duty_cycle_now:.1%}")
                print(f"  Input Voltage:  {m.v_in:.2f} V")
                fault = ord(m.mc_fault_code)
                if fault != 0:
                    print(f"  [WARNING] Fault code: {fault}")
        else:
            mode_str = "motor off"

        # ── MCSA Capture ──
        print_header(f"MCSA Capture ({args.duration:.1f}s, {mode_str})")
        print(f"  Capturing...")
        data = collect_mcsa(vesc, duration_s=args.duration)
        print_stats(data)

        # ── Stop motor ──
        if _motor_started:
            print(f"\n  Stopping motor...")
            vesc.set_current(0)
            time.sleep(0.5)
            _motor_started = False

            m = vesc.get_measurements()
            if m:
                print(f"  Post-stop RPM:  {m.rpm}")
                print(f"  Fault Code:     {ord(m.mc_fault_code)}")

        # ── MCSA Analysis ──
        if data['total_samples'] > 0:
            print_header("MCSA Spectral Analysis")
            analysis = mcsa_analysis(
                data,
                pole_pairs=args.pole_pairs,
                rpm_measured=rpm_measured if rpm_measured else None,
            )

            # ── Save data ──
            prefix = args.save or f"mcsa_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            print_header("Saving Data")
            save_data(data, analysis, prefix)

            # ── Plot ──
            if not args.no_plot:
                print_header("Plots")
                try_plot(data, analysis, args.pole_pairs, rpm_measured)

    _vesc = None
    print(f"\n{'=' * 65}")
    print(f"  Done.")
    print(f"{'=' * 65}\n")


if __name__ == '__main__':
    main()

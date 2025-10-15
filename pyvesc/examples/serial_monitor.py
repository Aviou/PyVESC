"""Example that starts a VESC controlled motor and streams measurements.

This script connects to a VESC over a serial port, applies a small duty
cycle so the motor starts spinning, and then prints values returned by the
``COMM_GET_VALUES`` command.  It is compatible with both legacy VESC 4/5 and
VESC 6 firmware where some of the field names were renamed.
"""

import argparse
import time
from typing import Any

from pyvesc import VESC


def _extract_measurement(measurements: Any, *attribute_names: str) -> float:
    """Return the first available attribute in ``attribute_names``.

    ``COMM_GET_VALUES`` renamed several fields with the VESC 6 firmware.  This
    helper mirrors :meth:`pyvesc.VESC.VESC._resolve_measurement_attribute`
    without exposing the private method to users of the example script.
    """

    for attr in attribute_names:
        if hasattr(measurements, attr):
            return getattr(measurements, attr)
    raise AttributeError(
        f"None of the expected attributes {attribute_names} are present on the "
        "GetValues response."
    )


def stream_measurements(serial_port: str, duty_cycle: float, duration: float, interval: float) -> None:
    """Spin the motor using ``duty_cycle`` and print measurement updates."""

    with VESC(serial_port=serial_port) as motor:
        firmware = motor.get_firmware_version()
        print(f"Connected to VESC firmware {firmware}")

        print(f"Applying duty cycle {duty_cycle:.3f} for {duration:.1f} seconds")
        motor.set_duty_cycle(duty_cycle)

        end_time = time.time() + duration
        try:
            while time.time() < end_time:
                measurements = motor.get_measurements()

                duty_now = _extract_measurement(measurements, "duty_cycle_now", "duty_now")
                motor_current = _extract_measurement(measurements, "avg_motor_current", "current_motor")
                input_current = _extract_measurement(measurements, "avg_input_current", "current_in")

                print(
                    "RPM: {rpm:8.0f} | Duty: {duty:7.3f} | Motor I: {motor_current:7.2f} A | "
                    "Input I: {input_current:7.2f} A | Vin: {vin:6.2f} V".format(
                        rpm=measurements.rpm,
                        duty=duty_now,
                        motor_current=motor_current,
                        input_current=input_current,
                        vin=measurements.v_in,
                    )
                )

                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nInterrupted by user, stopping motor...")
        finally:
            motor.set_duty_cycle(0.0)
            motor.set_current(0.0)
            print("Motor stopped.\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a VESC controlled motor over serial and stream COMM_GET_VALUES measurements.",
    )
    parser.add_argument(
        "serial_port",
        help="Serial port where the VESC is connected, e.g. COM3 or /dev/ttyACM0",
    )
    parser.add_argument(
        "--duty-cycle",
        type=float,
        default=0.05,
        help="Duty cycle to apply while streaming measurements (default: %(default)s)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Number of seconds to keep the motor running (default: %(default)s)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Delay between measurement requests in seconds (default: %(default)s)",
    )
    args = parser.parse_args()

    stream_measurements(
        serial_port=args.serial_port,
        duty_cycle=args.duty_cycle,
        duration=args.duration,
        interval=args.interval,
    )


if __name__ == "__main__":
    main()

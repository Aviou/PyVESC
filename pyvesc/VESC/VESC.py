from pyvesc.protocol.interface import encode_request, encode, decode
from pyvesc.VESC.messages import *
import time
import threading

# because people may want to use this library for their own messaging, do not make this a required package
try:
    import serial
except ImportError:
    serial = None


class VESC(object):
    def __init__(self, serial_port, has_sensor=False, start_heartbeat=True, baudrate=115200, timeout=0.05):
        """
        :param serial_port: Serial device to use for communication (i.e. "COM3" or "/dev/tty.usbmodem0")
        :param has_sensor: Whether or not the bldc motor is using a hall effect sensor
        :param start_heartbeat: Whether or not to automatically start the heartbeat thread that will keep commands
                                alive.
        :param baudrate: baudrate for the serial communication. Shouldn't need to change this.
        :param timeout: timeout for the serial communication
        """

        if serial is None:
            raise ImportError("Need to install pyserial in order to use the VESCMotor class.")

        self.serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)
        if has_sensor:
            self.serial_port.write(encode(SetRotorPositionMode(SetRotorPositionMode.DISP_POS_OFF)))

        self.alive_msg = [encode(Alive())]

        self.heart_beat_thread = threading.Thread(target=self._heartbeat_cmd_func)
        self._stop_heartbeat = threading.Event()

        if start_heartbeat:
            self.start_heartbeat()

        # check firmware version and set GetValue fields to old values if pre version 3.xx
        version = self.get_firmware_version()
        if version is not None:
            try:
                if int(version.split('.')[0]) < 3:
                    GetValues.fields = pre_v3_33_fields
            except (ValueError, IndexError):
                pass  # assume modern firmware

        # store message info for getting values so it doesn't need to calculate it every time
        msg = GetValues()
        self._get_values_msg = encode_request(msg)
        self._get_values_msg_expected_length = msg._full_msg_size

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop_heartbeat()
        if self.serial_port.is_open:
            self.serial_port.flush()
            self.serial_port.close()

    def _heartbeat_cmd_func(self):
        """
        Continuous function calling that keeps the motor alive
        """
        while not self._stop_heartbeat.is_set():
            time.sleep(0.1)
            for i in self.alive_msg:
                self.write(i)

    def start_heartbeat(self, can_id=None):
        """
        Starts a repetitive calling of the last set cmd to keep the motor alive.

        Args:
            can_id: Optional, used to specify the CAN ID to add to the existing heartbeat messaged
        """
        if can_id is not None:
            self.alive_msg.append(encode(Alive(can_id=can_id)))
        else:
            self.heart_beat_thread.start()

    def stop_heartbeat(self):
        """
        Stops the heartbeat thread and resets the last cmd function. THIS MUST BE CALLED BEFORE THE OBJECT GOES OUT OF
        SCOPE UNLESS WRAPPING IN A WITH STATEMENT (Assuming the heartbeat was started).
        """
        self._stop_heartbeat.set()
        if self.heart_beat_thread.is_alive():
            self.heart_beat_thread.join()

    def write(self, data, num_read_bytes=None):
        """
        A write wrapper function implemented like this to try and make it easier to incorporate other communication
        methods than UART in the future.
        :param data: the byte string to be sent
        :param num_read_bytes: number of bytes to read for decoding response
        :return: decoded response from buffer
        """
        self.serial_port.write(data)
        if num_read_bytes is not None:
            # Wait for response with timeout. Read incrementally until a valid
            # packet is decoded or we time out.
            start = time.monotonic()
            timeout = max(self.serial_port.timeout, 0.1)
            buf = bytearray()
            while (time.monotonic() - start) < timeout:
                avail = self.serial_port.in_waiting
                if avail > 0:
                    buf.extend(self.serial_port.read(avail))
                    response, consumed = decode(buf)
                    if response is not None:
                        return response
                time.sleep(0.001)
            # Final attempt with whatever is in the buffer
            avail = self.serial_port.in_waiting
            if avail > 0:
                buf.extend(self.serial_port.read(avail))
            response, consumed = decode(buf)
            return response

    def set_rpm(self, new_rpm, **kwargs):
        """
        Set the electronic RPM value (a.k.a. the RPM value of the stator)
        :param new_rpm: new rpm value
        """
        self.write(encode(SetRPM(new_rpm, **kwargs)))

    def set_current(self, new_current, **kwargs):
        """
        :param new_current: new current in milli-amps for the motor
        """
        self.write(encode(SetCurrent(new_current, **kwargs)))

    def set_duty_cycle(self, new_duty_cycle, **kwargs):
        """
        :param new_duty_cycle: Value of duty cycle to be set (range [-1e5, 1e5]).
        """
        self.write(encode(SetDutyCycle(new_duty_cycle, **kwargs)))

    def set_servo(self, new_servo_pos, **kwargs):
        """
        :param new_servo_pos: New servo position. valid range [0, 1]
        """
        self.write(encode(SetServoPosition(new_servo_pos, **kwargs)))

    def get_measurements(self):
        """
        :return: A msg object with attributes containing the measurement values
        """
        return self.write(self._get_values_msg, num_read_bytes=self._get_values_msg_expected_length)

    def get_firmware_version(self):
        msg = GetVersion()
        response = self.write(encode_request(msg), num_read_bytes=msg._full_msg_size)
        if response is None:
            return None
        return str(response)

    def get_rpm(self):
        """
        :return: Current motor rpm
        """
        return self.get_measurements().rpm

    def get_duty_cycle(self):
        """
        :return: Current applied duty-cycle
        """
        return self.get_measurements().duty_now

    def get_v_in(self):
        """
        :return: Current input voltage
        """
        return self.get_measurements().v_in

    def get_motor_current(self):
        """
        :return: Current motor current
        """
        return self.get_measurements().current_motor

    def get_incoming_current(self):
        """
        :return: Current incoming current
        """
        return self.get_measurements().current_in

    # ── MCSA Streaming ──────────────────────────────────────────────

    def mcsa_stream_start(self, **kwargs):
        """Start MCSA three-phase current streaming on the VESC."""
        self.write(encode(SetMcsaStreamStart(**kwargs)))

    def mcsa_stream_stop(self, **kwargs):
        """Stop MCSA three-phase current streaming on the VESC."""
        self.write(encode(SetMcsaStreamStop(**kwargs)))

    def mcsa_stream_read(self, timeout=None):
        """Read a single MCSA stream packet from the serial buffer.

        :param timeout: Optional read timeout override in seconds.
        :return: McsaStreamData message or None if no complete packet arrived.
        """
        old_timeout = self.serial_port.timeout
        if timeout is not None:
            self.serial_port.timeout = timeout
        try:
            buf = bytearray()
            # Read until we have enough bytes for at least one packet
            while True:
                chunk = self.serial_port.read(max(1, self.serial_port.in_waiting))
                if not chunk:
                    return None
                buf.extend(chunk)
                msg, consumed = decode(buf)
                if msg is not None:
                    return msg
        finally:
            self.serial_port.timeout = old_timeout

    def mcsa_stream_read_continuous(self, callback, duration=None, max_packets=None):
        """Read MCSA stream packets continuously and invoke *callback* for each.

        :param callback: ``callback(msg)`` — called with each McsaStreamData message.
                         Return ``False`` from the callback to stop early.
        :param duration: Maximum duration in seconds (``None`` = unlimited).
        :param max_packets: Maximum number of packets to read (``None`` = unlimited).
        """
        import time as _time
        deadline = None if duration is None else _time.monotonic() + duration
        count = 0
        buf = bytearray()
        while True:
            if deadline is not None and _time.monotonic() >= deadline:
                break
            if max_packets is not None and count >= max_packets:
                break
            chunk = self.serial_port.read(max(1, self.serial_port.in_waiting))
            if chunk:
                buf.extend(chunk)
            while True:
                msg, consumed = decode(buf)
                if msg is None:
                    break
                buf = buf[consumed:]
                count += 1
                if callback(msg) is False:
                    return
                if max_packets is not None and count >= max_packets:
                    return





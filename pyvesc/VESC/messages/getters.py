import struct

from pyvesc.protocol.base import VESCMessage
from pyvesc.VESC.messages import VedderCmd


pre_v3_33_fields = [('temp_mos1', 'h', 10),
                    ('temp_mos2', 'h', 10),
                    ('temp_mos3', 'h', 10),
                    ('temp_mos4', 'h', 10),
                    ('temp_mos5', 'h', 10),
                    ('temp_mos6', 'h', 10),
                    ('temp_pcb',  'h', 10),
                    ('current_motor', 'i', 100),
                    ('current_in',  'i', 100),
                    ('duty_now',    'h', 1000),
                    ('rpm',         'i', 1),
                    ('v_in',        'h', 10),
                    ('amp_hours',   'i', 10000),
                    ('amp_hours_charged', 'i', 10000),
                    ('watt_hours',  'i', 10000),
                    ('watt_hours_charged', 'i', 10000),
                    ('tachometer', 'i', 1),
                    ('tachometer_abs', 'i', 1),
                    ('mc_fault_code', 'c', 0)]


class GetVersion(metaclass=VESCMessage):
    """ Gets version fields
    """
    id = VedderCmd.COMM_FW_VERSION

    fields = [
            ('comm_fw_version', 'b', 0),
            ('fw_version_major', 'b', 0),
            ('fw_version_minor', 'b', 0)
    ]

    def __str__(self):
        return f"{self.comm_fw_version}.{self.fw_version_major}.{self.fw_version_minor}"


class GetValues(metaclass=VESCMessage):
    """ Gets internal sensor data
    """
    id = VedderCmd.COMM_GET_VALUES

    fields = [
        ('temp_fet', 'h', 10),
        ('temp_motor', 'h', 10),
        ('avg_motor_current', 'i', 100),
        ('avg_input_current', 'i', 100),
        ('avg_id', 'i', 100),
        ('avg_iq', 'i', 100),
        ('duty_cycle_now', 'h', 1000),
        ('rpm', 'i', 1),
        ('v_in', 'h', 10),
        ('amp_hours', 'i', 10000),
        ('amp_hours_charged', 'i', 10000),
        ('watt_hours', 'i', 10000),
        ('watt_hours_charged', 'i', 10000),
        ('tachometer', 'i', 1),
        ('tachometer_abs', 'i', 1),
        ('mc_fault_code', 'c', 0),
        ('pid_pos_now', 'i', 1000000),
        ('app_controller_id', 'c', 0),
        ('time_ms', 'i', 1),
    ]


class GetRotorPosition(metaclass=VESCMessage):
    """ Gets rotor position data

    Must be set to DISP_POS_MODE_ENCODER or DISP_POS_MODE_PID_POS (Mode 3 or
    Mode 4). This is set by SetRotorPositionMode (id=21).
    """
    id = VedderCmd.COMM_ROTOR_POSITION

    fields = [
            ('rotor_pos', 'i', 100000)
    ]


class McsaStreamData(metaclass=VESCMessage):
    """ MCSA stream data packet (variable length).

    Received from VESC when MCSA streaming is active. Contains batches of
    three-phase current samples captured at the full FOC sample rate.

    Packet layout after CMD byte:
        sample_cnt  : uint32  — running sample index (gap detection)
        sample_rate : float32 — FOC sample rate in Hz
        num_samples : uint8   — number of samples in this packet (1-32)
        ia[0..N]    : float32 — phase A current in Ampere (scaled ×1e3)
        ib[0..N]    : float32 — phase B current in Ampere (scaled ×1e3)
        ic[0..N]    : float32 — phase C current in Ampere (scaled ×1e3)
    """
    id = VedderCmd.COMM_MCSA_STREAM_DATA
    fields = []

    _HEADER_FMT = '!IfB'  # sample_cnt(u32), sample_rate(f32), num_samples(u8)
    _HEADER_SIZE = struct.calcsize(_HEADER_FMT)
    _SCALE = 1e3

    @classmethod
    def _custom_unpack(cls, data):
        sample_cnt, sample_rate, num_samples = struct.unpack_from(cls._HEADER_FMT, data, 0)
        offset = cls._HEADER_SIZE
        float_fmt = '!%uf' % num_samples
        float_size = struct.calcsize(float_fmt)
        ia = list(struct.unpack_from(float_fmt, data, offset))
        offset += float_size
        ib = list(struct.unpack_from(float_fmt, data, offset))
        offset += float_size
        ic = list(struct.unpack_from(float_fmt, data, offset))
        # Descale: firmware sends Ampere × 1e3
        ia = [v / cls._SCALE for v in ia]
        ib = [v / cls._SCALE for v in ib]
        ic = [v / cls._SCALE for v in ic]
        msg = cls.__new__(cls)
        msg.can_id = None
        msg.sample_cnt = sample_cnt
        msg.sample_rate = sample_rate
        msg.num_samples = num_samples
        msg.ia = ia
        msg.ib = ib
        msg.ic = ic
        return msg

    def _custom_pack(self):
        num = self.num_samples
        header = struct.pack(self._HEADER_FMT, self.sample_cnt, self.sample_rate, num)
        float_fmt = '!%uf' % num
        ia_bytes = struct.pack(float_fmt, *[v * self._SCALE for v in self.ia])
        ib_bytes = struct.pack(float_fmt, *[v * self._SCALE for v in self.ib])
        ic_bytes = struct.pack(float_fmt, *[v * self._SCALE for v in self.ic])
        return header + ia_bytes + ib_bytes + ic_bytes
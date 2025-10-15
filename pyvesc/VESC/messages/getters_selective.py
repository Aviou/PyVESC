# getters_selective.py
import struct
from ..protocol.base import VESCMessage

# Die IDs je nach Firmware; in vielen Forks:
# COMM_GET_VALUES_SELECTIVE = 50, COMM_GET_VALUES_SELECTIVE_REPLY = 51
# (Prüfe im Fork/deiner FW: datatypes.h / commands.c)
SELECTIVE_REQ_ID  = 50
SELECTIVE_RESP_ID = 51

class GetValuesSelective(VESCMessage, msg_id=SELECTIVE_REQ_ID):
    """
    Schickt eine 32-Bit Maske (Little Endian) zum VESC.
    Die Antwort enthält N Werte in der Reihenfolge der gesetzten Bits.
    """
    fields = [
        ('mask', 'I'),  # uint32 little-endian
    ]

    # Beispiel-Bitbelegung (bit-Nr. an deine FW anpassen!)
    BIT_V_IN             = 0
    BIT_RPM              = 1
    BIT_AVG_INPUT_CURR   = 2
    BIT_DUTY_NOW         = 3
    BIT_TEMP_MOS         = 4

    @staticmethod
    def build_mask(include_names):
        name_to_bit = {
            'v_in'             : GetValuesSelective.BIT_V_IN,
            'rpm'              : GetValuesSelective.BIT_RPM,
            'avg_input_current': GetValuesSelective.BIT_AVG_INPUT_CURR,
            'duty_now'         : GetValuesSelective.BIT_DUTY_NOW,
            'temp_mos'         : GetValuesSelective.BIT_TEMP_MOS,
        }
        mask = 0
        for n in include_names:
            b = name_to_bit.get(n)
            if b is not None:
                mask |= (1 << b)
        return mask

class ValuesSelectiveReply(VESCMessage, msg_id=SELECTIVE_RESP_ID):
    """
    Dynamisches Decoding: wir wissen nur, WAS wir abgefragt haben (Maske),
    und lesen die Felder in GENAU dieser Reihenfolge.
    """
    dynamic = True

    def decode(self, payload: bytes, **kwargs):
        # Minimal: v_in (float32), rpm (int32), avg_input_current (float32), duty_now (float32), temp_mos (float32)
        # Endianness je nach FW – oft < (Little Endian). Bei dir ggf. > nötig.
        res = {}
        ofs = 0
        try:
            res['v_in'], = struct.unpack_from('<f', payload, ofs); ofs += 4
            res['rpm'], = struct.unpack_from('<i', payload, ofs); ofs += 4
            res['avg_input_current'], = struct.unpack_from('<f', payload, ofs); ofs += 4
            res['duty_now'], = struct.unpack_from('<f', payload, ofs); ofs += 4
            res['temp_mos'], = struct.unpack_from('<f', payload, ofs); ofs += 4
        except struct.error:
            return None
        return res


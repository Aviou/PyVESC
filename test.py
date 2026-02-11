from unittest import TestCase


class TestPacket(TestCase):
    def exact_single_frame(self, length):
        """
        Simplest test possible. Create a packet, then parse a buffer containing this packet. Size of buffer is exactly
        one packet (no excess).
        :param length: Number of bytes in payload.
        """
        import random
        import pyvesc.protocol.packet.codec as vesc_packet
        correct_payload_index = None
        if length < 256:
            correct_payload_index = 2
        else:
            correct_payload_index = 3
        test_payload = bytes(random.getrandbits(8) for i in range(length))
        # test framing
        packet = vesc_packet.frame(test_payload)
        self.assertEqual(len(packet), correct_payload_index + length + 3, "size of packet")
        buffer = bytearray(packet)
        # test Parser
        parsed, consumed = vesc_packet.unframe(buffer)
        buffer = buffer[consumed:]
        self.assertEqual(parsed, test_payload)
        self.assertEqual(len(buffer), 0)

    def exact_two_frames(self, length1, length2):
        """
        Check that if there is more than one packet in a buffer, that the unpacker will properly unpack the packets.
        Size of buffer for this test is exactly two packets.
        :param length1: Length of first payload
        :param length2: Length of second payload
        """
        import random
        import pyvesc.protocol.packet.codec as vesc_packet
        correct_payload_index1 = None
        correct_payload_index2 = None
        if length1 < 256:
            correct_payload_index1 = 2
        else:
            correct_payload_index1 = 3
        if length2 < 256:
            correct_payload_index2 = 2
        else:
            correct_payload_index2 = 3
        test_payload1 = bytes(random.getrandbits(8) for i in range(length1))
        test_payload2 = bytes(random.getrandbits(8) for i in range(length2))
        # test framing
        packet1 = vesc_packet.frame(test_payload1)
        packet2 = vesc_packet.frame(test_payload2)
        self.assertEqual(len(packet1), correct_payload_index1 + length1 + 3, "size of packet")
        self.assertEqual(len(packet2), correct_payload_index2 + length2 + 3, "size of packet")
        buffer = bytearray(packet1 + packet2)
        # test Parser
        parsed, consumed = vesc_packet.unframe(buffer)
        buffer = buffer[consumed:]
        self.assertEqual(parsed, test_payload1)
        self.assertEqual(len(buffer), len(packet2))
        parsed, consumed = vesc_packet.unframe(buffer)
        buffer = buffer[consumed:]
        self.assertEqual(parsed, test_payload2)
        self.assertEqual(len(buffer), 0)

    def parse_buffer(self, length):
        import random
        import pyvesc.protocol.packet.codec as vesc_packet
        correct_payload_index = None
        if length < 256:
            correct_payload_index = 2
        else:
            correct_payload_index = 3
        test_payload = bytes(random.getrandbits(8) for i in range(length))
        packet = vesc_packet.frame(test_payload)

        # test on small buffers
        for n in range(0, 5):
            in_buffer = bytearray(packet[:n])
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, None)
            self.assertEqual(in_buffer, out_buffer)
        # test on buffer almost big enough
        for n in range(len(packet) - 4, len(packet)):
            in_buffer = bytearray(packet[:n])
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, None)
            self.assertEqual(in_buffer, out_buffer)
        # test on buffer slightly too big
        extension = b'\x02\x04\x07'
        extended_packet = packet + b'\x02\x04\x07'
        for n in range(len(packet) + 1, len(packet) + 4):
            in_buffer = bytearray(extended_packet[:n])
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, test_payload)
            self.assertEqual(out_buffer, extension[:n - len(packet)])

    def test_small_packets(self):
        for length in range(1, 5):
            self.exact_single_frame(length)
            self.exact_two_frames(length, length + 2)
            self.parse_buffer(length)

    def test_med_packets(self):
        for length in range(254, 258):
            self.exact_single_frame(length)
            self.exact_two_frames(length, length - 23)
            self.parse_buffer(length)

    def test_large_packets(self):
        for length in range(1022, 1024):
            self.exact_single_frame(length)
            self.exact_two_frames(length, length + 100)
            self.parse_buffer(length)

    def test_corrupt_detection(self):
        import pyvesc.protocol.packet.codec as vesc_packet
        # make a good packet
        test_payload = b'Te!'
        good_packet = b'\x02\x03Te!B\x92\x03'
        corrupt_packets = []
        # corrupt first byte
        corrupt = b'\x01\x03Te!B\x92\x03'
        corrupt_packets.append(corrupt)
        # corrupt payload_length (to be smaller and larger)
        smaller_corrupt = b'\x02\x02Te!B\x92\x03'
        larger_corrupt = b'\x02\x04Te!B\x92\x03\x03'
        corrupt_packets.append(smaller_corrupt)
        corrupt_packets.append(larger_corrupt)
        # corrupt first byte in payload
        corrupt = b'\x02\x03se!B\x92\x03'
        corrupt_packets.append(corrupt)
        # corrupt last byte in payload
        corrupt = b'\x02\x03TeyB\x92\x03'
        corrupt_packets.append(corrupt)
        # corrupt crc
        corrupt = b'\x02\x03Te!\xaa\x91\x03'
        corrupt_packets.append(corrupt)
        # corrupt terminator
        corrupt = b'\x02\x03Te!B\x92\x09'
        corrupt_packets.append(corrupt)
        # check that exceptions are given on each corrupt packet
        for corrupt in corrupt_packets:
            in_buffer = bytearray(corrupt)
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, None)
            self.assertTrue(consumed > 0)   # if a packet is corrupt then at least something should be consumed
            # get correct out_cuffer (in all of these cases it is just consuming to the next valid start byte (no more no less)
            self.assertEqual(consumed, vesc_packet.Stateless._next_possible_packet_index(in_buffer))
        # check that the good packet is parsed
        in_buffer = bytearray(good_packet)
        parsed, consumed = vesc_packet.unframe(in_buffer)
        out_buffer = in_buffer[consumed:]
        self.assertEqual(parsed, test_payload)
        self.assertEqual(out_buffer, b'')

    def test_corrupt_recovery(self):
        import pyvesc.protocol.packet.codec as vesc_packet
        # make a good packet
        test_payload = b'Te!'
        good_packet = b'\x02\x03Te!B\x92\x03'
        packet_to_recover = b'\x02\x04!\xe1$ 8\xbb\x03' # goal is to recover this packet
        payload_to_recover = b'!\xe1$ '
        after_goal = b'\x05\x09\x01' # mimic another corrupt packet after
        corrupt_packets = []
        # corrupt first byte
        corrupt = b'\x01\x03Te!B\x92\x03'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # corrupt payload_length (to be smaller and larger)
        smaller_corrupt = b'\x02\x02Te!B\x92\x03'
        larger_corrupt = b'\x02\x04Te!B\x92\x03\x03'
        corrupt_packets.append(smaller_corrupt + packet_to_recover + after_goal)
        corrupt_packets.append(larger_corrupt + packet_to_recover + after_goal)
        # corrupt first byte in payload
        corrupt = b'\x02\x03se!B\x92\x03'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # corrupt last byte in payload
        corrupt = b'\x02\x03TeyB\x92\x03'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # corrupt crc
        corrupt = b'\x02\x03Te!\xaa\x91\x03'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # corrupt terminator
        corrupt = b'\x02\x03Te!B\x92\x09'
        corrupt_packets.append(corrupt + packet_to_recover + after_goal)
        # check that exceptions are given on each corrupt packet
        for corrupt in corrupt_packets:
            in_buffer = bytearray(corrupt)
            parsed, consumed = vesc_packet.unframe(in_buffer)
            out_buffer = in_buffer[consumed:]
            self.assertEqual(parsed, payload_to_recover)
            found_packet_start = corrupt.find(packet_to_recover)
            self.assertTrue(consumed == (found_packet_start + len(packet_to_recover)))
        # check that the good packet is parsed
        in_buffer = bytearray(good_packet)
        parsed, consumed = vesc_packet.unframe(in_buffer)
        out_buffer = in_buffer[consumed:]
        self.assertEqual(parsed, test_payload)
        self.assertEqual(out_buffer, b'')

class TestMsg(TestCase):
    def setUp(self):
        import copy
        from pyvesc.protocol.base import VESCMessage
        self._initial_registry = copy.deepcopy(VESCMessage._msg_registry)

    def tearDown(self):
        from pyvesc.protocol.base import VESCMessage
        VESCMessage._msg_registry = self._initial_registry
        self._initial_registry = None

    def verify_packing_and_unpacking(self, msg):
        from pyvesc.protocol.base import VESCMessage
        payload_bytestring = VESCMessage.pack(msg)
        parsed_msg = VESCMessage.unpack(payload_bytestring)
        self.assertEqual(parsed_msg.id, msg.id)
        for name in [names[0] for names in msg.fields]:
            self.assertEqual(getattr(parsed_msg, name), getattr(msg, name))

    def test_single_message(self):
        from pyvesc.protocol.base import VESCMessage

        class TestMsg1(metaclass=VESCMessage):
            id = 0x12
            fields = [
                ('f1', 'B'),
                ('f2', 'H'),
                ('f3', 'i'),
                ('f4', 'L'),
                ('f5', 'b'),
                ('f6', 'I'),
            ]

        test_message = TestMsg1(27, 25367, -1124192846, 2244862237, 17, 73262)
        self.verify_packing_and_unpacking(test_message)

    def test_multiple_messages(self):
        from pyvesc.protocol.base import VESCMessage

        class testMsg1(metaclass=VESCMessage):
            id = 0x45
            fields = [
                ('f1', 'B'),
                ('f2', 'H'),
                ('f3', 'i'),
                ('f4', 'L'),
                ('f5', 'b'),
                ('f6', 'I'),
            ]

        class testMsg2(metaclass=VESCMessage):
            id = 0x19
            fields = [
                ('f1', 'B'),
                ('f2', 'B'),
            ]

        class testMsg3(metaclass=VESCMessage):
            id = 0x11
            fields = [
                ('f1', 'i'),
                ('f2', 'i'),
            ]

        class testMsg4(metaclass=VESCMessage):
            id = 0x24
            fields = [
                ('f1', 'i'),
                ('f2', 's'),
                ('f3', 'i'),
                ('f4', 'B'),
                ('f5', 'i'),
            ]

        test_message1 = testMsg1(27, 25367, -1124192846, 2244862237, 17, 73262)
        test_message12 = testMsg1(82, 45132, 382136436, 27374, 18, 72134)
        test_message2 = testMsg2(27, 13)
        test_message22 = testMsg2(52, 19)
        test_message3 = testMsg3(-7841, 4611)
        test_message32 = testMsg3(-123, 4123)
        test_message4 = testMsg4(4531, 'hello world', 1421, 34, 14215)
        self.verify_packing_and_unpacking(test_message1)
        self.verify_packing_and_unpacking(test_message2)
        self.verify_packing_and_unpacking(test_message3)
        self.verify_packing_and_unpacking(test_message4)

    def test_errors(self):
        from pyvesc.protocol.base import VESCMessage

        # try to make two messages with the same ID
        class testMsg1(metaclass=VESCMessage):
            id = 0x01
            fields = [
                ('f1', 'H'),
                ('f2', 'H'),
            ]
        caught = False
        try:
            class testMsg2(metaclass=VESCMessage):
                id = 0x01
                fields = [
                    ('f1', 'B'),
                    ('f2', 'B'),
                ]
        except TypeError as e:
            caught = True
        self.assertTrue(caught)

        # check that message classes are final
        caught = False
        try:
            class testMsg4(testMsg1):
                id = 0x01
                fields = [
                    ('f1', 'B'),
                    ('f2', 'B'),
                ]
        except TypeError as e:
            caught = True
        self.assertTrue(caught)

        # check that no more than 1 string field is allowed
        caught = False
        try:
            class testMsg7(metaclass=VESCMessage):
                id = 0x02
                fields = [
                    ('f1', 's'),
                    ('f2', 's'),
                ]
        except TypeError as e:
            caught = True
        self.assertTrue(caught)

        # check that 's' is used instead of 'p'
        caught = False
        try:
            class testMsg8(metaclass=VESCMessage):
                id = 0x31
                fields = [
                    ('f1', 'p'),
                ]
        except TypeError as e:
            caught = True
        self.assertTrue(caught)

        # try to fill a message with the wrong number of arguments
        caught = False
        try:
            testmessage1 = testMsg1(2, 4, 5) # should be 2 args
        except AttributeError as e:
            caught = True
        self.assertTrue(caught)


class TestMcsaStreamData(TestCase):
    def test_single_sample_unpack(self):
        """Test decoding a MCSA packet with 1 sample."""
        import struct
        from pyvesc.protocol.base import VESCMessage
        from pyvesc.VESC.messages.getters import McsaStreamData

        sample_cnt = 42
        sample_rate = 30000.0
        num_samples = 1
        ia_val, ib_val, ic_val = 1.5, -0.75, 0.25  # Ampere

        # Build payload: cmd_id + header + IEEE float32 (actual Ampere values)
        payload = struct.pack('!B', McsaStreamData.id)
        payload += struct.pack('!IfB', sample_cnt, sample_rate, num_samples)
        payload += struct.pack('!f', ia_val)
        payload += struct.pack('!f', ib_val)
        payload += struct.pack('!f', ic_val)

        msg = VESCMessage.unpack(payload)
        self.assertEqual(msg.sample_cnt, sample_cnt)
        self.assertAlmostEqual(msg.sample_rate, sample_rate, places=0)
        self.assertEqual(msg.num_samples, 1)
        self.assertEqual(len(msg.ia), 1)
        self.assertAlmostEqual(msg.ia[0], ia_val, places=5)
        self.assertAlmostEqual(msg.ib[0], ib_val, places=5)
        self.assertAlmostEqual(msg.ic[0], ic_val, places=5)

    def test_multi_sample_unpack(self):
        """Test decoding a MCSA packet with 32 samples."""
        import struct
        from pyvesc.protocol.base import VESCMessage
        from pyvesc.VESC.messages.getters import McsaStreamData

        sample_cnt = 1024
        sample_rate = 20000.0
        num_samples = 32
        ia_vals = [float(i) * 0.1 for i in range(num_samples)]
        ib_vals = [float(i) * -0.05 for i in range(num_samples)]
        ic_vals = [float(i) * 0.02 for i in range(num_samples)]

        payload = struct.pack('!B', McsaStreamData.id)
        payload += struct.pack('!IfB', sample_cnt, sample_rate, num_samples)
        fmt = '!%uf' % num_samples
        payload += struct.pack(fmt, *ia_vals)
        payload += struct.pack(fmt, *ib_vals)
        payload += struct.pack(fmt, *ic_vals)

        msg = VESCMessage.unpack(payload)
        self.assertEqual(msg.sample_cnt, sample_cnt)
        self.assertEqual(msg.num_samples, 32)
        self.assertEqual(len(msg.ia), 32)
        self.assertEqual(len(msg.ib), 32)
        self.assertEqual(len(msg.ic), 32)
        for i in range(num_samples):
            self.assertAlmostEqual(msg.ia[i], ia_vals[i], places=5)
            self.assertAlmostEqual(msg.ib[i], ib_vals[i], places=5)
            self.assertAlmostEqual(msg.ic[i], ic_vals[i], places=5)

    def test_pack_roundtrip(self):
        """Test that packing and unpacking McsaStreamData produces the same data."""
        from pyvesc.protocol.base import VESCMessage
        from pyvesc.VESC.messages.getters import McsaStreamData

        msg = McsaStreamData.__new__(McsaStreamData)
        msg.can_id = None
        msg.sample_cnt = 100
        msg.sample_rate = 25000.0
        msg.num_samples = 3
        msg.ia = [1.0, 2.0, 3.0]
        msg.ib = [-1.0, -2.0, -3.0]
        msg.ic = [0.5, 0.6, 0.7]

        packed = VESCMessage.pack(msg)
        unpacked = VESCMessage.unpack(packed)

        self.assertEqual(unpacked.sample_cnt, msg.sample_cnt)
        self.assertEqual(unpacked.num_samples, msg.num_samples)
        for i in range(msg.num_samples):
            self.assertAlmostEqual(unpacked.ia[i], msg.ia[i], places=2)
            self.assertAlmostEqual(unpacked.ib[i], msg.ib[i], places=2)
            self.assertAlmostEqual(unpacked.ic[i], msg.ic[i], places=2)

    def test_encode_decode_full_packet(self):
        """Test full encode/decode cycle through the packet framing layer."""
        import pyvesc
        from pyvesc.VESC.messages.getters import McsaStreamData

        msg = McsaStreamData.__new__(McsaStreamData)
        msg.can_id = None
        msg.sample_cnt = 500
        msg.sample_rate = 30000.0
        msg.num_samples = 5
        msg.ia = [0.1, 0.2, 0.3, 0.4, 0.5]
        msg.ib = [-0.1, -0.2, -0.3, -0.4, -0.5]
        msg.ic = [0.01, 0.02, 0.03, 0.04, 0.05]

        packet = pyvesc.encode(msg)
        decoded, consumed = pyvesc.decode(packet)
        self.assertEqual(consumed, len(packet))
        self.assertEqual(decoded.sample_cnt, 500)
        self.assertEqual(decoded.num_samples, 5)
        self.assertAlmostEqual(decoded.ia[2], 0.3, places=2)

    def test_start_stop_encode(self):
        """Test that start/stop messages encode correctly."""
        import pyvesc
        from pyvesc.VESC.messages.setters import SetMcsaStreamStart, SetMcsaStreamStop

        start_pkt = pyvesc.encode(SetMcsaStreamStart())
        stop_pkt = pyvesc.encode(SetMcsaStreamStop())
        self.assertIsNotNone(start_pkt)
        self.assertIsNotNone(stop_pkt)
        self.assertNotEqual(start_pkt, stop_pkt)


class TestInterface(TestCase):
    def setUp(self):
        import copy
        from pyvesc.protocol.base import VESCMessage
        self._initial_registry = copy.deepcopy(VESCMessage._msg_registry)

    def tearDown(self):
        from pyvesc.protocol.base import VESCMessage
        VESCMessage._msg_registry = self._initial_registry
        self._initial_registry = None

    def verify_encode_decode(self, msg):
        import pyvesc
        encoded = pyvesc.encode(msg)
        decoded, consumed = pyvesc.decode(encoded)
        self.assertEqual(consumed, len(encoded))
        for field in msg._field_names:
            self.assertEqual(getattr(msg, field), getattr(decoded, field))

    def test_interface(self):
        from pyvesc.VESCMotor.messages import VESCMessage

        class testMsg1(metaclass=VESCMessage):
            id = 0x45
            fields = [
                ('f1', 'B'),
                ('f2', 'H'),
                ('f3', 'i'),
                ('f4', 'L'),
                ('f5', 'b'),
                ('f6', 'I'),
            ]

        class testMsg2(metaclass=VESCMessage):
            id = 0x19
            fields = [
                ('f1', 'B'),
                ('f2', 'B'),
            ]

        class testMsg3(metaclass=VESCMessage):
            id = 0x11
            fields = [
                ('f1', 'i'),
                ('f2', 'i'),
            ]

        class testMsg4(metaclass=VESCMessage):
            id = 0x24
            fields = [
                ('f1', 'i'),
                ('f2', 's'),
                ('f3', 'i'),
                ('f4', 'B'),
                ('f5', 'i'),
            ]
        test_message1 = testMsg1(27, 25367, -1124192846, 2244862237, 17, 73262)
        test_message12 = testMsg1(82, 45132, 382136436, 27374, 18, 72134)
        test_message2 = testMsg2(27, 13)
        test_message22 = testMsg2(52, 19)
        test_message3 = testMsg3(-7841, 4611)
        test_message32 = testMsg3(-123, 4123)
        test_message4 = testMsg4(4531, 'hello world', 1421, 34, 14215)
        self.verify_encode_decode(test_message1)
        self.verify_encode_decode(test_message2)
        self.verify_encode_decode(test_message3)
        self.verify_encode_decode(test_message4)

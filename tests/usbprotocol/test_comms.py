#!/usr/bin/env python3
import random
import unittest

from opendbc.car.structs import CarParams
from panda import CANPACKET_HEAD_SIZE, DLC_TO_LEN, USBPACKET_MAX_SIZE, pack_can_buffer, unpack_can_buffer
from panda.tests.libpanda import libpanda_py

lpp = libpanda_py.libpanda

CHUNK_SIZE = USBPACKET_MAX_SIZE
TX_QUEUES = (lpp.tx1_q, lpp.tx2_q, lpp.tx3_q)


def unpackage_can_msg(pkt):
  dat_len = DLC_TO_LEN[pkt[0].data_len_code]
  dat = bytes(pkt[0].data[0:dat_len])
  return pkt[0].addr, dat, pkt[0].bus


def random_can_messages(n, bus=None):
  msgs = []
  for _ in range(n):
    if bus is None:
      bus = random.randint(0, 3)
    address = random.randint(1, (1 << 29) - 1)
    data = bytes([random.getrandbits(8) for _ in range(DLC_TO_LEN[random.randrange(0, len(DLC_TO_LEN))])])
    msgs.append((address, data, bus))
  return msgs


class TestPandaComms(unittest.TestCase):
  def setUp(self):
    lpp.set_safety_hooks(CarParams.SafetyModel.allOutput, 0)
    lpp.comms_can_reset()

  def test_tx_queues(self):
    for bus in range(len(TX_QUEUES)):
      message = (0x100, b"test", bus)

      can_pkt_tx = libpanda_py.make_CANPacket(message[0], message[2], message[1])
      can_pkt_rx = libpanda_py.ffi.new('CANPacket_t *')

      assert lpp.can_push(TX_QUEUES[bus], can_pkt_tx), "CAN push failed"
      assert lpp.can_pop(TX_QUEUES[bus], can_pkt_rx), "CAN pop failed"

      assert unpackage_can_msg(can_pkt_rx) == message

  def test_comms_reset_rx(self):
    # store some test messages in the queue
    test_msg = (0x100, b"test", 0)
    for _ in range(100):
      can_pkt_tx = libpanda_py.make_CANPacket(test_msg[0], test_msg[2], test_msg[1])
      lpp.can_push(lpp.rx_q, can_pkt_tx)

    # read a small chunk such that we have some overflow
    TINY_CHUNK_SIZE = 6
    dat = libpanda_py.ffi.new(f"uint8_t[{TINY_CHUNK_SIZE}]")
    rx_len = lpp.comms_can_read(dat, TINY_CHUNK_SIZE)
    assert rx_len == TINY_CHUNK_SIZE, "comms_can_read returned too little data"

    _, overflow = unpack_can_buffer(bytes(dat))
    assert len(overflow) > 0, "overflow buffer should not be empty"

    # reset the comms to clear the overflow buffer on the panda side
    lpp.comms_can_reset()

    # read a large chunk, which should now contain valid messages
    LARGE_CHUNK_SIZE = 512
    dat = libpanda_py.ffi.new(f"uint8_t[{LARGE_CHUNK_SIZE}]")
    rx_len = lpp.comms_can_read(dat, LARGE_CHUNK_SIZE)
    assert rx_len == LARGE_CHUNK_SIZE, "comms_can_read returned too little data"

    msgs, _ = unpack_can_buffer(bytes(dat))
    assert len(msgs) > 0, "message buffer should not be empty"
    for m in msgs:
      assert m == test_msg, "message buffer should contain valid test messages"

  def test_comms_reset_tx(self):
    # store some test messages in the queue
    test_msg = (0x100, b"test", 0)
    packed = pack_can_buffer([test_msg for _ in range(100)], chunk=True)

    # write a small chunk such that we have some overflow
    TINY_CHUNK_SIZE = 6
    lpp.comms_can_write(bytes(packed[0][:TINY_CHUNK_SIZE]), TINY_CHUNK_SIZE)

    # reset the comms to clear the overflow buffer on the panda side
    lpp.comms_can_reset()

    # write a full valid chunk, which should now contain valid messages
    lpp.comms_can_write(bytes(packed[1]), len(packed[1]))

    # read the messages from the queue and make sure they're valid
    queue_msgs = []
    pkt = libpanda_py.ffi.new('CANPacket_t *')
    while lpp.can_pop(TX_QUEUES[0], pkt):
      queue_msgs.append(unpackage_can_msg(pkt))

    assert len(queue_msgs) > 0, "message buffer should not be empty"
    for m in queue_msgs:
      assert m == test_msg, "message buffer should contain valid test messages"


  def test_comms_write_sanitizes_queue_private_flags(self):
    message = (0x123, b"format", 0)
    pkt = libpanda_py.make_CANPacket(message[0], message[2], message[1])
    pkt[0].returned = 1
    pkt[0].rejected = 1
    lpp.can_set_checksum(pkt)
    raw = bytes(libpanda_py.ffi.buffer(pkt, 6 + len(message[1])))

    # Exercise both parsing sites in comms_can_write: complete and split packets.
    for first_chunk in (len(raw), 4):
      with self.subTest(first_chunk=first_chunk):
        lpp.comms_can_reset()
        lpp.comms_can_write(raw[:first_chunk], first_chunk)
        if first_chunk < len(raw):
          lpp.comms_can_write(raw[first_chunk:], len(raw) - first_chunk)

        queued = libpanda_py.ffi.new('CANPacket_t *')
        self.assertTrue(lpp.can_pop(TX_QUEUES[0], queued))
        self.assertEqual(unpackage_can_msg(queued), message)
        self.assertEqual(queued[0].returned, 0)
        self.assertEqual(queued[0].rejected, 0)
        checksum = 0
        for b in bytes(libpanda_py.ffi.buffer(queued, 6 + len(message[1]))):
          checksum ^= b
        self.assertEqual(checksum, 0)

  def test_comms_write_rejects_bad_checksum_and_recovers(self):
    try:
      for flipped in (False, True):
        lpp.can_set_orientation(flipped)
        for bus in range(3):
          can_number = 2 - bus if flipped else bus
          for data_len in (8, 32):
            message = (0x123, bytes(range(data_len)), bus)
            pkt = libpanda_py.make_CANPacket(message[0], message[2], message[1])
            pkt[0].fd = data_len > 8
            for private_flags in (False, True):
              pkt[0].returned = private_flags
              pkt[0].rejected = private_flags
              lpp.can_set_checksum(pkt)
              raw = bytes(libpanda_py.ffi.buffer(pkt, CANPACKET_HEAD_SIZE + data_len))
              for corrupt_index in (CANPACKET_HEAD_SIZE - 1, len(raw) - 1):
                bad = bytearray(raw)
                bad[corrupt_index] ^= 1
                for first_chunk in (len(raw), 4, CANPACKET_HEAD_SIZE, len(raw) - 1):
                  with self.subTest(flipped=flipped, bus=bus, data_len=data_len, private_flags=private_flags,
                                    corrupt_index=corrupt_index, first_chunk=first_chunk):
                    lpp.comms_can_reset()
                    before = [lpp.get_can_tx_checksum_error_cnt(i) for i in range(3)]
                    lpp.comms_can_write(bytes(bad[:first_chunk]), first_chunk)
                    # A valid packet immediately following bad input must still be handled.
                    remaining = bytes(bad[first_chunk:]) + raw
                    lpp.comms_can_write(remaining, len(remaining))

                    queued = libpanda_py.ffi.new('CANPacket_t *')
                    self.assertTrue(lpp.can_pop(TX_QUEUES[bus], queued))
                    self.assertEqual(unpackage_can_msg(queued), message)
                    self.assertEqual(queued[0].fd, data_len > 8)
                    self.assertEqual(queued[0].returned, 0)
                    self.assertEqual(queued[0].rejected, 0)
                    self.assertFalse(lpp.can_pop(TX_QUEUES[bus], queued))
                    after = [lpp.get_can_tx_checksum_error_cnt(i) for i in range(3)]
                    self.assertEqual([a - b for a, b in zip(after, before, strict=True)],
                                     [int(i == can_number) for i in range(3)])
    finally:
      lpp.can_set_orientation(False)

  def test_comms_write_bad_checksum_invalid_bus(self):
    for bus in (3, 7):
      with self.subTest(bus=bus):
        lpp.comms_can_reset()
        pkt = libpanda_py.make_CANPacket(0x123, bus, b"invalid")
        pkt[0].checksum ^= 1
        raw = bytes(libpanda_py.ffi.buffer(pkt, CANPACKET_HEAD_SIZE + 7))
        before = [lpp.get_can_tx_checksum_error_cnt(i) for i in range(3)]
        lpp.comms_can_write(raw, len(raw))
        self.assertEqual([lpp.get_can_tx_checksum_error_cnt(i) for i in range(3)], before)
        queued = libpanda_py.ffi.new('CANPacket_t *')
        for queue in TX_QUEUES:
          self.assertFalse(lpp.can_pop(queue, queued))

  def test_can_send_usb(self):
    for bus in range(3):
      with self.subTest(bus=bus):
        for _ in range(100):
          msgs = random_can_messages(200, bus=bus)
          packed = pack_can_buffer(msgs)

          # Simulate USB bulk chunks
          for buf in packed:
            for i in range(0, len(buf), CHUNK_SIZE):
              chunk_len = min(CHUNK_SIZE, len(buf) - i)
              lpp.comms_can_write(bytes(buf[i:i+chunk_len]), chunk_len)

          # Check that they ended up in the right buffers
          queue_msgs = []
          pkt = libpanda_py.ffi.new('CANPacket_t *')
          while lpp.can_pop(TX_QUEUES[bus], pkt):
            queue_msgs.append(unpackage_can_msg(pkt))

          self.assertEqual(len(queue_msgs), len(msgs))
          self.assertEqual(queue_msgs, msgs)

  def test_can_receive_usb(self):
    msgs = random_can_messages(50000)
    packets = [libpanda_py.make_CANPacket(m[0], m[2], m[1]) for m in msgs]

    rx_msgs = []
    overflow_buf = b""
    while len(packets) > 0:
      # Push into queue
      while lpp.can_slots_empty(lpp.rx_q) > 0 and len(packets) > 0:
        lpp.can_push(lpp.rx_q, packets.pop(0))

      # Simulate USB bulk IN chunks
      MAX_TRANSFER_SIZE = 16384
      dat = libpanda_py.ffi.new(f"uint8_t[{CHUNK_SIZE}]")
      while True:
        buf = b""
        while len(buf) < MAX_TRANSFER_SIZE:
          max_size = min(CHUNK_SIZE, MAX_TRANSFER_SIZE - len(buf))
          rx_len = lpp.comms_can_read(dat, max_size)
          buf += bytes(dat[0:rx_len])
          if rx_len < max_size:
            break

        if len(buf) == 0:
          break
        unpacked_msgs, overflow_buf = unpack_can_buffer(overflow_buf + buf)
        rx_msgs.extend(unpacked_msgs)

    self.assertEqual(len(rx_msgs), len(msgs))
    self.assertEqual(rx_msgs, msgs)


if __name__ == "__main__":
  unittest.main()

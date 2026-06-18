import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = (ROOT / "Core" / "Inc" / "ble_comm.h").read_text(encoding="utf-8")
SOURCE = (ROOT / "Core" / "Src" / "ble_comm.c").read_text(encoding="utf-8")


def macro_body(name: str) -> str:
    match = re.search(rf"^#define\s+{name}\s+(.+)$", HEADER, re.MULTILINE)
    if match is None:
        raise AssertionError(f"missing macro {name}")
    return match.group(1).strip()


class BleCommProtocolTest(unittest.TestCase):
    def assertMacro(self, name: str, expected: str) -> None:
        self.assertEqual(macro_body(name), expected, name)

    def test_single_frame_layout_macros(self) -> None:
        self.assertMacro("BLE_COMM_SINGLE_FRAME_SIZE", "99U")
        self.assertMacro("BLE_COMM_IDX_ADC_START", "2U")
        self.assertMacro("BLE_COMM_ADC_VALUES_PER_CHANNEL", "6U")
        self.assertMacro("BLE_COMM_IDX_ADC_END", "73U")
        self.assertMacro("BLE_COMM_IDX_PPG_START", "74U")
        self.assertMacro("BLE_COMM_IDX_PPG_END", "82U")
        self.assertMacro("BLE_COMM_IDX_IMU_START", "83U")
        self.assertMacro("BLE_COMM_IDX_IMU_END", "94U")
        self.assertMacro("BLE_COMM_IDX_CHECKSUM", "95U")
        self.assertMacro("BLE_COMM_IDX_FRAME_SEQ_L", "96U")
        self.assertMacro("BLE_COMM_IDX_FRAME_SEQ_H", "97U")
        self.assertMacro("BLE_COMM_IDX_TAIL0", "98U")

    def test_batch_size_uses_frame_size_macro(self) -> None:
        body = macro_body("BLE_COMM_BATCH_TX_SIZE")
        self.assertIn("BLE_COMM_SINGLE_FRAME_SIZE", body)
        self.assertIn("BLE_COMM_BATCH_FRAME_COUNT", body)
        self.assertNotIn("490", body)

    def test_checksum_keeps_legacy_payload_only(self) -> None:
        self.assertMacro("BLE_COMM_XOR_START_IDX", "BLE_COMM_IDX_ADC_START")
        self.assertMacro("BLE_COMM_XOR_END_IDX", "BLE_COMM_IDX_IMU_END")

    def test_adc_packet_uses_six_raw_slots_per_channel(self) -> None:
        self.assertIn("slot_code[slot]", SOURCE)
        self.assertNotIn("early_code", SOURCE)
        self.assertNotIn("late_code", SOURCE)

    def test_frame_seq_is_static_uint16_and_written_little_endian(self) -> None:
        self.assertRegex(SOURCE, r"static\s+uint16_t\s+s_frame_seq\s*=\s*0U\s*;")
        self.assertRegex(
            SOURCE,
            r"out_buffer\s*\[\s*BLE_COMM_IDX_FRAME_SEQ_L\s*\]\s*="
            r"\s*\(uint8_t\)\s*\(\s*s_frame_seq\s*&\s*0xFFU\s*\)\s*;",
        )
        self.assertRegex(
            SOURCE,
            r"out_buffer\s*\[\s*BLE_COMM_IDX_FRAME_SEQ_H\s*\]\s*="
            r"\s*\(uint8_t\)\s*\(\s*\(\s*s_frame_seq\s*>>\s*8\s*\)\s*&\s*0xFFU\s*\)\s*;",
        )
        self.assertRegex(SOURCE, r"s_frame_seq\s*\+\+\s*;")

    def test_frame_seq_is_between_checksum_and_tail(self) -> None:
        checksum_pos = SOURCE.index("out_buffer[BLE_COMM_IDX_CHECKSUM]")
        seq_l_pos = SOURCE.index("out_buffer[BLE_COMM_IDX_FRAME_SEQ_L]")
        seq_h_pos = SOURCE.index("out_buffer[BLE_COMM_IDX_FRAME_SEQ_H]")
        tail_pos = SOURCE.index("out_buffer[BLE_COMM_IDX_TAIL0]")

        self.assertLess(checksum_pos, seq_l_pos)
        self.assertLess(seq_l_pos, seq_h_pos)
        self.assertLess(seq_h_pos, tail_pos)


if __name__ == "__main__":
    unittest.main()

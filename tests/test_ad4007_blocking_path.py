import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER_SOURCE = (ROOT / "Core" / "Inc" / "AD4007.h").read_text(encoding="utf-8")
DRIVER_SOURCE = (ROOT / "Core" / "Src" / "AD4007.c").read_text(encoding="utf-8")
MAIN_SOURCE = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
FRAME_SOURCE = (ROOT / "Core" / "Inc" / "sensor_ringbuffer.h").read_text(encoding="utf-8")


def function_body(source: str, signature: str, next_signature: str) -> str:
    match = re.search(
        rf"{re.escape(signature)}.*?(?={re.escape(next_signature)})",
        source,
        flags=re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"missing function block: {signature}")
    return match.group(0)


class Ad4007BlockingPathTest(unittest.TestCase):
    def test_public_api_exposes_five_microsecond_ll_read_and_stats(self) -> None:
        self.assertIn("#define AD4007_BLOCKING_TIMEOUT_US", HEADER_SOURCE)
        self.assertRegex(HEADER_SOURCE, r"AD4007_BLOCKING_TIMEOUT_US\s+\(5u\)")
        self.assertIn("AD4007_ReadBlocking_LL", HEADER_SOURCE)
        self.assertIn("AD4007_AverageValidSlots", HEADER_SOURCE)
        self.assertIn("AD4007_RuntimeStats_t", HEADER_SOURCE)
        self.assertIn("AD4007_GetRuntimeStats", HEADER_SOURCE)

    def test_driver_uses_ll_flags_and_dwt_cycle_deadline(self) -> None:
        self.assertIn('#include "stm32g4xx_ll_spi.h"', DRIVER_SOURCE)
        self.assertIn("DWT->CYCCNT", DRIVER_SOURCE)
        self.assertIn("LL_SPI_IsActiveFlag_TXE", DRIVER_SOURCE)
        self.assertIn("LL_SPI_IsActiveFlag_RXNE", DRIVER_SOURCE)
        self.assertIn("LL_SPI_IsActiveFlag_BSY", DRIVER_SOURCE)
        self.assertIn("LL_SPI_TransmitData8", DRIVER_SOURCE)
        self.assertIn("LL_SPI_ReceiveData8", DRIVER_SOURCE)

        body = function_body(
            DRIVER_SOURCE,
            "HAL_StatusTypeDef AD4007_ReadBlocking_LL",
            "HAL_StatusTypeDef AD4007_AverageValidSlots",
        )
        self.assertLess(
            body.index("start_cycles = DWT->CYCCNT"),
            body.index("LL_SPI_SetRxFIFOThreshold"),
        )

    def test_original_dma_api_is_retained_but_main_no_longer_uses_it(self) -> None:
        self.assertIn("AD4007_Start_DMA_Rx", HEADER_SOURCE)
        self.assertIn("AD4007_Start_DMA_Rx", DRIVER_SOURCE)
        self.assertNotIn("AD4007_Start_DMA_Rx", MAIN_SOURCE)
        self.assertNotIn("g_adc_dma_pending", MAIN_SOURCE)
        self.assertNotIn("g_adc_dma_raw", MAIN_SOURCE)
        self.assertNotIn("ADC_TryHarvestPendingSample", MAIN_SOURCE)

    def test_falling_edge_path_records_validity_and_always_advances_slot(self) -> None:
        body = function_body(
            MAIN_SOURCE,
            "void ADC_OnFallingEdgeTrigger",
            "/* USER CODE END 4 */",
        )
        self.assertIn("AD4007_ReadBlocking_LL", body)
        self.assertIn("slot_valid_mask", body)
        self.assertRegex(body, r"slot_code\s*\[\s*slot\s*\]\s*=\s*0")
        self.assertEqual(body.count("g_adc_pulse_start_index++"), 1)

    def test_frame_tracks_slot_validity_without_changing_raw_slot_count(self) -> None:
        self.assertRegex(FRAME_SOURCE, r"int32_t\s+slot_code\s*\[\s*6\s*\]")
        self.assertRegex(FRAME_SOURCE, r"uint8_t\s+slot_valid_mask")

    def test_valid_average_has_explicit_empty_range_failure(self) -> None:
        body = function_body(
            DRIVER_SOURCE,
            "HAL_StatusTypeDef AD4007_AverageValidSlots",
            "HAL_StatusTypeDef AD4007_Start_DMA_Rx",
        )
        self.assertIn("valid_mask", body)
        self.assertIn("valid_count", body)
        self.assertRegex(body, r"valid_count\s*==\s*0u")
        self.assertRegex(body, r"\*out_avg_code\s*=\s*0")
        self.assertIn("return HAL_ERROR", body)


if __name__ == "__main__":
    unittest.main()

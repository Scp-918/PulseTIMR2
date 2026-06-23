import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPI_SOURCE = (ROOT / "Core" / "Src" / "spi.c").read_text(encoding="utf-8")
IOC_SOURCE = (ROOT / "PulseDrive.ioc").read_text(encoding="utf-8")


def active_c_source(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//.*", "", source)


def source_between(source: str, start: str, end: str) -> str:
    start_index = source.index(start)
    end_index = source.index(end, start_index)
    return source[start_index:end_index]


class Spi3SpeedTest(unittest.TestCase):
    def test_runtime_spi3_clock_is_fifty_megahertz(self) -> None:
        source = active_c_source(SPI_SOURCE)
        init = source_between(source, "void MX_SPI3_Init", "void HAL_SPI_MspInit")

        self.assertIn(
            "hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;",
            init,
        )
        self.assertNotIn("SPI_BAUDRATEPRESCALER_4", init)

        apb1_match = re.search(r"^RCC\.APB1Freq_Value=(\d+)$", IOC_SOURCE, re.MULTILINE)
        self.assertIsNotNone(apb1_match)
        self.assertEqual(int(apb1_match.group(1)) // 2, 50_000_000)

    def test_spi3_gpio_uses_very_high_speed(self) -> None:
        msp = source_between(
            SPI_SOURCE,
            "else if(spiHandle->Instance==SPI3)",
            "void HAL_SPI_MspDeInit",
        )
        self.assertIn("GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;", msp)

    def test_ioc_matches_fifty_megahertz_spi3_configuration(self) -> None:
        self.assertIn(
            "SPI3.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_2",
            IOC_SOURCE,
        )
        self.assertIn("SPI3.CalculateBaudRate=50.0 MBits/s", IOC_SOURCE)

        for pin in ("PC10", "PC11", "PC12"):
            self.assertIn(f"{pin}.GPIOParameters=GPIO_Speed", IOC_SOURCE)
            self.assertIn(
                f"{pin}.GPIO_Speed=GPIO_SPEED_FREQ_VERY_HIGH",
                IOC_SOURCE,
            )


if __name__ == "__main__":
    unittest.main()

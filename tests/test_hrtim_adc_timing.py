import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HRTIM_SOURCE = (ROOT / "Core" / "Src" / "hrtim.c").read_text(encoding="utf-8")
MAIN_SOURCE = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
IOC_SOURCE = (ROOT / "PulseDrive.ioc").read_text(encoding="utf-8")


def active_c_source(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//.*", "", source)


def generated_timer_a_values() -> dict[str, int]:
    source = active_c_source(HRTIM_SOURCE)
    periods = [int(value) for value in re.findall(r"pTimeBaseCfg\.Period\s*=\s*(\d+);", source)]
    compares = [int(value) for value in re.findall(r"pCompareCfg\.CompareValue\s*=\s*(\d+);", source)]
    return {
        "period": periods[1],
        "cmp1": compares[-4],
        "cmp2": compares[-3],
        "cmp3": compares[-2],
        "cmp4": compares[-1],
    }


def generated_master_values() -> dict[str, int]:
    source = active_c_source(HRTIM_SOURCE)
    periods = [int(value) for value in re.findall(r"pTimeBaseCfg\.Period\s*=\s*(\d+);", source)]
    compares = [int(value) for value in re.findall(r"pCompareCfg\.CompareValue\s*=\s*(\d+);", source)]
    return {
        "period": periods[0],
        "cmp1": compares[0],
        "cmp2": compares[1],
        "cmp3": compares[2],
        "cmp4": compares[3],
    }


def ioc_value(name: str) -> int:
    match = re.search(rf"^{re.escape(name)}=(\d+)$", IOC_SOURCE, re.MULTILINE)
    if match is None:
        raise AssertionError(f"missing .ioc setting: {name}")
    return int(match.group(1))


class HrtimAdcTimingTest(unittest.TestCase):
    def test_timer_a_rst2_interrupt_is_disabled(self) -> None:
        hrtim_source = active_c_source(HRTIM_SOURCE)
        timer_interrupt_requests = re.findall(
            r"pTimerCfg\.InterruptRequests\s*=\s*([^;]+);",
            hrtim_source,
        )
        self.assertGreaterEqual(len(timer_interrupt_requests), 2)
        self.assertEqual(timer_interrupt_requests[1].strip(), "HRTIM_TIM_IT_NONE")

        main_source = active_c_source(MAIN_SOURCE)
        enable_match = re.search(
            r"__HAL_HRTIM_TIMER_ENABLE_IT\(\s*&hhrtim1\s*,\s*"
            r"HRTIM_TIMERINDEX_TIMER_A\s*,(.*?)\);",
            main_source,
            re.DOTALL,
        )
        self.assertIsNotNone(enable_match)
        enabled_interrupts = enable_match.group(1)
        for interrupt in ("HRTIM_TIM_IT_CMP1", "HRTIM_TIM_IT_CMP3", "HRTIM_TIM_IT_REP"):
            self.assertIn(interrupt, enabled_interrupts)
        self.assertNotIn("HRTIM_TIM_IT_RST2", enabled_interrupts)

        self.assertEqual(ioc_value("HRTIM1.NumberInterruptRequests-Output_TA1TA2"), 0)
        self.assertNotRegex(
            IOC_SOURCE,
            r"^HRTIM1\.InterruptRequests\d+-Output_TA1TA2=HRTIM_TIM_IT_RST2$",
        )

    def test_cnv_is_one_point_five_microseconds_high_and_fourteen_microseconds_low(self) -> None:
        values = generated_timer_a_values()
        self.assertEqual(
            values,
            {"period": 3250, "cmp1": 150, "cmp2": 1550, "cmp3": 1700, "cmp4": 3100},
        )

        set_ticks = [0, values["cmp2"], values["cmp4"]]
        reset_ticks = [values["cmp1"], values["cmp3"], values["period"]]
        self.assertEqual([reset - set_ for set_, reset in zip(set_ticks, reset_ticks)], [150] * 3)
        self.assertEqual(
            [set_ticks[i + 1] - reset_ticks[i] for i in range(2)],
            [1400] * 2,
        )

    def test_late_burst_finishes_before_master_compare4(self) -> None:
        master = generated_master_values()
        timer_a = generated_timer_a_values()
        self.assertEqual(master["cmp3"], 28500)

        late_last_falling_edge_ns = master["cmp3"] * 40 + timer_a["period"] * 10
        master_compare4_ns = master["cmp4"] * 40
        self.assertGreaterEqual(master_compare4_ns - late_last_falling_edge_ns, 10_000)

    def test_ioc_matches_generated_timer_a_timing(self) -> None:
        values = generated_timer_a_values()
        self.assertEqual(
            ioc_value("HRTIM1.CompareValue3-MasterTimer"),
            generated_master_values()["cmp3"],
        )
        self.assertEqual(ioc_value("HRTIM1.Periode_TA"), values["period"])
        for index in range(1, 5):
            self.assertEqual(
                ioc_value(f"HRTIM1.CompareValue{index}-Output_TA1TA2"),
                values[f"cmp{index}"],
            )


if __name__ == "__main__":
    unittest.main()

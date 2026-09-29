from __future__ import annotations

import dataclasses
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pipeline_overrides as po  # noqa: E402


@dataclasses.dataclass
class Config:
    gate: float = 0.2
    fit: bool = True


class OverridesTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp())

    def write(self, data) -> Path:
        p = self.tmp / "o.json"
        p.write_text(json.dumps(data))
        return p

    def test_no_file_is_no_overrides(self) -> None:
        self.assertEqual(po.load(None), {})

    def test_unknown_component_is_refused(self) -> None:
        with self.assertRaisesRegex(po.OverrideError, "unknown component 'slam'"):
            po.load(self.write({"slam": {"x": 1}}))

    def test_benchmark_refuses_sections_it_would_ignore(self) -> None:
        o = po.load(self.write({"slam_node": {"motion_model": "imu"}}))
        with self.assertRaisesRegex(po.OverrideError, "does not use slam_node"):
            po.only(o, "cone_detection")
        po.only(o, "slam_node", "cone_detection")

    def test_dataclass_fields_are_replaced_and_typos_fail(self) -> None:
        self.assertEqual(
            po.apply_dataclass(Config(), {"gate": 0.05}, "c"), Config(gate=0.05)
        )
        with self.assertRaisesRegex(po.OverrideError, "unknown parameter.*gaet"):
            po.apply_dataclass(Config(), {"gaet": 0.05}, "c")

    def test_effective_values_record_what_was_overridden(self) -> None:
        f = po.write_effective(
            self.tmp, "c", dataclasses.asdict(Config(gate=0.05)), {"gate": 0.05}
        )
        self.assertEqual(
            json.loads(f.read_text()),
            {"values": {"fit": True, "gate": 0.05}, "overridden": ["gate"]},
        )


if __name__ == "__main__":
    unittest.main()

"""Offline unit tests for the package checker; no third-party imports needed."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location(
    "gk_cbct_environment_check", HERE/"check_dicom_environment.py"
)
checker=importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(checker)


class DicomsSetupTests(unittest.TestCase):
    def test_requirements_cover_every_probed_package(self):
        lines=(HERE/"requirements.txt").read_text(encoding="utf-8").splitlines()
        packages={line.strip().split(">")[0].split("=")[0].lower()
                  for line in lines if line.strip() and not line.startswith("#")}
        self.assertEqual(packages,{name.lower() for name in checker.DEPENDENCIES.values()})

    def test_missing_package_names_are_actionable(self):
        with tempfile.TemporaryDirectory() as temp:
            detector=Path(temp)/"cbct_detect.py"
            detector.write_text("print('stub')",encoding="utf-8")
            def fake_import(name):
                if name=="torchio":
                    raise ImportError("test simulated missing dependency")
            with patch.object(checker.importlib,"import_module",side_effect=fake_import),\
                 patch.object(checker.importlib.metadata,"version",return_value="1.0"),\
                 patch.object(checker.subprocess,"run",return_value=SimpleNamespace(
                    returncode=0,stdout="--detector-variant",stderr=""
                 )):
                ok,message=checker.diagnose(detector)
            self.assertFalse(ok)
            self.assertIn("FAIL: torchio:",message)
            self.assertIn("NOT ready",message)

    def test_missing_detector_is_not_reported_as_ready(self):
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(checker.importlib,"import_module",return_value=None),\
                 patch.object(checker.importlib.metadata,"version",return_value="1.0"):
                ok,message=checker.diagnose(Path(temp)/"absent.py")
            self.assertFalse(ok)
            self.assertIn("missing bundled detector",message)

    def test_good_synthetic_dependencies_reports_ready(self):
        with tempfile.TemporaryDirectory() as temp:
            detector=Path(temp)/"cbct_detect.py"
            detector.write_text("print('stub')",encoding="utf-8")
            with patch.object(checker.importlib,"import_module",return_value=None),\
                 patch.object(checker.importlib.metadata,"version",return_value="1.0"),\
                 patch.object(checker.subprocess,"run",return_value=SimpleNamespace(
                    returncode=0,stdout="--detector-variant",stderr=""
                 )):
                ok,message=checker.diagnose(detector)
            self.assertTrue(ok)
            self.assertIn("ready for Original/Optimized",message)


if __name__=="__main__":
    unittest.main()

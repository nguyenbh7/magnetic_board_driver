"""Synthetic offline vendoring test: Hybrid imports without any source notebook."""
import ast
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest

import vendor_hybrid_board_detector as vendor


class VendorHybridDetectorTests(unittest.TestCase):
    def test_extraction_missing_methods_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"incomplete.ipynb"
            path.write_text(json.dumps({"nbformat":4,"cells":[{"cell_type":"code","source":"def _reslice(): pass\n"}]}))
            with self.assertRaisesRegex(ValueError,"Missing source functions"):
                vendor.extract_functions(path)

    def test_vendor_source_is_python_and_no_notebook_at_runtime(self):
        # Tiny synthetics use the same five names and signature as the
        # reviewed detector. They are not scientific detector geometry.
        source='''def _reslice(*args, t_step=None, cval=None, **kwargs):
    return ("special", t_step, cval)

def _side_level():
    return 1

def _contrast_cut():
    return 2

def _footprint():
    return 3

def _measure_board(data, spacing, affine, pts, grid, board_size,
                   max_thickness, min_rect, slab_mm, pitch_mm,
                   min_contrast, dense_thr):
    first = _reslice(data)
    second = _reslice(data, t_step=0.5, cval=float("nan"))
    return {"main": first, "side": second}, None
'''
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            notebook=root/"source.ipynb"
            notebook.write_text(json.dumps({
                "nbformat":4,
                "cells":[{"cell_type":"code","source":source.splitlines(keepends=True)}]
            }), encoding="utf-8")
            output=root/"hybrid_board_detector.py"
            self.assertTrue(vendor.vendor(notebook,output))
            ast.parse(output.read_text(encoding="utf-8"))
            self.assertFalse(vendor.vendor(notebook,output)) # idempotent
            digest=vendor.extract_functions(notebook)[1]
            with self.assertRaises(FileExistsError):
                changed=notebook.read_text().replace("return 3","return 4")
                notebook.write_text(changed,encoding="utf-8")
                vendor.vendor(notebook,output)
            notebook.unlink()  # prove the generated module does NOT read .ipynb
            spec=importlib.util.spec_from_file_location("test_vendored_hybrid",output)
            module=importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            core=types.SimpleNamespace(_reslice=lambda *args, **kw:("accelerated",))
            assert len(module.SOURCE_FUNCTIONS_SHA256)==64
            assert module.SOURCE_NOTEBOOK_SHA256==digest
            module.install(core)
            row, reason=core._measure_board(
                1,2,3,4,5,6,7,8,9,10,11,12, audit={}
            )
            self.assertIsNone(reason)
            self.assertEqual(row["main"],("accelerated",))
            self.assertEqual(row["side"][0],"special")
            self.assertEqual(row["side"][1],0.5)

    def test_no_external_notebook_import_or_runtime_pickle(self):
        output=vendor.render_module("def _reslice(): pass\n", "a"*64)
        self.assertNotIn("read_text(",output)
        self.assertNotIn("ipynb",output.split("def install(core):")[-1])
        ast.parse(output)


if __name__=="__main__":
    unittest.main()

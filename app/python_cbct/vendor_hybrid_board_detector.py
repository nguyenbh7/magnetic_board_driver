"""One-time vendor of the reviewed Hybrid board functions into maintained code.

Run on the developer's machine that holds the original, trusted source notebook.
The resulting hybrid_board_detector.py is ordinary Python source: it can be
reviewed, tested, committed, and distributed without an .ipynb at runtime.
No private DICOM, sensor data, or hidden notebook outputs are imported.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path

FUNCTIONS = ("_reslice", "_side_level", "_contrast_cut", "_footprint", "_measure_board")
SIGNATURE = (
    "data", "spacing", "affine", "pts", "grid", "board_size",
    "max_thickness", "min_rect", "slab_mm", "pitch_mm",
    "min_contrast", "dense_thr",
)


def extract_functions(path: Path) -> tuple[str, str]:
    raw = path.read_bytes()
    notebook = json.loads(raw.decode("utf-8"))
    if notebook.get("nbformat") != 4 or not isinstance(notebook.get("cells"), list):
        raise ValueError("Expected the reviewed Jupyter notebook format")
    functions = {}
    for cell in notebook["cells"]:
        if cell.get("cell_type") != "code":
            continue
        source = cell.get("source", [])
        source = "".join(source) if isinstance(source, list) else source
        try:
            tree = ast.parse(source)
        except SyntaxError:
            continue  # notebook magic and IPython-only cells are not executable in package
        for node in tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name in FUNCTIONS:
                if node.decorator_list or node.name in functions or isinstance(node, ast.AsyncFunctionDef):
                    raise ValueError(f"Unsafe, decorated or repeated Hybrid function: {node.name}")
                functions[node.name] = node
    if set(functions) != set(FUNCTIONS):
        raise ValueError(f"Missing source functions: {sorted(set(FUNCTIONS)-set(functions))}")
    signature = tuple(arg.arg for arg in functions["_measure_board"].args.args)
    if signature != SIGNATURE:
        raise ValueError(f"Unreviewed _measure_board signature: {signature}")
    # The same five AST nodes and source order were used in the validated
    # offline A/B research adapter. Do not silently change their bodies.
    tree = ast.fix_missing_locations(ast.Module(
        body=[functions[name] for name in FUNCTIONS], type_ignores=[]
    ))
    text = ast.unparse(tree) + "\n"
    return text, hashlib.sha256(raw).hexdigest()


def render_module(function_source: str, notebook_sha: str) -> str:
    digest = hashlib.sha256(function_source.encode("utf-8")).hexdigest()
    return (
        '"""Vendored GK Hybrid PCB detector. Reviewed source extracted once.\n'
        'Do not edit detection thresholds or board algorithms without an A/B review.\n'
        'This module is importable WITHOUT the source .ipynb being present.\n'
        '"""\n'
        'import warnings\n'
        'import numpy as np\n'
        'import scipy.ndimage as ndi\n'
        f'SOURCE_NOTEBOOK_SHA256 = "{notebook_sha}"\n'
        f'SOURCE_FUNCTIONS_SHA256 = "{digest}"\n\n'
        + function_source
        + '''
def install(core):
    """Install exact reviewed Hybrid board measurement into existing CT pipeline.

    Other magnet/BB detectors are unchanged. Ordinary face reslices reuse
    accelerated production interpolation; special side-layer reslices still
    use the alternate method. Match controlled offline research adapter.
    """
    for name, value in vars(core).items():
        if name not in globals() and not name.startswith("__"):
            globals()[name] = value
    # Preserve alternative reslice before routing ordinary slabs to core.
    local_reslice = _reslice
    accelerated_reslice = core._reslice

    def hybrid_reslice(*args, t_step=None, cval=None, **kwargs):
        if t_step is None and cval is None:
            return accelerated_reslice(*args, **kwargs)
        return local_reslice(*args, t_step=t_step, cval=cval, **kwargs)

    globals()["_reslice"] = hybrid_reslice

    def board_measure(data, spacing, affine, pts, grid, board_size,
                      max_thickness, min_rect, slab_mm, pitch_mm,
                      min_contrast, dense_thr, audit=None):
        # Accepted alternate _measure_board does not expose the optional
        # timing audit argument used by the maintained optimized detector.
        return _measure_board(
            data, spacing, affine, pts, grid, board_size, max_thickness,
            min_rect, slab_mm, pitch_mm, min_contrast, dense_thr
        )

    core._measure_board = board_measure
    return SOURCE_FUNCTIONS_SHA256
'''
    )


def vendor(source: Path, output: Path, *, replace: bool = False) -> bool:
    if not source.is_file():
        raise FileNotFoundError(f"Reviewed Hybrid notebook unavailable: {source}")
    body, original_digest = extract_functions(source)
    module_text = render_module(body, original_digest)
    ast.parse(module_text)  # Do not create malformed Python.
    if output.exists():
        current = output.read_text(encoding="utf-8")
        if current == module_text:
            print(f"Hybrid source already vendored and identical: {output}")
            return False
        if not replace:
            raise FileExistsError(
                f"Existing source differs: {output}. Review the new source, "
                "then rerun with --replace only if updating the approved Hybrid method."
            )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(module_text, encoding="utf-8")
    print(f"Vendored five reviewed Hybrid functions to: {output}")
    print(f"Original notebook SHA256: {original_digest}")
    print(f"Function source SHA256: {hashlib.sha256(body.encode('utf-8')).hexdigest()}")
    print("Next: run Hybrid regression tests and commit this .py file. No live approval is implied.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source", type=Path,
        default=Path.home() / "Downloads" / "find_magnets_arrays(1).ipynb",
        help="Existing reviewed source notebook (development-time input ONLY)",
    )
    parser.add_argument(
        "--output", type=Path,
        default=Path(__file__).with_name("hybrid_board_detector.py"),
        help="Maintained Python module to commit and ship with the detector",
    )
    parser.add_argument("--replace", action="store_true")
    args = parser.parse_args()
    vendor(args.source, args.output, replace=args.replace)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Where the SLG47910's device database is: Project Inkwell's forgefpga,
its `slg47910/` -- the routing graph, the template image, the format tables
(forgefpga's README says what each file is and where it comes from).

The checkout is `$FORGEFPGA`, by default `../Project-Inkwell/forgefpga`
beside this repository, as the Makefile's."""
import os
from pathlib import Path

_REPO = Path(__file__).resolve().parent.parent
FORGEFPGA = Path(os.environ.get("FORGEFPGA") or _REPO.parent / "Project-Inkwell" / "forgefpga")
DB = FORGEFPGA / "slg47910"
GRAPH = DB / "graph"
if not GRAPH.is_dir():
    raise SystemExit(f"no SLG47910 device database at {DB}: set FORGEFPGA to a Project Inkwell forgefpga checkout")

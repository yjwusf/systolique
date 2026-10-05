#!/usr/bin/env python3
"""One self-contained HTML page of a SystolicArray run (docs/systolic_array.md, "The viewer").

Runs a scenario through systolique_dump (tools/array_dump.cpp), which writes the run as JSON, and
puts that JSON into tools/array_view.html (inline CSS and JS, no network): a DIM x DIM grid of PEs per cycle coloured by the op whose weight (WS) or D/result
(OS) is in each PE's active register, an inner square for the shadow register's op, the PE state
of the cycle, a scrubber, counters, a timeline and the ops table.

  tools/array_view.py --build build --config dim4 --scenario ws_stream --tiles 3 --out page.html
  tools/array_view.py --build build --requests tools/ws_requests_example.json --out page.html
  tools/array_view.py --json run.json --out page.html     (JSON from systolique_dump)
  tools/array_view.py --check page.html [--expect-config c --expect-cycles n ...]
  tools/array_view.py --selftest --dump <systolique_dump> --out <dir>

--selftest (ctest systolique_view): pages for a tiny scenario and for the example request file
must pass --check with the expected MAC counts, and the committed examples (EXAMPLES, in
docs/examples/) must be byte for byte what their commands generate now.

--check parses a page, checks that it is well formed (every non-void element closed in order),
that it loads nothing from the network and that the embedded data is consistent (one frame per
cycle, per-cycle states summing to the PE count, per-request MACs summing to the total), and
prints "VIEW_CHECK ok ...".
"""
import argparse
import filecmp
import html.parser
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TEMPLATE = os.path.join(HERE, "array_view.html")
# The committed examples (docs/examples/) and the arguments that make them (README.md).
EXAMPLES = {
    "array_ws_dim4.html": ["--config", "dim4", "--scenario", "ws_stream", "--tiles", "3"],
    "array_ws_dim16.html": ["--config", "default", "--scenario", "ws_stream", "--tiles", "4"],
}
PLACEHOLDER = "/*DATA*/"
VOID = {"area", "base", "br", "col", "embed", "hr", "img", "input", "link", "meta", "source",
        "track", "wbr"}


def embed(data_text):
    """The page for this JSON: the data goes into a <script type="application/json">, with "</"
    escaped so no string in it can close the element."""
    with open(TEMPLATE, encoding="utf-8") as f:
        page = f.read()
    if page.count(PLACEHOLDER) != 1:
        raise SystemExit("template has no single data placeholder")
    json.loads(data_text)  # must be valid
    return page.replace(PLACEHOLDER, data_text.strip().replace("</", "<\\/"))


class Checker(html.parser.HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.stack, self.errors, self.data, self.in_data, self.external = [], [], [], False, []

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        for k in ("src", "href"):
            if k in a and a[k] and not a[k].startswith("#"):
                self.external.append(f"<{tag} {k}={a[k]}>")
        if tag not in VOID:
            self.stack.append(tag)
        self.in_data = tag == "script" and a.get("id") == "data"

    def handle_endtag(self, tag):
        if tag in VOID:
            return
        if not self.stack or self.stack[-1] != tag:
            self.errors.append(f"</{tag}> closes <{self.stack[-1] if self.stack else '-'}>")
            return
        self.stack.pop()
        self.in_data = False

    def handle_data(self, d):
        if self.in_data:
            self.data.append(d)


def check(path, expect):
    with open(path, encoding="utf-8") as f:
        text = f.read()
    c = Checker()
    c.feed(text)
    c.close()
    errs = list(c.errors)
    if c.stack:
        errs.append("unclosed: " + ",".join(c.stack))
    if c.external:
        errs.append("external resources: " + ", ".join(c.external))
    for word in ("http://", "https://"):
        if word in text.replace("https://github.com", ""):
            errs.append(f"{word} in the page")
    if not c.data:
        errs.append("no embedded data")
        print("VIEW_CHECK FAIL " + "; ".join(errs))
        return 1
    d = json.loads("".join(c.data))
    m = d["meta"]
    pes, cycles = m["dim"] * m["cols"], m["cycles"]
    if len(d["frames"]) != cycles:
        errs.append(f"{len(d['frames'])} frames for {cycles} cycles")
    if len(d["events"]) != pes:
        errs.append("register events not per PE")
    pc = d["per_cycle"]
    if len(pc) != 7 or any(len(x) != cycles for x in pc):
        errs.append("per-cycle counters do not cover the run")
    else:
        for t in range(cycles):
            if sum(pc[k][t] for k in range(5)) != pes:
                errs.append(f"cycle {t}: states do not sum to {pes}")
                break
            if pc[5][t] > pc[1][t] or pc[6][t] > pc[1][t]:
                errs.append(f"cycle {t}: concurrent flags outside MAC PE-cycles")
                break
    macs = sum(r["macs"] for r in d["requests"])
    if macs != d["totals"]["run"]["mac"] or macs != sum(pc[1]):
        errs.append("per-request MACs do not sum to the total")
    for p, pe in enumerate(d["per_pe"]):
        if sum(pe[:5]) != cycles:
            errs.append(f"PE {p}: states do not sum to the cycles")
            break
    for f in d["frames"]:
        n = len(f) - 1
        if f[0] not in "fd" or (f[0] == "f" and n != 3 * pes) or (f[0] == "d" and n % 5):
            errs.append("malformed frame")
            break
    for k, v in expect.items():
        if v is not None and str(m.get(k, d["totals"]["run"].get(k))) != str(v):
            errs.append(f"{k} = {m.get(k)}, expected {v}")
    if errs:
        print("VIEW_CHECK FAIL " + "; ".join(errs))
        return 1
    print(f"VIEW_CHECK ok config={m['config']} dim={m['dim']} cycles={cycles} "
          f"requests={len(d['requests'])} macs={macs} bytes={len(text)}")
    return 0


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.stdout.write(r.stdout + r.stderr)
        raise SystemExit(f"FAIL: {' '.join(cmd)} exited {r.returncode}")
    return r.stdout


def selftest(dump, out):
    os.makedirs(out, exist_ok=True)
    gen = [sys.executable, os.path.abspath(__file__), "--dump", dump]
    me = [sys.executable, os.path.abspath(__file__)]
    tiny = os.path.join(out, "tiny.html")
    run(gen + ["--config", "dim4", "--scenario", "ws_single", "--out", tiny])
    # ws_single on DIM 4: 2 requests (preload, compute), 4^3 = 64 useful MACs.
    print(run(me + ["--check", tiny, "--expect-config", "dim4", "--expect-mac", "64"]).strip())
    req = os.path.join(out, "requests.html")
    run(gen + ["--requests", os.path.join(HERE, "ws_requests_example.json"), "--out", req])
    print(run(me + ["--check", req, "--expect-config", "dim4", "--expect-mac", "192"]).strip())
    pages = 2
    for name, args in EXAMPLES.items():
        mine = os.path.join(out, name)
        run(gen + args + ["--out", mine])
        committed = os.path.join(ROOT, "docs", "examples", name)
        print(run(me + ["--check", committed]).strip())
        if not filecmp.cmp(mine, committed, shallow=False):
            raise SystemExit(f"FAIL: {committed} is not what its command generates now; regenerate "
                             "it (README.md, 'The viewer')")
        pages += 1
    print(f"VIEW_SELFTEST ok pages={pages}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", default="build", help="build directory with systolique_dump")
    ap.add_argument("--dump", help="path of systolique_dump (default: <build>/systolique_dump)")
    ap.add_argument("--config")
    ap.add_argument("--scenario", choices=["ws_single", "ws_stream", "os_single", "os_stream"])
    ap.add_argument("--tiles", type=int)
    ap.add_argument("--seed", type=int)
    ap.add_argument("--bubble", type=float)
    ap.add_argument("--requests", help="request list (JSON), see tools/array_dump.cpp")
    ap.add_argument("--json", help="use this systolique_dump output instead of running it")
    ap.add_argument("--out", help="HTML file to write")
    ap.add_argument("--check", help="check this page instead")
    ap.add_argument("--expect-config")
    ap.add_argument("--expect-cycles")
    ap.add_argument("--expect-mac")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        if not a.dump or not a.out:
            ap.error("--selftest needs --dump and --out (a directory)")
        return selftest(a.dump, a.out)
    if a.check:
        return check(a.check, {"config": a.expect_config, "cycles": a.expect_cycles,
                               "mac": a.expect_mac})
    if not a.out:
        ap.error("--out is required")
    if a.json:
        with open(a.json, encoding="utf-8") as f:
            data = f.read()
    else:
        dump = a.dump or os.path.join(a.build, "systolique_dump")
        cmd = [dump]
        for k in ("config", "scenario", "tiles", "seed", "bubble", "requests"):
            v = getattr(a, k)
            if v is not None:
                cmd += ["--" + k, str(v)]
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "run.json")
            r = subprocess.run(cmd + ["--out", out], capture_output=True, text=True)
            if r.returncode:
                sys.stderr.write(r.stdout + r.stderr)
                return r.returncode
            sys.stdout.write(r.stdout)
            with open(out, encoding="utf-8") as f:
                data = f.read()
    page = embed(data)
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(page)
    print(f"wrote {a.out} ({len(page)} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

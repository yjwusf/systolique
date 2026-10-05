#!/usr/bin/env python3
"""Provenance of the Gemmini RTL and of the stored reference traces (rtl/README.md).

  provenance --out DIR --gemmini G --hardfloat H --java J --sbt S --generator GEN
        write DIR/provenance.json for Verilog that rtl/elaborate.sh generated: the pinned
        sources, the tool versions, the configurations, a digest of every file
  record --build BUILD [--verilog DIR] [--ref-dir REF]
        run every lockstep bench of BUILD (rtl_<config> --write-ref) into
        REF/<config>/<test>.csv.gz and write REF/provenance.json (the Verilog provenance, the
        Verilator that ran it, sha256 and length of every trace)
  check-verilog --verilog DIR [--ref-dir REF]
        fail if the generated Verilog differs from the Verilog the stored traces were made from
  check-traces [--ref-dir REF]
        offline: every trace REF/provenance.json lists exists with its sha256 and cycle count and
        no other trace exists; its configurations equal src/config.cpp's and
        rtl/src/GemminiTops.scala's; the generator sources in rtl/ are the recorded ones
  correspondence --verilog DIR
        write DIR/<config>/correspondence.txt: the RTL registers behind the model's merged state
        (docs/microarchitecture.md "Cycle correspondence"), found by following the
        register-to-register copies in the generated Verilog; the lockstep bench reads them
        through VPI
"""
import argparse, glob, gzip, hashlib, json, os, re, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REF = os.path.join(ROOT, "tests", "reference", "gemmini_rtl")
GEN = os.path.join(ROOT, "rtl")
GEMMINI_REPO = "https://github.com/ucb-bar/gemmini"
GEMMINI_TAG = "v0.7.2"
GEMMINI_COMMIT = "709bc56b6dd859fc2b1a9027a96a0b5be6ad7ed6"
HARDFLOAT_REPO = "https://github.com/ucb-bar/berkeley-hardfloat"
HARDFLOAT_COMMIT = "9deaf1d49f487347be371641ba6ea637b669ad21"
TOPS = ("MeshTop", "MeshWithDelaysTop")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


LOCATOR = re.compile(r"[ \t]*// @\[[^\n]*")


def verilog_digest(path):
    """sha256 of a generated Verilog file without the `// @[file line:col]` source locators,
    which hold the absolute path of the Gemmini checkout."""
    with open(path) as f:
        text = LOCATOR.sub("", f.read())
    return hashlib.sha256(text.encode()).hexdigest()


def first_line(cmd):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True)
    except OSError:
        return None
    out = (p.stdout or p.stderr).strip()
    return out.splitlines()[0] if out else None


def git_head(repo):
    """HEAD of a checkout, read from its .git directory (detached or on a branch)."""
    gitdir = os.path.join(repo, ".git")
    try:
        head = open(os.path.join(gitdir, "HEAD")).read().strip()
    except OSError:
        return None
    if not head.startswith("ref: "):
        return head
    ref = head[5:]
    p = os.path.join(gitdir, ref)
    if os.path.exists(p):
        return open(p).read().strip()
    packed = os.path.join(gitdir, "packed-refs")
    if os.path.exists(packed):
        for line in open(packed):
            parts = line.split()
            if len(parts) == 2 and parts[1] == ref:
                return parts[0]
    return None


def scala_configs(path):
    """ArrayCfgs.all of GemminiTops.scala as {name: params}."""
    text = open(path).read()
    cfgs = {}
    for m in re.finditer(r'ArrayCfg\("(\w+)",\s*([^)]*)\)', text):
        v = [x.strip() for x in m.group(2).split(",")]
        if len(v) < 10:
            continue
        cfgs[m.group(1)] = {
            "in_bits": int(v[0]), "out_bits": int(v[1]), "acc_bits": int(v[2]),
            "dataflow": v[3].split(".")[-1], "tile_rows": int(v[4]), "tile_cols": int(v[5]),
            "mesh_rows": int(v[6]), "mesh_cols": int(v[7]), "tile_latency": int(v[8]),
            "output_delay": int(v[9]), "tag_bits": int(v[10]) if len(v) > 10 else 8}
    return cfgs


def cmd_provenance(a):
    head = git_head(a.gemmini)
    if head != GEMMINI_COMMIT:
        print(f"warning: {a.gemmini} is at {head}, not Gemmini {GEMMINI_TAG} ({GEMMINI_COMMIT})")
    files = {}
    for cfg in sorted(os.listdir(a.out)):
        for top in TOPS:
            p = os.path.join(a.out, cfg, top + ".v")
            if os.path.exists(p):
                files[f"{cfg}/{top}.v"] = verilog_digest(p)
    doc = {
        "_doc": "Verilog of Gemmini's systolic array, elaborated by rtl/elaborate.sh; "
                "written by tools/rtl_provenance.py provenance.",
        "gemmini": {"repo": GEMMINI_REPO, "tag": GEMMINI_TAG, "commit": head,
                    "files": ["src/main/scala/gemmini/" + f + ".scala" for f in
                              ("Arithmetic", "Dataflow", "Mesh", "MeshWithDelays", "PE", "Shifter",
                               "SyncMem", "TagQueue", "Tile", "Transposer", "Util")]},
        "hardfloat": {"repo": HARDFLOAT_REPO, "commit": HARDFLOAT_COMMIT,
                      "why": "rocket-chip's hardfloat submodule at the Chipyard commit Gemmini "
                             "v0.7.2 names (CHIPYARD.hash ef3409f)"},
        "chisel": "3.6.0, Scala FIRRTL compiler (chisel3.stage.ChiselStage.emitVerilog)",
        "scala": "2.13.10",
        "sbt": first_line([a.sbt, "--script-version"]),
        "java": first_line([a.java, "-version"]),
        "generator": {
            "command": "rtl/elaborate.sh",
            "sources": {f: sha256(os.path.join(a.generator, f))
                        for f in ("build.sbt", "project/build.properties", "src/GemminiTops.scala")},
        },
        "configs": scala_configs(os.path.join(a.generator, "src", "GemminiTops.scala")),
        "verilog_digest": "sha256 of each file without its // @[...] source locators",
        "verilog": files,
    }
    with open(os.path.join(a.out, "provenance.json"), "w") as f:
        json.dump(doc, f, indent=1)
        f.write("\n")
    print(f"wrote {os.path.join(a.out, 'provenance.json')}: {len(files)} Verilog files")


def verilator_version(build):
    cache = os.path.join(build, "CMakeCache.txt")
    exe = None
    if os.path.exists(cache):
        for line in open(cache):
            if line.startswith("VERILATOR_BIN:"):
                exe = line.split("=", 1)[1].strip()
    if exe:
        root = os.path.dirname(os.path.dirname(exe))
        for cand in (os.path.join(root, "bin", "verilator"), exe):
            v = first_line([cand, "--version"])
            if v:
                return v
    return None


def cmd_record(a):
    with open(os.path.join(a.verilog, "provenance.json")) as f:
        vprov = json.load(f)
    traces = {}
    for cfg in sorted(vprov["configs"]):
        exe = os.path.join(a.build, "rtl_" + cfg)
        out = os.path.join(a.ref_dir, cfg)
        os.makedirs(out, exist_ok=True)
        for old in glob.glob(os.path.join(out, "*.csv.gz")):
            os.remove(old)
        comments = [f"gemmini {vprov['gemmini']['tag']} {vprov['gemmini']['commit']}",
                    f"verilog digest {vprov['verilog'][cfg + '/MeshTop.v']} (MeshTop) "
                    f"{vprov['verilog'][cfg + '/MeshWithDelaysTop.v']} (MeshWithDelaysTop)"]
        cmd = [exe, "--write-ref", out]
        for c in comments:
            cmd += ["--comment", c]
        p = subprocess.run(cmd, capture_output=True, text=True)
        sys.stdout.write("".join(l + "\n" for l in p.stdout.splitlines() if "LOCKSTEP" in l))
        if p.returncode:
            sys.stdout.write(p.stdout + p.stderr)
            raise SystemExit(f"{exe} failed")
        for path in sorted(glob.glob(os.path.join(out, "*.csv.gz"))):
            with gzip.open(path, "rt") as f:
                rows = sum(1 for l in f if l and l[0].isdigit())
            traces[os.path.relpath(path, a.ref_dir)] = {"sha256": sha256(path), "cycles": rows}
    doc = {
        "_doc": "Reference traces of the Gemmini RTL (inputs and outputs of every cycle), made by "
                "the lockstep benches rtl_<config> while the model matched them; "
                "written by tools/rtl_provenance.py record.",
        "verilator": verilator_version(a.build),
        "rtl": vprov,
        "traces": traces,
    }
    with open(os.path.join(a.ref_dir, "provenance.json"), "w") as f:
        json.dump(doc, f, indent=1)
        f.write("\n")
    print(f"wrote {os.path.relpath(os.path.join(a.ref_dir, 'provenance.json'), ROOT)}: "
          f"{len(traces)} traces, {sum(t['cycles'] for t in traces.values())} cycles")


def cmd_check_verilog(a):
    with open(os.path.join(a.ref_dir, "provenance.json")) as f:
        want = json.load(f)["rtl"]["verilog"]
    bad = []
    for rel, sha in sorted(want.items()):
        p = os.path.join(a.verilog, rel)
        if not os.path.exists(p):
            bad.append(f"{rel}: missing")
        elif verilog_digest(p) != sha:
            bad.append(f"{rel}: differs from the Verilog the reference traces were made from")
    if bad:
        print("\n".join(bad))
        raise SystemExit("VERILOG differs (regenerate with rtl/elaborate.sh at "
                         f"Gemmini {GEMMINI_TAG}, or re-record the references)")
    print(f"VERILOG ok: {len(want)} files as recorded")


def cpp_configs(path):
    """named_configs() of src/config.cpp as {name: params}."""
    text = open(path).read()
    cfgs = {}
    for m in re.finditer(r'\{"(\w+)", (\d+), (\d+), (\d+), Dataflow::(\w+), (\d+), (\d+), (\d+), '
                         r'(\d+), (\d+), (\d+), (\d+)\}', text):
        v = m.groups()
        cfgs[v[0]] = {"in_bits": int(v[1]), "out_bits": int(v[2]), "acc_bits": int(v[3]),
                      "dataflow": v[4], "tile_rows": int(v[5]), "tile_cols": int(v[6]),
                      "mesh_rows": int(v[7]), "mesh_cols": int(v[8]), "tile_latency": int(v[9]),
                      "output_delay": int(v[10]), "tag_bits": int(v[11])}
    return cfgs


def cmd_check_traces(a):
    errors = []
    with open(os.path.join(a.ref_dir, "provenance.json")) as f:
        prov = json.load(f)
    rtl = prov["rtl"]
    if rtl["gemmini"]["commit"] != GEMMINI_COMMIT or rtl["gemmini"]["tag"] != GEMMINI_TAG:
        errors.append(f"the traces are from Gemmini {rtl['gemmini']['tag']} {rtl['gemmini']['commit']}, "
                      f"not {GEMMINI_TAG} {GEMMINI_COMMIT}")
    listed = set(prov["traces"])
    cycles = 0
    for rel, t in sorted(prov["traces"].items()):
        p = os.path.join(a.ref_dir, rel)
        if not os.path.exists(p):
            errors.append(f"{rel}: missing")
            continue
        if sha256(p) != t["sha256"]:
            errors.append(f"{rel}: changed since it was recorded")
        with gzip.open(p, "rt") as f:
            rows = sum(1 for l in f if l and l[0].isdigit())
        if rows != t["cycles"]:
            errors.append(f"{rel}: {rows} cycles, {t['cycles']} recorded")
        cycles += rows
    for p in glob.glob(os.path.join(a.ref_dir, "*", "*.csv.gz")):
        if os.path.relpath(p, a.ref_dir) not in listed:
            errors.append(f"{os.path.relpath(p, a.ref_dir)}: not in provenance.json")
    want = rtl["configs"]
    for what, got in (("src/config.cpp", cpp_configs(os.path.join(ROOT, "src", "config.cpp"))),
                      ("rtl/src/GemminiTops.scala",
                       scala_configs(os.path.join(GEN, "src", "GemminiTops.scala")))):
        if got != want:
            errors.append(f"the configurations of {what} differ from the traces' ({sorted(got)} vs "
                          f"{sorted(want)})")
    for rel, sha in sorted(rtl["generator"]["sources"].items()):
        p = os.path.join(GEN, rel)
        if not os.path.exists(p) or sha256(p) != sha:
            errors.append(f"rtl/{rel} is not the generator source the traces were made with")
    if errors:
        print("\n".join(errors))
        raise SystemExit("TRACES inconsistent")
    print(f"TRACES ok: {len(listed)} traces, {cycles} cycles, {len(want)} configurations, "
          f"Gemmini {GEMMINI_TAG} {GEMMINI_COMMIT[:7]}, generator sources as recorded")


COPY = re.compile(r"^    (\w+) <= (\w+);")
ASSIGN = re.compile(r"^  assign (\w+) = (\w+);")


def module_text(text, name):
    m = re.search(r"^module " + name + r"\(.*?^endmodule", text, re.S | re.M)
    if not m:
        raise SystemExit(f"module {name} not found")
    return m.group(0)


def copies(mod):
    """reg -> source for the unconditional `reg <= ident;` in the always blocks (shift stages)."""
    c = {}
    for line in mod.splitlines():
        m = COPY.match(line)
        if m:
            c[m.group(1)] = m.group(2)
    return c


def chain(cp, last, n):
    """The n registers ending in `last`, stage 0 first, or None if they are not a pure chain."""
    regs = [last]
    while len(regs) < n:
        if regs[0] not in cp:
            return None
        regs.insert(0, cp[regs[0]])
    return regs


def cmd_correspondence(a):
    for cfgdir in sorted(glob.glob(os.path.join(a.verilog, "*", "MeshWithDelaysTop.v"))):
        cfg = os.path.basename(os.path.dirname(cfgdir))
        prov = json.load(open(os.path.join(a.verilog, "provenance.json")))["configs"][cfg]
        depth = prov["tile_latency"] + 1
        text = open(cfgdir).read()
        mwd, mesh = module_text(text, "MeshWithDelays"), module_text(text, "Mesh")
        out, errors = [], []
        cp = copies(mwd)
        tc, tr = prov["tile_cols"], prov["tile_rows"]

        def add(kind, sig, group, lane, final, n):
            if n == 0:
                return
            regs = chain(cp, final, n)
            if regs is None:
                errors.append(f"{kind} {sig} {group} {lane}: {final} is not a chain of {n}")
                return
            out.extend(f"{kind} {sig} {group} {lane} {k} {r}" for k, r in enumerate(regs))

        for line in mwd.splitlines():
            m = ASSIGN.match(line)
            if not m:
                continue
            lhs, rhs = m.groups()
            f = re.fullmatch(r"mesh_io_in_(a|b|d|valid|id|last)_(\d+)_(\d+)", lhs)
            if f:
                g, j = int(f.group(2)), int(f.group(3))
                add("feed", f.group(1), g, j, rhs, g * depth)
            f = re.fullmatch(r"mesh_io_in_control_(\d+)_(\d+)_(dataflow|propagate|shift)", lhs)
            if f:
                g, j = int(f.group(1)), int(f.group(2))
                add("feed", f.group(3), g, j, rhs, g * depth)
            f = re.fullmatch(r"io_resp_bits_data_(\d+)_(\d+)", lhs)
            if f:
                g, j = int(f.group(1)), int(f.group(2))
                add("resp", "data", g, j, rhs, (prov["mesh_cols"] - 1 - g) * depth)
            if lhs in ("io_resp_valid", "io_resp_bits_last"):
                add("resp", "valid" if lhs == "io_resp_valid" else "last", 0, 0, rhs,
                    (prov["mesh_cols"] - 1) * depth)
        m = re.search(r"_io_resp_bits_tag_T = (\w+) == tagq_io_deq_bits_id", mwd)
        if m:
            add("resp", "id", 0, 0, m.group(1), (prov["mesh_cols"] - 1) * depth)
        # Mesh: the in_valid ShiftRegister of every tile and lane, and the Pipe valid flops.
        cp = copies(mesh)
        for line in mesh.splitlines():
            m = ASSIGN.match(line)
            f = m and re.fullmatch(r"mesh_(\d+)_(\d+)_io_in_valid_(\d+)", m.group(1))
            if f:
                r, c, j = (int(x) for x in f.groups())
                regs = chain(cp, m.group(2), depth)
                if regs is None:
                    errors.append(f"valid {r} {c} {j}: {m.group(2)} is not a chain of {depth}")
                    continue
                out.extend(f"valid {r} {c} {j} {k} {x}" for k, x in enumerate(regs))
        # A Pipe's valid flop copies the valid entering the tile: a Mesh input (tile row 0) or
        # the valid leaving the tile above (Mesh.scala:62, 71, 82-84); its name may not say
        # which tile it belongs to (pipe_v_9), the source does.
        stage = {}
        for reg, src in sorted(cp.items()):
            if not re.fullmatch(r"(?:\w+_)?pipe(?:_pipe)*_v(?:_\d+)?", reg):
                continue
            f = re.fullmatch(r"io_in_valid_(\d+)_(\d+)", src)
            if f:
                stage[reg] = (0, int(f.group(1)), int(f.group(2)), 0)
            f = re.fullmatch(r"mesh_(\d+)_(\d+)_io_out_valid_(\d+)", src)
            if f:
                stage[reg] = (int(f.group(1)) + 1, int(f.group(2)), int(f.group(3)), 0)
        for reg, src in sorted(cp.items()):  # deeper Pipe stages follow their predecessor
            if reg not in stage and src in stage and reg.endswith("_v"):
                r, c, j, k = stage[src]
                stage[reg] = (r, c, j, k + 1)
        out.extend(f"pipev {r} {c} {j} {k} {reg}" for reg, (r, c, j, k) in sorted(stage.items()))
        if errors:
            raise SystemExit(f"{cfg}: " + "; ".join(errors[:5]))
        path = os.path.join(a.verilog, cfg, "correspondence.txt")
        with open(path, "w") as f:
            f.write("# <kind> <signal|tile row> <group|tile col> <lane> <stage> <RTL register>; "
                    "written by tools/rtl_provenance.py correspondence\n")
            f.write("\n".join(out) + "\n")
        kinds = {}
        for l in out:
            kinds[l.split()[0]] = kinds.get(l.split()[0], 0) + 1
        print(f"wrote {path}: " + ", ".join(f"{v} {k}" for k, v in sorted(kinds.items())))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("provenance")
    for k in ("out", "gemmini", "hardfloat", "java", "sbt", "generator"):
        p.add_argument("--" + k, required=True)
    p = sub.add_parser("record")
    p.add_argument("--build", required=True)
    p.add_argument("--verilog", default=os.path.expanduser("~/opt/gemmini-verilog-709bc56"))
    p.add_argument("--ref-dir", default=REF)
    p = sub.add_parser("check-verilog")
    p.add_argument("--verilog", required=True)
    p.add_argument("--ref-dir", default=REF)
    p = sub.add_parser("correspondence")
    p.add_argument("--verilog", required=True)
    p = sub.add_parser("check-traces")
    p.add_argument("--ref-dir", default=REF)
    a = ap.parse_args()
    {"provenance": cmd_provenance, "record": cmd_record, "check-verilog": cmd_check_verilog,
     "check-traces": cmd_check_traces, "correspondence": cmd_correspondence}[a.cmd](a)


if __name__ == "__main__":
    main()

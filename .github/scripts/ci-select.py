#!/usr/bin/env python3
"""Pick which CI jobs a change has to run.

Every PR used to run the whole suite -- about 50 jobs and 150 runner-minutes --
although nearly every change lands in one SoC family: a README edit, or a
Novatek device fix, still booted fourteen HiSilicon SoCs and ran four VENC
pipelines.  This script maps the changed paths to the jobs they can reach.

The rule is one-directional: a path has to be *recognised* before it can
narrow anything.  qemu/setup.sh, the patches, the workflow and anything this
script does not know widen back out to the full suite, so a wrong
classification costs runner time, never coverage.  The weekly scheduled run
is the full suite regardless, as a backstop.

Which family a device model reaches is derived from the tree, not written
down.  A model belongs to the family its file name names (hisi-, nvt-,
ingenic-, ...), and also to every family whose code refers to it: by its QOM
type name ("hisi-smartsens"), its TYPE_ macro, or an #include of its header.
That is followed transitively, so hantro-h264.c reaches Xiongmai, Grain
Media, SigmaStar, Ingenic, Ambarella and HiSilicon through the encoders that
include it, and the HiSilicon models the Xiongmai and Goke machines
instantiate reach those families too.  A model that names no family and that
nothing refers to (an upstream file this repo overrides) widens to all.

Scripts under qemu-boot/ and qemu/tests/ run the jobs whose workflow text
names them, directly or through another script that does.  A script no job
runs (a run-*.sh helper, an SDK builder) runs nothing.

Usage:
    ci-select.py               # GitHub Actions: read the event, write $GITHUB_OUTPUT
    ci-select.py --stdin       # read changed paths on stdin, print the decision
    ci-select.py --all         # print the full-suite decision
    ci-select.py --self-test   # check this file still agrees with the tree
    ci-select.py --explain P   # print the families and jobs path P reaches
"""

import argparse
import collections
import fnmatch
import json
import os
import re
import subprocess
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORKFLOW = ".github/workflows/ci.yml"
BOOT_MATRIX = ".github/ci/boot-matrix.yml"

FAMILIES = [
    "hisilicon", "xiongmai", "goke710x", "novatek", "ingenic", "fullhan",
    "grainmedia", "sigmastar", "anyka", "ambarella", "allwinner",
]

# File-name prefix -> family.  Longest match wins, so "xm510-" and "xm-" both
# land in xiongmai and "gk-" does not swallow anything else.
PREFIX_FAMILY = {
    "hisi-": "hisilicon", "hisilicon": "hisilicon",
    "xm-": "xiongmai", "xm510-": "xiongmai", "xiongmai": "xiongmai",
    "gk-": "goke710x", "gk710x": "goke710x", "goke": "goke710x",
    "nvt-": "novatek", "novatek": "novatek",
    "ingenic": "ingenic",
    "fh-": "fullhan", "fullhan": "fullhan",
    "gm-": "grainmedia", "grainmedia": "grainmedia",
    "sstar-": "sigmastar", "mstar-": "sigmastar", "sigmastar": "sigmastar",
    "ak-": "anyka", "anyka": "anyka",
    "amb-": "ambarella", "ambarella": "ambarella",
    "aw-": "allwinner", "allwinner": "allwinner",
}

# Jobs in ci.yml, by the families whose code they exercise.  `boot` is
# per-row (its rows carry `group:` in .github/ci/boot-matrix.yml) and `build`
# runs whenever anything else does.  --self-test fails when ci.yml has a job
# this table does not name, so a new job cannot silently go always-on or
# never-on.
JOB_FAMILIES = {
    "ive-test": {"hisilicon"},
    "venc-hisilicon": {"hisilicon"},
    "sofia-isp": {"hisilicon"},
    "maskrom-av300": {"hisilicon"},
    "boot-hi3516cv610": {"hisilicon"},
    "boot-hi3516ev300-nand": {"hisilicon"},
    "boot-novatek": {"novatek"},
    "boot-novatek-recovery": {"novatek"},
    "maskrom-na51089": {"novatek"},
    "boot-fullhan": {"fullhan"},
    "boot-grainmedia": {"grainmedia"},
    "boot-ingenic": {"ingenic"},
    "majestic-ingenic": {"ingenic"},
}
SPECIAL_JOBS = {"select", "build", "boot", "ci-gate"}

# Files the workflow does not name but a job consumes.  The IVE initramfs is
# compiled in the build job and booted by ive-test.
EXTRA_JOB_EDGES = {
    "qemu-boot/test-ive-ops.c": {"ive-test"},
}

# Paths that never affect a CI result.
DOC_GLOBS = ["*.md", "docs/*", "LICENSE", ".gitignore", "demo/*"]

# Paths that can affect everything: the build, the patched upstream sources,
# the workflow and this selector.
ALL_GLOBS = ["qemu/setup.sh", "qemu/patches/*", ".github/*"]

# Directories whose scripts run only when a job names them.
SCRIPT_DIRS = ("qemu-boot/", "qemu/tests/", "scripts/", "tools/")

DEVICE_DIRS = ("qemu/hw/", "qemu/include/")

ALL = "ALL"

# The revision a change is read up to.  Only --head (replaying a past commit
# with --stdin) moves it off the checked-out tree.
HEAD_REV = "HEAD"


def sh(*args, check=True):
    return subprocess.run(args, cwd=ROOT, check=check, text=True,
                          capture_output=True).stdout


def tracked_files():
    return sh("git", "ls-files").split()


def read(path):
    try:
        with open(os.path.join(ROOT, path), encoding="utf-8",
                  errors="replace") as f:
            return f.read()
    except (FileNotFoundError, IsADirectoryError):
        return ""


def match_any(path, globs):
    return any(fnmatch.fnmatchcase(path, g) or
               fnmatch.fnmatchcase(os.path.basename(path), g) for g in globs)


def family_of_name(name):
    best = None
    for prefix, fam in PREFIX_FAMILY.items():
        if name.startswith(prefix) and (best is None or len(prefix) > len(best[0])):
            best = (prefix, fam)
    return best[1] if best else None


# ── Device-model reach ──────────────────────────────────────────────────

TYPE_DEFINE = re.compile(r'#define\s+(TYPE_\w+)\s+"([^"]+)"')
# `.name` only inside TypeInfo initialisers: ARMCPRegInfo, VMState and
# MemoryRegionOps have `.name`s too, and "CPUACTLR" is not a device.
TYPEINFO_BLOCK = re.compile(r'TypeInfo\s+\w+\s*(?:\[\s*\])?\s*=\s*\{(.*?)\n\};', re.S)
TYPEINFO_NAME = re.compile(r'\.name\s*=\s*"([^"]+)"')
TYPEINFO_MACRO = re.compile(r'\.name\s*=\s*(TYPE_\w+)')
INCLUDE = re.compile(r'#include\s+"([^"]+)"')
QUOTED = re.compile(r'"([^"\n]{4,64})"')
TYPE_IDENT = re.compile(r'\bTYPE_\w+')


class DeviceGraph:
    """Which families can reach each file under qemu/hw and qemu/include."""

    def __init__(self, files):
        self.files = [f for f in files if f.startswith(DEVICE_DIRS)
                      and f.endswith((".c", ".h"))]
        self.text = {f: read(f) for f in self.files}
        self.reach = self._compute()

    def _exports(self, f):
        """Tokens by which other code names this file."""
        t = self.text[f]
        infos = "\n".join(TYPEINFO_BLOCK.findall(t))
        strings = set(TYPEINFO_NAME.findall(infos))
        macros = set(TYPEINFO_MACRO.findall(infos))
        for macro, string in TYPE_DEFINE.findall(t):
            macros.add(macro)
            strings.add(string)
        headers = set()
        if f.endswith(".h"):
            headers.add(os.path.basename(f))
        else:
            # A .c is reached through its own header's functions too.
            stem = os.path.splitext(os.path.basename(f))[0]
            headers.add(stem + ".h")
        # Generic names would make everything reach everything.
        strings = {s for s in strings if len(s) > 3}
        return strings, macros, headers

    def _compute(self):
        # What each file mentions, tokenised once: quoted strings, TYPE_
        # identifiers, and the basenames it #includes.
        mentions = {}
        for g in self.files:
            t = self.text[g]
            mentions[g] = (set(QUOTED.findall(t)), set(TYPE_IDENT.findall(t)),
                           {os.path.basename(i) for i in INCLUDE.findall(t)})
        users = {f: set() for f in self.files}
        for f in self.files:
            strings, macros, headers = self._exports(f)
            for g in self.files:
                if g == f:
                    continue
                qs, ids, incs = mentions[g]
                if (strings & qs) or (macros & ids) or (headers & incs):
                    users[f].add(g)

        built = built_under()
        reach = {}
        for f in self.files:
            r = set(built.get(os.path.basename(f), ()))
            fam = family_of_name(os.path.basename(f))
            if fam:
                r.add(fam)
            reach[f] = r
        changed = True
        while changed:
            changed = False
            for f in self.files:
                for u in users[f]:
                    if not reach[u] <= reach[f]:
                        reach[f] |= reach[u]
                        changed = True
        self.users = users
        return {f: (r if r else {ALL}) for f, r in reach.items()}

    def families(self, path):
        if path in self.reach:
            return self.reach[path]
        # A new file (or a deleted one) we have no text for: by its name if
        # it has a family prefix, otherwise everything.
        fam = family_of_name(os.path.basename(path))
        return {fam} if fam else {ALL}


def built_under():
    """basename -> families whose Kconfig symbol qemu/setup.sh builds it under.

    The meson lines are `system_ss.add(when: 'CONFIG_X', if_true:
    files(...))`, some inside `for cfg in X Y; do ... CONFIG_$cfg ... done`.
    """
    setup = read("qemu/setup.sh")
    out = {}

    def add(cfg, flist):
        fam = CONFIG_FAMILY.get(cfg)
        if fam:
            for base in re.findall(r"'([^']+)'", flist):
                out.setdefault(base, set()).add(fam)

    for cfgs, body in re.findall(r"for cfg in ([A-Z0-9_ ]+); do(.*?)\ndone", setup, re.S):
        for flist in re.findall(r"when: 'CONFIG_\$cfg', if_true: files\(([^)]*)\)", body):
            for c in cfgs.split():
                add("CONFIG_" + c, flist)
    for cfg, flist in re.findall(r"when: '(CONFIG_\w+)', if_true: files\(([^)]*)\)", setup):
        add(cfg, flist)
    return out


# ── Workflow structure ──────────────────────────────────────────────────

class Workflow:
    def __init__(self):
        self.doc = yaml.safe_load(read(WORKFLOW))
        self.jobs = self.doc["jobs"]
        self.boot_rows = yaml.safe_load(read(BOOT_MATRIX))["include"]
        # Build-job steps that run one qemu/tests script, keyed by the script.
        self.qtests = {}
        for step in self.jobs["build"]["steps"]:
            run = step.get("run", "")
            m = re.search(r"qemu/tests/[\w.\-]+", run)
            if m:
                self.qtests.setdefault(m.group(0), [])
                self.qtests[m.group(0)].append(step.get("name", ""))

    def job_text(self, job):
        return yaml.dump(self.jobs[job])


def qtest_family(script):
    name = os.path.basename(script)
    name = re.sub(r"^test-", "", name)
    return family_of_name(name)


# ── Scripts and which jobs run them ─────────────────────────────────────

class ScriptGraph:
    """Which jobs, build-step qtests and boot rows each script feeds."""

    def __init__(self, files, wf):
        self.wf = wf
        self.scripts = [f for f in files if f.startswith(SCRIPT_DIRS)]
        self.text = {f: read(f) for f in self.scripts}
        texts = {j: wf.job_text(j) for j in wf.jobs if j not in ("build", "boot")}
        boot_steps = wf.jobs["boot"]["steps"]
        self.direct = {}
        for s in self.scripts:
            rx = self._name_rx(s)
            named = lambda t: s in t or rx.search(t)
            jobs = {j for j, t in texts.items() if named(t)}
            jobs |= EXTRA_JOB_EDGES.get(s, set())
            qt = {q for q in wf.qtests if q == s}
            # A boot step that runs only for rows with a matrix flag
            # (`if: matrix.stream_test`) reaches only those rows.
            rows = set()
            for step in boot_steps:
                if not named(yaml.dump(step)):
                    continue
                keys = re.findall(r"matrix\.(\w+)", str(step.get("if", "")))
                if not keys:
                    rows = None
                    break
                rows |= {r["name"] for r in wf.boot_rows
                         if any(r.get(k) for k in keys)}
            if rows is not None:
                rows |= {r["name"] for r in wf.boot_rows if named(yaml.dump(r))}
            self.direct[s] = (jobs, qt, rows)
        # Scripts that call or import another script pass its jobs on.
        self.callers = {s: set() for s in self.scripts}
        for s in self.scripts:
            rx = self._name_rx(s)
            for c in self.scripts:
                if c != s and rx.search(self.text[c]):
                    self.callers[s].add(c)

    @staticmethod
    def _name_rx(path):
        """A script's file name, or its stem as a Python import names it
        (`import ive_xnn`, `importlib` of "fake-ipcam")."""
        base = os.path.basename(path)
        stem = os.path.splitext(base)[0]
        names = {base}
        if base.endswith(".py"):
            names |= {stem, stem.replace("-", "_")}
        return re.compile(r"(?<![\w.\-])(?:%s)(?![\w\-])"
                          % "|".join(map(re.escape, sorted(names))))

    def reach(self, path):
        """(jobs, qtests, boot rows); rows None means every row."""
        jobs, qts, rows = set(), set(), set()
        seen, todo = set(), [path]
        while todo:
            s = todo.pop()
            if s in seen:
                continue
            seen.add(s)
            j, q, r = self.direct.get(s, (set(), set(), set()))
            jobs |= j
            qts |= q
            if r is None or rows is None:
                rows = None
            else:
                rows |= r
            if s == path and s not in self.direct:
                # New script: nothing names it yet.
                continue
            todo.extend(self.callers.get(s, ()))
        return jobs, qts, rows


# ── Classification ──────────────────────────────────────────────────────

class Decision:
    def __init__(self, wf):
        self.wf = wf
        self.families = set()
        self.jobs = set()
        self.qtests = set()
        self.all = False
        self.boot_names = set()
        self.why = []

    def widen(self, path, reason):
        self.all = True
        self.why.append((path, "ALL (%s)" % reason))

    def finish(self):
        wf = self.wf
        if self.all:
            self.families = set(FAMILIES)
        for job, fams in JOB_FAMILIES.items():
            if fams & self.families:
                self.jobs.add(job)
        for q in wf.qtests:
            if qtest_family(q) in self.families:
                self.qtests.add(q)
        rows = [r for r in wf.boot_rows
                if self.all or "boot" in self.jobs or r["group"] in self.families
                or r["name"] in self.boot_names]
        self.boot_rows = rows
        if rows:
            self.jobs.add("boot")
        else:
            self.jobs.discard("boot")
        self.build = bool(self.jobs or self.qtests)
        return self

    def outputs(self):
        alljobs = sorted(set(JOB_FAMILIES) | {"boot"})
        return {
            "build": "true" if self.build else "false",
            "jobs": json.dumps({j: j in self.jobs for j in alljobs},
                               separators=(",", ":")),
            "qtests": json.dumps({q: q in self.qtests for q in sorted(self.wf.qtests)},
                                 separators=(",", ":")),
            # Never an empty list: GitHub rejects a matrix with no rows even
            # when the job's `if:` skips it.  The `jobs` flag decides.
            "boot-matrix": json.dumps(self.boot_rows or self.wf.boot_rows[:1],
                                      separators=(",", ":")),
            "families": json.dumps(sorted(self.families)),
        }

    def report(self):
        lines = []
        lines.append("full suite: %s" % ("yes" if self.all else "no"))
        lines.append("families: %s" % (", ".join(sorted(self.families)) or "-"))
        lines.append("build: %s" % ("yes" if self.build else "no"))
        lines.append("jobs: %s" % (", ".join(sorted(self.jobs)) or "-"))
        lines.append("boot rows: %s" % (", ".join(r["name"] for r in self.boot_rows) or "-"))
        lines.append("qtests: %d of %d" % (len(self.qtests), len(self.wf.qtests)))
        lines.append("")
        lines.append("why:")
        for path, what in self.why:
            lines.append("  %-60s %s" % (path, what))
        return "\n".join(lines)


_GRAPHS = {}


def graphs(files):
    key = tuple(files)
    if key not in _GRAPHS:
        wf = Workflow()
        _GRAPHS[key] = (wf, DeviceGraph(files), ScriptGraph(files, wf))
    return _GRAPHS[key]


def show(rev, path):
    r = subprocess.run(["git", "show", "%s:%s" % (rev, path)], cwd=ROOT,
                       text=True, capture_output=True)
    return r.stdout if r.returncode == 0 else None


# Words of the cp / meson / Kconfig boilerplate setup.sh wraps around each
# model: a line of nothing else (a continuation line, a `fi`) is explained by
# the hunk it sits in.
SETUP_GLUE = set("""
QEMU_DIR SRC_DIR hw include misc i2c net ssi usb sd char audio intc timer gpio
display dma rtc watchdog arm mips nvram ide meson build Kconfig qemu
if then else elif fi for in do done while grep q E echo sed i s a cp mkdir p
added patched system_ss arm_ss specific_ss add when if_true files cfg k f
select config bool default y n depends on imply the to for and
""".split())


def setup_sh_diff(base, dev, d):
    """Narrow a qemu/setup.sh change to the families its hunks name.

    Nearly every new model also adds its cp line, meson line and Kconfig
    select to setup.sh, and those name the file or the family's symbol.  A
    changed line that names neither (the upstream patching, the configure
    line, the clone) widens to everything.  A build break shows in the build
    job, which every non-docs change runs.
    """
    r = subprocess.run(["git", "diff", "-U0", base, HEAD_REV, "--", "qemu/setup.sh"],
                       cwd=ROOT, text=True, capture_output=True)
    if r.returncode != 0:
        return False
    by_stem = {}
    for f, fams in dev.reach.items():
        stem = os.path.splitext(os.path.basename(f))[0]
        by_stem.setdefault(stem, set()).update(fams)
    # Per hunk, the words it adds or removes: appending one file to a meson
    # line that lists twenty must name that one file, not the twenty.
    word = re.compile(r"[A-Za-z0-9_+-]+(?:\.[ch])?")

    def recognise(w):
        stem = re.sub(r"\.[ch]$", "", w)
        if stem in by_stem:
            return by_stem[stem]
        fam = family_of_name(stem)
        if fam and (w != stem or any(stem.startswith(x) and x.endswith("-")
                                     for x in PREFIX_FAMILY)):
            # a file (xm-rtc2.c) or a marker (grep -q hisi-vi-fp)
            return {fam}
        fam = CONFIG_FAMILY.get(w) or CONFIG_FAMILY.get("CONFIG_" + w)
        return {fam} if fam else None
    hunks, cur = [], None
    for line in r.stdout.splitlines():
        if line.startswith("@@"):
            cur = {"+": [], "-": []}
            hunks.append(cur)
        elif cur is not None and line.startswith(("+", "-")) \
                and not line.startswith(("+++", "---")):
            body = line[1:].strip()
            if body and not body.startswith("#"):
                cur[line[0]].append(body)
    fams = set()
    for hunk in hunks:
        added = collections.Counter(word.findall("\n".join(hunk["+"])))
        removed = collections.Counter(word.findall("\n".join(hunk["-"])))
        changed = (added - removed) + (removed - added)
        if not changed:
            continue                # comments, whitespace or a reorder
        # Every changed line has to be explained on its own: by a model or
        # family symbol it names, or by being nothing but the shell and
        # meson glue around those.  One recognised word must not cover an
        # upstream sed patch that happens to sit in the same hunk.
        other = {"+": set(hunk["-"]), "-": set(hunk["+"])}
        for sign in "+-":
            for line in hunk[sign]:
                if line in other[sign]:
                    continue
                words = word.findall(line)
                if any(recognise(w) for w in words):
                    continue
                if all(w in SETUP_GLUE for w in words):
                    continue
                d.why.append(("qemu/setup.sh", "ALL (line names no model: %s)"
                              % line[:60]))
                return False
        found = set()
        for w in changed:
            found |= recognise(w) or set()
        if not found or ALL in found:
            d.why.append(("qemu/setup.sh", "ALL (hunk names no model: %s)"
                          % (hunk["+"] or hunk["-"])[0][:60]))
            return False
        fams |= found
    d.families |= fams
    d.why.append(("qemu/setup.sh", "families " + (", ".join(sorted(fams)) or "-")))
    return True


def workflow_diff(base, wf, d):
    """Narrow a ci.yml change to the jobs (and qtest steps) it changes."""
    old = show(base, WORKFLOW)
    if old is None:
        return False
    try:
        old = yaml.safe_load(old)
    except yaml.YAMLError:
        return False
    new = wf.doc if HEAD_REV == "HEAD" else yaml.safe_load(show(HEAD_REV, WORKFLOW) or "{}")
    for k in set(old) | set(new):
        if k != "jobs" and old.get(k) != new.get(k):
            d.why.append((WORKFLOW, "ALL (top-level %r changed)" % k))
            return False
    oj, nj = old.get("jobs", {}), new["jobs"]
    for job in sorted(set(oj) | set(nj)):
        if oj.get(job) == nj.get(job) or job not in nj:
            continue
        if job in ("select", "ci-gate"):
            d.why.append((WORKFLOW, "ALL (job %s changed)" % job))
            return False
        if job == "build" and job in oj:
            a = {k: v for k, v in oj[job].items() if k != "steps"}
            b = {k: v for k, v in nj[job].items() if k != "steps"}
            if a != b:
                d.why.append((WORKFLOW, "ALL (build job changed)"))
                return False
            # Only the qtest steps narrow.  Any other step added, removed,
            # changed or moved (the build itself, the uploads the other jobs
            # download) runs everything.
            def qtest(st):
                m = re.search(r"qemu/tests/[\w.\-]+", st.get("run", ""))
                return m.group(0) if m else None
            os_, ns_ = oj[job].get("steps", []), nj[job].get("steps", [])
            if [st for st in os_ if not qtest(st)] != [st for st in ns_ if not qtest(st)]:
                d.why.append((WORKFLOW, "ALL (a build step other than a qtest changed)"))
                return False
            before = [st for st in os_ if qtest(st)]
            for st in ns_:
                if qtest(st) and st not in before:
                    d.qtests.add(qtest(st))
                    d.why.append((WORKFLOW, "qtest %s" % qtest(st)))
            continue
        d.jobs.add(job)
        d.why.append((WORKFLOW, "job %s" % job))
    return True


def boot_matrix_diff(base, wf, d):
    """Narrow a boot-matrix.yml change to the rows it changes."""
    old = show(base, BOOT_MATRIX)
    if old is None:
        return False
    try:
        old = {r["name"]: r for r in yaml.safe_load(old)["include"]}
    except (yaml.YAMLError, KeyError, TypeError):
        return False
    for r in wf.boot_rows:
        if old.get(r["name"]) != r:
            d.boot_names.add(r["name"])
            d.why.append((BOOT_MATRIX, "row %s" % r["name"]))
    # A removed row: run the rest of its group, so the change still runs
    # something and the boot job itself is shown to work.
    names = {r["name"] for r in wf.boot_rows}
    for name, r in old.items():
        if name not in names:
            group = {x["name"] for x in wf.boot_rows if x["group"] == r.get("group")}
            if not group:
                d.why.append((BOOT_MATRIX, "ALL (removed row %s, its group is gone)" % name))
                return False
            d.boot_names |= group
            d.why.append((BOOT_MATRIX, "removed row %s: rows of %s" % (name, r.get("group"))))
    return True


# Shared files read with their diff when the base is known: narrowed by what
# the hunks change, or everything when they cannot tell.
DIFF_AWARE = {
    "qemu/setup.sh": lambda base, wf, dev, d: setup_sh_diff(base, dev, d),
    WORKFLOW: lambda base, wf, dev, d: workflow_diff(base, wf, d),
    BOOT_MATRIX: lambda base, wf, dev, d: boot_matrix_diff(base, wf, d),
}


def classify(paths, files=None, base=None):
    files = files if files is not None else tracked_files()
    wf, dev, scr = graphs(files)
    d = Decision(wf)
    for p in paths:
        if match_any(p, DOC_GLOBS):
            d.why.append((p, "nothing (docs)"))
        elif base and p in DIFF_AWARE:
            if not DIFF_AWARE[p](base, wf, dev, d):
                d.widen(p, "diff not narrowed")
        elif any(fnmatch.fnmatchcase(p, g) for g in ALL_GLOBS):
            d.widen(p, "build / workflow")
        elif p.startswith(DEVICE_DIRS):
            fams = dev.families(p)
            if ALL in fams:
                d.widen(p, "model no family claims")
            else:
                d.families |= fams
                d.why.append((p, "families " + ", ".join(sorted(fams))))
        elif p.startswith("qemu/tests/data/"):
            # Reference outputs read by the V5 IVE test.
            d.qtests.add("qemu/tests/test-hisi-ive-npu.py")
            d.why.append((p, "qtest test-hisi-ive-npu.py"))
        elif p.startswith(SCRIPT_DIRS):
            jobs, qts, rows = scr.reach(p)
            d.jobs |= jobs
            d.qtests |= qts
            what = sorted(jobs) + sorted(os.path.basename(q) for q in qts)
            if rows is None:
                d.jobs.add("boot")
                what.append("boot")
            elif rows:
                d.boot_names |= rows
                what.append("boot rows " + ",".join(sorted(rows)))
            d.why.append((p, ("runs " + ", ".join(what)) if what
                          else "nothing (no job runs it)"))
        else:
            d.widen(p, "unrecognised path")
    return d.finish()


def classify_all():
    wf = Workflow()
    d = Decision(wf)
    d.widen("-", "full run")
    return d.finish()


# ── Changed files from the GitHub event ─────────────────────────────────

def changed_from_event():
    """Return (changed paths, base revision), or None for the full suite."""
    event = os.environ.get("GITHUB_EVENT_NAME", "")
    if event in ("schedule", "workflow_dispatch"):
        print("event %s: full suite" % event)
        return None
    with open(os.environ["GITHUB_EVENT_PATH"]) as f:
        payload = json.load(f)
    if event == "pull_request":
        # actions/checkout checks out the merge commit; its first parent is
        # the base the PR merges into, so this is exactly what the PR adds.
        return sh("git", "diff", "--name-only", "HEAD^1", "HEAD").split(), "HEAD^1"
    if event == "push":
        before = payload.get("before", "")
        if payload.get("forced") or not before or set(before) == {"0"}:
            print("push without a usable 'before' (%s): full suite" % before)
            return None
        sh("git", "fetch", "--no-tags", "--depth=1", "origin", before, check=False)
        r = subprocess.run(["git", "diff", "--name-only", before, "HEAD"],
                           cwd=ROOT, text=True, capture_output=True)
        if r.returncode != 0:
            print("cannot diff %s..HEAD: full suite\n%s" % (before, r.stderr))
            return None
        return r.stdout.split(), before
    print("event %r: full suite" % event)
    return None


def emit(d):
    out = d.outputs()
    text = d.report()
    print(text)
    for k, v in out.items():
        print("%s=%s" % (k, v if len(v) < 400 else v[:400] + "..."))
    if os.environ.get("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a") as f:
            for k, v in out.items():
                f.write("%s=%s\n" % (k, v))
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as f:
            f.write("### CI selection\n\n```\n%s\n```\n" % text)


# ── Self-test ───────────────────────────────────────────────────────────

CONFIG_FAMILY = {
    "CONFIG_HISILICON": "hisilicon", "CONFIG_XIONGMAI": "xiongmai",
    "CONFIG_GOKE": "goke710x", "CONFIG_NOVATEK": "novatek",
    "CONFIG_INGENIC": "ingenic", "CONFIG_FULLHAN": "fullhan",
    "CONFIG_GRAINMEDIA": "grainmedia", "CONFIG_SIGMASTAR": "sigmastar",
    "CONFIG_ANYKA": "anyka", "CONFIG_AMBARELLA": "ambarella",
    "CONFIG_ALLWINNER_V83X": "allwinner", "CONFIG_ALLWINNER_CAM": "allwinner",
}


def self_test():
    errors = []
    files = tracked_files()
    wf, dev, _ = graphs(files)

    for job in wf.jobs:
        if job not in JOB_FAMILIES and job not in SPECIAL_JOBS:
            errors.append("ci.yml job %r is not in JOB_FAMILIES" % job)
    for job in list(JOB_FAMILIES) + list(SPECIAL_JOBS):
        if job not in wf.jobs:
            errors.append("JOB_FAMILIES names %r, which ci.yml does not have" % job)
    for job, fams in JOB_FAMILIES.items():
        for fam in fams - set(FAMILIES):
            errors.append("job %r: unknown family %r" % (job, fam))
        if job in wf.jobs:
            cond = str(wf.jobs[job].get("if", ""))
            if "'%s'" % job not in cond:
                errors.append("job %r has no `if:` on its selector output" % job)

    gate_needs = set(wf.jobs.get("ci-gate", {}).get("needs", []))
    for job in wf.jobs:
        if job != "ci-gate" and job not in gate_needs:
            errors.append("ci-gate does not need job %r, so its failure would "
                          "not block a merge" % job)

    for row in wf.boot_rows:
        if row.get("group") not in FAMILIES:
            errors.append("boot row %r: group %r is not a family"
                          % (row.get("name"), row.get("group")))

    for q, names in wf.qtests.items():
        if qtest_family(q) not in FAMILIES:
            errors.append("build step(s) %r: cannot tell the family of %s"
                          % (names, q))
    for step in wf.jobs["build"]["steps"]:
        m = re.search(r"qemu/tests/[\w.\-]+", step.get("run", ""))
        if m and m.group(0) not in str(step.get("if", "")):
            errors.append("build step %r has no `if:` on its qtest output"
                          % step.get("name"))
        if m and not os.path.exists(os.path.join(ROOT, m.group(0))):
            errors.append("build step %r runs missing %s"
                          % (step.get("name"), m.group(0)))

    # Every family is exercised by something, or narrowing to it would run
    # nothing at all.
    for fam in FAMILIES:
        if not (any(fam in f for f in JOB_FAMILIES.values())
                or any(qtest_family(q) == fam for q in wf.qtests)
                or any(r["group"] == fam for r in wf.boot_rows)):
            errors.append("family %r has no job or qtest" % fam)

    # Every model the build compiles under a family's Kconfig symbol must
    # reach that family.
    by_base = {}
    for f in dev.files:
        by_base.setdefault(os.path.basename(f), []).append(f)
    setup = read("qemu/setup.sh")
    for cfg, flist in re.findall(r"when: '(CONFIG_\w+)', if_true: files\(([^)]*)\)", setup):
        fam = CONFIG_FAMILY.get(cfg)
        if not fam:
            continue
        for base in re.findall(r"'([^']+)'", flist):
            for f in by_base.get(base, []):
                reach = dev.reach[f]
                if ALL not in reach and fam not in reach:
                    errors.append("%s is built under %s but reaches only %s"
                                  % (f, cfg, ", ".join(sorted(reach))))

    # Sanity of the classification on shapes we know.
    def expect(paths, want_all=None, has=(), lacks=(), build=None):
        d = classify(paths, files)
        tag = " ".join(paths)
        if want_all is not None and d.all != want_all:
            errors.append("%s: full suite %s, expected %s" % (tag, d.all, want_all))
        for j in has:
            if j not in d.jobs:
                errors.append("%s: expected job %s" % (tag, j))
        for j in lacks:
            if j in d.jobs:
                errors.append("%s: unexpected job %s" % (tag, j))
        if build is not None and d.build != build:
            errors.append("%s: build %s, expected %s" % (tag, d.build, build))

    expect(["README.md", "docs/novatek.md"], want_all=False, build=False)
    expect(["qemu/setup.sh"], want_all=True)
    expect(["qemu/patches/0001-arm-sp804-continuous-trigger.patch"], want_all=True)
    expect(["qemu/hw/misc/nvt-cg.c"], want_all=False, has=["boot-novatek"],
           lacks=["boot", "venc-hisilicon", "boot-ingenic"])
    expect(["qemu/hw/mips/ingenic.c"], want_all=False, has=["boot-ingenic"],
           lacks=["boot", "boot-novatek"])
    expect(["qemu/hw/arm/hisilicon.c"], want_all=False,
           has=["boot", "venc-hisilicon"], lacks=["boot-ingenic"])
    expect(["qemu-boot/test-novatek-boot.sh"], want_all=False,
           has=["boot-novatek"], lacks=["boot"])
    expect(["qemu-boot/run-ev300.sh"], want_all=False, build=False)

    for e in errors:
        print("self-test: " + e, file=sys.stderr)
    if errors:
        return 1
    print("self-test: ok (%d jobs, %d qtests, %d boot rows, %d models)"
          % (len(wf.jobs), len(wf.qtests), len(wf.boot_rows), len(dev.files)))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stdin", action="store_true")
    ap.add_argument("--base", metavar="REV",
                    help="with --stdin: the revision the change is against, "
                         "so setup.sh / ci.yml changes are read by their diff "
                         "(HEAD is the other side)")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--head", metavar="REV", default="HEAD",
                    help="with --stdin --base: the other side of the diff")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--explain", metavar="PATH")
    a = ap.parse_args()
    global HEAD_REV
    HEAD_REV = a.head

    if a.self_test:
        return self_test()
    if a.explain:
        files = tracked_files()
        if a.explain.startswith(DEVICE_DIRS):
            dev = DeviceGraph(files)
            print("reaches: %s" % ", ".join(sorted(dev.families(a.explain))))
            for u in sorted(dev.users.get(a.explain, ())):
                print("  named by %s" % u)
        print(classify([a.explain], files).report())
        return 0
    if a.all:
        emit(classify_all())
        return 0
    if a.stdin:
        paths = [l.strip() for l in sys.stdin if l.strip()]
        emit(classify(paths, base=a.base))
        return 0
    got = changed_from_event()
    emit(classify_all() if got is None else classify(got[0], base=got[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())

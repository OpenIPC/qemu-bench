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
VENC_MATRIX = ".github/ci/venc-matrix.yml"

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
    "vtund-neo": {"hisilicon"},
    "wifi-xiongmai": {"xiongmai"},
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
        self.type_macros = {}
        for f in self.files:
            self.type_macros.update(TYPE_DEFINE.findall(self.text[f]))
        self.reach = self._compute()

    def _exports(self, f):
        """Tokens by which other code names this file."""
        t = self.text[f]
        infos = "\n".join(TYPEINFO_BLOCK.findall(t))
        strings = set(TYPEINFO_NAME.findall(infos))
        macros = set(TYPEINFO_MACRO.findall(infos))
        # A type registered here through a macro another file defines
        # (hisi-f37.c registers TYPE_HISI_JXF22 from hisi-soi.h) is this
        # file's type too.
        strings |= {self.type_macros[m] for m in macros if m in self.type_macros}
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
        # The sensor catalogue lists every sensor model, and every machine
        # includes it, but being listed does not put a sensor on a machine:
        # sensor= or a default_sensor does.  So the catalogue's mention of a
        # model is not an edge; sensor_targets() finds who uses it.
        cat = sensor_catalogue(self.text.get(SENSOR_CATALOGUE, ""))
        self.sensor_names = {}
        users = {f: set() for f in self.files}
        for f in self.files:
            strings, macros, headers = self._exports(f)
            names = {n for n, t in cat if t in strings}
            if names and f != SENSOR_CATALOGUE:
                self.sensor_names[f] = names
            for g in self.files:
                if g == f or (names and g == SENSOR_CATALOGUE):
                    continue
                qs, ids, incs = mentions[g]
                if (strings & qs) or (macros & ids) or (headers & incs):
                    users[f].add(g)

        built = built_under()
        reach = {}
        for f in self.files:
            r = set()
            # A sensor model's file name says nothing about who uses it:
            # hisi-imx415 sits on Novatek and SigmaStar boards too.
            # A header reaches only through the files that include it:
            # hisi-soi.h is the Ingenic ISP's and the F37 model's, not a
            # HiSilicon machine's.
            if f not in self.sensor_names and not f.endswith(".h"):
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
        reach = {f: (r if r or f in self.sensor_names else {ALL})
                 for f, r in reach.items()}
        self.hisi = self._hisi_machines(reach)
        return reach

    def _hisi_machines(self, reach):
        """file -> the HiSilicon machines it is on, for files that reach
        HiSilicon; hisilicon.c's own edges are resolved per machine."""
        hm = HisiMachines()
        out = {f: set() for f in self.files}
        if HISI_MACHINES in out:
            out[HISI_MACHINES] = set(hm.all)
        for f in HISI_MACHINE_FILES & set(out):
            out[f] = set(hm.all)
        for f in self.files:
            if f not in HISI_MACHINE_FILES and HISI_MACHINE_FILES & self.users[f]:
                out[f] |= hm.for_tokens(*self._exports(f))
        changed = True
        while changed:
            changed = False
            for f in self.files:
                for u in self.users[f]:
                    if u not in HISI_MACHINE_FILES and not out[u] <= out[f]:
                        out[f] |= out[u]
                        changed = True
        for f in self.files:
            if "hisilicon" in reach[f] and not out[f]:
                out[f] = set(hm.all)        # nothing narrows it: all of them
        self.hisi_all = hm.all
        return out

    def sensor_users(self, path):
        """Sensor models that use this file, directly or through others
        (not through the catalogue, which reaches everything anyway)."""
        seen, todo, out = set(), [path], set()
        while todo:
            f = todo.pop()
            if f in seen:
                continue
            seen.add(f)
            if f != path and f in self.sensor_names:
                out.add(f)
            todo.extend(u for u in self.users.get(f, ()) if u != SENSOR_CATALOGUE)
        return out

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


SENSOR_CATALOGUE = "qemu/hw/i2c/image-sensor.c"
HISI_MACHINES = "qemu/hw/arm/hisilicon.c"
# hisilicon.c and its header: the config struct and the types it names.
HISI_MACHINE_FILES = {HISI_MACHINES, "qemu/include/hw/arm/hisilicon.h"}
CATALOGUE_ENTRY = re.compile(r'\{\s*"([\w.-]+)",\s*("[\w.-]+"|[A-Z_][A-Z0-9_]*),\s*0x[0-9A-Fa-f]+')


def sensor_catalogue(text):
    """[(catalogue name, QOM type)] from image-sensor.c's table, whose
    shared models are named through macros (SMARTSENS)."""
    macros = dict(re.findall(r'#define\s+([A-Z_][A-Z0-9_]*)\s+"([^"]+)"', text))
    out = []
    for name, t in CATALOGUE_ENTRY.findall(text):
        t = t.strip('"') if t.startswith('"') else macros.get(t)
        if t:
            out.append((name, t))
    # Entry macros: `#define GC8(n, a, i) { n, GALAXYCORE, ... }`, used as
    # GC8("gc2053", 0x37, ...).
    for mac, t in re.findall(r'#define\s+([A-Z_][A-Z0-9_]*)\(\s*n\b[^)]*\)\s*\{\s*n,\s*([A-Z_][A-Z0-9_]*)', text):
        if t in macros:
            for name in re.findall(r'\b%s\(\s*"([\w.-]+)"' % re.escape(mac), text):
                out.append((name, macros[t]))
    return out


def hisi_default_sensors():
    """HiSilicon machine name -> its config's default_sensor."""
    t = read(HISI_MACHINES)
    defaults = {}
    for name, body in re.findall(r"static const HisiSoCConfig (\w+) = \{(.*?)\n\};", t, re.S):
        m = re.findall(r'\.default_sensor\s*=\s*"([^"]+)"', body)
        if m:
            defaults[name] = m[-1]
    return {mach: defaults[cfg] for mach, cfg in
            re.findall(r'DEFINE_HISI_MACHINE\("([\w-]+)",\s*\w+,\s*(\w+)\)', t)
            if cfg in defaults}


def word_rx(words):
    return re.compile(r"(?<![\w-])(?:%s)(?![\w-])"
                      % "|".join(map(re.escape, sorted(words))))


class HisiMachines:
    """Which HiSilicon machines instantiate a model.

    hisilicon_common_init() builds every SoC from its HisiSoCConfig, so a
    model is on a machine either because the config names its type
    (`.fmc_type = "hisi-sfc350"`, a regbank `{ "hisi-vedu", ... }`, possibly
    through a family macro such as HISI_V4_VIDEO_PIPE), or because the code
    creating it is guarded by a config field (`if (c->ive_npu_base)`) that
    the config sets.  Code with no such guard is on every machine.
    """

    def __init__(self, text=None):
        t = self.text = read(HISI_MACHINES) if text is None else text
        macros = {}
        for m in re.finditer(r"#define\s+(\w+)((?:[^\n]*\\\n)*[^\n]*)", t):
            macros[m.group(1)] = (m.start(), m.end(), m.group(2))
        self.macros = macros
        self.configs = {}
        for m in re.finditer(r"static const HisiSoCConfig (\w+) = \{(.*?)\n\};", t, re.S):
            self.configs[m.group(1)] = (m.start(), m.end(), self._expand(m.group(2)))
        self.machine_of = {}
        for mach, cfg in re.findall(r'DEFINE_HISI_MACHINE\("([\w-]+)",\s*\w+,\s*(\w+)\)', t):
            self.machine_of.setdefault(cfg, set()).add(mach)
        self.all = set().union(*self.machine_of.values()) if self.machine_of else set()

    def _expand(self, body, depth=0):
        if depth > 4:
            return body
        extra = [self._expand(v[2], depth + 1) for k, v in self.macros.items()
                 if re.search(r"\b%s\b" % re.escape(k), body)]
        return body + "\n" + "\n".join(extra)

    def at_line(self, ln):
        """Machines a change to line ln (1-based) of this text affects: the
        config block or family macro it is in, or the DEFINE_HISI_MACHINE on
        it; None when it is shared code (every machine)."""
        lines = self.text.split("\n")
        if ln < 1 or ln > len(lines):
            return None
        line = lines[ln - 1].strip()
        if not line or line.startswith(("/*", "*", "//")):
            return set()                # a comment
        m = re.match(r'DEFINE_HISI_MACHINE\("([\w-]+)"', line)
        if m:
            return {m.group(1)}
        pos = sum(len(x) + 1 for x in lines[:ln - 1])
        for c, (a, b, _) in self.configs.items():
            if a <= pos < b:
                return self._machines([c])
        for k, (a, b, _) in self.macros.items():
            if a <= pos < b:
                users = {c for c, (_, _, body) in self.configs.items()
                         if re.search(r"\b%s\b" % re.escape(k), body)}
                # A family macro (HISI_V4_COMMON_PERIPH) is the configs that
                # expand it.  A constant the shared code uses too
                # (HISI_PPI_PHYSTIMER, HISI_MAX_SMP), or no config uses, is
                # every machine's.
                if not users or self._used_in_code(k):
                    return None
                return self._machines(users)
        return None

    def _used_in_code(self, name):
        """Named by the shared code itself, outside configs, macros and
        comments."""
        if not hasattr(self, "_comments"):
            self._comments = [(m.start(), m.end()) for m in
                              re.finditer(r"/\*.*?\*/|//[^\n]*", self.text, re.S)]
        rx = re.compile(r"\b%s\b" % re.escape(name))
        for m in rx.finditer(self.text):
            pos = m.start()
            if any(a <= pos < b for a, b in self._comments):
                continue
            if any(a <= pos < b for a, b, _ in self.configs.values()):
                continue
            if any(a <= pos < b for a, b, _ in self.macros.values()):
                continue
            return True
        return False

    def _machines(self, cfgs):
        out = set()
        for c in cfgs:
            out |= self.machine_of.get(c, set())
        return out

    def _sets_field(self, field):
        rx = re.compile(r"\.%s\s*=\s*(?!0\s*[,}]|0x0+\s*[,}]|NULL\b|false\b)" % re.escape(field))
        return {c for c, (_, _, body) in self.configs.items() if rx.search(body)}

    def for_tokens(self, strings, macros, headers):
        """Machines on which the code naming these tokens runs."""
        t = self.text
        pats = ['"%s"' % re.escape(x) for x in strings] + \
               [r"\b%s\b" % re.escape(x) for x in macros] + \
               [r'#include\s+"[^"]*\b%s"' % re.escape(x) for x in headers]
        if not pats:
            return set()
        out = set()
        lines = t.split("\n")
        for m in re.finditer("|".join(pats), t):
            pos = m.start()
            cfg = [c for c, (a, b, _) in self.configs.items() if a <= pos < b]
            if cfg:
                out |= self._machines(cfg)
                continue
            mac = [k for k, (a, b, _) in self.macros.items() if a <= pos < b]
            if mac:
                if mac[0].startswith("TYPE_"):
                    continue        # the #define itself, not a use
                users = {c for c, (_, _, body) in self.configs.items()
                         if re.search(r"\b%s\b" % re.escape(mac[0]), body)}
                out |= self._machines(users)
                continue
            ln = t.count("\n", 0, pos)
            col = pos - (t.rfind("\n", 0, pos) + 1)
            if re.search(r"str(?:n|case)?cmp\s*\([^;]*$", lines[ln][:col]):
                continue            # a test of the type, not its creation
            ind = len(lines[ln]) - len(lines[ln].lstrip())
            # Walk out through the enclosing blocks: each opener is the
            # nearest line above that is indented less.  The innermost one
            # that tests a config field -- `if (c->ive_npu_base)`,
            # `for (n = 0; n < c->num_himci; n++)` -- guards the creation.
            # Reaching the function's own line means it is unguarded.
            field = None
            for k in range(ln - 1, -1, -1):
                line = lines[k]
                stripped = line.strip()
                if not stripped or stripped.startswith(("/*", "*", "//", "#")):
                    continue
                # `}` closes a block; `} else ...` opens the branch we may be in.
                if stripped.startswith("}") and not re.match(r"\}\s*else\b", stripped):
                    continue
                i = len(line) - len(line.lstrip())
                if i >= ind:
                    continue
                ind = i
                # `} else {` is a branch whose own `if` tests something
                # else: walking on out treats it as unguarded (a superset).
                g = re.match(r"\s*(?:\}\s*else\s+)?if\s*\(\s*(?:c|cfg|config)->(\w+)", line) or \
                    re.match(r"\s*for\s*\([^;]*;[^;]*<\s*(?:c|cfg|config)->(\w+)", line)
                if g:
                    field = g.group(1)
                    break
                if i == 0:
                    break
            if field:
                out |= self._machines(self._sets_field(field))
            else:
                return set(self.all)
        return out


# ── Workflow structure ──────────────────────────────────────────────────

class Workflow:
    def __init__(self):
        self.doc = yaml.safe_load(read(WORKFLOW))
        self.jobs = self.doc["jobs"]
        self.boot_rows = yaml.safe_load(read(BOOT_MATRIX))["include"]
        self.venc_rows = yaml.safe_load(read(VENC_MATRIX))["include"]
        self._hisi_all = None
        # Build-job steps that run one qemu/tests script, keyed by the script.
        self.qtests = {}
        for step in self.jobs["build"]["steps"]:
            run = step.get("run", "")
            m = re.search(r"qemu/tests/[\w.\-]+", run)
            if m:
                self.qtests.setdefault(m.group(0), [])
                self.qtests[m.group(0)].append(step.get("name", ""))

    def job_text(self, job):
        t = yaml.dump(self.jobs[job])
        if job == "venc-hisilicon":
            t += read(VENC_MATRIX)
        return t

    def job_machines(self, job):
        """HiSilicon machines a job's own YAML names (`-M hi3516ev300`)."""
        if self._hisi_all is None:
            self._hisi_all = HisiMachines().all
        text = self.job_text(job)
        return {m for m in self._hisi_all if re.search(r"(?<![\w-])%s(?![\w-])" % re.escape(m), text)}


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
        self._corpora()

    def _corpora(self):
        """Each job's text with every script it runs (transitively), and
        the same for qtests and the boot rows' shared steps."""
        wf = self.wf
        self.corpus = {j: yaml.dump(v) for j, v in wf.jobs.items()}
        self.qtest_corpus = {q: self.text.get(q, read(q)) for q in wf.qtests}
        self.boot_corpus = ""
        for s in self.scripts:
            jobs, qts, rows = self.reach(s)
            for j in jobs:
                self.corpus[j] += self.text[s]
            for q in qts:
                self.qtest_corpus[q] += self.text[s]
            if rows is None:
                self.boot_corpus += self.text[s]

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
        self.hisi_machines = set()
        self.venc_socs = set()          # VENC rows chosen by name alone
        self.venc_whole = False
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
            elif "hisilicon" in fams and self.hisi_machines and job != "venc-hisilicon":
                # Only the jobs that run one of the machines the change is on.
                jm = wf.job_machines(job)
                if not jm or jm & self.hisi_machines:
                    self.jobs.add(job)
        for q in wf.qtests:
            if qtest_family(q) in self.families or \
                    (self.hisi_machines and qtest_family(q) == "hisilicon"):
                self.qtests.add(q)
        whole_venc = self.all or self.venc_whole or "hisilicon" in self.families \
            or "venc-hisilicon" in self.jobs
        self.venc_rows = [r for r in wf.venc_rows
                          if whole_venc or r["soc"] in self.hisi_machines
                          or r["soc"] in self.venc_socs]
        if self.venc_rows:
            self.jobs.add("venc-hisilicon")
        else:
            self.jobs.discard("venc-hisilicon")
        rows = [r for r in wf.boot_rows
                if self.all or "boot" in self.jobs or r["group"] in self.families
                or r["name"] in self.boot_names
                or (r["group"] == "hisilicon"
                    and r.get("machine", "").split(",")[0] in self.hisi_machines)]
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
            "venc-matrix": json.dumps(self.venc_rows or self.wf.venc_rows[:1],
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
        lines.append("venc rows: %s" % (", ".join(r["soc"] for r in self.venc_rows) or "-"))
        lines.append("qtests: %d of %d" % (len(self.qtests), len(self.wf.qtests)))
        lines.append("")
        lines.append("why:")
        for path, what in self.why:
            lines.append("  %-60s %s" % (path, what))
        return "\n".join(lines)


def sensor_targets(path, dev, scr, wf, d):
    sensor_targets_names(path, dev.sensor_names[path],
                         {f for f in dev.reach[path] if f != ALL},
                         dev, scr, wf, d)


def sensor_targets_names(label, names, fams, dev, scr, wf, d):
    """A sensor model runs what can put that sensor on a bus.

    Those are: the families whose own code names the model (novatek.c's
    sensor table); the families whose machine file names one of its
    catalogue names (a default_sensor), except HiSilicon, whose machines are
    resolved one by one; the HiSilicon machines that default to it, through
    the boot rows that run them without a sensor= of their own and the jobs
    whose workflow text or scripts name them; and any job, qtest or boot row
    that names the sensor itself (sensor=sc2315e).
    """
    fams = set(fams)
    name_rx = word_rx(names)
    for f in dev.files:
        if f.startswith(("qemu/hw/arm/", "qemu/hw/mips/")) and f != HISI_MACHINES \
                and name_rx.search(dev.text[f]):
            fams |= {fam for fam in dev.reach[f] if fam != ALL}
    machines = {m for m, n in hisi_default_sensors().items() if n in names}
    # A job that starts one of those machines itself, with no sensor= of its
    # own, gets the default (ive-test's -M hi3516ev300).  Machine names in
    # its scripts do not count: they pass a sensor= or only download.
    # A line of the job's own YAML that names the machine and no sensor=
    # starts it with the default: `-M hi3516ev300`, `for mach in hi3516cv608
    # hi3516cv610`.  `soc:` matrix values are not machines: those jobs'
    # scripts pass their sensor (VENC runs sensor=sc2315e).
    mach_rx = word_rx(machines) if machines else None

    def starts_default(job):
        for line in yaml.dump(wf.jobs[job], width=1 << 20).splitlines():
            if mach_rx.search(line) and "sensor=" not in line \
                    and not re.match(r"\s*-?\s*soc:", line):
                return True
        return False

    jobs, qts, rows = set(), set(), set()
    venc = set()
    for job, text in scr.corpus.items():
        if job not in JOB_FAMILIES:
            continue
        if job == "venc-hisilicon":
            # By row: a row that names the sensor (`wide: imx335`) or is a
            # machine it is the default of.  Only when the job's scripts
            # name it and no row does is every row a candidate.
            for r in wf.venc_rows:
                if name_rx.search(yaml.dump(r)) or r["soc"] in machines:
                    venc.add(r["soc"])
            if not venc and name_rx.search(text):
                venc = {r["soc"] for r in wf.venc_rows}
            continue
        if name_rx.search(text) or (mach_rx and starts_default(job)):
            jobs.add(job)
    # A qtest that names the sensor in any case (comments say "IMX335"), or
    # starts one of its default machines without a sensor= of its own
    # (test-hisi-vpss.py: QTest(..., 'hi3516ev300')).
    iname_rx = re.compile(name_rx.pattern, re.I)
    for q in wf.qtests:
        corpus = scr.qtest_corpus.get(q, "")
        if iname_rx.search(corpus) or (mach_rx and any(
                mach_rx.search(line) and "sensor=" not in line
                for line in corpus.splitlines())):
            qts.add(q)
    for r in wf.boot_rows:
        mach = r.get("machine", "")
        base = mach.split(",")[0]
        if (base in machines and "sensor=" not in mach) or \
                name_rx.search(yaml.dump(r)) or name_rx.search(scr.boot_corpus):
            rows.add(r["name"])
    d.families |= fams
    d.jobs |= jobs
    d.qtests |= qts
    d.boot_names |= rows
    d.venc_socs |= venc
    if venc:
        jobs = jobs | {"venc-hisilicon (%s)" % ",".join(sorted(venc))}
    what = []
    if fams:
        what.append("families " + ", ".join(sorted(fams)))
    if machines:
        what.append("default on " + ", ".join(sorted(machines)))
    what += sorted(jobs) + sorted(os.path.basename(q) for q in qts)
    if rows:
        what.append("boot rows " + ",".join(sorted(rows)))
    d.why.append((label, "sensor %s: %s" % (",".join(sorted(names)),
                                          "; ".join(what) or "nothing runs it")))


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


def catalogue_diff(base, wf, dev, d):
    """An image-sensor.c change that only adds, removes or edits catalogue
    entries runs what those sensors reach; anything else (the interface,
    image_sensor_create()) reaches every family."""
    r = subprocess.run(["git", "diff", "-U0", base, HEAD_REV, "--", SENSOR_CATALOGUE],
                       cwd=ROOT, text=True, capture_output=True)
    if r.returncode != 0:
        return False
    _, _, scr = graphs(tracked_files())
    entry = re.compile(r'^\s*(?:\{\s*"([\w.-]+)",|[A-Z_][A-Z0-9_]*\(\s*"([\w.-]+)",)')
    names = set()
    for line in r.stdout.splitlines():
        if line.startswith(("+++", "---")) or not line.startswith(("+", "-")):
            continue
        body = line[1:].strip()
        if not body or body.startswith(("/*", "*", "//")):
            continue
        m = entry.match(body)
        if not m:
            d.why.append((SENSOR_CATALOGUE, "ALL (not a catalogue entry: %s)" % body[:50]))
            return False
        names.add(m.group(1) or m.group(2))
    if names:
        models = {f for f, n in dev.sensor_names.items() if n & names}
        fams = set()
        for f in models:
            fams |= {x for x in dev.reach[f] if x != ALL}
        sensor_targets_names(SENSOR_CATALOGUE, names, fams, dev, scr, wf, d)
    return True


def venc_matrix_diff(base, wf, d):
    """A venc-matrix.yml change runs the rows it changes."""
    old = show(base, VENC_MATRIX)
    if old is None:
        return False
    try:
        old = {r["soc"]: r for r in yaml.safe_load(old)["include"]}
    except (yaml.YAMLError, KeyError, TypeError):
        return False
    for r in wf.venc_rows:
        if old.get(r["soc"]) != r:
            # This row only: not the other jobs that boot the same SoC.
            d.venc_socs.add(r["soc"])
            d.why.append((VENC_MATRIX, "row %s" % r["soc"]))
    if set(old) - {r["soc"] for r in wf.venc_rows}:
        if not wf.venc_rows:
            return False
        d.venc_whole = True             # a removed row: run the rest
        d.why.append((VENC_MATRIX, "removed row(s): every remaining row"))
    return True


def hisi_machine_diff(base, wf, dev, d):
    """A hisilicon.c change that stays inside SoC configs (or the family
    macros they expand, or DEFINE_HISI_MACHINE lines) runs those SoCs'
    machines; one that touches the shared init code runs them all."""
    r = subprocess.run(["git", "diff", "-U0", base, HEAD_REV, "--", HISI_MACHINES],
                       cwd=ROOT, text=True, capture_output=True)
    old_t, new_t = show(base, HISI_MACHINES), show(HEAD_REV, HISI_MACHINES)
    if r.returncode != 0 or old_t is None or new_t is None:
        return False
    old_m, new_m = HisiMachines(old_t), HisiMachines(new_t)
    machines = set()
    for h in re.finditer(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@", r.stdout, re.M):
        o, on = int(h.group(1)), int(h.group(2) or 1)
        n, nn = int(h.group(3)), int(h.group(4) or 1)
        spans = [(old_m, ln) for ln in range(o, o + on)] + \
                [(new_m, ln) for ln in range(n, n + nn)]
        for hm, ln in spans:
            got = hm.at_line(ln)
            if got is None:
                d.families.add("hisilicon")
                d.why.append((HISI_MACHINES, "hisilicon (shared code at line %d)" % ln))
                return True
            machines |= got
    d.hisi_machines |= machines
    d.why.append((HISI_MACHINES, "hisilicon on " + (",".join(sorted(machines)) or "-")))
    return True


# Shared files read with their diff when the base is known: narrowed by what
# the hunks change, or everything when they cannot tell.
DIFF_AWARE = {
    "qemu/setup.sh": lambda base, wf, dev, d: setup_sh_diff(base, dev, d),
    WORKFLOW: lambda base, wf, dev, d: workflow_diff(base, wf, d),
    BOOT_MATRIX: lambda base, wf, dev, d: boot_matrix_diff(base, wf, d),
    VENC_MATRIX: lambda base, wf, dev, d: venc_matrix_diff(base, wf, d),
    SENSOR_CATALOGUE: catalogue_diff,
    HISI_MACHINES: hisi_machine_diff,
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
        elif p in dev.sensor_names:
            sensor_targets(p, dev, scr, wf, d)
        elif p.startswith(DEVICE_DIRS):
            fams = set(dev.families(p))     # a copy: the graph keeps its own
            sensors = dev.sensor_users(p) if p in dev.reach else set()
            if ALL in fams and not sensors:
                d.widen(p, "model no family claims")
            else:
                fams.discard(ALL)
                what = ", ".join(sorted(fams - {"hisilicon"})) or "-"
                mach = dev.hisi.get(p, set())
                if "hisilicon" in fams and mach and mach != dev.hisi_all:
                    fams.discard("hisilicon")
                    d.hisi_machines |= mach
                    what += "; hisilicon on " + ",".join(sorted(mach))
                elif "hisilicon" in fams:
                    what = ", ".join(sorted(fams))
                d.families |= fams
                d.why.append((p, "families " + what))
                for sm in sorted(sensors):
                    sensor_targets(sm, dev, scr, wf, d)
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


# ── Check against a built QEMU ──────────────────────────────────────────

def check_machines(qemu_dir):
    """Boot every HiSilicon machine to its device tree (-S, `info qtree`)
    and fail when a model sits on a machine the selector does not derive
    for it: then a change to that model would skip a job that runs it.
    Sensors are checked against default_sensor, which is how they get there.
    """
    files = tracked_files()
    _, dev, _ = graphs(files)
    hm = HisiMachines()
    typefile = {}
    for f in dev.files:
        for t in dev._exports(f)[0]:
            typefile.setdefault(t, f)
    defaults = hisi_default_sensors()
    cat = dict(sensor_catalogue(read(SENSOR_CATALOGUE)))
    errors = []
    for m in sorted(hm.all):
        qtree = ""
        for arch in ("arm", "aarch64"):
            exe = os.path.join(qemu_dir, "qemu-system-" + arch)
            if not os.path.exists(exe):
                continue
            try:
                r = subprocess.run([exe, "-M", m, "-S", "-display", "none",
                                    "-serial", "null", "-nodefaults", "-monitor", "stdio"],
                                   input="info qtree\nquit\n", capture_output=True,
                                   text=True, timeout=60)
            except subprocess.TimeoutExpired:
                continue
            if "dev: " in r.stdout:
                qtree = r.stdout
                break
        if not qtree:
            errors.append("%s: no device tree from QEMU" % m)
            continue
        for t in sorted(set(re.findall(r"dev: ([\w.-]+),", qtree))):
            f = typefile.get(t)
            if not f or "hisilicon" not in dev.reach.get(f, ()) and f not in dev.sensor_names:
                continue
            if f in dev.sensor_names:
                if cat.get(defaults.get(m)) != t:
                    errors.append("%s: QEMU puts %s on it, but its default_sensor "
                                  "does not name it" % (m, t))
            elif m not in dev.hisi.get(f, set()):
                errors.append("%s: QEMU puts %s (%s) on it, but the selector "
                              "does not derive that machine for it" % (m, t, f))
    for e in errors:
        print("check-machines: " + e, file=sys.stderr)
    if errors:
        return 1
    print("check-machines: ok (%d machines)" % len(hm.all))
    return 0


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

    hisi_all = HisiMachines().all
    for row in wf.venc_rows:
        if row.get("soc") not in hisi_all:
            errors.append("venc row %r is not a HiSilicon machine" % row.get("soc"))

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
    # A sensor runs where it can be attached, not everywhere the catalogue
    # is included: the IMX415 is only AV300's default and novatek's option.
    expect(["qemu/hw/i2c/hisi-imx415.c"], want_all=False,
           has=["maskrom-av300"],
           lacks=["venc-hisilicon", "boot", "boot-ingenic", "sofia-isp"])
    # The SmartSens model is the sensor VENC and Sofia run (sensor=sc2315e),
    # and the cv6xx default.
    expect(["qemu/hw/i2c/hisi-smartsens.c"], want_all=False,
           has=["venc-hisilicon", "sofia-isp", "boot-hi3516cv610"])
    # A HiSilicon model runs only the jobs and rows of the machines that
    # create it: the HiSFC350 is on the V1/V2 parts, none of the VENC SoCs.
    expect(["qemu/hw/misc/hisi-sfc350.c"], want_all=False, has=["boot"],
           lacks=["venc-hisilicon", "sofia-isp", "ive-test"])
    expect(["qemu/hw/misc/hisi-ive-npu.c"], want_all=False,
           has=["boot-hi3516cv610"], lacks=["boot", "sofia-isp", "ive-test"])
    d = classify(["qemu/hw/misc/hisi-ive-npu.c"], files)
    if [r["soc"] for r in d.venc_rows] != ["hi3516cv610"]:
        errors.append("hisi-ive-npu.c: venc rows %s, expected hi3516cv610 only"
                      % [r["soc"] for r in d.venc_rows])
    # The F37 file registers JX-F22 through hisi-soi.h's macro, and the
    # Ingenic qtest starts t20,sensor=jxf22.
    expect(["qemu/hw/i2c/hisi-f37.c"], want_all=False)
    d = classify(["qemu/hw/i2c/hisi-f37.c"], files)
    if "qemu/tests/test-ingenic.py" not in d.qtests:
        errors.append("hisi-f37.c: test-ingenic.py (sensor=jxf22) not selected")
    # The IMX335 is EV300's default and a `wide:` sensor of its VENC row only.
    d = classify(["qemu/hw/i2c/hisi-imx335.c"], files)
    if [r["soc"] for r in d.venc_rows] != ["hi3516ev300"]:
        errors.append("hisi-imx335.c: venc rows %s, expected hi3516ev300"
                      % [r["soc"] for r in d.venc_rows])
    if "qemu/tests/test-hisi-vpss.py" not in d.qtests:
        errors.append("hisi-imx335.c: test-hisi-vpss.py (boots hi3516ev300) not selected")
    # A shared constant is every machine's, not the configs that use it.
    hm = HisiMachines()
    for k in ("HISI_PPI_PHYSTIMER", "HISI_MAX_SMP"):
        if k in hm.macros:
            ln = hm.text.count("\n", 0, hm.macros[k][0]) + 1
            if hm.at_line(ln) is not None:
                errors.append("%s: a change to it does not run every machine" % k)
    # ...but a family macro is the configs that expand it, comments or not.
    if "HISI_V4_COMMON_PERIPH" in hm.macros:
        ln = hm.text.count("\n", 0, hm.macros["HISI_V4_COMMON_PERIPH"][0]) + 1
        got = hm.at_line(ln)
        if got is None or got == hm.all:
            errors.append("HISI_V4_COMMON_PERIPH: resolves to every machine")
    if not hisi_default_sensors():
        errors.append("no default_sensor found in %s" % HISI_MACHINES)
    if not sensor_catalogue(read(SENSOR_CATALOGUE)):
        errors.append("no entries found in %s's catalogue" % SENSOR_CATALOGUE)

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
    ap.add_argument("--check-machines", metavar="QEMU_BUILD_DIR",
                    help="check the derived HiSilicon machine map against "
                         "the device trees a built QEMU creates")
    a = ap.parse_args()
    global HEAD_REV
    HEAD_REV = a.head

    if a.self_test:
        return self_test()
    if a.check_machines:
        return check_machines(a.check_machines)
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

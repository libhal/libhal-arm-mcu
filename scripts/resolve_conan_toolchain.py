#!/usr/bin/env python3
# Copyright 2026 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Discover every Conan-built binary under a directory tree and regenerate
VSCode's .vscode/{c_cpp_properties,launch,tasks}.json to point at them.

Conan 2's local package cache (~/.conan2/p/...) uses short, non-deterministic
per-package hash folders that change whenever a package gets rebuilt or
re-downloaded -- hand-pasting one into launch.json rots the moment that
happens. Conan already writes the correct, current absolute paths into every
build's own generators/conan_toolchain.cmake (CMAKE_PROGRAM_PATH); this
script reads that instead of guessing, so it stays correct across cache
rebuilds and across the many different toolchain versions/platforms a
library's demos/ and test_package/ sub-builds tend to accumulate.

Unlike a single-binary application, a library repo's demos/ produces *many*
executables per build folder (one per demos/applications/*.cpp), and a
platform library like this one accumulates many build folders (one per
platform x compiler x build type). This script therefore:

  1. Finds every generators/conan_toolchain.cmake under --root.
  2. For each, resolves the exact compiler + debugger it used.
  3. Finds every real ELF executable in that build folder by reading the
     ELF header itself (e_type == ET_EXEC/ET_DYN) rather than assuming a
     naming convention -- test_package's output isn't even named *.elf.
  4. Emits one launch.json config per (build folder, executable) pair.

Run any time after a `conan build` (new platform, new build type, or the
Conan cache was cleaned) via the "Resolve Conan Toolchain" VSCode task, or
directly:

    python3 scripts/resolve_conan_toolchain.py [--root demos]
"""

from __future__ import annotations

import argparse
import json
import re
import struct
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# This project's own LLVM builds tool_require the "arm-embedded" variant of
# llvm-toolchain, which ships only clang/lld/binutils -- no debugger. GDB
# from the GCC-based toolchain debugs a clang-built ELF just fine (it only
# reads DWARF), so that's the fallback whenever a build's own toolchain bin
# directory has no gdb of its own.
FALLBACK_GDB_RECIPE = "multiarch-gnu-toolchain"
FALLBACK_GDB_VARIANT_CONF = "user.multiarch-gnu-toolchain:variant"
FALLBACK_GDB_VARIANT = "arm-none-eabi"
GDB_CANDIDATE_NAMES = ["arm-none-eabi-gdb", "gdb-multiarch", "gdb"]

# pyOCD's target pack has no dedicated stm32f103c8 entry -- stm32f103rc is
# the confirmed-working target for that chip. Extend this as other chips
# turn out to need a different pyOCD target name than their libhal platform
# name.
PYOCD_TARGET_OVERRIDES = {
    "stm32f103c8": "stm32f103rc",
}
PYOCD_SERVER_ARGS = ["--semihost", "-Osemihost_console_type=True"]


def parse_cmake_list_prepend(text: str, variable: str) -> list[str]:
    pattern = re.compile(
        r"list\(PREPEND\s+" + re.escape(variable) + r"\s+(.*?)\)", re.DOTALL
    )
    paths: list[str] = []
    for match in pattern.finditer(text):
        paths.extend(re.findall(r'"([^"]*)"', match.group(1)))
    return paths


def parse_cmake_set(text: str, variable: str) -> str | None:
    matches = re.findall(
        r'set\(' + re.escape(variable) + r'\s+"?([^")\s]+)"?\)', text
    )
    return matches[-1] if matches else None


def is_executable_elf(path: Path) -> bool:
    """True for a linked ELF binary (ET_EXEC/ET_DYN); false for .o/.a/etc.

    Reads the ELF header directly instead of trusting a naming convention --
    demos name their output <driver>.elf, but test_package's own CMakeLists
    just calls it "test_package" with no extension.
    """
    try:
        with open(path, "rb") as f:
            header = f.read(20)
    except OSError:
        return False
    if len(header) < 20 or header[:4] != b"\x7fELF":
        return False
    is_le = header[5] == 1
    e_type = struct.unpack_from("<H" if is_le else ">H", header, 16)[0]
    return e_type in (2, 3)  # ET_EXEC, ET_DYN


def find_build_dirs(root: Path) -> list[Path]:
    return sorted(
        p.parent.parent
        for p in root.rglob("generators/conan_toolchain.cmake")
    )


def find_executables(build_dir: Path) -> list[Path]:
    return sorted(
        p for p in build_dir.iterdir()
        if p.is_file() and is_executable_elf(p)
    )


class ToolchainInfo:
    def __init__(self, build_dir: Path):
        text = (build_dir / "generators" / "conan_toolchain.cmake").read_text()
        self.build_dir = build_dir
        self.is_cross = "set(CMAKE_SYSTEM_NAME Generic)" in text
        self.cxx_compiler = parse_cmake_set(text, "CMAKE_CXX_COMPILER")
        self.cxx_standard = parse_cmake_set(text, "CMAKE_CXX_STANDARD") or "23"
        self.program_paths = parse_cmake_list_prepend(text, "CMAKE_PROGRAM_PATH")
        self.include_paths = parse_cmake_list_prepend(text, "CMAKE_INCLUDE_PATH")
        self.compiler_dir = self._find_compiler_dir()

    def _find_compiler_dir(self) -> Path | None:
        if not self.cxx_compiler:
            return None
        for candidate in self.program_paths:
            if (Path(candidate) / self.cxx_compiler).exists():
                return Path(candidate)
        return None

    @property
    def compiler_path(self) -> Path | None:
        if self.compiler_dir is None or self.cxx_compiler is None:
            return None
        return self.compiler_dir / self.cxx_compiler

    def find_local_gdb(self) -> Path | None:
        if self.compiler_dir is None:
            return None
        for name in GDB_CANDIDATE_NAMES:
            candidate = self.compiler_dir / name
            if candidate.exists():
                return candidate
        return None


_fallback_gdb_cache: dict[str, str | None] = {}


def resolve_fallback_gdb() -> str | None:
    """Resolve arm-none-eabi-gdb via `conan cache path` (memoized per run).

    Used only when a build's own toolchain (e.g. LLVM's arm-embedded fork)
    doesn't ship a debugger itself.
    """
    if "gdb" in _fallback_gdb_cache:
        return _fallback_gdb_cache["gdb"]

    result: str | None = None
    try:
        listing = subprocess.run(
            ["conan", "list", f"{FALLBACK_GDB_RECIPE}/*:*", "--format=json"],
            check=True, capture_output=True, text=True,
        )
        data = json.loads(listing.stdout)
        candidates: list[tuple[str, str, str]] = []
        for ref, ref_data in data.get("Local Cache", {}).items():
            name, _, version = ref.partition("/")
            if name != FALLBACK_GDB_RECIPE:
                continue
            for rrev, rrev_data in ref_data.get("revisions", {}).items():
                for pkg_id, pkg_data in rrev_data.get("packages", {}).items():
                    variant = (
                        pkg_data.get("info", {}).get("conf", {})
                        .get(FALLBACK_GDB_VARIANT_CONF)
                    )
                    if variant == FALLBACK_GDB_VARIANT:
                        candidates.append((version, rrev, pkg_id))
        if candidates:
            version, rrev, pkg_id = sorted(candidates, key=lambda c: c[0])[-1]
            ref = f"{FALLBACK_GDB_RECIPE}/{version}#{rrev}:{pkg_id}"
            cache_path = subprocess.run(
                ["conan", "cache", "path", ref],
                check=True, capture_output=True, text=True,
            ).stdout.strip()
            gdb_path = Path(cache_path) / "bin" / "arm-none-eabi-gdb"
            if gdb_path.exists():
                result = str(gdb_path)
    except (subprocess.CalledProcessError, FileNotFoundError, json.JSONDecodeError) as error:
        print(f"warning: fallback gdb lookup failed: {error}", file=sys.stderr)

    _fallback_gdb_cache["gdb"] = result
    return result


def guess_platform(build_dir_name: str) -> str:
    match = re.match(r"^(.*?)-(?:clang|gcc)-", build_dir_name)
    return match.group(1) if match else build_dir_name


def write_c_cpp_properties(vscode_dir: Path, infos: list[ToolchainInfo],
                            project_root: Path) -> None:
    """Merge-write, keyed by build-dir name -- same rationale as launch.json:
    a scoped run (--root v2/demos) must not drop configs an earlier scoped
    run (--root v1/demos) already wrote for a different subtree.
    """
    path = vscode_dir / "c_cpp_properties.json"
    if path.exists():
        existing = json.loads(path.read_text())
    else:
        existing = {"configurations": [], "version": 4}

    by_name = {c["name"]: c for c in existing.get("configurations", [])}

    for info in infos:
        if info.compiler_path is None:
            print(f"warning: could not resolve compiler for {info.build_dir}",
                  file=sys.stderr)
            continue
        rel = str(info.build_dir.relative_to(project_root))
        by_name[rel] = {
            "name": rel,
            "compilerPath": str(info.compiler_path),
            "cStandard": "c17",
            "cppStandard": f"c++{info.cxx_standard}",
            "intelliSenseMode": "clang-arm" if "clang" in info.cxx_compiler else "gcc-arm",
            "includePath": [
                "${workspaceFolder}/include/**",
                "${workspaceFolder}/src/**",
                *info.include_paths,
            ],
            "defines": [],
        }

    existing["configurations"] = list(by_name.values())
    existing["version"] = 4
    path.write_text(json.dumps(existing, indent=2) + "\n")
    print(f"wrote {path} ({len(by_name)} configuration(s))")


def build_debug_config(name: str, executable: Path, gdb_path: str | None,
                        platform: str) -> dict:
    pyocd_target = PYOCD_TARGET_OVERRIDES.get(platform, platform)
    config = {
        "name": name,
        "cwd": "${workspaceFolder}",
        "request": "launch",
        "type": "cortex-debug",
        "servertype": "pyocd",
        "device": pyocd_target,
        "serverArgs": PYOCD_SERVER_ARGS,
        "interface": "swd",
        "runToEntryPoint": "main",
        "preLaunchTask": "Resolve Conan Toolchain",
    }
    if gdb_path:
        config["gdbPath"] = gdb_path
    return config


def write_launch_json(vscode_dir: Path, infos: list[ToolchainInfo],
                       project_root: Path) -> None:
    path = vscode_dir / "launch.json"
    if path.exists():
        existing = json.loads(path.read_text())
    else:
        existing = {"version": "0.2.0", "configurations": []}

    by_name = {c["name"]: c for c in existing.get("configurations", [])
               if c.get("preLaunchTask") == "Resolve Conan Toolchain"}
    other_configs = [c for c in existing.get("configurations", [])
                      if c.get("preLaunchTask") != "Resolve Conan Toolchain"]

    for info in infos:
        if not info.is_cross:
            continue  # native/local debugging is out of scope for this pass
        gdb_path = info.find_local_gdb() or resolve_fallback_gdb()
        gdb_path_str = str(gdb_path) if gdb_path else None
        rel_build_dir = info.build_dir.relative_to(project_root)
        platform = guess_platform(info.build_dir.name)

        for executable in find_executables(info.build_dir):
            name = f"{rel_build_dir} :: {executable.name}"
            if name in by_name:
                # Preserve everything the user may have hand-tuned (device,
                # serverArgs, ...); only refresh what we can actually derive.
                config = by_name[name]
                config["executable"] = str(executable)
                if gdb_path_str:
                    config["gdbPath"] = gdb_path_str
            else:
                config = build_debug_config(name, executable, gdb_path_str, platform)
                config["executable"] = str(executable)
                by_name[name] = config

    existing["configurations"] = other_configs + list(by_name.values())
    path.write_text(json.dumps(existing, indent=2) + "\n")
    print(f"wrote {path} ({len(by_name)} debug configuration(s))")


def write_tasks_json(vscode_dir: Path, root_arg: str | None) -> None:
    path = vscode_dir / "tasks.json"
    args = ["${workspaceFolder}/scripts/resolve_conan_toolchain.py"]
    if root_arg:
        args += ["--root", root_arg]
    task = {
        "label": "Resolve Conan Toolchain",
        "type": "shell",
        "command": "python3",
        "args": args,
        "problemMatcher": [],
        "presentation": {"reveal": "silent"},
    }
    if path.exists():
        existing = json.loads(path.read_text())
    else:
        existing = {"version": "2.0.0", "tasks": []}
    tasks = [t for t in existing.get("tasks", []) if t.get("label") != task["label"]]
    tasks.append(task)
    existing["tasks"] = tasks
    path.write_text(json.dumps(existing, indent=2) + "\n")
    print(f"wrote {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=str, default=None,
        help="Directory to scan for build folders, relative to the repo root "
             "(default: whole repo). Pass e.g. 'demos' to scope to one subtree.",
    )
    args = parser.parse_args()

    scan_root = (REPO_ROOT / args.root) if args.root else REPO_ROOT
    if not scan_root.exists():
        print(f"error: {scan_root} does not exist", file=sys.stderr)
        return 1

    build_dirs = find_build_dirs(scan_root)
    if not build_dirs:
        print(f"error: no generators/conan_toolchain.cmake found under {scan_root}",
              file=sys.stderr)
        return 1

    infos = []
    for build_dir in build_dirs:
        info = ToolchainInfo(build_dir)
        rel = build_dir.relative_to(REPO_ROOT)
        kind = "cross" if info.is_cross else "native"
        n_exec = len(find_executables(build_dir))
        print(f"found {kind} build: {rel} ({n_exec} executable(s), "
              f"compiler={info.cxx_compiler})")
        infos.append(info)

    vscode_dir = REPO_ROOT / ".vscode"
    vscode_dir.mkdir(exist_ok=True)

    write_c_cpp_properties(vscode_dir, infos, REPO_ROOT)
    write_launch_json(vscode_dir, infos, REPO_ROOT)
    write_tasks_json(vscode_dir, args.root)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Build a headless compatibility COPY; third_party/bcd_planner is always read-only."""
from __future__ import annotations

import fcntl
import hashlib
from pathlib import Path
import re
import shlex
import subprocess

UPSTREAM_COMMIT = 'c90ffa61f4b84c3c09b41c1ba5a837f49d44d278'
UPSTREAM_MAIN_SHA256 = '3b033d4e5442b2896f12bbecfa713fc27ad77f5ab64da3a261aea6ea8f95e95c'
COMPATIBILITY = [
    'Add explicit standard headers: climits, numeric, cmath, limits, fstream.',
    'Add opencv2/imgcodecs.hpp: OpenCV4 no longer includes image IO through highgui.',
    'Replace example main with headless CLI; disable standalone namedWindow/imshow/waitKey calls in generated copy.',
    'Set failbit on original std::cout debug stream in wrapper only.',
]


def build(source_root: Path) -> tuple[Path, dict]:
    original = source_root / 'third_party/bcd_planner/main.cpp'
    original_bytes = original.read_bytes()
    fingerprint = hashlib.sha256(original_bytes).hexdigest()
    if fingerprint != UPSTREAM_MAIN_SHA256:
        raise RuntimeError(f'BCD original main.cpp differs from pinned upstream: {fingerprint}')
    wrapper = Path(__file__).with_name('bcd_wrapper.cpp.in').read_text()
    source = original_bytes.decode()
    # Narrow compatibility-only edits: no planner decisions or geometric logic is replaced.
    source = source[:source.index('int main()')]
    source = re.sub(r'^\s*cv::(?:namedWindow|imshow|waitKey)\([^\n]*\);', '    (void)0;', source, flags=re.MULTILINE)
    source = ('#include <climits>\n#include <numeric>\n#include <cmath>\n#include <limits>\n'
              '#include <fstream>\n#include <opencv2/imgcodecs.hpp>\n' + source + '\n' + wrapper)
    key = hashlib.sha256(source.encode()).hexdigest()[:16]
    build_dir = source_root / 'coverage_benchmark/.build/bcd' / key
    build_dir.mkdir(parents=True, exist_ok=True)
    generated = build_dir / 'bcd_headless.cpp'
    executable = build_dir / 'bcd_headless'
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'opencv4'], text=True))
    command = ['g++', '-std=c++14', '-O2', '-I/usr/include/eigen3', str(generated), '-o', str(executable), *flags]
    with (build_dir / '.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if not executable.exists():
            generated.write_text(source)
            completed = subprocess.run(command, capture_output=True, text=True, timeout=120)
            (build_dir / 'compile.log').write_text(completed.stdout + completed.stderr)
            if completed.returncode:
                raise RuntimeError(f'BCD compilation failed ({completed.returncode}): {completed.stderr[-12000:]}')
    details = {
        'upstream_commit': UPSTREAM_COMMIT,
        'upstream_main_sha256': fingerprint,
        'generated_source_sha256': hashlib.sha256(source.encode()).hexdigest(),
        'executable_sha256': hashlib.sha256(executable.read_bytes()).hexdigest(),
        'compile_command': shlex.join(command),
        'compatibility': COMPATIBILITY,
        'executable': str(executable),
    }
    return executable, details

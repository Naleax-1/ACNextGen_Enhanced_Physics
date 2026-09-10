"""Run paired microbenchmarks in fresh Lua VMs and optionally save a JSON report."""
import argparse
import datetime
import hashlib
import json
import platform
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.git/test-deps'))
from lupa.luajit21 import LuaRuntime as LuaJIT
from lupa.lua54 import LuaRuntime as Lua54

def plain(value):
    if not hasattr(value, 'items'):
        return value
    items = dict(value.items())
    if items and set(items) == set(range(1, len(items) + 1)):
        return [plain(items[i]) for i in range(1, len(items) + 1)]
    return {k: plain(v) for k, v in items.items()}

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if Path.cwd() != ROOT:
        raise SystemExit('Run from repository root')
    output = args.output.resolve() if args.output else None
    if output and not output.is_relative_to(ROOT):
        raise SystemExit('Report must stay within repository')
    report = {
        'measured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'platform': platform.platform(), 'python': platform.python_version(),
        'git_head_at_measurement': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted([*ROOT.glob('Engine/*.lua'), *ROOT.glob('Send/*.lua'),
                                           ROOT / 'modules/brake_fade.json', ROOT / 'tests/benchmark.lua'])},
        'interpretation': 'Microbenchmark only. Ratios below 1 mean faster pilot CPU; no FPS or whole-app inference. '
                          'Memory figures are incremental Lua heap, not process RSS, startup footprint or peak usage.',
        'runtimes': [],
    }
    for runtime in (LuaJIT, Lua54):
        lua = runtime(unpack_returned_tuples=True)
        result = plain(lua.execute((ROOT / 'tests/benchmark.lua').read_text()))
        result['median'] = {}
        for mode in ('legacy', 'pilot'):
            result['median'][mode] = {key: statistics.median(s[key] for s in result[mode])
                                      for key in result[mode][0]}
        result['pilot_to_legacy_cpu_ratio'] = (result['median']['pilot']['cpu_us_per_update'] /
                                               result['median']['legacy']['cpu_us_per_update'])
        report['runtimes'].append(result)
    serialized = json.dumps(report, indent=2) + '\n'
    if output:
        output.write_text(serialized)
    print(serialized)

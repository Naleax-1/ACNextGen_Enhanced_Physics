"""Replay real module bodies in independent Lua VMs; all AC APIs remain synthetic."""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.git/test-deps'))
from lupa.luajit21 import LuaRuntime as LuaJIT
from lupa.lua54 import LuaRuntime as Lua54

checks = 0
max_error = 0.0

def compare(a, b, path='root'):
    global checks, max_error
    checks += 1
    if hasattr(a, 'items'):
        assert hasattr(b, 'items'), path
        aa, bb = dict(a.items()), dict(b.items())
        assert aa.keys() == bb.keys(), (path, aa.keys() ^ bb.keys())
        for key in aa:
            compare(aa[key], bb[key], f'{path}.{key}')
    elif isinstance(a, (float, int)) and not isinstance(a, bool):
        assert isinstance(b, (float, int)), path
        if math.isnan(a):
            assert math.isnan(b), path
        elif math.isinf(a):
            assert a == b, path
        else:
            diff = abs(a - b)
            max_error = max(max_error, diff)
            assert diff <= 1e-12 + 1e-12 * max(abs(a), abs(b)), (path, a, b)
    else:
        assert a == b, (path, a, b)

def replay(runtime, mode, corrupt=False, observer_failure=False):
    old_vm, new_vm = runtime(unpack_returned_tuples=True), runtime(unpack_returned_tuples=True)
    source = (ROOT / 'tests/integration.lua').read_text()
    old = old_vm.execute(source)('legacy', True, False, False)
    new = new_vm.execute(source)(mode, False, corrupt, observer_failure)
    for frame in range(1, 161):
        compare(old.step(frame), new.step(frame), f'frame{frame}')
    status = new.status()
    assert status.ui_calls > 0
    assert status.pilot_active == (mode == 'pilot' and not corrupt)
    assert status.legacy_brake_loaded == (mode == 'legacy' or corrupt)
    if corrupt:
        assert status.migration_errors.brake_fade
    if observer_failure:
        assert status.migration_errors.observer
    # Existing module limitations are reported, not hidden behind overall equivalence.
    assert status.runtime_active_errors == 0, status.runtime_last_error
    return status

if __name__ == '__main__':
    if Path.cwd() != ROOT:
        raise SystemExit('Run from repository root: python tests/integration.py')
    for runtime in (LuaJIT, Lua54):
        for mode, corrupt, observer_failure in [('legacy', False, False), ('pilot', False, False),
                                                ('pilot', True, False), ('pilot', False, True)]:
            result = replay(runtime, mode, corrupt, observer_failure)
            print(f'PASS real module replay: {runtime.__module__} {mode=} {corrupt=} {observer_failure=}', flush=True)
    print(f'PASS {checks} comparisons; maximum numeric difference {max_error:.17g}')

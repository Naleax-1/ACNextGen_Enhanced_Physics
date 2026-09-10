"""Optional runner when standalone Lua binaries are unavailable (pip install lupa==2.8)."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# Workspace-local optional dependencies keep the sandbox isolated.
sys.path.insert(0, str(ROOT / '.git/test-deps'))
from lupa.luajit21 import LuaRuntime as LuaJIT
from lupa.lua54 import LuaRuntime as Lua54

if Path.cwd() != ROOT:
    raise SystemExit('Run from the repository root: python tests/run.py')
for runtime in (LuaJIT, Lua54):
    lua = runtime(unpack_returned_tuples=True)
    print('Testing', lua.eval('_VERSION'), runtime.__module__, flush=True)
    lua.execute((ROOT / 'tests/run.lua').read_text())

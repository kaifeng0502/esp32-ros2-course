#!/usr/bin/env python3
"""Check course completeness, syntax and local links without opening hardware."""
import ast
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
SKIP = {'.git', '.venv', 'build', 'results', 'dist', '.run', 'sdkconfig', '__pycache__'}


def main():
    errors = []
    lessons = json.loads((ROOT/'course.json').read_text())['lessons']
    assert [x['id'] for x in lessons] == list(range(1, 22))
    for lesson in lessons:
        folder = ROOT/'lessons'/lesson['slug']
        metadata = json.loads((folder/'lesson.json').read_text())
        if metadata['id'] != lesson['id'] or not (folder/'README.md').is_file() or not any((folder/'code').iterdir()):
            errors.append(f'Incomplete lesson: {folder.name}')
        for source in metadata['sources']:
            if not (ROOT/source).exists(): errors.append(f'Missing lesson source: {source}')
    files = [p for p in ROOT.rglob('*') if p.is_file() and not (set(p.relative_to(ROOT).parts) & SKIP)]
    for p in files:
        if p.suffix == '.py':
            try: ast.parse(p.read_text(), filename=str(p.relative_to(ROOT)))
            except SyntaxError as e: errors.append(str(e))
        if p.suffix == '.sh':
            result = subprocess.run(['bash', '-n', str(p)], capture_output=True, text=True)
            if result.returncode: errors.append(result.stderr)
        if p.suffix == '.md':
            for target in re.findall(r'\]\(([^\s)]+)\)', p.read_text()):
                if target.startswith(('https:', 'http:', 'mailto:', '#')): continue
                target = unquote(target.split('#')[0]).strip('<>')
                if not (p.parent/target).exists(): errors.append(f'{p.relative_to(ROOT)}: broken link {target}')
        if p.suffix in {'.py', '.c', '.h', '.md', '.json', '.txt', '.yaml', '.yml', '.sh'}:
            text = p.read_text()
            private_patterns = ['/'+'Users/kai/', '/'+'home/kai/', '192.168.'+'31.57', '冯'+'凯', '周'+'月', 'codex-'+'spritesheet']
            for pattern in private_patterns:
                if pattern in text: errors.append(f'{p.relative_to(ROOT)}: personal source material {pattern}')
    if errors: raise SystemExit('\n'.join(errors))
    print(f'OK: 21 lessons; {len(files)} files checked; local links, Python and shell syntax valid.')


if __name__ == '__main__': main()

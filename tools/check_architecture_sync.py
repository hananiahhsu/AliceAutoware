#!/usr/bin/env python3
import json
import pathlib
import sys


def main() -> int:
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    inventory_path = repo_root / 'docs' / 'architecture' / 'module_inventory.json'
    inventory = json.loads(inventory_path.read_text(encoding='utf-8'))

    missing_paths = []
    for module in inventory['modules']:
        rel_path = repo_root / module['path']
        if not rel_path.exists():
            missing_paths.append(module['path'])

    architecture_md = (repo_root / 'docs' / 'architecture' / '01_system_architecture.md').read_text(encoding='utf-8')
    missing_in_doc = [m['name'] for m in inventory['modules'] if m['name'] not in architecture_md]

    if missing_paths or missing_in_doc:
        print('[MAD] architecture sync check failed')
        if missing_paths:
            print('  missing paths   :', missing_paths)
        if missing_in_doc:
            print('  missing in docs :', missing_in_doc)
        return 2

    print('[MAD] architecture sync check passed')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

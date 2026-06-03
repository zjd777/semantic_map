#!/usr/bin/env python3
# encoding: utf-8

import argparse
import json
import os
from pathlib import Path


DEFAULT_MAP_FILE = '~/.ros/semantic_voxel_map.json'


def _expand_path(path):
    return Path(os.path.abspath(os.path.expanduser(os.path.expandvars(path))))


def _find_map_files(requested_path):
    requested = _expand_path(requested_path)
    candidates = []
    seen = set()

    def add(path):
        path = Path(path)
        if path.exists() and path.is_file() and path not in seen:
            seen.add(path)
            candidates.append(path)

    add(requested)
    add(_expand_path(DEFAULT_MAP_FILE))

    search_roots = [
        Path.home() / '.ros',
        Path.home() / 'ros2_ws',
        Path.cwd(),
    ]
    for root in search_roots:
        if not root.exists():
            continue
        for path in root.rglob('semantic_voxel_map.json'):
            add(path)

    return sorted(candidates, key=lambda path: path.stat().st_mtime, reverse=True)


def _resolve_map_file(requested_path):
    requested = _expand_path(requested_path)
    if requested.exists():
        return requested
    candidates = _find_map_files(requested_path)
    if candidates:
        print(f'Map file not found at {requested}; using {candidates[0]}')
        return candidates[0]
    raise FileNotFoundError(
        f'No semantic_voxel_map.json found. Expected location: {requested}'
    )


def _load_map(path):
    with path.open('r', encoding='utf-8') as file:
        return json.load(file)


def _save_map(path, data):
    tmp_path = path.with_suffix(path.suffix + '.tmp')
    with tmp_path.open('w', encoding='utf-8') as file:
        json.dump(data, file, ensure_ascii=False, indent=2)
    os.replace(tmp_path, path)


def locate_command(args):
    candidates = _find_map_files(args.map_file)
    if not candidates:
        print(f'No semantic map JSON found. Expected default: {_expand_path(args.map_file)}')
        return 1
    for path in candidates:
        print(path)
    return 0


def list_command(args):
    path = _resolve_map_file(args.map_file)
    data = _load_map(path)
    objects = data.get('objects', [])
    print(f'map_file: {path}')
    print(f'frame: {data.get("frame_id", "map")}')
    print(f'objects: {len(objects)}')
    for obj in objects:
        position = obj.get('position', [0.0, 0.0, 0.0])
        display_name = obj.get('display_name', '')
        display = f' name={display_name}' if display_name else ''
        print(
            f'{obj.get("id", ""):<20} {obj.get("class_name", ""):<16}'
            f' x={position[0]:.2f} y={position[1]:.2f} z={position[2]:.2f}'
            f' obs={obj.get("observations", 0)} conf={obj.get("confidence", 0.0):.2f}'
            f'{display}'
        )
    return 0


def rename_command(args):
    path = _resolve_map_file(args.map_file)
    data = _load_map(path)
    matched = None
    for obj in data.get('objects', []):
        if obj.get('id') == args.object_id:
            matched = obj
            break
    if matched is None:
        raise ValueError(f'Object id not found: {args.object_id}')

    previous = matched.get('display_name', '')
    if args.clear:
        matched.pop('display_name', None)
        updated = ''
    else:
        matched['display_name'] = args.name
        updated = args.name
    _save_map(path, data)
    print(
        f'Updated {args.object_id}: display_name "{previous}" -> "{updated}" in {path}'
    )
    return 0


def reclassify_command(args):
    path = _resolve_map_file(args.map_file)
    data = _load_map(path)
    matched = None
    for obj in data.get('objects', []):
        if obj.get('id') == args.object_id:
            matched = obj
            break
    if matched is None:
        raise ValueError(f'Object id not found: {args.object_id}')

    updated = str(args.class_name).strip().lower().replace('_', ' ')
    if not updated:
        raise ValueError('class_name must not be empty')
    previous = matched.get('class_name', '')
    matched['class_name'] = updated
    _save_map(path, data)
    print(
        f'Updated {args.object_id}: class_name "{previous}" -> "{updated}" in {path}'
    )
    print('This corrects the saved semantic target only; it does not retrain YOLO.')
    return 0


def build_parser():
    parser = argparse.ArgumentParser(
        description='Locate, list, rename, or correct classes of saved semantic map objects.'
    )
    parser.add_argument('--map-file', default=DEFAULT_MAP_FILE)
    subparsers = parser.add_subparsers(dest='command', required=True)
    subparsers.add_parser('locate', help='Find semantic voxel JSON files.')
    subparsers.add_parser('list', help='List semantic objects in the JSON file.')
    rename_parser = subparsers.add_parser('rename', help='Set an object display name.')
    rename_parser.add_argument('object_id', help='Stable object id such as suitcase_1.')
    rename_parser.add_argument('name', nargs='?', default='', help='New display name.')
    rename_parser.add_argument('--clear', action='store_true', help='Remove display name.')
    reclassify_parser = subparsers.add_parser(
        'reclassify', help='Correct the semantic class of a falsely detected saved object.'
    )
    reclassify_parser.add_argument('object_id', help='Stable object id such as tv_1.')
    reclassify_parser.add_argument('class_name', help='Correct class such as suitcase.')
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        if args.command == 'locate':
            return locate_command(args)
        if args.command == 'list':
            return list_command(args)
        if args.command == 'rename':
            if not args.clear and not args.name:
                parser.error('rename requires NAME unless --clear is used')
            return rename_command(args)
        if args.command == 'reclassify':
            return reclassify_command(args)
    except (FileNotFoundError, ValueError, OSError, json.JSONDecodeError) as exc:
        print(f'Error: {exc}')
        return 1
    return 1


if __name__ == '__main__':
    raise SystemExit(main())

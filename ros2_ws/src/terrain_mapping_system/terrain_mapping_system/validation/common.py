"""Shared helpers for offline validation and batch orchestration."""

from __future__ import annotations

import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def read_json(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as stream:
        return json.load(stream)


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        stream.write(content)


def read_yaml(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as stream:
        payload = yaml.safe_load(stream)
    if not isinstance(payload, dict):
        raise ValueError(f'YAML file does not contain a mapping: {path}')
    return payload


def write_yaml(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(payload, stream, sort_keys=False)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def file_fingerprint(path: Optional[Path]) -> Optional[Dict[str, Any]]:
    if path is None:
        return None
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        return {
            'path': str(resolved),
            'exists': False,
        }
    stat = resolved.stat()
    return {
        'path': str(resolved),
        'exists': True,
        'sha256': sha256_file(resolved),
        'size_bytes': stat.st_size,
        'modified_at_epoch_s': round(stat.st_mtime, 6),
    }


def software_context() -> Dict[str, Any]:
    return {
        'python_version': sys.version.split()[0],
        'platform': platform.platform(),
        'numpy_version': np.__version__,
    }

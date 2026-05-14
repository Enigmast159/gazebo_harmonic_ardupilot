"""Machine-readable mission artifact writer."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def resolve_run_id(run_id: str) -> str:
    return run_id or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


class MissionArtifactWriter:
    """Writes plan, events, trajectory, summary, and status files."""

    def __init__(self, results_root: str, run_id: str):
        self.results_dir = Path(results_root).expanduser().resolve() / run_id
        self.results_dir.mkdir(parents=True, exist_ok=True)

        self.plan_path = self.results_dir / 'planned_path.json'
        self.trajectory_path = self.results_dir / 'actual_trajectory.jsonl'
        self.events_path = self.results_dir / 'mission_events.jsonl'
        self.summary_path = self.results_dir / 'mission_summary.json'
        self.status_path = self.results_dir / 'mission_status.json'

    def write_plan(self, payload: Dict[str, Any]) -> None:
        self._write_json(self.plan_path, payload)

    def append_trajectory(self, payload: Dict[str, Any]) -> None:
        self._append_jsonl(self.trajectory_path, payload)

    def append_event(self, payload: Dict[str, Any]) -> None:
        self._append_jsonl(self.events_path, payload)

    def write_summary(self, payload: Dict[str, Any]) -> None:
        self._write_json(self.summary_path, payload)

    def write_status(self, payload: Dict[str, Any]) -> None:
        self._write_json(self.status_path, payload)

    def snapshot(self) -> Dict[str, str]:
        return {
            'results_dir': str(self.results_dir),
            'planned_path': str(self.plan_path),
            'actual_trajectory': str(self.trajectory_path),
            'mission_events': str(self.events_path),
            'mission_summary': str(self.summary_path),
            'mission_status': str(self.status_path),
        }

    def _write_json(self, path: Path, payload: Dict[str, Any]) -> None:
        with path.open('w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write('\n')

    def _append_jsonl(self, path: Path, payload: Dict[str, Any]) -> None:
        with path.open('a', encoding='utf-8') as stream:
            json.dump(payload, stream, sort_keys=True)
            stream.write('\n')


def with_timestamp(payload: Dict[str, Any], *, timestamp_key: str = 'timestamp') -> Dict[str, Any]:
    stamped = dict(payload)
    stamped[timestamp_key] = utc_now_iso()
    return stamped


def sequence_payload(items: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    return {'items': list(items)}

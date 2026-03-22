"""onnx_nav_model_io.py

役割:
- モデル入出力のファイルロギング（JSONL）を担当
- 推論入力/出力の統計値整形を担当
- ノード本体からI/O記録処理を分離
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import numpy as np


class ModelIoFileLogger:
    def __init__(
        self,
        enabled: bool,
        log_dir: str,
        every_n: int,
        *,
        node_name: str,
        action_output_name: str,
        input_names: list[str],
        output_names: list[str],
        logger: Any,
        stamp_ns_provider: Callable[[], int],
    ) -> None:
        self.enabled = enabled
        self.log_dir = log_dir
        self.every_n = every_n
        self.node_name = node_name
        self.action_output_name = action_output_name
        self.input_names = input_names
        self.output_names = output_names
        self._stamp_ns_provider = stamp_ns_provider

        self._file_log_count = 0
        self._file_log_path: Path | None = None

        if self.every_n <= 0:
            raise ValueError(f'model_io_log_every_n must be > 0: {self.every_n}')

        if self.enabled:
            self._init_file(logger)

    def _init_file(self, logger: Any) -> None:
        log_dir = Path(self.log_dir).expanduser()
        if not log_dir.is_absolute():
            log_dir = Path.cwd() / log_dir

        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._file_log_path = log_dir / f'model_io_{timestamp}.jsonl'

        start_record = {
            'event': 'start',
            'node': self.node_name,
            'action_output_name': self.action_output_name,
            'input_names': self.input_names,
            'output_names': self.output_names,
        }

        with self._file_log_path.open('w', encoding='utf-8') as fp:
            fp.write(f'{json.dumps(start_record, ensure_ascii=False)}\n')

        logger.info(f'model_io file logging enabled: {self._file_log_path}')

    @staticmethod
    def _arr_stats(arr: np.ndarray) -> dict:
        flat = arr.reshape(-1)
        sample = flat[:8].tolist()
        return {
            'shape': list(arr.shape),
            'dtype': str(arr.dtype),
            'min': float(np.min(arr)),
            'max': float(np.max(arr)),
            'mean': float(np.mean(arr)),
            'sample': [float(v) for v in sample],
        }

    def write(
        self,
        feed: dict,
        vec: np.ndarray,
        action: np.ndarray,
        *,
        goal_xy: np.ndarray | None,
        robot_xyyaw: np.ndarray | None,
        logger: Any,
    ) -> None:
        if not self.enabled or self._file_log_path is None:
            return

        self._file_log_count += 1
        if self._file_log_count % self.every_n != 0:
            return

        feed_stats = {}
        for name, arr in feed.items():
            if isinstance(arr, np.ndarray):
                feed_stats[name] = self._arr_stats(arr)
            else:
                feed_stats[name] = {'type': type(arr).__name__}

        record = {
            'event': 'inference',
            'stamp_ns': int(self._stamp_ns_provider()),
            'goal_xy': goal_xy.tolist() if goal_xy is not None else None,
            'robot_xyyaw': robot_xyyaw.tolist() if robot_xyyaw is not None else None,
            'vec_obs': [float(v) for v in vec.reshape(-1).tolist()],
            'feed': feed_stats,
            'action_output_name': self.action_output_name,
            'action': [float(v) for v in action.tolist()],
        }

        try:
            with self._file_log_path.open('a', encoding='utf-8') as fp:
                fp.write(f'{json.dumps(record, ensure_ascii=False)}\n')
        except Exception as exc:
            logger.error(f'Failed to write model_io log file: {exc}')

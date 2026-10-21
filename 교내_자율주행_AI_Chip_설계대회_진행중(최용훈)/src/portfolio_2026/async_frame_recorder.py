"""Non-blocking frame recorder core with injectable encoding for host tests."""

from __future__ import annotations

import csv
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


Encoder = Callable[[Path, object], bool]


@dataclass(frozen=True)
class RecorderStats:
    saved: int
    dropped: int
    failed: int


class AsyncFrameRecorder:
    """Write frames on a worker; never wait when the bounded queue is full."""

    def __init__(self, output_dir: Path, encoder: Encoder, queue_size: int = 64) -> None:
        if queue_size < 1:
            raise ValueError("queue_size must be positive")
        self.output_dir = Path(output_dir)
        self.encoder = encoder
        self._queue: queue.Queue[tuple[int, float, object] | None] = queue.Queue(queue_size)
        self._worker_thread: threading.Thread | None = None
        self._csv_file = None
        self._csv_writer = None
        self._next_index = 0
        self._saved = 0
        self._dropped = 0
        self._failed = 0

    def start(self) -> None:
        if self._worker_thread is not None:
            raise RuntimeError("recorder already started")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_file = (self.output_dir / "frames.csv").open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(["frame_index", "monotonic_s", "filename"])
        self._worker_thread = threading.Thread(target=self._worker, daemon=True)
        self._worker_thread.start()

    def submit(self, frame: object, now_s: float | None = None) -> bool:
        if self._worker_thread is None:
            raise RuntimeError("recorder is not started")
        item = (self._next_index, time.monotonic() if now_s is None else now_s, frame)
        self._next_index += 1
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            self._dropped += 1
            return False

    def stop(self) -> RecorderStats:
        if self._worker_thread is None:
            return self.stats
        self._queue.put(None)
        self._worker_thread.join(timeout=30.0)
        if self._worker_thread.is_alive():
            raise TimeoutError("frame writer did not stop")
        self._worker_thread = None
        self._csv_file.flush()
        self._csv_file.close()
        self._csv_file = None
        return self.stats

    @property
    def stats(self) -> RecorderStats:
        return RecorderStats(self._saved, self._dropped, self._failed)

    def _worker(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                index, timestamp, frame = item
                filename = f"frame_{index:06d}.jpg"
                if self.encoder(self.output_dir / filename, frame):
                    self._saved += 1
                    self._csv_writer.writerow([index, f"{timestamp:.6f}", filename])
                else:
                    self._failed += 1
            finally:
                self._queue.task_done()


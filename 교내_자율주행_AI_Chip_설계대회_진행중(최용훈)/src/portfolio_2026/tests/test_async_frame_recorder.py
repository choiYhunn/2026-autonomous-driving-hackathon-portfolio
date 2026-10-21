import tempfile
import threading
import unittest
from pathlib import Path

from portfolio_2026.async_frame_recorder import AsyncFrameRecorder


class AsyncFrameRecorderTest(unittest.TestCase):
    def test_records_metadata_and_stops_cleanly(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            calls = []

            def encoder(path: Path, frame: object) -> bool:
                calls.append((path.name, frame, threading.current_thread().name))
                path.write_bytes(b"jpeg")
                return True

            recorder = AsyncFrameRecorder(Path(temp_dir), encoder, queue_size=2)
            recorder.start()
            self.assertTrue(recorder.submit("frame", now_s=1.25))
            stats = recorder.stop()
            self.assertEqual(stats.saved, 1)
            self.assertEqual(calls[0][0], "frame_000000.jpg")
            self.assertTrue((Path(temp_dir) / "frames.csv").exists())


if __name__ == "__main__":
    unittest.main()


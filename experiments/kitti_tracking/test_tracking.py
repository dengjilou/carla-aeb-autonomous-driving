import tempfile
from collections import defaultdict
from pathlib import Path
import unittest

import numpy as np
from evaluate import associate, track, overlaps


class TrackingTests(unittest.TestCase):
    def test_hungarian_respects_gate_and_preserves_two_matches(self):
        scores = np.array([[.9, .8], [.7, .2]])
        self.assertEqual(list(associate(scores, True)), [(0, 0)])
        self.assertEqual(set(associate(scores, False)), {(0, 1), (1, 0)})
        self.assertEqual(list(associate(np.zeros((1, 1)), False)), [])

    def test_short_gap_keeps_identity_without_emitting_guesses(self):
        frames = defaultdict(list, {0: [[0, 0, 20, 20, 1]], 2: [[1, 0, 21, 20, 1]],
                                    6: [[1, 0, 21, 20, 1]]})
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "tracks.txt"
            track(frames, 7, "kalman_hungarian", output)
            rows = [line.split() for line in output.read_text().splitlines()]
        self.assertEqual([int(s[0]) for s in rows], [0, 2, 6])
        self.assertEqual([int(s[1]) for s in rows], [0, 0, 1])
        np.testing.assert_allclose(overlaps(np.array([[0, 0, 20, 20]]),
                                           np.array([[0, 0, 20, 20], [30, 0, 50, 20]])), [[1, 0]])


if __name__ == "__main__":
    unittest.main()

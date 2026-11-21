import unittest
from queue import Queue
from threading import Event
from unittest.mock import patch

from paired_validation import PAIR_SEEDS, ValidationMetrics, compare_runs, run_pairs
from simulator import Robot


class PairedValidationTests(unittest.TestCase):
    @staticmethod
    def runs(times):
        return {seed: ValidationMetrics(True, seconds, 5, 0)
                for seed, seconds in zip(PAIR_SEEDS, times)}

    def test_faster_screening_alone_does_not_win(self):
        reference = self.runs((10, 10, 10))
        result = compare_runs(reference, self.runs((9, 10, 11)))
        self.assertEqual(result["delta_mediano_s"], 0)
        self.assertFalse(result["aceita"])

    def test_all_candidate_runs_must_be_competitive(self):
        reference = self.runs((10, 10, 10))
        candidate = self.runs((9, 9, 10.5))
        self.assertTrue(compare_runs(reference, candidate)["aceita"])
        candidate[83] = ValidationMetrics(True, 9, 5, 0, .01)
        result = compare_runs(reference, candidate)
        self.assertEqual(result["validas"], 2)
        self.assertFalse(result["aceita"])
        self.assertIsNone(result["pares"][2]["delta_s"])

    def test_cache_reuses_only_matching_profile_and_seed(self):
        reference = self.runs((10, 10, 10))
        candidate = self.runs((9,))
        row = {"t_s": 9.2, "enc_left": 100, "enc_right": 100, "line_lost": 0, "body_on_line": 1}
        with patch("paired_validation.simulate", return_value=([row], [], "volta completa")) as simulate:
            run_pairs(None, Robot(), {"id": 1}, {"id": 2}, reference, candidate, Queue(), Event())
        self.assertEqual([(call.args[2]["id"], call.args[3]) for call in simulate.call_args_list],
                         [(2, 37), (2, 83)])

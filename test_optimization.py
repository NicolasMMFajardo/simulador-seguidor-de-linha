"""Exercita o ciclo automático sem abrir a janela ou aguardar tempo de parede."""
import unittest
from collections import deque
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from optimization import AutoOptimizer
from paired_validation import ValidationMetrics


class ScheduledCallbacks:
    def __init__(self):
        self.jobs = {}
        self.next_id = 0

    def after(self, delay, callback, *args):
        self.next_id += 1
        self.jobs[self.next_id] = (callback, args)
        return self.next_id

    def after_cancel(self, job):
        self.jobs.pop(job, None)

    def until(self, condition):
        for _ in range(50):
            if condition():
                return
            if not self.jobs:
                break
            self.tick()
        raise AssertionError("O ciclo automático não alcançou o estado esperado.")

    def tick(self):
        callback, args = self.jobs.pop(next(iter(self.jobs)))
        callback(*args)


def profile(speed):
    return {"segments": [{"v": speed, "kp": 100., "kd": 6., "turn_feedforward": 0.}]}


class OptimizationBench:
    """Mantém a mesma passagem de proposta, triagem e referência da bancada."""
    def __init__(self, chances):
        self.root = ScheduledCallbacks()
        self.status = Mock()
        self.update_buttons = Mock()
        self.profile = profile(1.)
        self.reference_id = self.last_iteration_id = self.latest_validation_id = 0
        self.pending_id = self.proposal = self.confirmation_stop = None
        self.iterations = [SimpleNamespace(profile=self.profile, reference_id=None, diagnosed=False,
                                           metrics=ValidationMetrics(True, 10., 5., 0.), decision="Referência inicial")]
        self.validation = ([], "volta completa", None, self.profile)
        self.map_history_start = 0
        self.chances = deque(chances)
        self.validations = []
        self.diagnoses = []

    def run_diagnosis(self):
        self.diagnoses.append(self.last_iteration_id)
        self.iterations[self.last_iteration_id].diagnosed = True
        self.proposal = self.chances.popleft() if self.chances else None

    def apply_proposal(self):
        self.profile, self.proposal = self.proposal, None
        self.pending_id = len(self.iterations)
        self.iterations.append(SimpleNamespace(profile=self.profile, reference_id=self.reference_id,
                                               diagnosed=False, metrics=None, decision="Aguardando validação"))
        self.validation = None

    def start_auto_validation(self):
        future = Future()
        self.validations.append(future)
        return future

    def accept_auto_validation(self, decision):
        if decision == "Confirmando":
            self.iterations[self.pending_id].decision = decision
            self.confirmation_stop = object()
            return
        self.finish_candidate(decision)

    def finish_candidate(self, decision):
        self.confirmation_stop = None
        tested = self.iterations[self.pending_id]
        tested.decision = decision
        tested.metrics = ValidationMetrics(True, 9. if decision == "Aceita" else 11., 5., 0.)
        self.last_iteration_id = self.latest_validation_id = self.pending_id
        self.pending_id = None
        self.validation = ([], "volta completa", None, tested.profile)
        if decision == "Aceita":
            self.reference_id = self.last_iteration_id
        self.profile = self.iterations[self.reference_id].profile


class AutoOptimizerTests(unittest.TestCase):
    def setUp(self):
        self.first = profile(1.07)
        self.refined = profile(1.035)
        self.app = OptimizationBench([self.first, self.refined])
        self.loop = AutoOptimizer(self.app)

    def test_no_proposal_finishes_without_another_validation(self):
        self.app.chances.clear()
        self.loop.start()
        self.app.root.until(lambda: not self.loop.active)
        self.assertEqual(self.app.diagnoses, [0])
        self.assertEqual(self.app.validations, [])
        self.assertEqual(len(self.app.iterations), 1)

    def test_rejection_is_followed_by_refinement_from_the_reference(self):
        self.loop.start()
        self.app.root.until(lambda: len(self.app.validations) == 1)
        self.app.validations[0].set_result("Rejeitada")
        self.app.root.until(lambda: len(self.app.validations) == 2)
        self.assertEqual(self.app.reference_id, 0)
        self.assertEqual(self.app.iterations[2].reference_id, 0)
        self.assertEqual(self.app.iterations[2].profile, self.refined)
        self.app.validations[1].set_result("Aceita")
        self.app.root.until(lambda: not self.loop.active)
        self.assertEqual(self.app.reference_id, 2)
        self.assertEqual(self.app.diagnoses, [0, 1, 2])
        self.assertEqual(len(self.app.iterations), 3)

    def test_pause_completes_the_running_validation_then_stops(self):
        self.loop.start()
        self.app.root.until(lambda: bool(self.app.validations))
        self.loop.pause()
        self.app.validations[0].set_result("Aceita")
        self.app.root.until(lambda: not self.loop.active)
        self.assertIsNone(self.app.pending_id)
        self.assertEqual(self.app.reference_id, 1)
        self.assertEqual(len(self.app.validations), 1)
        self.assertEqual(len(self.app.iterations), 2)

    def test_next_proposal_waits_for_paired_confirmation(self):
        self.loop.start()
        self.app.root.until(lambda: bool(self.app.validations))
        self.app.validations[0].set_result("Confirmando")
        self.app.root.until(lambda: self.app.confirmation_stop is not None)
        self.app.root.tick()
        self.assertEqual(len(self.app.validations), 1)
        self.assertEqual(self.app.diagnoses, [0])
        self.app.finish_candidate("Aceita")
        self.app.root.until(lambda: len(self.app.validations) == 2)
        self.assertEqual(self.app.iterations[2].reference_id, 1)

    def test_repeated_rejected_candidate_stops_at_the_reference(self):
        self.app.chances = deque([self.first, self.first])
        self.loop.start()
        self.app.root.until(lambda: bool(self.app.validations))
        self.app.validations[0].set_result("Rejeitada")
        self.app.root.until(lambda: not self.loop.active)
        self.assertEqual(self.app.diagnoses, [0, 1])
        self.assertEqual(len(self.app.validations), 1)
        self.assertEqual(self.app.profile, self.app.iterations[0].profile)

    def test_pending_manual_proposal_is_validated_once(self):
        self.app.proposal = self.first
        self.app.apply_proposal()
        self.loop.start()
        self.app.root.until(lambda: bool(self.app.validations))
        self.assertEqual(self.app.diagnoses, [])
        self.assertEqual(len(self.app.iterations), 2)
        self.loop.pause()
        self.app.validations[0].set_result("Aceita")
        self.app.root.until(lambda: not self.loop.active)
        self.assertEqual(self.app.reference_id, 1)
        self.assertEqual(len(self.app.validations), 1)

    def test_cancel_removes_scheduled_work(self):
        self.loop.start()
        self.loop.cancel()
        self.assertFalse(self.loop.active)
        self.assertFalse(self.app.root.jobs)
        self.assertEqual(len(self.app.iterations), 1)
        self.assertEqual(self.app.validations, [])


if __name__ == "__main__":
    unittest.main()

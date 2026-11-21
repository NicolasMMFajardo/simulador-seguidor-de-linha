import unittest
from copy import deepcopy
from math import pi

from curve_analysis import analyze_curves
from simulator import Robot


class CurveAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.robot = Robot(diameter=1/pi, ticks_per_turn=100)
        self.profile = {"length": 4., "ends": [2.], "segments": [
            {"label": "Reta", "v": 1., "kp": 100., "kd": 6., "curvature": 0.} for _ in range(2)]}
        self.rows = [
            dict(t_s=i/10, track_progress_m=i/10, enc_left=i*10, enc_right=i*10,
                 line_error_m=0., body_on_line=1, motor_sat=0, line_lost=0, line_edge=0,
                 traction_usage_pct=20., encoder_speed_m_s=1.)
            for i in range(1, 41)]

    def analyze(self, bends=(), **options):
        return analyze_curves(self.rows, self.profile, self.robot, bends, 4., **options)

    def test_wrapping_curve_uses_both_sides_of_start_and_reference_exit(self):
        reference = [dict(r, t_s=.8*r["t_s"]) for r in self.rows]
        _, report = self.analyze([(-.25, .25, 1), (1., 2., -1)], reference_rows=reference)
        first = next(row for row in report if row["curva"] == 1)
        self.assertTrue(first["cruza_largada"])
        self.assertAlmostEqual(first["marco_m"], .75)
        self.assertAlmostEqual(first["tempo_s"], 1.)
        self.assertAlmostEqual(first["delta_s"], .20)
        self.assertAlmostEqual(first["vel_saida_m_s"], 1.)

    def test_rejected_group_refines_the_step_and_mixed_trial_is_ignored(self):
        first, _ = self.analyze()
        for sector in first["segments"]:
            self.assertAlmostEqual(sector["v"], 1.07)
        refined, _ = self.analyze(trials=[(self.profile, first, False)])
        for sector in refined["segments"]:
            self.assertAlmostEqual(sector["v"], 1.035)
        manual = deepcopy(first)
        manual["segments"][0]["kd"] += 1
        again, _ = self.analyze(trials=[(self.profile, manual, False)])
        self.assertEqual(first, again)

    def test_historical_reference_uses_same_distance_source_in_both_laps(self):
        reference = [dict(row) for row in self.rows]
        for row in reference:
            del row["track_progress_m"]
        for row in self.rows:
            row["track_progress_m"] *= .9
        _, report = self.analyze([(1., 2., 1)], reference_rows=reference)
        self.assertTrue(all(row["alinhamento"] == "odometria estimada" for row in report))
        self.assertTrue(all(row["delta_s"] == 0 for row in report))

    def test_short_passage_and_incomplete_lap_do_not_promote_a_proposal(self):
        self.rows = self.rows[:15]
        candidate, report = self.analyze([(1.25, 2., 1)], complete=False)
        self.assertEqual(candidate, self.profile)
        self.assertFalse(any(r["selecionado"] for r in report))
        curve = next(r for r in report if r["curva"])
        self.assertIsNone(curve["tempo_s"])
        self.assertIsNone(curve["vel_saida_m_s"])
        self.assertIsNone(curve["delta_s"])

    def test_oscillation_and_turn_lag_trigger_distinct_experiments(self):
        for i, row in enumerate(self.rows):
            row["line_error_m"] = .010 if i % 4 < 2 else -.010
        candidate, report = self.analyze()
        self.assertEqual(report[0]["parametro"], "kd")
        self.assertGreater(candidate["segments"][0]["kd"], 6.)
        for sector in self.profile["segments"]:
            sector.update(label="D2", curvature=2.)
        for row in self.rows:
            row.update(gyro_z_rad_s=.5, line_edge=1)
        candidate, report = self.analyze([(0., 4., 1)])
        self.assertEqual(report[0]["parametro"], "turn_feedforward")
        self.assertAlmostEqual(candidate["segments"][0]["turn_feedforward"], .2)


class CurveLossDiagnosisTests(unittest.TestCase):
    def setUp(self):
        self.robot = Robot(diameter=1/pi, ticks_per_turn=100)
        self.profile = {"length": 1., "ends": [], "segments": [
            {"label": "D3", "curvature": 1.5, "v": 1., "kp": 100., "kd": 6.}]}
        self.rows = [
            dict(t_s=i/100, track_progress_m=i/100, enc_left=i, enc_right=i,
                 encoder_speed_m_s=1., gyro_z_rad_s=1.5+self.robot.gyro_bias,
                 line_error_m=.020, line_edge=1, line_lost=0, body_on_line=1, motor_sat=0,
                 planned_m_s=1., target_m_s=1., lateral_grip_pct=50., lateral_grip_limited=0,
                 traction_long_m_s2=0., traction_usage_pct=50.)
            for i in range(1, 101)]

    def curve(self, start=0., end=1., rows=None):
        _, report = analyze_curves(self.rows if rows is None else rows, self.profile, self.robot,
                                   [(start, end, 1)], 1.)
        return next(row for row in report if row["curva"] == 1)

    def lose_line_and_brake(self):
        for row in self.rows[50:56]:
            row.update(line_lost=1, line_edge=0, line_error_m="")
        for row in self.rows[50:60]:
            row.update(traction_long_m_s2=-3.5, traction_usage_pct=127., target_m_s=.2)

    def test_recovery_braking_does_not_mask_steering_opportunity(self):
        self.lose_line_and_brake()
        csv_rows = [{key: str(value) for key, value in row.items()} for row in self.rows]
        curve = self.curve(rows=csv_rows)
        self.assertEqual(curve["episodios_perda_qtr"], 1)
        self.assertAlmostEqual(curve["tracao_p95_pct"], 127.)
        self.assertAlmostEqual(curve["tracao_pre_perda_max_pct"], 50.)
        self.assertAlmostEqual(curve["tracao_fora_recuperacao_max_pct"], 50.)
        self.assertAlmostEqual(curve["frenagem_recuperacao_s"], .1)
        self.assertEqual(curve["parametro"], "turn_feedforward")

    def test_pre_loss_peak_outside_curve_counts_at_its_entry(self):
        self.lose_line_and_brake()
        self.rows[48].update(traction_usage_pct=124., traction_long_m_s2=-3.4)
        curve = self.curve(start=.50, end=.82)
        self.assertEqual(curve["episodios_perda_qtr"], 1)
        self.assertAlmostEqual(curve["tracao_pre_perda_max_pct"], 124.)
        self.assertFalse(curve["selecionado"])

    def test_entering_curve_mid_loss_does_not_create_another_episode(self):
        self.lose_line_and_brake()
        curve = self.curve(start=.54, end=.90)
        self.assertEqual(curve["episodios_perda_qtr"], 0)
        self.assertGreater(curve["frenagem_recuperacao_s"], .04)

    def test_lateral_limit_during_recovery_remains_a_limit(self):
        self.lose_line_and_brake()
        for row in self.rows[50:60]:
            row.update(lateral_grip_pct=110., lateral_grip_limited=1, traction_usage_pct=160.)
        curve = self.curve()
        self.assertAlmostEqual(curve["lateral_max_pct"], 110.)
        self.assertFalse(curve["selecionado"])

    def test_steady_signed_qtr_error_can_trigger_anticipation_at_nominal_yaw(self):
        curve = self.curve()
        self.assertAlmostEqual(curve["giro_relativo_entrada"], 1.)
        self.assertEqual(curve["episodios_perda_qtr"], 0)
        self.assertIsNone(curve["tracao_pre_perda_max_pct"])
        self.assertGreater(curve["erro_persistente_s"], .9)
        self.assertEqual(curve["parametro"], "turn_feedforward")

    def test_wrong_qtr_bias_for_turn_does_not_trigger_anticipation(self):
        for row in self.rows:
            row["line_error_m"] = -.020
        curve = self.curve()
        self.assertFalse(curve["selecionado"])

    def test_bias_at_finish_and_launch_are_two_separate_observations(self):
        for row in self.rows[6:-6]:
            row.update(line_error_m=0., line_edge=0)
        curve = self.curve(start=-.06, end=.06)
        self.assertAlmostEqual(curve["erro_persistente_s"], .06)
        self.assertFalse(curve["selecionado"])

    def test_opposite_map_does_not_authorize_anticipation(self):
        self.profile["segments"][0].update(label="E3", curvature=-1.5)
        curve = self.curve()
        self.assertEqual(curve["mapa_coerente_pct"], 0.)
        self.assertFalse(curve["selecionado"])

    def test_unknown_grip_does_not_authorize_anticipation(self):
        for row in self.rows:
            for key in ("traction_usage_pct", "traction_long_m_s2", "lateral_grip_pct",
                        "lateral_grip_limited", "gyro_z_rad_s"):
                del row[key]
        self.assertFalse(self.curve()["selecionado"])


if __name__ == "__main__":
    unittest.main()

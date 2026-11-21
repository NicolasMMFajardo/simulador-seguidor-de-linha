"""Confirma propostas com o mesmo ruído para a referência e a candidata."""
from dataclasses import dataclass
from statistics import median

from simulator import simulate


SCREENING_SEED = 7
PAIR_SEEDS = (SCREENING_SEED, 37, 83)


@dataclass(frozen=True)
class ValidationMetrics:
    complete: bool
    duration: float
    distance: float
    lost_fraction: float
    body_off_fraction: float = 0.0

    @classmethod
    def from_run(cls, rows, reason, robot):
        last = rows[-1]
        return cls(
            reason == "volta completa", last["t_s"],
            (last["enc_left"] + last["enc_right"])*robot.meters_per_tick/2,
            sum(row["line_lost"] for row in rows)/len(rows),
            sum(1-row["body_on_line"] for row in rows)/len(rows),
        )

    @property
    def competitive(self):
        return self.complete and self.body_off_fraction == 0

    @property
    def rank(self):
        if self.competitive:
            return (0, round(self.duration, 3), self.lost_fraction)
        if self.complete:
            return (1, self.body_off_fraction, round(self.duration, 3), self.lost_fraction)
        return (2, -round(self.distance, 3), self.body_off_fraction, self.lost_fraction)


def compare_runs(reference, candidate):
    pairs = []
    for seed in PAIR_SEEDS:
        before, after = reference[seed], candidate[seed]
        delta = round(after.duration-before.duration, 3) if before.competitive and after.competitive else None
        pairs.append({
            "semente": seed, "referencia_valida": before.competitive, "candidata_valida": after.competitive,
            "referencia_tempo_s": before.duration if before.complete else None,
            "candidata_tempo_s": after.duration if after.complete else None, "delta_s": delta,
        })
    valid_runs = sum(pair["candidata_valida"] for pair in pairs)
    deltas = [pair["delta_s"] for pair in pairs if pair["delta_s"] is not None]
    typical = round(median(deltas), 3) if deltas else None
    return {"pares": pairs, "validas": valid_runs, "delta_mediano_s": typical,
            "aceita": valid_runs == len(PAIR_SEEDS) and typical is not None and typical < 0}


def run_pairs(track, robot, reference_profile, candidate_profile, reference, candidate, events, stop):
    """Executa sem Tk. Os dicionários de métricas são cópias privadas da bateria."""
    try:
        for number, seed in enumerate(PAIR_SEEDS, 1):
            for name, profile, results in (("referência", reference_profile, reference),
                                          ("candidata", candidate_profile, candidate)):
                if stop.is_set():
                    return
                if seed not in results:
                    events.put(("progress", f"Par {number}/{len(PAIR_SEEDS)} · semente {seed} · {name}"))
                    rows, _, reason = simulate(track, robot, profile, seed)
                    results[seed] = ValidationMetrics.from_run(rows, reason, robot)
        events.put(("done", (compare_runs(reference, candidate), reference, candidate)))
    except Exception as error:
        events.put(("error", str(error)))

"""Diagnóstico de uma passagem por distância e busca local por curva física."""
from bisect import bisect_left
from collections import Counter, deque
from copy import deepcopy
from dataclasses import dataclass
from math import hypot, sqrt

from simulator import segment_index

LOSS_WINDOW_S = .10


@dataclass(frozen=True)
class Region:
    number: int
    side: int
    start: float
    exit: float
    checkpoint: float
    label: str


def _regions(bends, length):
    """As curvas têm a mesma numeração do contador da pista."""
    if not bends:
        return [Region(0, 0, 0, length, length, "Reta")]
    regions = []
    for i, (start, end, side) in enumerate(bends):
        following = bends[i+1][0] if i+1 < len(bends) else bends[0][0]+length
        checkpoint = min(end+.50, following)
        direction = "D" if side > 0 else "E"
        regions.append(Region(i+1, side, start, end, checkpoint, f"Curva {i+1:02} · {direction}"))
        for lo, hi in _pieces(end, following, length):
            if hi-lo >= .10:
                label = "Reta de largada" if lo == 0 else f"Reta após C{i+1:02}"
                regions.append(Region(0, 0, lo, hi, hi, label))
    return sorted(regions, key=lambda region: region.start)


def _pieces(start, end, length):
    """Intervalos em uma única volta, inclusive para uma curva sobre a largada."""
    if start < 0:
        return [(length+start, length), (0, end)]
    if end > length:
        return [(start, length), (0, end-length)]
    return [(start, end)]


def _number(row, key, fallback=0.0):
    value = row.get(key)
    return fallback if value is None or value == "" else float(value)


@dataclass(frozen=True)
class _Grip:
    combined: float | None
    lateral: float | None
    recovery_braking: bool
    loss_start: bool
    before_loss: float | None
    outside_recovery_before_loss: float | None
    signed_bias_before_loss: float


class _LossHistory:
    """Contexto cronológico antes de dividir a passagem por regiões da pista."""
    def __init__(self, robot):
        self.limit = robot.effective_lateral_limit
        self.error_threshold = .18*robot.sensor_span
        self.recent = deque()
        self.last_lost = float("-inf")
        self.was_lost = False
        self.signed_bias = 0.0

    def observe(self, row, time, dt):
        limit = _number(row, "lateral_limit_m_s2", self.limit)
        ax = _number(row, "traction_long_m_s2", _number(row, "acc_x_m_s2", None))
        ay = _number(row, "lateral_demand_m_s2", _number(row, "acc_y_m_s2", None))
        lateral = _number(row, "lateral_grip_pct", None)
        if lateral is None and ay is not None and limit > 0:
            lateral = 100*abs(ay)/limit
        if _number(row, "lateral_grip_limited"):
            lateral = max(100., lateral or 0.)
        combined = _number(row, "traction_usage_pct", None)
        if combined is None and ax is not None and lateral is not None and limit > 0:
            combined = hypot(100*ax/limit, lateral)
        lost = bool(_number(row, "line_lost"))
        onset = lost and not self.was_lost
        if lost:
            self.last_lost = time
        target = _number(row, "target_m_s", None)
        planned = _number(row, "planned_m_s", None)
        below_plan = target is None or planned is None or target < planned*.95
        recovery = (ax is not None and ax < 0 and below_plan
                    and time-self.last_lost <= LOSS_WINDOW_S+1e-9)
        while self.recent and time-self.recent[0][0] > LOSS_WINDOW_S+1e-9:
            self.recent.popleft()
        previous = previous_clean = None
        if onset:
            previous = max((g for _, g, _ in self.recent), default=None)
            previous_clean = max((g for _, g, braking in self.recent if not braking), default=None)
        bias_before_loss = self.signed_bias if onset else 0.0
        if combined is not None:
            self.recent.append((time, combined, recovery))
        error = _number(row, "line_error_m", 0.0)
        if lost or abs(error) < self.error_threshold:
            self.signed_bias = 0.0
        else:
            sign = 1 if error > 0 else -1
            self.signed_bias = sign*(max(0., sign*self.signed_bias)+dt)
        self.was_lost = lost
        return _Grip(combined, lateral, recovery, onset, previous, previous_clean, bias_before_loss)


class _Lap:
    def __init__(self, rows, profile, robot, length, measured_progress):
        self.length = length
        self.distance = [0.0]
        self.time = [0.0]
        self.velocity = [0.0]
        self.steps = []
        previous_odom = 0.0
        history = _LossHistory(robot)
        for row in rows:
            time = float(row["t_s"])
            dt = time-self.time[-1]
            if dt <= 0:
                continue
            odom = (float(row["enc_left"])+float(row["enc_right"]))*robot.meters_per_tick/2
            position = float(row["track_progress_m"]) if measured_progress else odom
            # A primeira passagem num ponto é a que define a parcial, mesmo havendo recuo.
            self.distance.append(max(self.distance[-1], min(length, position)))
            self.time.append(time)
            self.velocity.append(_number(row, "encoder_speed_m_s", (odom-previous_odom)/dt))
            index = segment_index(profile, max(0, previous_odom))
            self.steps.append((row, dt, index, self.velocity[-1], history.observe(row, time, dt)))
            previous_odom = odom

    def at(self, distance, values):
        index = bisect_left(self.distance, distance)
        if index == len(self.distance):
            # A condição de conclusão do simulador tem tolerância de 1 mm.
            if distance == self.length and distance-self.distance[-1] <= .001:
                return values[-1]
            return None
        if index == 0:
            return values[0]
        lo, hi = self.distance[index-1:index+1]
        fraction = (distance-lo)/(hi-lo)
        return values[index-1] + fraction*(values[index]-values[index-1])

    def duration(self, start, end):
        total = 0.0
        for lo, hi in _pieces(start, end, self.length):
            t0, t1 = self.at(lo, self.time), self.at(hi, self.time)
            if t0 is None or t1 is None:
                return None
            total += t1-t0
        return total

    def samples(self, start, end):
        result = []
        for lo, hi in _pieces(start, end, self.length):
            first = max(0, bisect_left(self.distance, lo)-1)
            last = min(len(self.steps), bisect_left(self.distance, hi))
            for i in range(first, last):
                before, after = self.distance[i:i+2]
                if after > before:
                    overlap = max(0, min(after, hi)-max(before, lo))/(after-before)
                else:
                    overlap = float(lo <= before < hi)
                if overlap > 0:
                    row, dt, sector, speed, grip = self.steps[i]
                    # O QTR foi lido no início deste passo. Um evento pertence a uma única região.
                    result.append((row, dt*overlap, sector, speed, grip, lo <= before < hi))
        return result


def _metrics(samples, segments, robot, side):
    duration = sum(sample[1] for sample in samples)
    totals = Counter()
    sector_times, at_target, grip_distribution = Counter(), Counter(), []
    last_sign = reversals = 0
    last_crossing = -1.0
    elapsed = 0.0
    seen_gyro = False
    loss_count = 0
    pre_loss, pre_loss_clean, outside_recovery, lateral_peaks = [], [], [], []
    current_bias = max_bias = max_bias_before_loss = 0.
    error_threshold = .18*robot.sensor_span
    previous_time = None
    for row, dt, i, speed, grip, starts_here in samples:
        time = float(row["t_s"])
        if previous_time is not None and time < previous_time:
            # A cauda e a cabeça de uma curva sobre a largada são medidas em
            # instantes distintos desta passada, com o robô parado na largada.
            last_sign = 0
            last_crossing = -1.
            current_bias = 0.
        previous_time = time
        sector = segments[i]
        nominal = _number(row, "profile_target_m_s", sector["v"])
        planned = _number(row, "planned_m_s", nominal)
        target = _number(row, "target_m_s", planned)
        sector_times[i] += dt
        if abs(speed-nominal) <= .08*nominal and target >= .95*nominal:
            at_target[i] += dt
        for key in ("line_edge", "line_lost", "motor_sat"):
            totals[key] += _number(row, key)*dt
        totals["off"] += (1-_number(row, "body_on_line", 1))*dt
        totals["brake"] += (planned < nominal*.92)*dt
        totals["ramp"] += (target < planned*.92 and not _number(row, "line_lost"))*dt
        error = 0.0
        if row.get("line_error_m") not in (None, "") and not _number(row, "line_lost"):
            error = float(row["line_error_m"])
            totals["squares"] += dt*error**2
            totals["valid_qtr"] += dt
            # Banda morta e tempo mínimo eliminam cruzamentos causados só pelo ruído do QTR.
            sign = 1 if error > .003 else -1 if error < -.003 else 0
            if sign and sign != last_sign and elapsed-last_crossing >= .04:
                reversals += int(last_sign != 0)
                last_sign, last_crossing = sign, elapsed
        current_bias = current_bias+dt if side*error >= error_threshold else 0.
        max_bias = max(max_bias, current_bias)
        if grip.combined is not None:
            grip_distribution.append((grip.combined, dt))
            if not grip.recovery_braking:
                outside_recovery.append(grip.combined)
        if grip.lateral is not None:
            lateral_peaks.append(grip.lateral)
        totals["recovery"] += dt*grip.recovery_braking
        if grip.loss_start and starts_here:
            loss_count += 1
            if grip.before_loss is not None:
                pre_loss.append(grip.before_loss)
            if grip.outside_recovery_before_loss is not None:
                pre_loss_clean.append(grip.outside_recovery_before_loss)
            max_bias_before_loss = max(max_bias_before_loss, side*grip.signed_bias_before_loss)
        # Olhar o início do giro evita confundir um atraso de entrada com o regime da curva.
        curvature = sector.get("curvature", 0)
        if side*curvature >= .30:
            totals["aligned"] += dt
        if elapsed <= .5*duration and side*curvature >= .45 and row.get("gyro_z_rad_s") not in (None, ""):
            expected = speed*abs(curvature)
            direction = 1 if curvature > 0 else -1
            actual = direction*(float(row["gyro_z_rad_s"])-robot.gyro_bias)
            totals["expected_turn"] += dt*expected
            totals["actual_turn"] += dt*actual
            seen_gyro = True
        elapsed += dt
    cumulative = 0.0
    p95 = None
    measured_grip_s = sum(dt for _, dt in grip_distribution)
    for value, dt in sorted(grip_distribution):
        cumulative += dt
        if cumulative >= measured_grip_s*.95:
            p95 = value
            break
    denom = duration or 1
    gyro_ratio = None
    if seen_gyro and totals["expected_turn"] >= .03:
        gyro_ratio = totals["actual_turn"]/totals["expected_turn"]
    return {
        "erro_rms_mm": 1000*sqrt(totals["squares"]/totals["valid_qtr"]) if totals["valid_qtr"] else None,
        "borda_pct": 100*totals["line_edge"]/denom, "perda_qtr_pct": 100*totals["line_lost"]/denom,
        "motor_sat_pct": 100*totals["motor_sat"]/denom, "corpo_fora_pct": 100*totals["off"]/denom,
        "tracao_p95_pct": p95, "inversoes_qtr": reversals, "giro_relativo_entrada": gyro_ratio,
        "episodios_perda_qtr": loss_count, "tracao_pre_perda_max_pct": max(pre_loss, default=None),
        "tracao_pre_perda_sem_recuperacao_max_pct": max(pre_loss_clean, default=None),
        "tracao_max_pct": max((value for value, _ in grip_distribution), default=None),
        "tracao_fora_recuperacao_max_pct": max(outside_recovery, default=None),
        "lateral_max_pct": max(lateral_peaks, default=None), "frenagem_recuperacao_s": totals["recovery"],
        "erro_persistente_s": max_bias, "erro_pre_perda_s": max_bias_before_loss,
        "mapa_coerente_pct": 100*totals["aligned"]/denom if side else None,
        "no_alvo_pct": 100*sum(at_target.values())/denom,
        "frenagem_pct": 100*totals["brake"]/denom, "rampa_pct": 100*totals["ramp"]/denom,
    }, sector_times, at_target


def _hypothesis(region, report, robot):
    """Hipótese e um experimento por vez. A prioridade estima tempo exposto ao limitante."""
    if not report["completa"]:
        return "Passagem parcial", "Concluir a passagem", None
    if report["corpo_fora_pct"]:
        return "Corpo fora da faixa", "Recuperar a linha", None
    p95 = report["tracao_p95_pct"]
    lateral = report["lateral_max_pct"]
    before = report["tracao_pre_perda_sem_recuperacao_max_pct"]
    outside = report["tracao_fora_recuperacao_max_pct"]
    if lateral is not None and lateral >= 95:
        return "Demanda lateral alta", "Rever velocidade de entrada", None
    if before is not None and before >= 95:
        return "Tração alta antes da perda do QTR", "Rever velocidade de entrada", None
    if p95 is None:
        return "Tração sem medição", "Conferir a telemetria de aderência", None
    # Só atribuir o pico à frenagem quando a perda ocorreu com margem e a componente
    # lateral continuou baixa, inclusive durante a recuperação.
    recovery = (report["frenagem_recuperacao_s"] > 0 and before is not None and before < 80
                and outside is not None and outside < 80 and lateral is not None and lateral < 80)
    if p95 >= 95 and not recovery:
        return "Aderência combinada", "Rever velocidade de entrada", None
    if report["motor_sat_pct"] >= 5:
        return "Motor saturado", "Rever a demanda nas rodas", None
    rms = report["erro_rms_mm"] or 0.0
    lost, edge = report["perda_qtr_pct"], report["borda_pct"]
    if not region.side and report["inversoes_qtr"] >= 3 and rms >= 4:
        return "Oscilação na reta", "Testar maior amortecimento (Kd)", ("kd", .10, .02+.002*rms)
    yaw_ratio = report["giro_relativo_entrada"]
    if region.side:
        bias = report["erro_pre_perda_s"] if lost else report["erro_persistente_s"]
        if edge > 12 or lost > 0 or bias >= LOSS_WINDOW_S:
            if report["mapa_coerente_pct"] < 60 or (yaw_ratio is not None and yaw_ratio < -.20):
                return "Giro e mapa desalinhados", "Conferir o alinhamento do mapa", None
            # A resultante medida também limita por cima a componente lateral se
            # um CSV antigo só trouxer a demanda combinada.
            lateral_bound = lateral if lateral is not None else report["tracao_max_pct"]
            margin = outside is not None and outside < 80 and lateral_bound is not None and lateral_bound < 80
            if lost:
                margin = margin and before is not None and before < 80 and (p95 < 80 or recovery)
            if margin and bias >= LOSS_WINDOW_S-1e-9:
                if lost:
                    return ("Erro persistente antes da perda do QTR", "Testar antecipação do giro",
                            ("turn_feedforward", .20, .05+.001*edge+.003*lost))
                return ("QTR deslocado no sentido da curva", "Testar antecipação do giro",
                        ("turn_feedforward", .20, .035))
            if margin and yaw_ratio is not None and yaw_ratio < .90:
                return "Giro atrasado na entrada", "Testar antecipação do giro", ("turn_feedforward", .20, .035+.001*edge)
    if lost > 0 or edge > 20 or rms > 1000*.22*robot.sensor_span:
        return "QTR na borda ou sem leitura", "Examinar o traçado do QTR", None
    if report["tracao_p95_pct"] < 80 and report["no_alvo_pct"] >= 25:
        return "Velocidade nominal", "Testar mais velocidade", ("v", .07, report["no_alvo_pct"]*.00065)
    if report["rampa_pct"] > 20 and report["rampa_pct"] >= report["frenagem_pct"]:
        return "Rampa de aceleração", "Verificar a velocidade de saída anterior", None
    if report["frenagem_pct"] > 20:
        return "Frenagem planejada", "Examinar a próxima entrada", None
    if report["tracao_p95_pct"] >= 80:
        return "Margem de aderência pequena", "Manter e medir", None
    return "Alvo pouco atingido", "Comparar alvo aplicado e encoder", None


def _step_fraction(trials, segments, targets, key):
    """Bisseção somente para uma configuração rejeitada na mesma direção de busca."""
    fraction = 1.0
    expected_fields = {(i, key) for i in targets}
    for base, candidate, accepted in trials:
        if accepted:
            continue
        changed = set()
        for i, (before, after) in enumerate(zip(base["segments"], candidate["segments"])):
            for field in before.keys() | after.keys():
                if before.get(field) != after.get(field):
                    changed.add((i, field))
        if changed != expected_fields:
            continue
        ratios = []
        for i, value in targets.items():
            current = segments[i].get(key, 0.0)
            failed = candidate["segments"][i]
            if any(failed.get(field, 0) != segments[i].get(field, 0)
                   for field in ("v", "kp", "kd", "turn_feedforward") if field != key):
                break
            ratios.append((failed.get(key, 0.0)-current)/(value-current))
        if len(ratios) != len(targets):
            continue
        # Uma alteração manual com proporções diferentes é outro experimento.
        if min(ratios) > 0 and max(ratios) <= 1.000001 and max(ratios)-min(ratios) <= .01:
            fraction = min(fraction, .5*min(ratios))
    return fraction


def _changes(profile, indices, key, step, trials, robot):
    segments = profile["segments"]
    targets = {}
    minimums = {}
    for i in indices:
        current = segments[i].get(key, 0.0)
        if key == "turn_feedforward":
            value, minimum = min(1.0, current+step), .02
        elif key == "kd":
            value, minimum = min(15, current+max(.4, current*step)), .12
        else:
            value, minimum = min(.96*robot.max_wheel_speed, current*(1+step)), .005*current
        if value > current:
            targets[i] = value
            minimums[i] = minimum
    if not targets:
        return {}
    fraction = _step_fraction(trials, segments, targets, key)
    changes = {i: segments[i].get(key, 0.0)+(v-segments[i].get(key, 0.0))*fraction
               for i, v in targets.items()}
    if any(v-segments[i].get(key, 0.0) >= minimums[i] for i, v in changes.items()):
        return changes
    return {}


def analyze_curves(rows, profile, robot, bends, track_length, *, reference_rows=None, trials=(), complete=True):
    """Compara passagens espaciais. Em uma volta competitiva escolhe um experimento local."""
    measured = bool(rows) and all(row.get("track_progress_m") not in ("", None)
                                  for passage in (rows, reference_rows or ()) for row in passage)
    lap = _Lap(rows, profile, robot, track_length, measured)
    reference = _Lap(reference_rows, profile, robot, track_length, measured) if reference_rows else None
    competitive = complete and bool(rows) and all(_number(r, "body_on_line", 1) for r in rows)
    segments = profile["segments"]
    report, candidates = [], []
    for region in _regions(bends, track_length):
        samples = lap.samples(region.start, region.exit)
        metrics, sector_times, at_target = _metrics(samples, segments, robot, region.side)
        duration = lap.duration(region.start, region.checkpoint)
        earlier = reference.duration(region.start, region.checkpoint) if reference else None
        label = region.label
        if region.side:
            side_label = "D" if region.side > 0 else "E"
            notes = Counter()
            for i, seconds in sector_times.items():
                note = segments[i].get("label", "")
                if note.startswith(side_label):
                    notes[note] += seconds
            if notes:
                label = f"Curva {region.number:02} · {notes.most_common(1)[0][0]}"
        row = dict(curva=region.number, regiao=label, inicio_m=region.start,
                   saida_m=region.exit, marco_m=region.checkpoint, tempo_s=duration,
                   delta_s=duration-earlier if duration is not None and earlier is not None else None,
                   vel_saida_m_s=lap.at(region.exit, lap.velocity), completa=duration is not None and bool(samples),
                   setores=", ".join(str(i+1) for i in sorted(sector_times)),
                   alinhamento="pista" if measured else "odometria estimada",
                   cruza_largada=region.start < 0 or region.checkpoint > track_length,
                   **metrics)
        limit, adjustment, experiment = _hypothesis(region, row, robot)
        row.update(limitante=limit, ajuste=adjustment, selecionado=False, setores_propostos="",
                   parametro="", passo_pct=None)
        report.append(row)
        if not competitive or experiment is None:
            continue
        key, step, weight = experiment
        compatible = []
        for index, seconds in sector_times.items():
            sector = segments[index]
            curvature = sector.get("curvature", 0)
            side = 1 if curvature >= .3 else -1 if curvature <= -.3 else 0
            if side != region.side or seconds < .025:
                continue
            if key == "v" and at_target[index] < .25*seconds:
                continue
            compatible.append(index)
        changes = _changes(profile, compatible, key, step, trials, robot)
        if not changes:
            row["ajuste"] = "Rever o alinhamento dos setores" if not compatible else "Sem novo passo neste ajuste"
            continue
        if key == "v":
            priority = sum(at_target[i]*(1-segments[i]["v"]/speed) for i, speed in changes.items())
        else:
            # Um passo refinado após rejeição também disputa prioridade com os outros locais.
            priority = 0.0
            for i, value in changes.items():
                previous = segments[i].get(key, 0.0)
                full_step = step if key == "turn_feedforward" else max(.4, step*previous)
                priority += weight*sector_times[i]*(value-previous)/full_step
        candidates.append((priority, row, key, changes))
    proposal = deepcopy(profile)
    if candidates:
        _, row, key, changes = max(candidates, key=lambda item: item[0])
        for i, value in changes.items():
            proposal["segments"][i][key] = value
        row.update(selecionado=True, parametro=key,
                   setores_propostos=", ".join(str(i+1) for i in sorted(changes)))
        old_values = sum(segments[i].get(key, 0) for i in changes)
        if old_values:
            row["passo_pct"] = 100*(sum(changes.values())/old_values-1)
    return proposal, report

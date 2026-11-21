"""Geometria e arbitragem simulada do regulamento RoboCore Pro de 03/12/2021."""
from dataclasses import dataclass
from itertools import combinations, groupby
from math import acos, ceil, cos, degrees, hypot, pi, radians, sin

from simulator import DT, LINE, body_over_line, clamp

MAX_LENGTH = 60
FINISH_GAP = 1.0
ZONE_HALF_WIDTH = .20
STRAIGHT_MARGIN = .25
MIN_RADIUS = .10
MAX_ATTEMPTS = 3
SESSION_SECONDS = 300
STOP_SECONDS = 2
VALID_LAP = "volta válida"


def pro_example():
    """Estádio com a largada no meio de uma reta longa."""
    points = [(1.65, .34), (3.10, .34)]
    points.extend((3.10 + .34*cos(a), .68 + .34*sin(a))
                  for a in (-pi/2 + i*pi/40 for i in range(1, 41)))
    points.append((.34, 1.02))
    points.extend((.34 + .34*cos(a), .68 + .34*sin(a))
                  for a in (pi/2 + i*pi/40 for i in range(1, 41)))
    return points


def straight(track, center, half_length):
    ax, ay, _ = track.pose(center-half_length)
    bx, by, _ = track.pose(center+half_length)
    dx, dy = bx-ax, by-ay
    chord = hypot(dx, dy)
    if abs(chord - 2*half_length) > .006:
        return False
    count = ceil(2*half_length/.025)
    for i in range(1, count):
        x, y, _ = track.pose(center-half_length+2*half_length*i/count)
        if abs((x-ax)*dy-(y-ay)*dx)/chord > .003:
            return False
    return True


def crossings(track):
    seen, found = set(), []
    for indices in track.grid.values():
        for i, j in combinations(indices, 2):
            if (i, j) in seen:
                continue
            seen.add((i, j))
            if j == i+1 or (i == 0 and j == len(track.segments)-1):
                continue
            x, y, dx, dy, length = track.segments[i]
            tx, ty, ex, ey, other = track.segments[j]
            det = dx*ey-dy*ex
            if abs(det) < 1e-10:
                continue
            a = ((tx-x)*ey-(ty-y)*ex)/det
            b = ((tx-x)*dy-(ty-y)*dx)/det
            if 0 <= a <= 1 and 0 <= b <= 1:
                px, py = x+a*dx, y+a*dy
                if all(hypot(px-qx, py-qy) > .003 for qx, qy, *_ in found):
                    angle = degrees(acos(clamp(abs(dx*ex+dy*ey)/(length*other), 0, 1)))
                    found.append((px, py, track.starts[i]+a*length, track.starts[j]+b*other, angle))
    return found


def curvature_at(track, position, span=.035):
    ax, ay, _ = track.pose(position-span)
    bx, by, _ = track.pose(position)
    cx, cy, _ = track.pose(position+span)
    u, v, w, z = bx-ax, by-ay, cx-bx, cy-by
    denominator = hypot(u, v)*hypot(w, z)*hypot(u+w, v+z)
    return 2*(u*z-v*w)/denominator if denominator > 1e-10 else 0.0


def curvature_markers(track):
    count = ceil(track.length/.04)
    step = track.length/count
    values = [curvature_at(track, i*step) for i in range(count)]
    last = -.12
    for i, value in enumerate(values):
        change = abs(value-values[i-1])
        if change >= .6 and i*step-last >= .12:
            last = i*step
            yield last


def track_bends(track):
    """Numera curvas contínuas na linha central a partir da largada."""
    count = min(12000, ceil(track.length/.025))
    step = track.length/count
    values = [curvature_at(track, (i+.5)*step, span=.05) for i in range(count)]
    directions = [1 if value >= .45 else -1 if value <= -.45 else 0 for value in values]

    def runs():
        spans = []
        for side, block in groupby(range(count), key=directions.__getitem__):
            items = list(block)
            spans.append((items[0], items[-1]+1, side))
        return spans

    for start, end, side in runs():
        before, after = directions[start-1], directions[end % count]
        if not side and before and before == after and (end-start)*step <= .075:
            directions[start:end] = [before]*(end-start)
    bends = [(start*step, end*step, side, side*sum(values[start:end])*step)
             for start, end, side in runs() if side]
    if len(bends) > 1 and directions[0] and directions[0] == directions[-1]:
        start, _, side, tail_turn = bends.pop()
        _, end, _, head_turn = bends[0]
        bends[0] = (start-track.length, end, side, head_turn+tail_turn)
    return [(start, end, side) for start, end, side, turn in bends if turn >= radians(6)]


def track_checks(track, robot):
    length = track.length
    launch_ok = length >= 1.5 and straight(track, -.5, .75)
    size_ok = (0 < robot.overall_length <= .250 and 0 < robot.overall_width <= .250
               and 0 < robot.body_height <= .200)
    findings = [
        (length <= MAX_LENGTH, f"Faixa branca de {LINE*1000:.0f} mm · percurso de {length:.2f} m (máx. 60 m)"),
        (size_ok, f"Robô de {robot.overall_length*1000:.0f} × {robot.overall_width*1000:.0f} × "
                  f"{robot.body_height*1000:.0f} mm (máx. 250 × 250 × 200 mm)"),
        (launch_ok, "Reta de 1,50 m centrada na zona partida–chegada · portões separados por 1 m"),
    ]
    if length > MAX_LENGTH or length < 1.5:
        return findings
    intersections = crossings(track)
    bad_cross = [(s, angle) for _, _, s, t, angle in intersections
                 if angle < 85 or not (straight(track, s, .25) and straight(track, t, .25))]
    cross_text = (f"Cruzamento a {bad_cross[0][0]:.2f} m (ângulo agudo de {bad_cross[0][1]:.1f}°)"
                  if bad_cross else f"{len(intersections)} cruzamento(s) · 90° ± 5° e 25 cm retos de cada lado")
    findings.append((not bad_cross, cross_text))
    count = ceil(length/.020)
    step = length/count
    positions = [track.pose(i*step)[:2] for i in range(count)]
    peak = max(abs(curvature_at(track, i*step)) for i in range(count))
    radius = 1/peak if peak > 1e-6 else float("inf")
    findings.append((radius >= MIN_RADIUS-.002, f"Raio mínimo estimado de {radius*1000:.0f} mm (mín. 100 mm)"))

    def gap(a, b):
        return abs((a-b+length/2) % length-length/2)

    def crossing_zone(s, t):
        return any((gap(s,a) < .29 and gap(t,b) < .29) or (gap(s,b) < .29 and gap(t,a) < .29)
                   for _, _, a, b, _ in intersections)

    buckets = {}
    separation = None
    for i, (x, y) in enumerate(positions):
        col, row = int(x//.20), int(y//.20)
        for a in range(col-1, col+2):
            for b in range(row-1, row+2):
                for j in buckets.get((a,b), ()):
                    t, s = j*step, i*step
                    if gap(s,t) <= .40:
                        continue
                    tx, ty = positions[j]
                    width = hypot(x-tx, y-ty)
                    if width < .198 and not crossing_zone(s,t):
                        if separation is None or width < separation[0]:
                            separation = width, s, t
        buckets.setdefault((col,row), []).append(i)
    if separation:
        width, s, t = separation
        findings.append((False, f"Separação estimada de {width*1000:.0f} mm entre {s:.2f} e {t:.2f} m (mín. 200 mm)"))
    else:
        findings.append((True, "Separação entre trechos estimada em pelo menos 200 mm"))
    clearance = True
    for gate in (0, length-FINISH_GAP):
        x, y, _ = track.pose(gate)
        if any(gap(i*step, gate) > .30 and hypot(tx-x, ty-y) < .198
               for i, (tx, ty) in enumerate(positions)):
            clearance = False
    findings.append((clearance, "Distância da faixa não local aos portões de pelo menos 200 mm"))
    return findings


def calibrate_stop_odom(rows, frames, robot, track_length):
    """Lê a distância dos encoders no meio da zona, numa volta de validação."""
    middle = track_length - FINISH_GAP/2
    scale = robot.meters_per_tick/2
    earlier_odom = 0.0
    for row, previous, current in zip(rows, frames, frames[1:]):
        odom = (float(row["enc_left"]) + float(row["enc_right"]))*scale
        if previous.progress_m < middle <= current.progress_m:
            fraction = (middle-previous.progress_m)/(current.progress_m-previous.progress_m)
            return earlier_odom + fraction*(odom-earlier_odom)
        earlier_odom = odom
    return None


@dataclass
class OfficialTrial:
    budget: float
    launch_offset: float = .18
    stop_offset: float = .55
    start_s: float | None = None
    finish_s: float | None = None
    stopped_at_s: float | None = None
    stopped_for_s: float = 0.0
    furthest_m: float = -.18
    stop_odom_m: float | None = None

    def stop_distance(self, profile_length):
        if self.stop_odom_m is not None:
            return self.stop_odom_m
        return profile_length - FINISH_GAP + self.stop_offset

    @property
    def lap_s(self):
        return self.finish_s-self.start_s if self.finish_s is not None else None

    def observe(self, track, robot, x, y, heading, previous, progress, time, stopped, *, on_line=None):
        half_l, half_w = robot.body_length/2, robot.body_width/2
        if on_line is None:
            on_line = body_over_line(track, x, y, heading, half_l, half_w, forward=robot.body_center)
        if not on_line:
            return "volta inválida · corpo saiu da faixa"
        self.furthest_m = max(self.furthest_m, progress)
        if self.furthest_m-progress > .06:
            return "volta inválida · percorreu a faixa no sentido contrário"
        if self.start_s is None and previous < 0 <= progress:
            self.start_s = time-DT + DT*(-previous)/(progress-previous)
        finish = track.length-FINISH_GAP
        if self.start_s is not None and self.finish_s is None and previous < finish <= progress:
            self.finish_s = time-DT + DT*(finish-previous)/(progress-previous)
        if not stopped:
            self.stopped_at_s = None
            return None
        if self.finish_s is None:
            return "volta inválida · parou antes da chegada"
        sx, sy, direction = track.pose(0)
        c, s = cos(direction), sin(direction)
        relative = heading-direction
        along = (x-sx)*c+(y-sy)*s
        lateral = -(x-sx)*s+(y-sy)*c
        front_axis, side_axis = cos(relative), sin(relative)
        for offset, box_half_l, box_half_w in robot.footprint_boxes:
            box_along = along + offset*front_axis
            box_lateral = lateral + offset*side_axis
            envelope_l = abs(front_axis)*box_half_l + abs(side_axis)*box_half_w
            envelope_w = abs(side_axis)*box_half_l + abs(front_axis)*box_half_w
            if not (-FINISH_GAP+envelope_l <= box_along <= -envelope_l
                    and abs(box_lateral)+envelope_w <= ZONE_HALF_WIDTH):
                return "volta inválida · parou fora da área partida–chegada"
        if self.stopped_at_s is None:
            self.stopped_at_s = time
        self.stopped_for_s = time-self.stopped_at_s
        if self.stopped_for_s >= STOP_SECONDS-1e-9:
            return VALID_LAP
        return None

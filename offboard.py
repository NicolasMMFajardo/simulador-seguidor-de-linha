"""Arquivos intercambiáveis com o ESP32 e execução offboard por terminal."""
import argparse
import csv
import json
from pathlib import Path

from curve_analysis import analyze_curves
from regulation import track_bends
from simulator import Robot, Track, diagnose, generate


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_csv(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as file:
        return list(csv.DictReader(file))


def write_csv(path, rows):
    with Path(path).open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def describe_proposal(report, tested, base, proposal, action):
    """Liga o relatório técnico medido ao próximo perfil que será testado."""
    for row, measured, before, after in zip(report, tested["segments"], base["segments"], proposal["segments"]):
        row.update(v_proposta_m_s=after["v"], kp_proposto=after["kp"], kd_proposto=after["kd"],
                   antecipacao_proposta=after.get("turn_feedforward", 0))
        if after != before:
            row["acao"] = action
        elif measured != before:
            row["acao"] = "restaurar referência"
        else:
            row["acao"] = "manter" if row["tempo_s"] else "sem amostras"


def diagnose_pass(rows, profile, robot, complete, bends, length, *, reference_rows=None, trials=()):
    recovery, report = diagnose(rows, profile, robot, complete, search_speed=False)
    candidate, curves = analyze_curves(rows, profile, robot, bends, length,
                                       reference_rows=reference_rows, trials=trials, complete=complete)
    if complete and rows and all(int(row.get("body_on_line", 1)) for row in rows):
        action = next((row["ajuste"] for row in curves if row["selecionado"]), "manter")
        describe_proposal(report, profile, profile, candidate, action)
    else:
        candidate = recovery
        for row, sector in zip(report, candidate["segments"]):
            row["antecipacao_proposta"] = sector.get("turn_feedforward", 0)
    return candidate, report, curves


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", help="JSON opcional com os campos de Robot")
    commands = parser.add_subparsers(dest="command", required=True)
    mapping = commands.add_parser("mapa")
    mapping.add_argument("telemetria")
    mapping.add_argument("perfil")
    mapping.add_argument("--peso-gyro", type=float, default=.75)
    review = commands.add_parser("diagnostico")
    review.add_argument("telemetria")
    review.add_argument("perfil")
    review.add_argument("proposta")
    review.add_argument("relatorio")
    review.add_argument("--incompleta", action="store_true")
    review.add_argument("--pista", help="JSON da pista para o diagnóstico por curva e saída")
    review.add_argument("--referencia", help="CSV de outra validação na mesma pista para calcular os deltas")
    args = parser.parse_args()
    config = read_json(args.robot) if args.robot else {}
    config.pop("yaw_inertia_kg_m2", None)
    robot = Robot(**config)
    rows = read_csv(args.telemetria)
    if not rows:
        parser.error("CSV vazio")
    if args.command == "mapa":
        if not 0 <= args.peso_gyro <= 1:
            parser.error("--peso-gyro deve ficar entre 0 e 1")
        write_json(args.perfil, generate(rows, robot, args.peso_gyro))
    else:
        profile = read_json(args.perfil)
        if args.referencia and not args.pista:
            parser.error("Informe --pista para comparar as passagens por curva.")
        if args.pista:
            track = Track(read_json(args.pista)["points_m"])
            reference = read_csv(args.referencia) if args.referencia else None
            proposal, report, curves = diagnose_pass(rows, profile, robot, not args.incompleta,
                                                     track_bends(track), track.length, reference_rows=reference)
            output = Path(args.relatorio)
            write_csv(output.with_name(output.stem+"-curvas.csv"), curves)
        else:
            proposal, report = diagnose(rows, profile, robot, not args.incompleta)
        write_json(args.proposta, proposal)
        write_csv(args.relatorio, report)


if __name__ == "__main__":
    main()

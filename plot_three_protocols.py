#!/usr/bin/env python3
"""Create research plots from actual NS-3 per-protocol CSV/JSON outputs.
No synthetic or placeholder measurements are generated.
"""
import argparse, csv, json
from pathlib import Path
import matplotlib.pyplot as plt

PROTOCOLS = ["Direct", "LEACH", "C-LEACH"]


def read_csv(path):
    if not path.exists():
        raise FileNotFoundError(f"Missing simulation output: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"No rows in {path}")
    return rows


def read_metrics(root, protocol):
    path = root / protocol / "candidate_metrics.json"
    if not path.exists():
        raise FileNotFoundError(f"Missing simulation metrics: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def save(fig, output, name):
    fig.tight_layout()
    fig.savefig(output / name, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", type=Path, default=Path("leach_results"))
    ap.add_argument("--output", type=Path, default=Path("leach_graphs"))
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    state = {}
    metrics = {}
    for p in PROTOCOLS:
        state[p] = read_csv(args.results / p / "node_states.csv")
        metrics[p] = read_metrics(args.results, p)

    # Residual energy over rounds: average across sensors at each round.
    fig, ax = plt.subplots(figsize=(8, 5))
    for p in PROTOCOLS:
        by_round = {}
        for r in state[p]:
            rnd = int(r["round"])
            by_round.setdefault(rnd, []).append(float(r["residual_energy_j"]))
        xs = sorted(by_round)
        ys = [sum(by_round[x]) / len(by_round[x]) for x in xs]
        ax.plot(xs, ys, marker="o", label=p)
    ax.set(title="Average Residual Energy vs Simulation Round",
           xlabel="Simulation round", ylabel="Average residual energy (J)")
    ax.grid(True, alpha=.3); ax.legend()
    save(fig, args.output, "residual_energy.png")

    # Network lifetime. FND/HND/LND are rounds at which 1/50/100% of
    # nodes have reached zero energy. Values are derived from node-state rows.
    lifetime = {}
    for p in PROTOCOLS:
        death_round = {}
        for r in state[p]:
            node = int(r["node_id"]); rnd = int(r["round"])
            energy = float(r["residual_energy_j"])
            if energy <= 0 and node not in death_round:
                death_round[node] = rnd
        total_nodes = len({int(r["node_id"]) for r in state[p]})
        vals = list(death_round.values())
        fnd = min(vals) if vals else None
        hnd = None
        lnd = max(vals) if len(vals) >= total_nodes else None
        # First round where at least half of nodes have died.
        if vals:
            for rnd in sorted(set(vals)):
                if sum(v <= rnd for v in vals) >= (total_nodes + 1)//2:
                    hnd = rnd; break
        lifetime[p] = (fnd, hnd, lnd)

    fig, ax = plt.subplots(figsize=(8, 5))
    labels = ["FND", "HND", "LND"]
    x = list(range(len(PROTOCOLS))); width = .24
    for j, label in enumerate(labels):
        vals = [lifetime[p][j] if lifetime[p][j] is not None else float("nan") for p in PROTOCOLS]
        ax.bar([i + (j-1)*width for i in x], vals, width, label=label)
    ax.set_xticks(x, PROTOCOLS)
    ax.set(title="Network Lifetime Comparison", xlabel="Protocol", ylabel="Round")
    ax.grid(axis="y", alpha=.3); ax.legend()
    save(fig, args.output, "network_lifetime.png")

    # PDR, PLR, throughput, delay from actual FlowMonitor-derived metrics.
    metric_specs = [
        ("pdr_percent", "Packet Delivery Ratio (%)", "pdr.png"),
        ("packet_loss_ratio", "Packet Loss Ratio", "packet_loss.png"),
        ("throughput_mbps", "Throughput (Mbps)", "throughput.png"),
        ("mean_delay_ms", "Average End-to-End Delay (ms)", "delay.png"),
        ("energy_consumed_j", "Energy Consumed (J)", "energy_consumed.png"),
    ]
    for key, title, filename in metric_specs:
        vals = [float(metrics[p].get(key, float("nan"))) for p in PROTOCOLS]
        fig, ax = plt.subplots(figsize=(7, 5))
        ax.bar(PROTOCOLS, vals)
        ax.set(title=title + " by Protocol", xlabel="Protocol", ylabel=title)
        ax.grid(axis="y", alpha=.3)
        save(fig, args.output, filename)

    # Save exact metric table for paper/review.
    with (args.output / "metrics_comparison.csv").open("w", newline="", encoding="utf-8") as f:
        fields = ["protocol", "pdr_percent", "packet_loss_ratio", "throughput_mbps",
                  "mean_delay_ms", "energy_consumed_j", "remaining_energy_j"]
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
        for p in PROTOCOLS:
            writer.writerow({"protocol": p, **{k: metrics[p].get(k, "") for k in fields[1:]}})
    print(f"Generated plots and comparison CSV in: {args.output.resolve()}")

if __name__ == "__main__":
    main()

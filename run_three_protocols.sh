#!/usr/bin/env bash
set -euo pipefail

# Run from the NS-3 root directory, e.g. ~/ns3/ns-3-dev.
# Ensure scratch/leach_three_protocols.cc is copied here first.
ROOT="$(pwd)"
OUT="${ROOT}/leach_results"
mkdir -p "$OUT"

for protocol in Direct LEACH C-LEACH; do
  dir="$OUT/$protocol"
  mkdir -p "$dir"
  echo "========== Running $protocol =========="
  ./ns3 run "scratch/leach_three_protocols --protocol=${protocol} --scenarioId=${protocol} --stateCsv=${dir}/node_states.csv --metricsOut=${dir}/candidate_metrics.json --truncateStateCsv=1 --seed=1 --run=1 --nSensors=100 --chProbability=0.05 --rounds=100 --roundDuration=10"
done

python3 plot_three_protocols.py --results "$OUT" --output "${ROOT}/leach_graphs"
echo "Results: $OUT"
echo "Graphs:  ${ROOT}/leach_graphs"

# LoRa Mesh Routing Simulator

A packet-level Python simulator for evaluating LoRa mesh routing strategies under
shared propagation, airtime, collision, node-failure, and energy assumptions.
The project is designed for researchers and developers who want to compare
routing behavior fairly across different protocol ideas without reproducing
firmware internals from a specific product.

This simulator currently includes two behavior-equivalent baselines:

- `meshtastic`: managed flooding with delayed rebroadcast and suppression when a
  duplicate copy is heard.
- `meshcore`: route discovery with RREQ/RREP exchange followed by cached source
  routes for repeated unicast conversations.

The code also supports interference, static obstacles, node-failure schedules,
energy accounting, and richer survival analysis for network resilience studies.

See [docs/terminology-and-formulas.md](docs/terminology-and-formulas.md) for the
project terminology, parameter list, and the propagation/LoRa formulas used in
simulation.

## Quick start

Clone the repository and run a baseline comparison:

```bash
git clone <repository-url>
cd lora-mesh-routing-sim
python3 lora_mesh_sim.py --protocol both --nodes 40 --duration-s 1800 --traffic unicast --seeds 3
```

Run a repeated-unicast scenario that is better suited to route caching:

```bash
python3 lora_mesh_sim.py --protocol both --nodes 40 --duration-s 1800 --traffic unicast --pair-count 6 --seeds 5
```

Write the results to CSV for later analysis:

```bash
python3 lora_mesh_sim.py --protocol both --nodes 80 --duration-s 3600 --traffic mixed --pair-count 10 --seeds 20 --csv results/baselines.csv
```

Aggregate a multi-seed CSV with the helper script:

```bash
python3 analyze_results.py results/baselines.csv
```

## Core parameters

Useful simulation knobs include:

```bash
--protocol meshtastic|meshcore|both
--nodes 80
--area-m 3000
--duration-s 3600
--rate-per-min 6
--traffic unicast|broadcast|mixed
--pair-count 10
--seeds 30
--seed0 1
--max-hops 7
--repeater-ratio 0.0
--sf 9
--bw-hz 125000
--cr 1
--payload-bytes 32
--tx-power-dbm 17
--path-loss-exp 2.7
--shadow-sigma-db 4
--capture-threshold-db 6
--csv results/output.csv
```

Additional research extensions supported by the current code:

```bash
--interference-enabled
--interference-rate 2.0
--interference-power-dbm 20.0
--obstacles-file obstacles.json
--node-failures-file failures.json
--energy-j 1000
--rx-power-w 0.05
--idle-power-w 0.001
--detailed-csv results/detailed.csv
```

## Example scenarios

```bash
# Repeated unicast traffic, where route caching can help.
python3 lora_mesh_sim.py --protocol both --nodes 50 --area-m 3000 --duration-s 1200 --traffic unicast --rate-per-min 6 --pair-count 8 --seeds 20 --csv results/exp_50n_unicast_pairs.csv

# Mixed traffic, with both broadcast and unicast traffic.
python3 lora_mesh_sim.py --protocol both --nodes 50 --area-m 3000 --duration-s 1200 --traffic mixed --rate-per-min 6 --pair-count 8 --seeds 20 --csv results/exp_50n_mixed.csv
```

## Model and assumptions

The PHY model includes:

- log-distance path loss with log-normal shadowing
- fixed LoRa SF/BW/CR settings for all nodes
- LoRa time-on-air estimation
- half-duplex radio behavior
- collision handling with capture threshold logic
- probabilistic packet success based on SNR/SINR margins
- optional background interference and obstacle attenuation
- energy accounting for TX, RX, and idle power consumption

The simulator produces virtual RSSI and SNR values from the modeled channel,
which makes different protocols comparable under the same topology, traffic, and
radio assumptions.

## Metrics reported

The summary table and CSV include metrics such as:

- `unicast_pdr`
- `broadcast_coverage`
- `avg_delay_s`
- `tx_count`
- `data_tx`
- `control_tx`
- `total_airtime_s`
- `airtime_per_delivery_s`
- `collision_fail`
- `duplicate_rx`
- `suppressed_forwards`
- `route_cache_hits`
- `route_cache_misses`
- `avg_rssi_dbm`
- `avg_snr_db`
- `total_energy_consumed_j`

These values are useful for comparing reliability, overhead, delay, and survival
performance across scenarios.

## Included example results

The repository ships with a few sample outputs in the `results/` folder:

- `meshcore_survival.csv`
- `meshtastic_survival.csv`
- `surviva1l_data.csv`

These files are examples of the output format produced by the simulator and can be
replaced or regenerated for your own experiments as the model evolves.

## Extending the simulator

Add a new protocol class in `lora_mesh_sim.py` and register it in
`build_protocol()`. The main hooks are:

```python
def send_app(self, src: int, dst: int, flow_id: int) -> None:
    ...

def on_receive(self, receiver: int, packet: Packet, rx: RxInfo) -> None:
    ...
```

`RxInfo` contains simulated receive values such as receive power, SNR, SINR,
collision status, and link-quality information. This is the place to implement
forwarding policies that consider airtime, SNR margin, or route availability.

## Scope

This project is a packet-level research simulator. It is not a direct clone of
Meshtastic or MeshCore firmware, and it is not a physical waveform simulator.
Its purpose is to compare protocol behaviors under a common LoRa PHY, topology,
traffic model, and energy/robustness assumptions.

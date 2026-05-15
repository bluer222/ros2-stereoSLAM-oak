# RTAB-Map Tuning Workflow

This workflow is separate from the existing launch files so you can tune RTAB-Map without disturbing the current setup.

It is built around one idea:

1. Record one representative bag.
2. Replay that same bag many times.
3. Change only RTAB-Map parameters between runs.
4. Keep notes and scores for each run.
5. Review each generated RTAB-Map database and rank the results.

## What This Gives You

- `record_processed_bag.sh`
  Brings up the live stereo/depth pipeline and records the processed topics that RTAB-Map actually consumes.
- `run_trial.sh`
  Runs `rgbd_odometry` + `rtabmap` against a bag with one preset.
- `presets/*.yaml`
  A few starter parameter sets.
- `results/summary.csv`
  A growing ledger of runs and scores.
- `rank_results.py`
  Prints the best run overall and the best run per metric.

## Recommended Bag Contents

For tuning RTAB-Map specifically, record the already-processed topics instead of raw camera data:

- `/stereo/left/image_rect`
- `/stereo/left/camera_info`
- `/stereo/depth/image_rect`
- `/tf`
- `/tf_static`

That isolates RTAB-Map tuning from changes in the stereo/depth generation pipeline.

## Quick Start

Record a reusable bag:

```bash
./tools/rtabmap_tuning/record_processed_bag.sh my_loop_bag
```

By default this starts `/workspace/launch/launch_all.py` so the processed stereo/depth topics exist while recording. If you want a different live pipeline, pass:

```bash
./tools/rtabmap_tuning/record_processed_bag.sh \
  --launch-file /workspace/launch/launch_raft.py \
  my_loop_bag
```

Run one experiment:

```bash
./tools/rtabmap_tuning/run_trial.sh \
  --bag /workspace/tools/rtabmap_tuning/bags/my_loop_bag \
  --preset /workspace/tools/rtabmap_tuning/presets/00_baseline.yaml \
  --open-db-viewer
```

Run another preset against the same bag:

```bash
./tools/rtabmap_tuning/run_trial.sh \
  --bag /workspace/tools/rtabmap_tuning/bags/my_loop_bag \
  --preset /workspace/tools/rtabmap_tuning/presets/10_conservative.yaml
```

See which runs scored best:

```bash
python3 ./tools/rtabmap_tuning/rank_results.py
```

## Suggested Tuning Session

Use one bag with:

- straight motion
- turns
- some low-texture area
- some textured area
- at least one loop closure

Then run:

1. `00_baseline.yaml`
2. `10_conservative.yaml`
3. `20_aggressive_loops.yaml`

After each run:

1. Inspect RViz during playback if you want.
2. Open the generated database with `rtabmap-databaseViewer`.
3. Score the run.
4. Add a short note about what failed or improved.

## Scoring Rubric

Use `0-5` for each category.

- `odom_stability`
  Did odometry stay locked and smooth?
- `loop_closure`
  Did it close loops correctly without obvious false matches?
- `map_quality`
  Was the graph and map geometrically clean?
- `speed`
  Did it keep up well enough for your target deployment?
- `failure_recovery`
  Did it recover if tracking got shaky?

The script computes `overall` as the average of those five scores.

## Database Review

The viewer you are thinking of is RTAB-Map's database viewer, not RViz itself.

Typical command:

```bash
rtabmap-databaseViewer /path/to/rtabmap.db
```

Use it to compare runs side by side over time:

- open each run's `rtabmap.db`
- check graph deformation
- inspect loop closures
- look for repeated drift patterns
- note where one preset beats another

## Result Layout

Each run gets its own folder under `results/runs/`:

- `metadata.txt`
- `preset.yaml`
- `rtabmap.db`
- `notes.txt`
- `commands.sh`

The global scoreboard is:

- `results/summary.csv`

## Good Workflow Rules

- Change only one cluster of parameters at a time.
- Reuse the same bag for all comparisons.
- Start with odometry stability before chasing loop-closure aggressiveness.
- Keep the environment and playback rate consistent.
- Do not reuse old databases across trials.

## Preset Strategy

- `00_baseline`
  A balanced default starting point.
- `10_conservative`
  Safer matching, fewer risky loop closures.
- `20_aggressive_loops`
  More eager loop closure behavior for testing recovery and relocalization.

Once you find a direction that works, clone the best preset and make smaller edits from there.

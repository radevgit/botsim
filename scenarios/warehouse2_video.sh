#!/usr/bin/env bash
# Makes data/warehouse2.mp4 (10 minutes, 1920x1080, 30 fps) from scenarios/warehouse2.toml:
# 6000 simulated seconds at speed 10 = 18000 frames. It takes long: about 15 minutes to
# simulate (the recording is about 750 MB), 15 to 20 to draw the frames, 10 to encode.
# Run it from anywhere:   bash scenarios/warehouse2_video.sh
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p data

echo "0/3 remove the previous export"
rm -rf data/warehouse2_frames

echo "1/3 simulate and record"
cargo run --release -p botsim -- --record data/warehouse2.jsonl --record-interval 0.5 --geometry scenarios/warehouse2.geometry.json scenarios/warehouse2.toml > data/warehouse2_summary.json
wc -l data/warehouse2.jsonl
grep -E '"(collision_count|deadlock_count)"' data/warehouse2_summary.json

echo "2/3 draw the frames (18000 of them, about 15 to 20 minutes)"
cargo run --release -p botsim-export -- data/warehouse2.jsonl data/warehouse2_frames --speed 10

echo "3/3 make the movie (about 10 minutes)"
ffmpeg -y -framerate 30 -i data/warehouse2_frames/frame_%06d.png -c:v libx264 -preset slow -crf 18 -vf "scale=out_color_matrix=bt709:out_range=tv,format=yuv420p" -colorspace bt709 -movflags +faststart data/warehouse2.mp4

echo "done: data/warehouse2.mp4"

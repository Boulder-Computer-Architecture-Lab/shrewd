#!/bin/bash

# Copy simpoint BB files from results/dir/subdir/simpoint.bb.gz
# into simpoints/dir_subdir.bb.gz

RESULTS_DIR="results"
OUTPUT_DIR="simpoints"

mkdir -p "$OUTPUT_DIR"

for bbfile in "$RESULTS_DIR"/*/*/simpoint.bb.gz; do
    # Extract dir and subdir from the path
    dir=$(basename "$(dirname "$(dirname "$bbfile")")")
    subdir=$(basename "$(dirname "$bbfile")")

    dest="$OUTPUT_DIR/${dir}_${subdir}.bb"

    echo "Copying: $bbfile -> $dest"
    gunzip -c "$bbfile" > "$dest"
done

echo "Done."
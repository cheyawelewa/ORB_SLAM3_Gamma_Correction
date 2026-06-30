#!/usr/bin/env bash
#
# download_euroc.sh
# Downloads the EuRoC MAV dataset (ASL format) into the ORB-SLAM3 dataset dir.
# Mirrors the DOWNLOAD_DATASET logic from ORB_SLAM3/CMakeLists.txt, but runs
# standalone so you can re-pull data without re-running a full cmake configure.
#
# Sequences land at: $DATASET_DIR/<sequence>/mav0/...
# The .bag rosbags inside each bundle are ignored (ORB-SLAM3 only needs ASL).
#
# By default, sequences that are already present (mav0/cam0/data exists and is
# non-empty) are skipped, and a bundle whose sequences are all present is not
# downloaded at all. Use --force to wipe and re-pull everything.

set -euo pipefail

# --- args --------------------------------------------------------------------
FORCE=0
usage() {
  cat <<EOF
Usage: $(basename "$0") [-f|--force]

  -f, --force   Re-download and overwrite every sequence, even if present.
  -h, --help    Show this help.

Env:
  DATASET_DIR   Target dir (default: \$PWD/Datasets/EuRoc)
EOF
}
while [ $# -gt 0 ]; do
  case "$1" in
    -f|--force) FORCE=1; shift ;;
    -h|--help)  usage; exit 0 ;;
    *) echo "ERROR: unknown argument '$1'" >&2; usage; exit 1 ;;
  esac
done

# Default: Datasets/EuRoc under the directory you run the script from ($PWD),
# i.e. download in place into the current repo root. Override by exporting
# DATASET_DIR before running, e.g.
#   DATASET_DIR=/some/other/path ./download_euroc.sh
DATASET_DIR="${DATASET_DIR:-$PWD/Datasets/EuRoc}"
STAGING_DIR="$DATASET_DIR/_tmp_dl"

echo ">>> Downloading into: $DATASET_DIR"

# Unified per-environment bundles from the ETH Research Collection:
# https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f
declare -A BUNDLES=(
  [machine_hall]="https://www.research-collection.ethz.ch/server/api/core/bitstreams/7b2419c1-62b5-4714-b7f8-485e5fe3e5fe/content"
  [vicon_room1]="https://www.research-collection.ethz.ch/server/api/core/bitstreams/02ecda9a-298f-498b-970c-b7c44334d880/content"
  [vicon_room2]="https://www.research-collection.ethz.ch/server/api/core/bitstreams/ea12bc01-3677-4b4c-853d-87c7870b8c44/content"
)

# Standard EuRoC ASL sequence names per bundle. Used only for the bundle-level
# "all present -> skip download" check. If a name doesn't match what's actually
# inside the zip, the worst case is an unnecessary re-download (no data loss).
declare -A BUNDLE_SEQS=(
  [machine_hall]="MH_01_easy MH_02_easy MH_03_medium MH_04_difficult MH_05_difficult"
  [vicon_room1]="V1_01_easy V1_02_medium V1_03_difficult"
  [vicon_room2]="V2_01_easy V2_02_medium V2_03_difficult"
)

# --- helpers -----------------------------------------------------------------
# A sequence counts as "present" if mav0/cam0/data exists and has >=1 png.
seq_present() {
  local d="$DATASET_DIR/$1"
  [ -d "$d/mav0/cam0/data" ] || return 1
  local n
  n=$(find "$d/mav0/cam0/data" -maxdepth 1 -name '*.png' 2>/dev/null | wc -l)
  [ "$n" -gt 0 ]
}

# --- sanity checks -----------------------------------------------------------
for cmd in wget unzip find; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "ERROR: '$cmd' is required but not installed." >&2
    exit 1
  fi
done

mkdir -p "$STAGING_DIR"

# --- download + extract ------------------------------------------------------
for name in "${!BUNDLES[@]}"; do
  url="${BUNDLES[$name]}"

  # Bundle-level skip: if every expected sequence is already present, don't
  # even download this bundle.
  if [ "$FORCE" -ne 1 ]; then
    all_present=1
    for s in ${BUNDLE_SEQS[$name]}; do
      if ! seq_present "$s"; then all_present=0; break; fi
    done
    if [ "$all_present" -eq 1 ]; then
      echo ">>> Skipping $name (all sequences already present)"
      continue
    fi
  fi

  zip="$STAGING_DIR/$name.zip"
  extract="$STAGING_DIR/$name"
  mkdir -p "$extract"

  echo ">>> Downloading $name ..."
  # -c resumes a partial download from a previous interrupted run
  if ! wget -c "$url" -O "$zip"; then
    echo "ERROR: failed to download $name from $url" >&2
    exit 1
  fi

  echo ">>> Extracting $name ..."
  (
    cd "$extract"
    unzip -o "$zip" >/dev/null

    # The bundle contains a single top-level dir holding per-sequence folders.
    # Each per-sequence folder holds an inner ASL <sequence>.zip (the one with
    # mav0/ inside) plus a .bag we ignore.
    top=$(find . -maxdepth 1 -mindepth 1 -type d | head -n1)
    [ -n "$top" ] && cd "$top"

    found_any=0
    for d in */; do
      d="${d%/}"

      # Per-sequence skip: leave a present sequence untouched.
      if [ "$FORCE" -ne 1 ] && seq_present "$d"; then
        echo "    skip $d (already present)"
        found_any=1
        continue
      fi

      inner=$(find "$d" -maxdepth 1 -name '*.zip' | head -n1 || true)
      if [ -n "$inner" ]; then
        rm -rf "$DATASET_DIR/$d"
        mkdir -p "$DATASET_DIR/$d"
        unzip -o "$inner" -d "$DATASET_DIR/$d" >/dev/null
        found_any=1
      elif [ -d "$d/mav0" ]; then
        rm -rf "$DATASET_DIR/$d"
        mv "$d" "$DATASET_DIR/"
        found_any=1
      fi
    done

    # Fallback: per-sequence zips sitting flat at this level.
    for z in *.zip; do
      [ -e "$z" ] || continue
      seq="${z%.zip}"
      if [ "$FORCE" -ne 1 ] && seq_present "$seq"; then
        echo "    skip $seq (already present)"
        found_any=1
        continue
      fi
      rm -rf "$DATASET_DIR/$seq"
      mkdir -p "$DATASET_DIR/$seq"
      unzip -o "$z" -d "$DATASET_DIR/$seq" >/dev/null
      found_any=1
    done

    if [ "$found_any" -ne 1 ]; then
      echo "ERROR: no sequences found in $name" >&2
      exit 1
    fi
  )

  rm -f "$zip"
  rm -rf "$extract"
done

rm -rf "$STAGING_DIR"

echo ""
echo "EuRoC dataset at: $DATASET_DIR"
echo ""
echo "Sequences:"
find "$DATASET_DIR" -maxdepth 1 -mindepth 1 -type d | sort | while read -r d; do
  n=$(find "$d/mav0/cam0/data" -maxdepth 1 -name '*.png' 2>/dev/null | wc -l)
  printf "  %-24s  %s cam0 frames\n" "$(basename "$d")" "$n"
done
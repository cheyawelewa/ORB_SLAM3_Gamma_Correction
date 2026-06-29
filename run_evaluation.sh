#!/bin/bash
# Evaluate ATE and RPE for all EuRoC result folders matching 2026-*
# Results are saved as JSON per run, then one plot per sequence is generated.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATASETS_DIR="$SCRIPT_DIR/Datasets/EuRoc"
EVAL_SCRIPT="$SCRIPT_DIR/evaluation/evaluate_trajectory.py"
PLOT_SCRIPT="$SCRIPT_DIR/evaluation/plot_summary.py"
RESULTS_DIR="$SCRIPT_DIR/evaluation_results"

# Map dataset ID -> EuRoC sequence folder name
declare -A GT_MAP
GT_MAP["MH01"]="MH_01_easy"
GT_MAP["MH02"]="MH_02_easy"
GT_MAP["MH03"]="MH_03_medium"
GT_MAP["MH04"]="MH_04_difficult"
GT_MAP["MH05"]="MH_05_difficult"
GT_MAP["V101"]="V1_01_easy"
GT_MAP["V102"]="V1_02_medium"
GT_MAP["V103"]="V1_03_difficult"
GT_MAP["V201"]="V2_01_easy"
GT_MAP["V202"]="V2_02_medium"
GT_MAP["V203"]="V2_03_difficult"

folders=("$SCRIPT_DIR"/2026-*)
if [ ! -d "${folders[0]}" ]; then
    echo "No folders matching '2026-*' found in $SCRIPT_DIR"
    exit 1
fi

echo "Found ${#folders[@]} result folder(s)"
mkdir -p "$RESULTS_DIR"

for folder in "${folders[@]}"; do
    folder_name="$(basename "$folder")"

    for frame_file in "$folder"/f_dataset-*_stereo_inertial.txt; do
        [ -f "$frame_file" ] || continue

        fname="$(basename "$frame_file")"
        dataset_id="$(echo "$fname" | sed 's/f_dataset-\(.*\)_stereo_inertial\.txt/\1/')"

        seq_folder="${GT_MAP[$dataset_id]}"
        if [ -z "$seq_folder" ]; then
            echo "[SKIP] $folder_name — no groundtruth mapping for: $dataset_id"
            continue
        fi

        gt_path="$DATASETS_DIR/$seq_folder/mav0/state_groundtruth_estimate0/data.csv"
        if [ ! -f "$gt_path" ]; then
            echo "[SKIP] groundtruth not found: $gt_path"
            continue
        fi

        kf_file="$folder/kf_dataset-${dataset_id}_stereo_inertial.txt"

        echo "================================================================="
        echo "Folder  : $folder_name"
        echo "Dataset : $dataset_id  ->  $seq_folder"

        if [ -f "$kf_file" ]; then
            python3 "$EVAL_SCRIPT" "$gt_path" \
                --frames "$frame_file" \
                --keyframes "$kf_file" \
                --output_dir "$RESULTS_DIR" \
                --dataset "$dataset_id" \
                --run "$folder_name"
        else
            python3 "$EVAL_SCRIPT" "$gt_path" \
                --frames "$frame_file" \
                --output_dir "$RESULTS_DIR" \
                --dataset "$dataset_id" \
                --run "$folder_name"
        fi
    done
done

# Generate one plot per sequence from all collected results
echo "================================================================="
echo "Generating plots..."
python3 "$PLOT_SCRIPT" "$RESULTS_DIR"
echo "Done. Plots saved to: $RESULTS_DIR"

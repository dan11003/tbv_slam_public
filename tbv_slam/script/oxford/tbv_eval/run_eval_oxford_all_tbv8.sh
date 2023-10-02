TBV_DIR=`rospack find tbv_slam`
SCRIPT_PATH="$TBV_DIR/script/run_eval.sh"
PARAMETER_FILE="oxford/par_oxford_all_tbv_8.csv"
DISABLE_OUTPUT="False"
"$SCRIPT_PATH" "-p" "$PARAMETER_FILE" "-o" "$DISABLE_OUTPUT"

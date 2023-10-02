TBV_DIR=`rospack find tbv_slam`
SCRIPT_PATH="$TBV_DIR/script/run_eval.sh"
PARAMETER_FILE="par_oxford_tbv_8.csv"
DISABLE_OUTPUT="True"
"$SCRIPT_PATH" "-p" "$PARAMETER_FILE" "-o" "$DISABLE_OUTPUT"
TBV_DIR=`rospack find tbv_slam`
SCRIPT_PATH="$TBV_DIR/script/run_eval.sh"
PARAMETER_FILE="oxford/par_oxford_loop_closure.csv"
DISABLE_OUTPUT="True"
PARMETER_ORDER="True"
"$SCRIPT_PATH" "-p" "$PARAMETER_FILE" "-o" "$DISABLE_OUTPUT" "-r" "$PARMETER_ORDER"

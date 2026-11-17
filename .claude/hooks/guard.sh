#!/bin/sh
# Claude Code PreToolUse hook (AGENTS.md, "The machine" and "Recordings are evidence"). Reads
# the tool call as JSON on stdin and prints a decision, or nothing to let it through:
# - mc601 against the machine: refused. Only the operator runs that, at the machine.
# - mc601 writing the operator's own sessions, calibration file or config: asks first.
# - edits to recordings and blessed programs: refused.
set -uf

input=$(cat)
root=$(cd "${CLAUDE_PROJECT_DIR:-.}" 2>/dev/null && pwd -P) || exit 0

say() { # decision reason
  jq -n --arg d "$1" --arg r "$2" \
    '{hookSpecificOutput: {hookEventName: "PreToolUse", permissionDecision: $d, permissionDecisionReason: $r}}'
  exit 0
}

if ! command -v jq >/dev/null 2>&1; then
  # The call can't be read: stop only what might matter.
  case $input in *mc601* | *testdata/*)
    echo "guard.sh needs jq to check this call" >&2
    exit 2
    ;;
  esac
  exit 0
fi

field() { printf '%s' "$input" | jq -r "$1 // \"\""; }

# "@" and the mc601 arguments of one simple command, or nothing if it doesn't run mc601.
mc601_args() {
  s=$(printf '%s' "$1" | sed -E -e 's/^[[:space:]]+//' -e ':a' \
    -e 's/^([A-Za-z_][A-Za-z0-9_]*=("[^"]*"|'"'"'[^'"'"']*'"'"'|[^[:space:]]*)|time|env|nohup|exec|command|rtk|timeout[[:space:]]+[0-9.]+[smh]?)[[:space:]]+//' \
    -e 'ta')
  first=${s%%[[:space:]]*}
  case ${first##*/} in
  mc601) printf '@%s\n' "${s#"$first"}" ;;
  cargo)
    case $s in
    *[[:space:]]run[[:space:]]*mc601*" -- "*) printf '@%s\n' "${s#*" -- "}" ;;
    *[[:space:]]run[[:space:]]*mc601*) echo @ ;;
    esac
    ;;
  esac
}

# Whether VAR sends mc601 somewhere other than the operator's own files.
given() {
  case $cmd in *"$1="*) return 0 ;; esac
  [ "$1" = MC601_CALIBRATION ] && [ -n "$calflag" ] && return 0
  [ -n "$(printenv "$1")" ]
}

check_mc601() { # args
  sub="" sub2="" sim="" help="" calflag="" cfgwrite="" skip=""
  # shellcheck disable=SC2086 # split into words on purpose; globbing is off
  set -- $1
  for w; do
    if [ -n "$skip" ]; then skip=""; continue; fi
    case $w in
    --sim | --sim-limits) sim=1 ;;
    -h | --help | -V | --version) help=1 ;;
    --calibration) calflag=1 skip=1 ;;
    --calibration=*) calflag=1 ;;
    --port | --port=* | --forget-port | --data-dir | --data-dir=*) cfgwrite=1 ;;
    -*) ;;
    *) if [ -z "$sub" ]; then sub=$w; elif [ -z "$sub2" ]; then sub2=$w; fi ;;
    esac
  done
  if [ -n "$help" ] || [ "$sub" = help ]; then return 0; fi

  # Whether it opens the port, and the variables that keep it out of the operator's files.
  port="" writes=""
  case "$sub $sub2" in
  "run "* | "probe "* | "check "*) port=1 writes=MC601_DATA ;;
  "calibrate heads" | "calibrate runons") port=1 writes="MC601_DATA MC601_CALIBRATION" ;;
  "calibrate read") writes="MC601_DATA MC601_CALIBRATION" ;;
  "calibration set" | "calibration unset" | "calibration revert") writes=MC601_CALIBRATION ;;
  "config "*) [ -z "$cfgwrite" ] || writes=MC601_CONFIG ;;
  esac

  if [ -n "$port" ] && [ -z "$sim" ]; then
    say deny "mc601 $sub without --sim talks to the real 601. Agents never do that (AGENTS.md, The machine). Use --sim with MC601_DATA, MC601_CONFIG and MC601_CALIBRATION set to a scratch folder, or ask the operator to run it at the machine and bring back the log."
  fi
  missing=""
  for v in $writes; do given "$v" || missing="$missing $v"; done
  if [ -n "$missing" ]; then
    say ask "mc601 $sub $sub2 writes the operator's own files: the sessions in ~/Documents/mc601 (the newest is what 'calibrate read' takes), the machine's calibration file, or the config. Set$missing to a scratch path unless the operator asked for this."
  fi
}

case $(field .tool_name) in
Bash)
  cmd=$(field .tool_input.command)
  case $cmd in *mc601*) ;; *) exit 0 ;; esac
  # A heredoc's body is text, not commands: read only the line that starts it.
  case $cmd in *"<<"*) cmd=$(printf '%s\n' "$cmd" | sed -n 1p) ;; esac
  printf '%s\n' "$cmd" | tr ';|&()`' '\n\n\n\n\n\n' | while IFS= read -r seg; do
    args=$(mc601_args "$seg")
    case $args in @*) check_mc601 "${args#@}" ;; esac
  done
  ;;
Edit | Write | MultiEdit | NotebookEdit)
  path=$(field '.tool_input.file_path // .tool_input.notebook_path')
  case $path in "") exit 0 ;; /*) ;; *) path=$PWD/$path ;; esac
  dir=$(cd "$(dirname "$path")" 2>/dev/null && pwd -P) && path=$dir/${path##*/}
  case $path in
  "$root"/testdata/captures/*)
    say deny "testdata/captures/ holds recordings of the serial line: never edited (AGENTS.md, Recordings are evidence)."
    ;;
  "$root"/testdata/sessions/*.log)
    say deny "testdata/sessions/*.log are logs from the machine: never edited or written by hand. A replay that differs means the simulator is wrong: fix crates/sim or SimConfig::default(). To add a log, copy it in (the session-log skill)."
    ;;
  "$root"/testdata/programs/*.json)
    say deny "testdata/programs/*.json are written from the captures: BLESS=1 cargo test -p machine --test golden, and only when lowering changed on purpose (AGENTS.md, Recordings are evidence)."
    ;;
  esac
  ;;
esac
exit 0

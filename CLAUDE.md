Read AGENTS.md first: it is the instructions for working in this repository.

@AGENTS.md

## In Claude Code

- `.claude/hooks/guard.sh` runs before every shell command and file edit. It refuses `mc601`
  against the machine, asks before `mc601` writes the operator's own files, and refuses edits to
  recordings and blessed programs. If it stops you, the reason says what to do instead. Don't
  work around it.
- Before committing, run the `simplify` skill on the diff (AGENTS.md, Code). For a change to
  `protocol`, `link`, `controller`, `runner`, `machine` or `sim`, have the `machine-safety`
  agent review it too.
- The `session-log` skill takes a log from the machine into `testdata/sessions/`, replayed by
  the simulator.

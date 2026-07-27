# Guidance for Claude Code

- When running `find`, always scope the search to a directory you actually have access to (e.g. `.` or a specific project path). Never run `find` against `/`, since scanning the whole filesystem can exhaust system resources.
- Avoid shell commands that set variables inline (e.g. `VAR=value cmd`, `echo $VAR`, backtick/`$()` command substitution assigned to a variable) when the same command can be formulated without them. These trigger "simple expansion" allow/deny prompts instead of matching a plain allowed command, causing unnecessary permission prompts.
- To interrupt a hung or long-running `cargo` or `probe-rs` invocation, prefer `killall probe-rs` / `killall cargo` over `kill <pid>` — PID-based `kill` only ever matches a one-off permission grant for that PID, so it prompts again next time. `killall` matches by process name machine-wide, but that's fine here: the user doesn't run `cargo` in parallel with Claude, so there's no unrelated process to catch.

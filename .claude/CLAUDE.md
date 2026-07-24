# Guidance for Claude Code

- When running `find`, always scope the search to a directory you actually have access to (e.g. `.` or a specific project path). Never run `find` against `/`, since scanning the whole filesystem can exhaust system resources.
- Avoid shell commands that set variables inline (e.g. `VAR=value cmd`, `echo $VAR`, backtick/`$()` command substitution assigned to a variable) when the same command can be formulated without them. These trigger "simple expansion" allow/deny prompts instead of matching a plain allowed command, causing unnecessary permission prompts.

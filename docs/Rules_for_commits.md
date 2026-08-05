# Rules for commits

Good commits make scientific software easier to review, reproduce, maintain, and cite.

## Commit-message rules

1. Separate the subject from the body with a blank line.
2. Keep the subject concise; approximately 50 characters is a useful target.
3. Capitalize the subject.
4. Do not end the subject with a period.
5. Use the imperative mood: `Add`, `Fix`, `Move`, `Document`, or `Refactor`.
6. Wrap the body at approximately 72 characters where practical.
7. Explain **what changed and why**. The diff should explain most implementation details.

Example:

```text
Move external interfaces to top level

Expose external-program interfaces as a primary LADAC area rather
than treating them as general utilities. This makes ArduPilot,
FlightGear, and related integrations easier to discover.

Update affected library links and documentation paths.

Resolves: #123
```

## Keep commits focused

Avoid:

- mixing whitespace-only changes with functional changes,
- mixing unrelated changes,
- combining directory moves with large behavioral changes,
- committing generated files accidentally,
- one giant commit for a feature that can be reviewed in stages.

A useful sequence for a structural change is:

1. move or rename files without behavioral changes,
2. repair links and paths,
3. update documentation,
4. make functional improvements in later commits.

## Commit bodies

Include relevant context:

- What problem does the change solve?
- Why was this approach selected?
- What behavior changes?
- Are there compatibility consequences?
- Are paths, interfaces, parameters, or generated files affected?
- What tests were run?
- What remains unsupported?

Do not assume that future readers know the original issue.

## Referencing issues

Put issue references at the end:

```text
Resolves: #123
See also: #456
```

## Submodules

When updating a Git submodule:

- state which dependency was updated,
- record the old and new commits in the pull-request description,
- summarize relevant upstream changes,
- run affected tests,
- avoid combining unrelated submodule updates.

## Sources

These conventions are based on established Git commit-message guidance, including Chris Beams' *How to Write a Git Commit Message*, OpenStack's Git commit-message guidance, and earlier GENIVI/Atlassian recommendations.

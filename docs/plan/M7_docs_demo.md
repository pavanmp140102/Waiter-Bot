# M7: Documentation and demo

**Goal:** a reviewer can understand, run, and judge the project from the repository alone, in minutes.

**Depends on:** M5 and M6.

## Sub-tasks

- [ ] Rewrite the root `README.md` (the only README in the repository): one-paragraph summary, demo recording, quick start of three commands, the architecture diagram, a table of the seven situations with their scenario command, a link to `docs/index.md`.
- [ ] Per-requirement walkthroughs in `docs/walkthroughs.md`: for each of R1 to R7, the situation, the state path that handles it, the command that replays it, the expected log and state output, and a recording.
- [ ] Record the demos: one short clip per scenario, plus one full run with the dashboard. Keep each file small (under 3 MB; GIF or WebM) in `docs/demo/`.
- [ ] Final pass on every document in `docs/`: parameter defaults, command lines, test IDs and file names match the code.
- [ ] Fill in the **Result** section of every milestone file, with what was built, what was measured, and what differed from the plan. This is the approach record the assessment asks for.
- [ ] Design notes: what worked, what was hard, and what the first submission got wrong, in `docs/design_notes.md`. Do not link `REVIEW.md`: it is an internal file and stays out of the repository.
- [ ] Remove anything unused: dead launch files, staging folders from M0, old assets.
- [ ] Check package metadata, versions and licences.
- [ ] Run the README quick start on a clean checkout (MAN3).

## Files created or changed

```
README.md
docs/{walkthroughs.md,design_notes.md,demo/}
docs/plan/*.md (Result sections)
```

## Gate

- MAN3: the quick start works from a clean machine in under 10 minutes.
- S5 and S8 still pass: no foreign paths, repository under 15 MB.
- Every link in `README.md` and `docs/` resolves (checked with a small script).

## Notes

- There is exactly one README, at the repository root. Other documents are named for their content.
- Show the real behaviour, including limitations. A short "known limitations" section is part of a credible write-up.

## Result

(filled in when the milestone lands)

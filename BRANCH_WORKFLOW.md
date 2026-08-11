# Branch workflow

Use these two long-lived working branches for GK development and measurements.

## `agent-dev`

Development branch. All experimental changes, debugging, diagnostics, and unvalidated fitting improvements start here.

Before promoting a change:

1. Build and run the app locally.
2. Run the relevant Rust tests/checks.
3. Validate the behavior needed for the next measurement.
4. Commit and push the tested state to `agent-dev`.

Do not use `agent-dev` as the assumed measurement build merely because it compiles.

## `gk_measurement`

Measurement-ready branch. This should point only to a state that has been explicitly tested and accepted for data acquisition.

Normal promotion procedure:

```bash
git switch agent-dev
git status --short
# test/validate here, then commit and push

git switch gk_measurement
git pull --ff-only origin gk_measurement
git merge --ff-only agent-dev
git push origin gk_measurement

git switch agent-dev
```

If `--ff-only` cannot promote the branch, stop and inspect the history instead of force-updating `gk_measurement`.

After a measurement session, continue new development on `agent-dev`; do not modify `gk_measurement` until another tested build is intentionally promoted.

## Historical branches

`agent-live-fit-fixes` and `rotation-stage-ready-2026-08-10` are historical names superseded by `agent-dev` and `gk_measurement`, respectively. Keep them only as needed for transition/history; do not use them for new work.

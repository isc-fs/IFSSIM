<!--
Pull request template. Fill in each section; delete bullets that
don't apply rather than leaving them blank. The CI gate will catch
the mechanical issues; this template's job is to surface the human
ones (intent, scope, test plan, user-visible impact).
-->

## What this PR does

<!--
One paragraph: the goal, the approach, and the trade-off you picked
if there was one. Don't repeat the diff — explain the *why* the diff
doesn't make obvious.
-->

## Test plan

<!--
- [ ] Concrete steps you ran to verify the change works
- [ ] Edge cases or regressions you specifically checked for
- [ ] If applicable: screenshot, log excerpt, or sample output
-->

## Checklist

- [ ] CI gate is green (Bridge build / Frontend / Backend / Tracks all passing)
- [ ] PR targets `dev`, not `main`
- [ ] If user-visible: a one-liner is added under `## [Unreleased]` in `CHANGELOG.md`
- [ ] If touching `Plugins/`, `Config/`, `Content/`, `IFSSIM.uproject`, or `package_*.{sh,ps1}`: I cooked locally and the staged sim runs (UE5 dev side has no GitHub-hosted CI gate)
- [ ] If adding a new sensor / RPC / topic: `docs/REFERENCE.md` or `docs/AUTONOMY.md` is updated to match

## Closes

<!--
Issue this PR closes — the auto-tracking workflow opens an issue per
branch under [feat/N] / [fix/N], so the corresponding number lives
in the Issues tab. `Closes #NNN` triggers GitHub to close it on
merge automatically.
-->

Closes #

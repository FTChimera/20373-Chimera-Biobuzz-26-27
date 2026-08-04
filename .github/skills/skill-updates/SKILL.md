---
name: skill-updates
description: Use this skill when asked to update `.github/skills/README.md` and one or more `SKILL.md` files so they reflect the latest repository changes/commit history or the current codebase state.
---

# Skill Updates

Use this skill when the user asks for documentation maintenance work on the repo-specific Copilot skills under `.github/skills/`.

Typical prompts include requests like:

- “Update all the `SKILL.md` files, and the `.github/skills/README.md` on all the latest changes.”
- “Refresh the skills docs based on recent commits.”
- “Make the skills match the current TeamCode architecture.”
- “Refresh the repo-specific programming guide in `ftcdev-20373.html` so its docs cards, snippets, and branding still match TeamCode.”

## Default contract for skill-update requests

- Start by checking what the user actually wants updated.
- Unless the user explicitly insists on updating **all** skill files, only update the files that are actually necessary.
- If the user says “update all,” you should still determine whether some files truly need no changes; if so, tell the user that those files were reviewed and left unchanged.
- If the update scope or history boundary is ambiguous, state the assumption you used and proceed.
- Prefer direct verification against the current tree, and use commit history only when the user allows it and the request actually depends on recent-change boundaries.

## Important repo-specific rule about history range

This repo already uses commits that explicitly mention skills updates.

When the user wants a history-bounded refresh and allows git-history inspection, that means:

- you do **not** need to re-document every older repo change from the beginning of time
- instead, find the most recent commit that already updated the skills docs
- then inspect the commits **after** that point and update only what is newly relevant

If you discover an older documentation gap that is still materially wrong and should be corrected, tell the user and update it if appropriate.

If the user explicitly wants a codebase-only audit or bans history / diff inspection, skip the history-bounding step and audit the current tree directly.

## If history is allowed, start with commit history

A good recent-history snapshot is:

```zsh
git --no-pager log --stat --decorate -4
```

Use that as an initial recent-history snapshot when the request is recent-history-driven.

Also use commands like these to find the correct doc-update boundary and the latest relevant code changes:

```zsh
git --no-pager log --stat --decorate -4
git --no-pager log --decorate --oneline --grep='skill' -i --all | cat
git --no-pager log --decorate --oneline -- .github/skills TeamCode | cat
git --no-pager diff --name-only <LAST_SKILLS_COMMIT>..HEAD | cat
git --no-pager log --stat --decorate <LAST_SKILLS_COMMIT>..HEAD -- TeamCode .github/skills | cat
```

In this repo, a command like this is especially useful for locating the most recent skills-update boundary from scratch:

```zsh
git --no-pager log --decorate --oneline --grep='skill' -i --all | cat
```

If the most recent skills-update commit title already says it updated the skills, you usually only need to inspect the commits after that point.

Useful ways to figure out the latest commits from scratch:

```zsh
git --no-pager log --decorate --oneline -20 | cat
git --no-pager log --stat --decorate -10 | cat
git --no-pager log --decorate --oneline --all --graph | cat
```

If you need to inspect only the recent code under `TeamCode/`:

```zsh
git --no-pager log --stat --decorate -10 -- TeamCode | cat
```

## Workflow for a skill-doc update request

1. Read `.github/copilot-instructions.md` and `.github/skills/README.md` first.
2. If history is allowed and relevant, find the most recent skills-update commit.
3. If history is allowed and relevant, inspect commits after that point.
4. Inspect the actual current code, not just commit messages.
5. Map which skill files are affected by those changes.
6. `Constants.currentBot` can change quickly, do not update SKILL files/README.md to state the current profile as it might become outdated quickly.
7. Update only the necessary skills unless the user explicitly asks for a blanket refresh.
8. If the index file `.github/skills/README.md` should mention a new pattern, package, or skill, update it too.
9. If `ftcdev-20373.html` is present and acts as repo documentation, review whether its branding, links, docs cards, and TeamCode-derived snippets also need to change alongside the skill docs.
10. After editing docs, validate the changed markdown files for obvious consistency issues and summarize exactly which skill files changed.

## What kinds of code changes usually require skill updates

- new architecture conventions in `Constants`, `RobotConstants`, `RobotConfigurations`, or `Mechanism`
- new mechanisms or wrappers under `mechanisms/` or `library/hardware/...`
- a new `library/` utility that changes house style, such as `OpModeHelper.kt`
- new custom FTC hardware devices under `library/hardware/hardwareDevice/`
- changes to how tests should reference hardware names or profile-managed factories
- new op mode base classes, variants, or pose-handoff patterns
- new Pedro/Ivy workflow changes
- new tuning packages or calibration op modes such as `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pid/autoTune/`
- new third-party build wiring in `build.dependencies.gradle`, especially new repositories or driver-station-facing libraries
- new build, lint, or validation expectations
- new skill directories or renamed skills
- repo documentation drift in `ftcdev-20373.html`, especially stale snippets, outdated BioBuzz architecture notes, or non-FTChimera links

## How to decide which skill files to touch

Examples:

- If the change adds a new slides mechanism layer around `SlidesMotor`, update:
  - `mechanisms-and-hardware-wrappers`
  - `teamcode-library-architecture`
  - `tests-and-tuning-opmodes`
  - `library-utilities-and-shared-types` if the lower-level device layer changed expectations
- If the change centralizes test hardware naming through `RobotConfigurations` or `*_PARAMS`, update:
  - `tests-and-tuning-opmodes`
  - `build-and-validation`
  - `robot-configurations-and-follower-builder`
  - any skill whose examples referenced stale test-only strings
- If the change adds a lifecycle helper like `OpModeHelper.kt`, update:
  - `library-utilities-and-shared-types`
  - `ftc-sdk-hardwaremap-and-lifecycle`
  - possibly `teamcode-library-architecture`
- If the change adds a new tuning package or driver-station-visible calibration flow like `pid/autoTune`, update:
  - `tests-and-tuning-opmodes`
  - `build-and-validation`
  - `ftc-sdk-opmodes`
  - `ftc-sdk-hardwaremap-and-lifecycle`
  - `constants-not-configured` if the new package introduces its own configurable tuning constants

## Rules for editing the skills docs

- Keep the docs repo-specific and practical.
- Prefer concrete file names and actual current symbols.
- Remove stale references when current architecture supersedes them.
- Do not claim a pattern is active unless you verified it in the current code.
- When updating `ftcdev-20373.html`, source examples from `TeamCode/` instead of inventing generic FTC snippets, and keep FTChimera / 20373 RMS Chimera branding current.
- Distinguish between:
  - profile-managed `Mechanism`/factory flows
  - lower-level direct `hardwareMap.get(...)` custom devices
  - test/proof-of-concept exceptions
- If a behavior is intentionally provisional or preseason, say so.
- If a recent change only affected one narrow area, do not rewrite unrelated skill files just for style consistency.

## Recommended final report shape

When done, report:

1. which code/doc scope you used, and if allowed, which commits/history range you used
2. which skill files were updated
3. which skill files were reviewed but did not need changes
4. any older gaps you noticed that predate the last skills-update commit
5. any assumptions or open questions that would help future skill maintenance

## Assumptions to surface when needed

If you would normally ask these, state the assumption you chose and proceed:

- Should I update only the skill files affected since the last skills-update commit, or do you want a full sweep anyway?
- If I find older stale skill content that predates the last skills update, do you want me to fix it now or just report it?
- Do you want me to also update non-skill documentation such as `README.md` when the same architecture changes apply there?

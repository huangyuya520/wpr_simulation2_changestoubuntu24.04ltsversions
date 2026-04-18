# Architecture Notes

## 2026-04-18 Git divergence handling

- Root cause: local `main` and `origin/main` each had one unique commit, so `git pull` stopped because no pull strategy was configured.
- Operational decision: prefer `rebase` for this repository so local feature work is replayed on top of the latest `origin/main` and the history stays linear.
- Pitfall found: Python cache artifacts under `launch/__pycache__/` and `src/__pycache__/` were versioned. These binary files caused the rebase conflict because Git cannot safely auto-merge `.pyc`.
- Remediation: add a repository-level `.gitignore` for `__pycache__/` and `*.pyc`, then remove tracked cache artifacts from version control to prevent repeated sync failures.

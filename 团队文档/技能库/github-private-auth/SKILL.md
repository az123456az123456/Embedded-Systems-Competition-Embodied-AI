---
name: github-private-auth
description: Authenticate to GitHub and clone/push private repositories using the GitHub CLI device flow, with fixes for the token-not-persisted issue and git credential helper setup. Use when a private repo requires credentials, gh login reports success but no token is saved, or git can't push because credentials are missing.
metadata:
  short-description: Set up GitHub private-repo access (gh device flow + git credentials)
---

# GitHub private-repo authentication

## Recommended: gh device flow (token stays on the machine)
1. Install the gh binary (put it in a writable folder if HOME is read-only). GitHub release tarballs work: `gh_<ver>_linux_amd64.tar.gz` from `github.com/cli/cli/releases`.
2. Set config locations to writable paths before running anything:
   - `GH_CONFIG_DIR=<tools>/gh-config`
   - `GIT_CONFIG_GLOBAL=<tools>/gitconfig` (gh writes `git_protocol` there; `~/.gitconfig` may be read-only)
3. Run `gh auth login --hostname github.com --git-protocol https --web` in a PTY. Answer Y, then give the user the one-time code to enter at `https://github.com/login/device`.
4. **Gotcha:** if gh fails to write `~/.gitconfig` (read-only), it aborts *before saving the oauth_token* — login looks successful but `grep -c oauth_token hosts.yml` is 0. Fix by setting `GIT_CONFIG_GLOBAL` to a writable file first, then redo the device flow. If it still won't persist, run the device flow manually via `POST https://github.com/login/device/code` + polling `https://github.com/login/oauth/access_token` (client_id `178c6fc778ccc68e1d6a`) and write `hosts.yml` yourself with `oauth_token`.
5. Set up git credentials: `gh auth setup-git` (writes the gh credential helper into `GIT_CONFIG_GLOBAL`). Cloning then works with `git clone https://github.com/<owner>/<repo>.git`.
6. For commits, set repo-local `user.name` / `user.email` (e.g. `<login>@users.noreply.github.com`).

## Alternative: fine-grained PAT
Create a fine-grained token limited to the one repo (Contents: Read and write) at `https://github.com/settings/personal-access-tokens/new`, then use it as the git password or store via a credential helper. Revoke when done.

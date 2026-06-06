#!/usr/bin/env bash
# Configure Docker CLI to use the GNOME secretservice keyring for credentials.
# Result: `docker login` stores tokens in the OS keyring, encrypted at rest,
# unlocked by your desktop login. No plaintext in ~/.docker/config.json.
#
# Run with: bash scripts/setup_docker_creds.sh

set -euo pipefail
c_grn=$'\e[32m'; c_red=$'\e[31m'; c_off=$'\e[0m'
log()  { echo "${c_grn}[creds]${c_off} $*"; }
warn() { echo "${c_red}[creds]${c_off} $*" >&2; }

# 1) helper binary
if ! command -v docker-credential-secretservice >/dev/null; then
  log "installing golang-docker-credential-helpers (sudo)"
  sudo apt-get update -qq
  sudo apt-get install -y golang-docker-credential-helpers libsecret-1-0
else
  log "docker-credential-secretservice already installed"
fi

# 2) sanity-check the keyring is up
if ! pgrep -x gnome-keyring-d >/dev/null && ! pgrep -f 'gnome-keyring-daemon' >/dev/null; then
  warn "gnome-keyring-daemon doesn't appear to be running"
  warn "  (log out and back in, or start it: gnome-keyring-daemon --start --components=secrets)"
fi

# 3) wire Docker config
CFG="${HOME}/.docker/config.json"
mkdir -p "$(dirname "$CFG")"
if [ -f "$CFG" ]; then
  cp "$CFG" "${CFG}.bak.$(date +%s)"
  log "backed up existing config to ${CFG}.bak.*"
fi

python3 - <<'PY'
import json, os, pathlib
cfg_path = pathlib.Path.home() / ".docker" / "config.json"
data = {}
if cfg_path.exists():
    try:
        data = json.loads(cfg_path.read_text())
    except json.JSONDecodeError:
        data = {}
data["credsStore"] = "secretservice"
# Strip any stale plaintext auths from before this change
data.setdefault("auths", {})
cfg_path.write_text(json.dumps(data, indent=2) + "\n")
print("Wrote", cfg_path)
PY

log "config now uses credsStore=secretservice. To log in:"
echo
echo "  docker login nvcr.io"
echo "  Username: \$oauthtoken"
echo "  Password: <paste NGC API key, characters won't show>"
echo
log "after login, pull Isaac Sim:"
echo "  cd ${HOME}/GR_Assignment/docker && docker compose pull isaac"

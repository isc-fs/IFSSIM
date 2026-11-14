#!/usr/bin/env bash
# Sets up the bench PC (or any Linux GPU box) to run IFSSIM's GPU runner, on a clean
# Ubuntu 24.04 LTS: the roadmap's "host script". Each step is idempotent, so running it
# again repairs a box rather than breaking it.
#
#   sudo tools/ci/setup_bench_host.sh --dry-run          # print what it would do
#   sudo tools/ci/setup_bench_host.sh                    # every step but the runner's registration
#   sudo RUNNER_TOKEN=<token> tools/ci/setup_bench_host.sh runner   # register (token: org settings ->
#                                                        # Actions -> Runners -> New runner)
#   sudo tools/ci/setup_bench_host.sh docker nvidia      # only these steps
#
# Steps (in order): base driver docker nvidia vulkan gh tailscale user dirs runner
#   base       git, git-lfs, curl, jq, Python, flock
#   driver     the NVIDIA driver ubuntu-drivers recommends, then held (apt-mark hold), so an
#              unattended upgrade can't swap it under a running sim
#   docker     Docker CE and its compose plugin from Docker's apt repository (not the snap:
#              it can't use --gpus and hides /tmp from containers)
#   nvidia     the NVIDIA Container Toolkit, as Docker's nvidia runtime
#   vulkan     vulkan-tools (vulkaninfo) and the Vulkan loader
#   gh         the GitHub CLI
#   tailscale  Tailscale (then, by hand: sudo tailscale up --ssh)
#   user       the runner's user ($RUNNER_USER, default `runner`): no sudo, in the docker group
#   dirs       $CI_HOME (default /build/ifssim-ci), owned by the runner's user
#   runner     the GitHub Actions runner in ~runner/actions-runner, registered to the org with
#              only its own labels (self-hosted,Linux,X64,gpu), as a systemd service; needs
#              RUNNER_TOKEN. Its .env sets CI_HOME and UE5_ROOT for the workflows.
#
# Not here: Unreal Engine (built from source with an Epic-linked GitHub account, see
# tools/ci/README.md), BIOS settings, and the LattePanda.
#
# Env:
#   RUNNER_USER     default runner
#   CI_HOME         default /build/ifssim-ci
#   UE5_ROOT        written into the runner's .env (default $CI_HOME/UE_5.8)
#   RUNNER_TOKEN    the registration token (step runner only)
#   RUNNER_URL      default https://github.com/isc-fs
#   RUNNER_GROUP    default Default (the org's group limited to IFSSIM and IFS09, when it exists)
#   RUNNER_NAME     default the host name
#   RUNNER_LABELS   default self-hosted,Linux,X64,gpu
set -euo pipefail

RUNNER_USER="${RUNNER_USER:-runner}"
CI_HOME="${CI_HOME:-/build/ifssim-ci}"
UE5_ROOT="${UE5_ROOT:-$CI_HOME/UE_5.8}"
RUNNER_URL="${RUNNER_URL:-https://github.com/isc-fs}"
RUNNER_GROUP="${RUNNER_GROUP:-Default}"
RUNNER_NAME="${RUNNER_NAME:-$(hostname)}"
RUNNER_LABELS="${RUNNER_LABELS:-self-hosted,Linux,X64,gpu}"
ALL=(base driver docker nvidia vulkan gh tailscale user dirs runner)
DEFAULT=(base driver docker nvidia vulkan gh tailscale user dirs)

DRY=0
steps=()
for a in "$@"; do
  case "$a" in
    --dry-run|-n) DRY=1 ;;
    -h|--help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *)
      printf '%s\n' "${ALL[@]}" | grep -qx "$a" || { echo "unknown step: $a (steps: ${ALL[*]})" >&2; exit 2; }
      steps+=("$a") ;;
  esac
done
[ ${#steps[@]} -gt 0 ] || steps=("${DEFAULT[@]}")

if [ "$DRY" = 0 ] && [ "$(id -u)" != 0 ]; then
  echo "run it with sudo (or --dry-run to see what it does)" >&2
  exit 1
fi
if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${VERSION_ID:-}" = "24.04" ] || echo "warning: written for Ubuntu 24.04, this is ${PRETTY_NAME:-unknown}" >&2
fi
CODENAME="${VERSION_CODENAME:-noble}"
ARCH="$(dpkg --print-architecture 2>/dev/null || echo amd64)"

# run: a command, or its line with --dry-run
run() {
  if [ "$DRY" = 1 ]; then printf '  $ %s\n' "$*"; else "$@"; fi
}
# write <file> <content>: a file, or what it would hold
write() {
  # shellcheck disable=SC2001  # indent every line of a multi-line file
  if [ "$DRY" = 1 ]; then printf '  > %s\n%s\n' "$1" "$(sed 's/^/      /' <<<"$2")"; else printf '%s\n' "$2" > "$1"; fi
}
apt_install() { run env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"; }
# keyring <url> <file> [binary]: an apt repository's signing key; an ASCII-armored one is
# dearmored, a binary one (gh's, Tailscale's) saved as it is
keyring() {
  run install -m 0755 -d /etc/apt/keyrings
  if [ "${3:-}" = binary ]; then
    run curl -fsSL -o "$2" "$1"
  elif [ "$DRY" = 1 ]; then
    printf '  $ curl -fsSL %s | gpg --dearmor -o %s\n' "$1" "$2"
  else
    curl -fsSL "$1" | gpg --dearmor --yes -o "$2"
  fi
  run chmod a+r "$2"
}
runner_home() { getent passwd "$RUNNER_USER" | cut -d: -f6 || true; }

step_base() {
  run apt-get update
  apt_install ca-certificates curl gnupg git git-lfs jq python3 python3-venv python3-yaml \
    util-linux rsync unzip
  run git lfs install --system
}

step_driver() {
  apt_install ubuntu-drivers-common
  if command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
    echo "  an NVIDIA driver already answers: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
  else
    run ubuntu-drivers install
    echo "  reboot once this script is done, so the new driver loads"
  fi
  # hold whatever nvidia-driver-* is installed
  local pkgs
  pkgs="$(dpkg-query -W -f '${Package} ${Status}\n' 'nvidia-driver-*' 2>/dev/null | awk '/ installed$/ {print $1}' || true)"
  if [ -n "$pkgs" ]; then
    # shellcheck disable=SC2086  # one word per package
    run apt-mark hold $pkgs
  else
    echo "  (after the reboot, run the driver step again to hold the installed driver)"
  fi
}

step_docker() {
  if command -v snap >/dev/null && snap list docker >/dev/null 2>&1; then
    echo "  removing the snap Docker (no --gpus, and its own /tmp)"
    run snap remove --purge docker
  fi
  keyring https://download.docker.com/linux/ubuntu/gpg /etc/apt/keyrings/docker.gpg
  write /etc/apt/sources.list.d/docker.list \
    "deb [arch=$ARCH signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $CODENAME stable"
  run apt-get update
  apt_install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  run systemctl enable --now docker
}

step_nvidia() {
  keyring https://nvidia.github.io/libnvidia-container/gpgkey /etc/apt/keyrings/nvidia-container-toolkit.gpg
  write /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    "deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/\$(ARCH) /"
  run apt-get update
  apt_install nvidia-container-toolkit
  run nvidia-ctk runtime configure --runtime=docker
  run systemctl restart docker
  echo "  check: docker run --rm --gpus all ubuntu nvidia-smi"
}

step_vulkan() {
  apt_install vulkan-tools libvulkan1
}

step_gh() {
  keyring https://cli.github.com/packages/githubcli-archive-keyring.gpg /etc/apt/keyrings/githubcli.gpg binary
  write /etc/apt/sources.list.d/github-cli.list \
    "deb [arch=$ARCH signed-by=/etc/apt/keyrings/githubcli.gpg] https://cli.github.com/packages stable main"
  run apt-get update
  apt_install gh
}

step_tailscale() {
  keyring "https://pkgs.tailscale.com/stable/ubuntu/$CODENAME.noarmor.gpg" /etc/apt/keyrings/tailscale.gpg binary
  write /etc/apt/sources.list.d/tailscale.list \
    "deb [signed-by=/etc/apt/keyrings/tailscale.gpg] https://pkgs.tailscale.com/stable/ubuntu $CODENAME main"
  run apt-get update
  apt_install tailscale
  echo "  then, by hand: sudo tailscale up --ssh   (joins the team tailnet)"
}

step_user() {
  if id "$RUNNER_USER" >/dev/null 2>&1; then
    echo "  $RUNNER_USER exists"
  else
    run useradd --create-home --shell /bin/bash "$RUNNER_USER"
  fi
  # docker = root-equivalent: accepted because only team code runs here (roadmap, Security)
  run usermod -aG docker "$RUNNER_USER"
  if id -nG "$RUNNER_USER" 2>/dev/null | tr ' ' '\n' | grep -qx sudo; then
    echo "  warning: $RUNNER_USER is in the sudo group; the runner shouldn't be" >&2
  fi
}

step_dirs() {
  run install -d -o "$RUNNER_USER" -g "$RUNNER_USER" -m 0755 "$CI_HOME" "$CI_HOME/builds" "$CI_HOME/baseline" \
    "$CI_HOME/runs" "$CI_HOME/ddc"
}

step_runner() {
  : "${RUNNER_TOKEN:?set RUNNER_TOKEN (org settings -> Actions -> Runners -> New runner)}"
  local home dir version url sha body
  home="$(runner_home)"
  if [ -z "$home" ]; then
    [ "$DRY" = 1 ] || { echo "no user $RUNNER_USER: run the user step first" >&2; exit 1; }
    home="/home/$RUNNER_USER"
  fi
  dir="$home/actions-runner"
  if [ "$DRY" = 1 ]; then
    version="<latest>"
  else
    body="$(curl -fsSL https://api.github.com/repos/actions/runner/releases/latest)"
    version="$(jq -r .tag_name <<<"$body" | sed 's/^v//')"
    # the release notes carry each package's checksum
    sha="$(jq -r .body <<<"$body" | sed -n 's/.*<!-- BEGIN SHA linux-x64 -->\([0-9a-f]\{64\}\)<!-- END SHA linux-x64 -->.*/\1/p')"
    [ -n "$sha" ] || { echo "no checksum for linux-x64 in runner v$version's release notes" >&2; exit 1; }
  fi
  url="https://github.com/actions/runner/releases/download/v$version/actions-runner-linux-x64-$version.tar.gz"
  run install -d -o "$RUNNER_USER" -g "$RUNNER_USER" "$dir"
  if [ "$DRY" = 1 ]; then
    printf '  $ curl -fsSL %s (checked against the release notes'"'"' sha256) | tar xz -C %s\n' "$url" "$dir"
  elif [ ! -x "$dir/config.sh" ]; then
    curl -fsSL -o "/tmp/actions-runner.tgz" "$url"
    echo "$sha  /tmp/actions-runner.tgz" | sha256sum -c -
    sudo -u "$RUNNER_USER" tar xzf /tmp/actions-runner.tgz -C "$dir"
    rm -f /tmp/actions-runner.tgz
    "$dir/bin/installdependencies.sh"
  fi
  # what the workflows read from the runner's environment (bench.yml, sim_build.sh, sim_pr_check.sh)
  write "$dir/.env" "CI_HOME=$CI_HOME
UE5_ROOT=$UE5_ROOT
IFSSIM_RIG=$dir/_work/IFSSIM/IFSSIM/rigs/bench-pc.yaml
LANG=C.UTF-8"
  run chown "$RUNNER_USER:$RUNNER_USER" "$dir/.env"
  local tok="$RUNNER_TOKEN"
  [ "$DRY" = 1 ] && tok="***"
  run sudo -u "$RUNNER_USER" "$dir/config.sh" --unattended --replace --url "$RUNNER_URL" \
    --token "$tok" --name "$RUNNER_NAME" --runnergroup "$RUNNER_GROUP" \
    --no-default-labels --labels "$RUNNER_LABELS" --work _work
  if [ "$DRY" = 1 ]; then
    printf '  $ (cd %s && ./svc.sh install %s && ./svc.sh start)\n' "$dir" "$RUNNER_USER"
  else
    (cd "$dir" && ./svc.sh install "$RUNNER_USER" && ./svc.sh start)
  fi
}

[ "$DRY" = 1 ] && echo "dry run: nothing is changed"
for s in "${steps[@]}"; do
  echo "== $s"
  "step_$s"
done
echo "done: ${steps[*]}"

#!/usr/bin/env bash
# =============================================================================
# agv.sh -- installer / updater / checker / launcher for the AGV on the Raspberry Pi 5
#
# Your workflow:  upload code to GitHub (web)  ->  on the Pi desktop double-click an icon (or type `agv ...`)
#
#   agv install            ONE TIME: packages, code download, desktop icons, `agv` command
#   agv update             download the newest code from GitHub (keeps tuning + logs)
#   agv check              test Python libs, camera (30 fps?), Nano (sonar) and the display
#   agv run [options]      update, then start the car program + open the tuning dashboard (starts in HOLD)
#       --go               start driving immediately (race mode, no HOLD)
#       --no-update        do not touch GitHub; run the saved copy (use this at the competition)
#       --no-browser       do not auto-open the dashboard (open http://localhost:8080 yourself)
#       (any other option is passed to main_vision.py, e.g. --log-level DEBUG)
#   agv status             what is installed, where files are, what is plugged in
#
# The Nano is flashed by you with the Arduino IDE. This script never touches it.
# =============================================================================
set -Eeuo pipefail

# ----------------------------- YOUR REPOSITORY -------------------------------
GH_USER="${GH_USER:-robotahmid07-debug}"
GH_REPO="${GH_REPO:-Techfest-iit-Bombay-International-round}"
BRANCH="${BRANCH:-main}"
CODE_SUBDIR="${CODE_SUBDIR:-Autonomou car/codes/Iteration 3}"     # folder inside the repo (spaces are fine)
# -----------------------------------------------------------------------------

REPO_URL="${REPO_URL:-https://github.com/$GH_USER/$GH_REPO.git}"
BASE_DIR="${BASE_DIR:-$HOME/agv-car}"
REPO_DIR="$BASE_DIR/repo"           # downloaded copy of your GitHub folder (overwritten by updates)
DATA_DIR="$BASE_DIR/data"           # tuning.json + logs: NEVER touched by updates
BIN_LINK="${BIN_LINK:-/usr/local/bin/agv}"
WEB_PORT="${WEB_PORT:-8080}"
VENV="$BASE_DIR/venv"            # python environment with OpenCV (pip wheels, no apt conflicts)
PY="$VENV/bin/python"
CODE_DIR=""
NEED_RELOGIN=0
FAILS=0

c_ok=$'\e[32m'; c_warn=$'\e[33m'; c_err=$'\e[31m'; c_off=$'\e[0m'
log()  { echo "${c_ok}[agv]${c_off} $*"; }
warn() { echo "${c_warn}[agv] WARN:${c_off} $*" >&2; }
die()  { echo "${c_err}[agv] ERROR:${c_off} $*" >&2; exit 1; }
trap 'echo "${c_err}[agv] stopped: a command failed near line $LINENO${c_off}" >&2' ERR

USER="${USER:-$(id -un)}"
[ "$(id -u)" -ne 0 ] || [ -n "${AGV_ALLOW_ROOT:-}" ] || die "run as your normal user, not root/sudo (the script asks for sudo itself when needed)."
SUDO="sudo"; [ "$(id -u)" -eq 0 ] && SUDO=""

online() { curl -fsS -m 6 -o /dev/null https://github.com >/dev/null 2>&1; }
web_url() { echo "https://github.com/$GH_USER/$GH_REPO/tree/$BRANCH/${CODE_SUBDIR// /%20}"; }

# ------------------------------- packages / permissions ----------------------
install_packages() {
  local pkgs=(git curl ca-certificates python3 python3-venv python3-pip python3-numpy python3-serial v4l-utils)
  local missing=() p
  for p in "${pkgs[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p"); done
  if [ ${#missing[@]} -eq 0 ]; then log "all system packages already installed"; return 0; fi
  online || die "no internet. Connect the Pi to Wi-Fi/phone hotspot, then run: agv install"
  log "installing: ${missing[*]}  (asks for your password once)"
  $SUDO apt-get update -y
  if ! $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing[@]}"; then
    # Typical on a freshly flashed OS: base libraries are older than the repository (held-back packages).
    warn "install failed (usually an out-of-date base system). Upgrading the system, then retrying (a few minutes)..."
    $SUDO env DEBIAN_FRONTEND=noninteractive apt-get full-upgrade -y
    $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing[@]}"
  fi
}

# OpenCV from pip wheels: avoids Debian's heavy python3-opencv dependency chain (VTK/MPI), which can break apt.
ensure_venv() {
  if [ -x "$PY" ] && "$PY" -c "import cv2, numpy, serial" >/dev/null 2>&1; then
    log "python environment ready"; return 0
  fi
  online || die "no internet: cannot download OpenCV. Connect the Pi to the internet, then run: agv install"
  log "creating the python environment (downloads OpenCV, about 60 MB)..."
  mkdir -p "$BASE_DIR"
  python3 -m venv --system-site-packages "$VENV"
  "$VENV/bin/pip" install --no-cache-dir opencv-python-headless pyserial
  "$PY" -c "import cv2, numpy, serial; print('[agv] OpenCV', cv2.__version__, 'ready')"
}

ensure_groups() {
  local g
  for g in dialout video; do
    if ! id -nG "$USER" | tr ' ' '\n' | grep -qx "$g"; then
      log "adding $USER to group $g"; $SUDO usermod -aG "$g" "$USER"
    fi
    if ! id -nG | tr ' ' '\n' | grep -qx "$g"; then NEED_RELOGIN=1; fi
  done
}

# Group changes only apply after a new login; until then open the device nodes temporarily.
ensure_access() {
  local dev
  for dev in "$@"; do
    [ -e "$dev" ] || continue
    if [ ! -r "$dev" ] || [ ! -w "$dev" ]; then
      warn "granting temporary access to $dev (permanent after a reboot)"; $SUDO chmod a+rw "$dev"
    fi
  done
}

find_port() {
  local p
  for p in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$p" ]; then echo "$p"; return 0; fi
  done
  return 1
}

# ------------------------------- code from GitHub ----------------------------
locate_code() {
  local f=""
  if [ -f "$REPO_DIR/$CODE_SUBDIR/main_vision.py" ]; then
    CODE_DIR="$REPO_DIR/$CODE_SUBDIR"; return 0
  fi
  if [ -d "$REPO_DIR" ]; then
    f=$(find "$REPO_DIR" -path "$REPO_DIR/.git" -prune -o -name main_vision.py -print -quit)
  fi
  if [ -n "$f" ]; then CODE_DIR=$(dirname "$f"); return 0; fi
  CODE_DIR=""; return 1
}

fetch_code() {
  mkdir -p "$BASE_DIR" "$DATA_DIR/logs"
  if [ -f "$DATA_DIR/tuning.json" ]; then cp "$DATA_DIR/tuning.json" "$DATA_DIR/tuning.backup.json"; fi
  if [ -d "$REPO_DIR/.git" ]; then
    if online; then
      log "checking GitHub for new code..."
      if git -C "$REPO_DIR" fetch -q --depth 1 origin "$BRANCH" && git -C "$REPO_DIR" reset -q --hard FETCH_HEAD; then
        log "code is up to date: $(git -C "$REPO_DIR" log -1 --format='%h %cd' --date=short)"
      else
        warn "could not update; using the saved copy"
      fi
    else
      warn "no internet: using the saved copy"
    fi
  else
    online || die "no internet and no saved copy yet. Connect to the internet once, then run: agv install"
    log "downloading only your folder '$CODE_SUBDIR' from GitHub..."
    rm -rf "$REPO_DIR"
    if ! git clone -q --depth 1 --filter=blob:none --sparse --branch "$BRANCH" "$REPO_URL" "$REPO_DIR"; then
      warn "partial download not supported here, downloading the whole repo instead"
      rm -rf "$REPO_DIR"; git clone -q --depth 1 --branch "$BRANCH" "$REPO_URL" "$REPO_DIR"
    fi
    git -C "$REPO_DIR" sparse-checkout set "$CODE_SUBDIR" 2>/dev/null || true
  fi
  if locate_code; then
    log "code folder: $CODE_DIR"
  else
    return 1
  fi
}

need_code_msg() {
  warn "main_vision.py is not in your GitHub folder yet."
  echo "   Upload it here (Add file -> Upload files):  $(web_url)"
  echo "   then run:  agv update"
}

# agv.sh keeps a copy of itself so the `agv` command works; if you upload a newer agv.sh it replaces it.
install_self() {
  local src="" dest="$BASE_DIR/agv.sh" tmp="$BASE_DIR/.agv.sh.new"
  if [ -f "$REPO_DIR/$CODE_SUBDIR/agv.sh" ]; then src="$REPO_DIR/$CODE_SUBDIR/agv.sh"
  elif [ -f "${BASH_SOURCE[0]:-}" ]; then src="${BASH_SOURCE[0]}"; fi
  if [ -z "$src" ]; then warn "agv.sh not found in your GitHub folder; upload it so the 'agv' command works."; return 0; fi
  mkdir -p "$BASE_DIR"
  if [ ! -f "$dest" ] || ! cmp -s "$src" "$dest"; then
    cp "$src" "$tmp"; chmod +x "$tmp"; mv -f "$tmp" "$dest"      # mv = new inode: safe while this script runs
  fi
  $SUDO ln -sf "$dest" "$BIN_LINK"
}

make_shortcuts() {
  local apps="$HOME/.local/share/applications" desk
  desk="$(xdg-user-dir DESKTOP 2>/dev/null || true)"; [ -n "$desk" ] || desk="$HOME/Desktop"
  mkdir -p "$apps" "$desk" "$BASE_DIR"
  cat > "$BASE_DIR/launch.sh" <<'EOF'
#!/usr/bin/env bash
# used by the desktop icons: run an agv command, then wait so errors stay readable
/usr/local/bin/agv "$@"; rc=$?
echo; read -rp "Finished (exit code $rc). Press Enter to close... " _
EOF
  chmod +x "$BASE_DIR/launch.sh"
  mk() {   # file  title  args  icon
    local d
    for d in "$apps" "$desk"; do
      cat > "$d/$1" <<EOF
[Desktop Entry]
Type=Application
Name=$2
Exec=$BASE_DIR/launch.sh $3
Icon=$4
Terminal=true
Categories=Utility;
EOF
      chmod +x "$d/$1"
    done
  }
  mk agv-check.desktop  "AGV 1 - Check hardware"           "check"                "utilities-system-monitor"
  mk agv-run.desktop    "AGV 2 - Run + tune (starts HOLD)" "run"                  "applications-engineering"
  mk agv-race.desktop   "AGV 3 - RACE (drives now)"        "run --go --no-update" "media-playback-start"
  mk agv-update.desktop "AGV - Update code from GitHub"    "update"               "system-software-update"
  log "desktop icons created (first double-click may ask 'Execute?': choose Execute)"
}

# ------------------------------- checks --------------------------------------
pass() { echo "  ${c_ok}[ OK ]${c_off} $*"; }
bad()  { echo "  ${c_err}[FAIL]${c_off} $*"; FAILS=$((FAILS + 1)); }
note() { echo "  ${c_warn}[ !! ]${c_off} $*"; }

check_camera() {
  local dev="/dev/video0" rc=0
  if [ ! -e "$dev" ]; then bad "no /dev/video0: plug in the Logitech camera"; return 0; fi
  ensure_access "$dev"
  "$PY" - "$dev" <<'PY' || rc=$?
import sys, time, cv2
cap = cv2.VideoCapture(sys.argv[1], cv2.CAP_V4L2)
if not cap.isOpened(): print("cannot open camera (used by another program?)"); sys.exit(1)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30); cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
for _ in range(10): cap.read()
n, lum, t0, shape = 0, 0.0, time.time(), None
while n < 60 and time.time() - t0 < 6:
    ok, f = cap.read()
    if ok: n += 1; lum += float(f.mean()); shape = f.shape
dt = time.time() - t0; cap.release()
if n < 10: print("camera opened but delivers no frames"); sys.exit(1)
fps = n / dt
print(f"         {shape[1]}x{shape[0]} at {fps:.1f} fps, brightness {lum/n:.0f}/255")
sys.exit(0 if fps >= 24 else 2)
PY
  case "$rc" in
    0) pass "camera works";;
    2) note "camera is below 30 fps (dim light? add light or lower exposure)";;
    *) bad "camera problem (see message above)";;
  esac
}

check_nano() {
  mkdir -p "$BASE_DIR"
  if ! ( exec 9>"$BASE_DIR/run.lock" && flock -n 9 ) 2>/dev/null; then     # lock is held while `agv run` is active
    note "car program is running: skipping Nano test (opening the port would reset the Nano). Close it first."; return 0
  fi
  local port rc=0
  if ! port=$(find_port); then bad "no Nano on USB (use a DATA cable; try the other Pi USB port)"; return 0; fi
  ensure_access "$port"
  "$PY" - "$port" <<'PY' || rc=$?
import sys, time, serial
ser = serial.Serial(sys.argv[1], 115200, timeout=0.3)
time.sleep(2.3); ser.reset_input_buffer()                  # the Nano resets when the port opens
t0, seen = time.time(), None
while time.time() - t0 < 4:
    line = ser.readline().decode("ascii", "ignore").strip()
    if line.startswith("$TEL"):
        seen = line; break
ser.close()
if not seen: print("         no telemetry: wrong sketch, wrong board settings, or Nano not booting"); sys.exit(1)
f = seen[1:].split("*")[0].split(",")                      # TEL,ms,dist,flags,bad,dropped
dist, flags = int(f[2]), int(f[3])
print(f"         sonar {'no echo (nothing within ~3 m)' if dist == 9999 else str(dist) + ' mm'}, flags={flags}")
sys.exit(2 if flags & 4 else 0)
PY
  case "$rc" in
    0) pass "Nano answers on $port";;
    2) note "Nano answers on $port but the SONAR looks stale: check Trig=D4, Echo=D2, 5V, GND";;
    *) bad "Nano does not answer on $port (re-upload the sketch in the Arduino IDE)";;
  esac
}

attach_display() {   # lets `agv run` open its window on the Pi Connect desktop even from a Remote-shell terminal
  if [ -n "${WAYLAND_DISPLAY:-}${DISPLAY:-}" ]; then return 0; fi
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  local s
  for s in wayland-0 wayland-1; do
    if [ -S "$XDG_RUNTIME_DIR/$s" ]; then export WAYLAND_DISPLAY="$s"; break; fi
  done
  if [ -S /tmp/.X11-unix/X0 ]; then export DISPLAY=":0"; fi
  return 0
}

do_check() {
  echo "AGV hardware check"
  if "$PY" -c "import cv2, numpy, serial; print('         opencv', cv2.__version__, '| numpy', numpy.__version__)" 2>/dev/null; then
    pass "python libraries"
  else
    bad "Python libraries missing: run  agv install"
  fi
  if locate_code; then pass "code found: $CODE_DIR"; else bad "main_vision.py not downloaded yet (upload it to GitHub, then: agv update)"; fi
  attach_display
  if [ -n "${WAYLAND_DISPLAY:-}${DISPLAY:-}" ]; then pass "desktop session found (dashboard will open automatically)"
  else note "no desktop session: open Pi Connect SCREEN SHARING once, or open http://$(hostname).local:$WEB_PORT from a laptop"; fi
  check_camera
  check_nano
  echo
  if [ "$FAILS" -eq 0 ]; then log "all checks passed"; else warn "$FAILS check(s) failed: fix those first"; fi
}

# ------------------------------- run -----------------------------------------
open_dashboard() {   # opens the dashboard in the Pi's browser (the one you see through Pi Connect)
  local url="http://localhost:$WEB_PORT" b=""
  for b in chromium chromium-browser firefox; do command -v "$b" >/dev/null 2>&1 && break || b=""; done
  (
    sleep 5
    if [ -n "$b" ]; then "$b" "$url" >/dev/null 2>&1
    else xdg-open "$url" >/dev/null 2>&1 || true
    fi
  ) &
  disown 2>/dev/null || true
}

run_car() {
  local update=1 hold=1 browser=1 args=() a
  for a in "$@"; do
    case "$a" in
      --no-update)  update=0;;
      --go)         hold=0;;
      --no-browser) browser=0;;
      *)            args+=("$a");;
    esac
  done
  mkdir -p "$BASE_DIR" "$DATA_DIR/logs"
  exec 9>"$BASE_DIR/run.lock"
  flock -n 9 || die "the car program is already running (another window?). Close it first."

  if [ "$update" -eq 1 ]; then
    fetch_code || true
    install_self || true
  else
    log "--no-update: running the saved copy"
    locate_code || true
  fi
  if [ -z "$CODE_DIR" ]; then need_code_msg; exit 1; fi

  ensure_access /dev/video0
  local port
  if port=$(find_port); then ensure_access "$port"; else warn "Nano not detected: vision will run without motors until it is plugged in"; fi
  attach_display

  [ -x "$PY" ] || die "python environment missing: run  agv install"
  local cmd=("$PY" -u main_vision.py --params "$DATA_DIR/tuning.json" --log-dir "$DATA_DIR/logs" --web --web-port "$WEB_PORT")
  if [ "$hold" -eq 1 ]; then cmd+=(--start-hold); fi
  cmd+=("${args[@]}")
  log "starting: ${cmd[*]}"
  log "DASHBOARD: http://localhost:$WEB_PORT   (from a laptop on the same network: http://$(hostname).local:$WEB_PORT)"
  if [ "$hold" -eq 1 ]; then log "car is in HOLD: press the Run / Hold button in the dashboard to drive. Ctrl+C here = quit."; fi
  if [ "$browser" -eq 1 ]; then open_dashboard; fi
  cd "$CODE_DIR"
  exec "${cmd[@]}"
}

do_status() {
  echo "code repo : $REPO_URL  ($CODE_SUBDIR)"
  if locate_code; then
    echo "code dir  : $CODE_DIR"
    echo "version   : $(git -C "$REPO_DIR" log -1 --format='%h %cd' --date=short 2>/dev/null || echo unknown)"
  else
    echo "code dir  : main_vision.py NOT downloaded yet"
  fi
  echo "tuning    : $([ -f "$DATA_DIR/tuning.json" ] && echo "$DATA_DIR/tuning.json" || echo "none saved yet")"
  echo "logs      : $DATA_DIR/logs ($(ls "$DATA_DIR/logs" 2>/dev/null | wc -l) files)"
  echo "camera    : $(ls /dev/video0 2>/dev/null || echo 'not found')"
  echo "nano      : $(find_port || echo 'not found')"
  echo "internet  : $(online && echo yes || echo no)"
}

do_install() {
  install_packages
  ensure_venv
  ensure_groups
  fetch_code || need_code_msg
  install_self
  make_shortcuts
  log "install complete."
  if [ "$NEED_RELOGIN" -eq 1 ]; then warn "reboot the Pi once (sudo reboot) so the new permissions apply."; fi
  echo
  echo "Next: plug in the camera + Nano and double-click 'AGV 1 - Check hardware' on the Pi desktop (or type: agv check)."
}

usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; }

cmd="${1:-help}"; shift || true
case "$cmd" in
  install) do_install;;
  update)  fetch_code || need_code_msg; install_self; make_shortcuts >/dev/null 2>&1 || true;;
  check)   do_check;;
  run)     run_car "$@";;
  status)  do_status;;
  help|-h|--help) usage;;
  *)       usage; exit 1;;
esac

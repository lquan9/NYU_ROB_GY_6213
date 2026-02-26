#!/usr/bin/env bash
# install-camera.sh — Deploy the camera-stream systemd service
#
# Usage:
#   sudo ./configs/install-camera.sh                        # install & enable (defaults)
#   sudo ./configs/install-camera.sh --camera 1 --port 9000 # custom camera & port
#   sudo ./configs/install-camera.sh --user myuser           # install for a specific user
#   sudo ./configs/install-camera.sh --remove                # remove the service
#   sudo ./configs/install-camera.sh --status                # show service status & logs
#   sudo ./configs/install-camera.sh --dry-run               # show what would be installed
#
set -euo pipefail

SERVICE_NAME="camera-stream"
SERVICE_TEMPLATE="$(cd "$(dirname "$0")" && pwd)/${SERVICE_NAME}.service"
SYSTEMD_DIR="/etc/systemd/system"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENV_DIR="${PROJECT_DIR}/rob6213_env"
ENTRY_POINT="${VENV_DIR}/bin/camera-stream"
GENERATED_SERVICE="/tmp/${SERVICE_NAME}.service.generated"

# Defaults
TARGET_USER="${SUDO_USER:-$(whoami)}"
TARGET_GROUP=""
CAMERA_ID=0
PORT=8090
QUALITY=80
DRY_RUN=false

# ── Colors ──────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
dry()   { echo -e "${CYAN}[DRY-RUN]${NC} $*"; }

# ── Helpers ─────────────────────────────────────────────────────
check_root() {
    if [[ $EUID -ne 0 ]]; then
        error "This script must be run as root (use sudo)."
    fi
}

resolve_group() {
    if [[ -n "${TARGET_GROUP}" ]]; then
        return
    fi
    TARGET_GROUP="$(id -gn "${TARGET_USER}" 2>/dev/null || echo "${TARGET_USER}")"
}

generate_service_file() {
    sed \
        -e "s|__USER__|${TARGET_USER}|g" \
        -e "s|__GROUP__|${TARGET_GROUP}|g" \
        -e "s|__PROJECT_DIR__|${PROJECT_DIR}|g" \
        -e "s|__VENV_DIR__|${VENV_DIR}|g" \
        -e "s|__CAMERA_ID__|${CAMERA_ID}|g" \
        -e "s|__PORT__|${PORT}|g" \
        -e "s|__QUALITY__|${QUALITY}|g" \
        "${SERVICE_TEMPLATE}" > "${GENERATED_SERVICE}"
}

# ── Remove ──────────────────────────────────────────────────────
remove_service() {
    check_root
    info "Stopping ${SERVICE_NAME}..."
    systemctl stop "${SERVICE_NAME}" 2>/dev/null || true
    systemctl disable "${SERVICE_NAME}" 2>/dev/null || true
    rm -f "${SYSTEMD_DIR}/${SERVICE_NAME}.service"
    systemctl daemon-reload
    info "Service ${SERVICE_NAME} removed."
    exit 0
}

# ── Status ──────────────────────────────────────────────────────
show_status() {
    echo -e "${BOLD}── Service Status ──${NC}"
    systemctl status "${SERVICE_NAME}" --no-pager 2>/dev/null || echo "Service not found."
    echo ""
    echo -e "${BOLD}── Recent Logs (last 20 lines) ──${NC}"
    journalctl -u "${SERVICE_NAME}" -n 20 --no-pager 2>/dev/null || echo "No logs found."
    exit 0
}

# ── Install ─────────────────────────────────────────────────────
install_service() {
    check_root
    resolve_group

    echo -e "${BOLD}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}║   ROB-GY 6213 — Camera Stream Installer      ║${NC}"
    echo -e "${BOLD}╚══════════════════════════════════════════════╝${NC}"
    echo ""

    if [[ ! -f "${SERVICE_TEMPLATE}" ]]; then
        error "Service template not found: ${SERVICE_TEMPLATE}"
    fi
    info "Template:     ${SERVICE_TEMPLATE}"

    info "Project dir:  ${PROJECT_DIR}"
    info "Venv dir:     ${VENV_DIR}"
    info "Run as user:  ${TARGET_USER}:${TARGET_GROUP}"
    info "Camera index: ${CAMERA_ID}"
    info "Stream port:  ${PORT}"
    info "JPEG quality: ${QUALITY}"
    echo ""

    if ! id "${TARGET_USER}" &>/dev/null; then
        error "User '${TARGET_USER}' does not exist. Create it first or use --user <name>."
    fi

    if [[ ! -d "${VENV_DIR}" ]]; then
        if $DRY_RUN; then
            dry "Would create venv at ${VENV_DIR}"
            dry "Would run: pip install -e ${PROJECT_DIR}[dev]"
        else
            warn "Virtual environment not found at ${VENV_DIR}"
            info "Creating venv and installing package..."
            sudo -u "${TARGET_USER}" python3 -m venv "${VENV_DIR}"
            sudo -u "${TARGET_USER}" "${VENV_DIR}/bin/pip" install --upgrade pip
            sudo -u "${TARGET_USER}" "${VENV_DIR}/bin/pip" install -e "${PROJECT_DIR}[dev]"
        fi
    fi

    if [[ ! -f "${ENTRY_POINT}" ]] && ! $DRY_RUN; then
        warn "camera-stream entry point not found. Installing package into venv..."
        sudo -u "${TARGET_USER}" "${VENV_DIR}/bin/pip" install -e "${PROJECT_DIR}[dev]"
    fi

    if [[ ! -f "${ENTRY_POINT}" ]] && ! $DRY_RUN; then
        error "After install, entry point not found at ${ENTRY_POINT}.\n       Check pyproject.toml [project.scripts]."
    fi

    if ! $DRY_RUN; then
        info "Entry point:  ${ENTRY_POINT} ✓"
    fi

    generate_service_file

    echo ""
    echo -e "${BOLD}── Generated service file ──${NC}"
    cat "${GENERATED_SERVICE}"
    echo ""

    if $DRY_RUN; then
        dry "Would copy to ${SYSTEMD_DIR}/${SERVICE_NAME}.service"
        dry "Would run: systemctl daemon-reload && systemctl enable --now ${SERVICE_NAME}"
        rm -f "${GENERATED_SERVICE}"
        exit 0
    fi

    info "Installing ${SERVICE_NAME}.service -> ${SYSTEMD_DIR}/"
    cp "${GENERATED_SERVICE}" "${SYSTEMD_DIR}/${SERVICE_NAME}.service"
    rm -f "${GENERATED_SERVICE}"

    systemctl daemon-reload
    systemctl enable "${SERVICE_NAME}"
    info "Service enabled. Starting ${SERVICE_NAME}..."
    systemctl start "${SERVICE_NAME}"

    echo ""
    systemctl status "${SERVICE_NAME}" --no-pager || true
    echo ""
    info "Done! The stream is available at http://<this-ip>:${PORT}/video"
    echo ""
    info "Useful commands:"
    info "  sudo systemctl status  ${SERVICE_NAME}     # check status"
    info "  sudo systemctl restart ${SERVICE_NAME}     # restart"
    info "  sudo systemctl stop    ${SERVICE_NAME}     # stop"
    info "  journalctl -u ${SERVICE_NAME} -f           # follow logs"
    info "  sudo $0 --status                           # quick status + logs"
    info "  sudo $0 --remove                           # uninstall"
    echo ""
    info "On the robot, set in parameters.py:"
    info "  camera_source = \"http://<this-ip>:${PORT}/video\""
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --remove|-r)
            remove_service
            ;;
        --status|-s)
            show_status
            ;;
        --dry-run|-n)
            DRY_RUN=true
            shift
            ;;
        --user|-u)
            TARGET_USER="$2"
            shift 2
            ;;
        --group|-g)
            TARGET_GROUP="$2"
            shift 2
            ;;
        --camera|-c)
            CAMERA_ID="$2"
            shift 2
            ;;
        --port|-p)
            PORT="$2"
            shift 2
            ;;
        --quality|-q)
            QUALITY="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: sudo $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  (no args)          Install and enable the ${SERVICE_NAME} systemd service"
            echo "  --user, -u NAME    Run the service as this user (default: \$SUDO_USER)"
            echo "  --group, -g NAME   Run the service as this group (default: user's primary group)"
            echo "  --camera, -c ID    Camera device index (default: 0)"
            echo "  --port, -p PORT    HTTP stream port (default: 8090)"
            echo "  --quality, -q Q    JPEG quality 1-100 (default: 80)"
            echo "  --dry-run, -n      Show what would be installed without doing it"
            echo "  --status, -s       Show current service status and recent logs"
            echo "  --remove, -r       Stop, disable, and remove the service"
            echo "  --help, -h         Show this help message"
            echo ""
            echo "The service template at configs/camera-stream.service uses placeholders"
            echo "that are filled in at install time."
            exit 0
            ;;
        *)
            error "Unknown option: $1 (try --help)"
            ;;
    esac
done

install_service

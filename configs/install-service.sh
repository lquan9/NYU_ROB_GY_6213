#!/usr/bin/env bash
# install-service.sh — Deploy the robot-gui systemd service
#
# Usage:
#   sudo ./configs/install-service.sh                    # install & enable (runs as current user)
#   sudo ./configs/install-service.sh --user myuser      # install for a specific user
#   sudo ./configs/install-service.sh --remove            # remove the service
#   sudo ./configs/install-service.sh --status            # show service status & logs
#   sudo ./configs/install-service.sh --dry-run           # show what would be installed without doing it
#
set -euo pipefail

SERVICE_NAME="robot-gui"
SERVICE_TEMPLATE="$(cd "$(dirname "$0")" && pwd)/${SERVICE_NAME}.service"
SYSTEMD_DIR="/etc/systemd/system"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENV_DIR="${PROJECT_DIR}/rob6213_env"
GUI_ENTRY="${VENV_DIR}/bin/gui"
GENERATED_SERVICE="/tmp/${SERVICE_NAME}.service.generated"

# Defaults — overridable via flags
TARGET_USER="${SUDO_USER:-$(whoami)}"
TARGET_GROUP=""
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
        return  # already set via flag
    fi
    # Use the primary group of the target user
    TARGET_GROUP="$(id -gn "${TARGET_USER}" 2>/dev/null || echo "${TARGET_USER}")"
}

generate_service_file() {
    # Replace placeholders in the template with actual values
    sed \
        -e "s|__USER__|${TARGET_USER}|g" \
        -e "s|__GROUP__|${TARGET_GROUP}|g" \
        -e "s|__PROJECT_DIR__|${PROJECT_DIR}|g" \
        -e "s|__VENV_DIR__|${VENV_DIR}|g" \
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
    echo -e "${BOLD}║   ROB-GY 6213 — Service Installer            ║${NC}"
    echo -e "${BOLD}╚══════════════════════════════════════════════╝${NC}"
    echo ""

    # ── 1. Validate template ────────────────────────────────────
    if [[ ! -f "${SERVICE_TEMPLATE}" ]]; then
        error "Service template not found: ${SERVICE_TEMPLATE}"
    fi
    info "Template:     ${SERVICE_TEMPLATE}"

    # ── 2. Show resolved configuration ──────────────────────────
    info "Project dir:  ${PROJECT_DIR}"
    info "Venv dir:     ${VENV_DIR}"
    info "Run as user:  ${TARGET_USER}:${TARGET_GROUP}"
    echo ""

    # ── 3. Validate user exists ─────────────────────────────────
    if ! id "${TARGET_USER}" &>/dev/null; then
        error "User '${TARGET_USER}' does not exist. Create it first or use --user <name>."
    fi

    # ── 4. Ensure venv + package installed ──────────────────────
    if [[ ! -d "${VENV_DIR}" ]]; then
        if $DRY_RUN; then
            dry "Would create venv at ${VENV_DIR}"
            dry "Would run: pip install -e ${PROJECT_DIR}[dev]"
        else
            warn "Virtual environment not found at ${VENV_DIR}"
            info "Creating venv and installing package..."
            sudo -u "${TARGET_USER}" python3 -m venv --system-site-packages "${VENV_DIR}"
            sudo -u "${TARGET_USER}" "${VENV_DIR}/bin/pip" install --upgrade pip
            sudo -u "${TARGET_USER}" "${VENV_DIR}/bin/pip" install -e "${PROJECT_DIR}[dev]"
        fi
    fi

    if [[ ! -f "${GUI_ENTRY}" ]] && ! $DRY_RUN; then
        warn "GUI entry point not found. Installing package into venv..."
        sudo -u "${TARGET_USER}" "${VENV_DIR}/bin/pip" install -e "${PROJECT_DIR}[dev]"
    fi

    if [[ ! -f "${GUI_ENTRY}" ]] && ! $DRY_RUN; then
        error "After install, GUI entry point not found at ${GUI_ENTRY}.\n       Check pyproject.toml [project.scripts]."
    fi

    if ! $DRY_RUN; then
        info "GUI entry:    ${GUI_ENTRY} ✓"
    fi

    # ── 5. Generate service file from template ──────────────────
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

    # ── 6. Deploy ───────────────────────────────────────────────
    info "Installing ${SERVICE_NAME}.service -> ${SYSTEMD_DIR}/"
    cp "${GENERATED_SERVICE}" "${SYSTEMD_DIR}/${SERVICE_NAME}.service"
    rm -f "${GENERATED_SERVICE}"

    # Reload systemd, enable and start
    systemctl daemon-reload
    systemctl enable "${SERVICE_NAME}"
    info "Service enabled. Starting ${SERVICE_NAME}..."
    systemctl start "${SERVICE_NAME}"

    # ── 7. Report ───────────────────────────────────────────────
    echo ""
    systemctl status "${SERVICE_NAME}" --no-pager || true
    echo ""
    info "Done! Useful commands:"
    info "  sudo systemctl status  ${SERVICE_NAME}     # check status"
    info "  sudo systemctl restart ${SERVICE_NAME}     # restart"
    info "  sudo systemctl stop    ${SERVICE_NAME}     # stop"
    info "  journalctl -u ${SERVICE_NAME} -f           # follow logs"
    info "  sudo $0 --status                           # quick status + logs"
    info "  sudo $0 --remove                           # uninstall"
}

# ── Argument parsing ────────────────────────────────────────────
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
        --help|-h)
            echo "Usage: sudo $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  (no args)          Install and enable the ${SERVICE_NAME} systemd service"
            echo "  --user, -u NAME    Run the service as this user (default: \$SUDO_USER)"
            echo "  --group, -g NAME   Run the service as this group (default: user's primary group)"
            echo "  --dry-run, -n      Show what would be installed without doing it"
            echo "  --status, -s       Show current service status and recent logs"
            echo "  --remove, -r       Stop, disable, and remove the service"
            echo "  --help, -h         Show this help message"
            echo ""
            echo "The service template at configs/robot-gui.service uses placeholders"
            echo "(__USER__, __PROJECT_DIR__, etc.) that are filled in at install time."
            exit 0
            ;;
        *)
            error "Unknown option: $1 (try --help)"
            ;;
    esac
done

install_service

info "=== STAGE: final (build workspace + configs) ==="

# Refresh apt state (lists might have been cleaned in middle)
# Only run minimal base steps to keep final fast.
source "$CHROOT_ROOT/base/10_apt.sh"
source "$CHROOT_ROOT/base/20_locales.sh"
source "$CHROOT_ROOT/base/30_repos.sh"

# Build drone if requested
run_dir "$CHROOT_ROOT/build"

# Now workspace-dependent configs
run_dir "$CHROOT_ROOT/services"
run_dir "$CHROOT_ROOT/web"

# Finalize
source "$CHROOT_ROOT/finalize/10_systemd_reload.sh"
source "$CHROOT_ROOT/finalize/20_generate_web.sh"
source "$CHROOT_ROOT/finalize/30_write_version.sh"
source "$CHROOT_ROOT/finalize/99_cleanup.sh"

info "=== STAGE: middle (heavy packages / ROS / deps) ==="

# NOTE: middle stage should NOT depend on catkin workspace presence.
# Ensure INSTALL_ROS=true, INSTALL_DRONE=false at caller.

run_dir "$CHROOT_ROOT/base"
run_dir "$CHROOT_ROOT/pkgs"
run_dir "$CHROOT_ROOT/pkgs_heavy"
run_dir "$CHROOT_ROOT/ros"
run_dir "$CHROOT_ROOT/build_deps"

# Services/apps that don't depend on workspace will run; those that do will self-skip
run_dir "$CHROOT_ROOT/services"
run_dir "$CHROOT_ROOT/apps"
run_dir "$CHROOT_ROOT/web"

# Finalize (no web generation here unless roswww script exists)
source "$CHROOT_ROOT/finalize/10_systemd_reload.sh"
source "$CHROOT_ROOT/finalize/30_write_version.sh"
source "$CHROOT_ROOT/finalize/99_cleanup.sh"

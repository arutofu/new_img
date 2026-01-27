# Only enable autostart if drone package is present (workspace built)
if [[ -d "${CATKIN_WS:-/home/${PI_USER:-pi}/catkin_ws}/src/drone" ]]; then
  setup_drone_launch_service
else
  info "No drone workspace yet -> skip drone launch service"
fi

# Only meaningful if workspace present
if [[ -d "${CATKIN_WS:-/home/${PI_USER:-pi}/catkin_ws}/src/drone" ]]; then
  setup_drone_entrypoints
else
  info "No drone workspace yet -> skip entrypoints"
fi

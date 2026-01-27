# Only meaningful if workspace present
if [[ -d "${CATKIN_WS:-/home/${PI_USER:-pi}/catkin_ws}/src/drone" ]]; then
  setup_roswww_static_update_service
else
  info "No drone workspace yet -> skip roswww update service"
fi

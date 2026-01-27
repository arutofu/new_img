if [[ -x /usr/local/sbin/drone-www-update.sh ]]; then
  info "Generate web pages now (best-effort)"
  su - "${PI_USER}" -c "/usr/local/sbin/drone-www-update.sh" || true
  systemctl restart nginx 2>/dev/null || true
else
  info "drone-www-update.sh not present -> skip web generation"
fi

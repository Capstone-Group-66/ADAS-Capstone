sudo systemctl daemon-reload
sudo systemctl restart tailscaled

sudo tailscale up --advertise-tags=tag:jetson

sudo systemctl status tailscaled

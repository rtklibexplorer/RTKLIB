#/bin/sh
# System helper scripts
sudo cp strsvr.service /lib/systemd/system
sudo cp rtksvr.service /lib/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable gpsstream.service
exit 0

# REMARKS

sudo mv ~/.config/systemd/user/python_demo_service.service /etc/systemd/system/
sudo chown root:root /etc/systemd/system/python_demo_service.service
sudo chmod 644 /etc/systemd/system/python_demo_service.service
systemctl list-unit-files | grep pythond
journalctl --unit python
journalctl --unit pythond

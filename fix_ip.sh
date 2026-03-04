#!/bin/bash

CONN="Wired connection 1"
IP="172.16.0.3/24"

echo "Fixing IP for: $CONN"
echo "Setting address to $IP"

# Authenticate once
sudo -v || exit 1

# Configure IPv4 manually
sudo nmcli con mod "$CONN" ipv4.addresses "$IP"
sudo nmcli con mod "$CONN" ipv4.method manual
sudo nmcli con mod "$CONN" ipv4.gateway ""
sudo nmcli con mod "$CONN" ipv4.dns ""

# Restart the connection
sudo nmcli con down "$CONN"
sudo nmcli con up "$CONN"

echo "Done ✅"

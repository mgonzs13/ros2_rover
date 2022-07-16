# dependencies
echo "--------INSTALLING DEPENDENCIES"
apt install ros-galactic-joy-linux ros-galactic-teleop-twist-joy ros-galactic-urg-node -y >>/dev/null

# copy rover project
echo "--------COPYING SH FILE"
cp rover.sh /usr/local/bin/rover.sh >>/dev/null

# copy rover service
echo "--------COPYING SERVICE FILE"
cp rover.service /etc/systemd/system/rover.service

# add permissions
echo "--------ADDING PERMISSIONS"
chmod 744 /usr/local/bin/rover.sh
chmod 664 /etc/systemd/system/rover.service

# enable service
echo "--------ENABLING SERVICE"
systemctl daemon-reload
systemctl enable rover.service

echo "--------STARTING SERVICE"
# start service
systemctl start rover

# temp readme

```bash
sudo cp ~/catkin_ws/src/bluerov_apps/debian/99-bluerov-apps.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

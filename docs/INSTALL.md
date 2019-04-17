Copy `99-st.rules` to your udev directory and reload rules. For example:
```shell
sudo cp 99-st.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
```
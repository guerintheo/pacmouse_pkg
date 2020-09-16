Note by Theo about configuring roslaunch to launch nodes on another machine.

The <machine> tag of a launch file (see http://wiki.ros.org/roslaunch/XML/machine) has an attribute for machine password, which is "strongly discouraged". Instead, they suggest setting up SSH keys. I generated SSH keys for pacmouse_ap and copied the public key over to the slave Pi (hostname: pacmouse). See https://www.raspberrypi.org/documentation/remote-access/ssh/passwordless.md. In order to not be prompted to enter SSH key password when trying to call `ssh pi@pacmouse` from pacmouse_ap, I also ran `ssh-add` on the pacmouse_ap client (see https://unix.stackexchange.com/questions/187418/ssh-with-rsa-keys-under-raspbian-keeps-requesting-passphrase).
#!/bin/sh

apt-get update && apt-get install -y expect
expect -c "
spawn apt install -y libk4abt1.0-dev
expect \"Do you accept the EULA license terms?\"
send yes\n
expect $
exit
"
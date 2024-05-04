#!/bin/bash

set -e
echo "Installing Unity Testing Framework to [$PWD/tests/unity]"
sudo mkdir -p $PWD/tests/tests
# Give read, write, and execute to everyone
# u for users, g for group, o for others, and ugo or a (for all)
sudo chmod ugo+rwx $PWD/tests
curl -o $PWD/Unity-master.zip -OL https://github.com/ThrowTheSwitch/Unity/archive/master.zip

wait

sudo unzip $PWD/Unity-master.zip -d $PWD/tests
sudo mv $PWD/tests/Unity-master "$PWD/tests/unity"
sudo rm $PWD/Unity-master.zip


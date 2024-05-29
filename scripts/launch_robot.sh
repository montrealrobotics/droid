#!/bin/bash -i
source ~/.bashrc
cd $( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
python main.py
sudo usbreset 2b03:f682
lsusb -d 2b03:f780 | while read _ bus _ device _; do
        sudo usbreset "${bus}/${device%:}"
done
read -p "Press enter to continue"


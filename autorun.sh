#!/bin/bash
check_gpio_file() {
    local file_in_python="gpio_in_python"
    touch "$file_in_python"
    echo "0" > "$file_in_python"

    local file_out_cpp="gpio_out_cpp"
    touch "$file_out_cpp"
    echo "1" > "$file_out_cpp"

    local file_result_cpp="gpio_result_cpp"
    touch "$file_result_cpp"
    echo "1" > "$file_result_cpp"

}

check_gpio_file

#release/app &
#/home/ushine/FlyCamera/release/app &

#echo "鍵入 OK 開始啟動機台"
#read INPUT
#echo $INPUT
# should add sudo python
echo ushine | sudo -S /home/ushine/FlyCamera/gpio_work_stop.py

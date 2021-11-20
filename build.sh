#! /bin/bash

if [[ "$1" == "clean" ]]; then
    rm -rf build
fi

if [[ ! -e ../pico-sdk ]]; then
    echo "Raspberry Pi Pico SDK missing"
    exit 1
fi

if [[ ! -e build ]]; then
    cmake -G Ninja -B build -S src -DPICO_SDK_PATH=../pico-sdk
fi

ninja -C build

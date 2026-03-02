# xcalibur_ares
Firmware used for the ARES project

## Git initialization

To initialize the Git repository and its submodules, run the following command:

```
git submodule update --init --recursive
```

## Control Board

The control board is based on the STM32G494 microcontroller. The firmware for the control board is located in the `control_board` directory. The main firmware file is `main.c`, which contains the main loop and initialization code for the control board.

## HMI

The HMI (Human-Machine Interface) is based on the STM32H743 microcontroller. The firmware for the HMI is located in the `hmi` directory.
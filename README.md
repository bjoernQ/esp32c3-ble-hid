## ESP32-C3 BLE HID Keyboard

Example/experiment using bare-metal Rust to simulate a BLE HID keyboard on ESP32-C3.

Seems to work on Android but Windows 11 apparently hates it.

You can pair it with your Android phone. Pressing the boot button should enter 'esp32'.

Currently the LTK isn't persisted - when you reboot the ESP32-C3 it won't be paired anymore.

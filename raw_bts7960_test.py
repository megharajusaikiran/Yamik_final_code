import os

os.environ['GPIOZERO_PIN_FACTORY'] = 'lgpio'
os.environ['RPI_LGPIO_CHIP'] = '4'

import time
from gpiozero import PWMOutputDevice, DigitalOutputDevice

LEFT_RPWM = 18
LEFT_LPWM = 23
LEFT_REN = 17
LEFT_LEN = 27

RIGHT_RPWM = 12
RIGHT_LPWM = 16
RIGHT_REN = 22
RIGHT_LEN = 24

left_rpwm = PWMOutputDevice(
    LEFT_RPWM,
    frequency=1000,
    initial_value=0
)

left_lpwm = PWMOutputDevice(
    LEFT_LPWM,
    frequency=1000,
    initial_value=0
)

right_rpwm = PWMOutputDevice(
    RIGHT_RPWM,
    frequency=1000,
    initial_value=0
)

right_lpwm = PWMOutputDevice(
    RIGHT_LPWM,
    frequency=1000,
    initial_value=0
)

left_ren = DigitalOutputDevice(
    LEFT_REN,
    initial_value=True
)

left_len = DigitalOutputDevice(
    LEFT_LEN,
    initial_value=True
)

right_ren = DigitalOutputDevice(
    RIGHT_REN,
    initial_value=True
)

right_len = DigitalOutputDevice(
    RIGHT_LEN,
    initial_value=True
)

def stop():
    left_rpwm.value = 0
    left_lpwm.value = 0
    right_rpwm.value = 0
    right_lpwm.value = 0

def forward(speed=0.80):
    left_lpwm.value = 0
    right_lpwm.value = 0

    left_rpwm.value = speed
    right_rpwm.value = speed

def reverse(speed=0.80):
    left_rpwm.value = 0
    right_rpwm.value = 0

    left_lpwm.value = speed
    right_lpwm.value = speed

try:
    print('Raw BTS7960 test started.')
    print('Both driver enable pins are HIGH.')
    print('Keep wheels lifted. Ctrl+C stops motors.')

    while True:
        print('FORWARD: 80% PWM for 2 seconds')
        forward()
        time.sleep(2)

        print('STOP: 2 seconds')
        stop()
        time.sleep(2)

        print('REVERSE: 80% PWM for 2 seconds')
        reverse()
        time.sleep(2)

        print('STOP: 2 seconds')
        stop()
        time.sleep(2)

except KeyboardInterrupt:
    print('\\nStopping all motors...')

finally:
    stop()

    for device in [
        left_rpwm,
        left_lpwm,
        right_rpwm,
        right_lpwm,
        left_ren,
        left_len,
        right_ren,
        right_len,
    ]:
        device.close()

    print('Raw motor test finished.')

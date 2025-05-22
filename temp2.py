import gpiod
import time

chip = gpiod.Chip('gpiochip0')
line = chip.get_line(4)  # GPIO4

config = gpiod.LineRequest()
config.request_type = gpiod.LineRequest.DIRECTION_OUTPUT
config.consumer = "buzzer"

line.request(config)

try:
    for _ in range(5):
        line.set_value(1)  # Turn buzzer ON
        time.sleep(0.1)
        line.set_value(0)  # Turn buzzer OFF
        time.sleep(0.1)
finally:
    line.release()

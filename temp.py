import smbus2
import time

# LCD Address (from your i2cdetect): 0x27
LCD_ADDR = 0x27

bus = smbus2.SMBus(1)

# Commands and flags (based on HD44780 and PCF8574)
LCD_CHR = 1  # Mode - Sending data
LCD_CMD = 0  # Mode - Sending command

LCD_BACKLIGHT = 0x08  # On
ENABLE = 0b00000100   # Enable bit

def lcd_strobe(data):
    bus.write_byte(LCD_ADDR, data | ENABLE | LCD_BACKLIGHT)
    bus.write_byte(LCD_ADDR, (data & ~ENABLE) | LCD_BACKLIGHT)
    time.sleep(0.001)

def lcd_write_byte(bits, mode):
    high_bits = mode | (bits & 0xF0) | LCD_BACKLIGHT
    low_bits = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    bus.write_byte(LCD_ADDR, high_bits)
    lcd_strobe(high_bits)

    bus.write_byte(LCD_ADDR, low_bits)
    lcd_strobe(low_bits)

def lcd_init():
    lcd_write_byte(0x33, LCD_CMD)  # Initialize
    lcd_write_byte(0x32, LCD_CMD)  # Set to 4-bit mode
    lcd_write_byte(0x28, LCD_CMD)  # 2 line, 5x8 font
    lcd_write_byte(0x0C, LCD_CMD)  # Display ON, cursor OFF, blink OFF
    lcd_write_byte(0x06, LCD_CMD)  # Entry mode set
    lcd_write_byte(0x01, LCD_CMD)  # Clear display
    time.sleep(0.0005)

def lcd_string(message, line):
    if line == 1:
        lcd_write_byte(0x80, LCD_CMD)
    elif line == 2:
        lcd_write_byte(0xC0, LCD_CMD)

    for char in message.ljust(16):
        lcd_write_byte(ord(char), LCD_CHR)

if __name__ == "__main__":
    lcd_init()
    lcd_string("Hello ROS2!", 1)
    lcd_string("Ubuntu 24.04 Pi", 2)


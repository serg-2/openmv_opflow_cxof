# This script sends out OpticalFlow detections using the CXOF protocol to
# controller for position control using your OpenMV Cam.
#
# P4 = TXD

import image, math, pyb, sensor, struct, time

# Parameters #################################################################
# For Default INAV Builds (Under 5.0) set 19200 (Fixed for CXOF protocol)
uart_baudrate = 115200

##############################################################################

# LED control
led = pyb.LED(2) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.
led_state = 0

def update_led():
    global led_state
    led_state = led_state + 1
    if led_state == 10:
        led.on()
    elif led_state >= 20:
        led.off()
        led_state = 0

# Link Setup
uart = pyb.UART(3, uart_baudrate, timeout_char = 1000)

def checksum(x, y):
    return ((x & 0xFF) + ((x >> 8) & 0xFF) + (y & 0xFF) + ((y >> 8) & 0xFF)) & 0xFF

# Struct manual
# https://docs.python.org/3/library/struct.html
def send_optical_flow_packet(x, y, c):
    temp = struct.pack("<bbhhbbb",  # < - little endian
                       0xFE,            # b - Header
                       0x04,            # b - Number of data packet.
                       int(x),          # h - short. 2 byte. Flow in x-sensor direction (dpix).
                       int(y),          # h - short. 2 byte. Flow in y-sensor direction (dpix).
                       checksum(x, y),  # b - Checksum.
                       int(c * 255),    # b - Quality.
                       0xAA)            # b - Footer.
    uart.write(temp)
    update_led()

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.B64X32) # Set frame size to 64x32... (or 64x64)...
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

# Take from the main frame buffer's RAM to allocate a second frame buffer.
# There's a lot more RAM in the frame buffer than in the MicroPython heap.
# However, after doing this you have a lot less RAM for some algorithms...
# So, be aware that it's a lot easier to get out of RAM issues now.
extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
extra_fb.replace(sensor.snapshot())

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    displacement = extra_fb.find_displacement(img)
    extra_fb.replace(img)

    # Offset results are noisy without filtering so we drop some accuracy.
    sub_pixel_x = int(-displacement.x_translation() * 35)
    sub_pixel_y = int(displacement.y_translation() * 53)

    send_optical_flow_packet(sub_pixel_x, sub_pixel_y, displacement.response())

    print("{0:+f}x {1:+f}y {2} {3} FPS".format(sub_pixel_x, sub_pixel_y,
          displacement.response(),
          clock.fps()))

import pygame
import pygame.locals
import serial
import time

# serialPort = '/dev/ttyUSB0'
# baudRate = 9600

# ser = serial.Serial(serialPort, baudRate, timeout=1)

pygame.init()

pad = pygame.joystick.Joystick(0)

pad.init()

print 'Initialized Joystick : %s ' % pad.get_name()

print 'Number of hats %i' % pad.get_numhats()
buttons = pad.get_numbuttons()

# def send_command(i):
#     if i is 10:
#         ser.write('*')
#     elif i is 11:
#         ser.write('s')

try:
    while True:
        pygame.event.pump()
        for evt in pygame.event.get():
            print 'Axis 0 %f' % pad.get_axis(0)
            print 'Axis 1 %f' % pad.get_axis(1)
            print 'Axis 2 %f' % pad.get_axis(2)
            print 'Axis 3 %f' % pad.get_axis(3)

        #     print 'Axis 3 %f' % pad.get_axis(3)
        for evt in pygame.event.get():
            for i in range(buttons):
                button = pad.get_button(i)
                if evt.type == pygame.locals.JOYBUTTONDOWN and button == 1:
                    print 'Button %i  reads %i' % (i, button)
                    # send_command(i)

        time.sleep(0.2)

except KeyboardInterrupt:
    pad.quit()

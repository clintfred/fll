#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.motor import *
from ev3dev2.wheel import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor import *
from ev3dev2.button import *
from ev3dev2.display import Display
from ev3dev2.sound import Sound
import ev3dev2.fonts as fonts
import logging
import time

logging.basicConfig(level=logging.DEBUG)

gyro = GyroSensor(INPUT_4)
disp = Display()
b = Button()
s = Sound()

color_left = ColorSensor(INPUT_2)
color_right = ColorSensor(INPUT_3)
# 14, 18, ...
# See all: https://python-ev3dev.readthedocs.io/en/latest/display.html
f = fonts.load('luBS18')

"""
Please depending on the robot change the number below:                                <<<       IMPORTANT!!!!                                         HEY!!!!
WHICH_ROBOT = 0 is if the robot has one color sensor                               <<<<<<OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
WHICH_ROBOT = 1 is if the robot has two color sensor                                  <<<                          LOOK!!!!
PLEASE AND THANK YOU
"""
WHICH_ROBOT = 2
if WHICH_ROBOT == 0:
    L_MOTOR = OUTPUT_A
    R_MOTOR = OUTPUT_B
    GYRO_PORT = INPUT_3
    IS_INVERTED = True
    DEGREES_PER_INCH = 43.0 * 2  # guess
elif WHICH_ROBOT == 1:
    # LRPB
    L_MOTOR = OUTPUT_D
    R_MOTOR = OUTPUT_A
    GYRO_PORT = INPUT_4
    IS_INVERTED = False
    DEGREES_PER_INCH = 53.0
elif WHICH_ROBOT == 2:
    # LRPB
    L_MOTOR = OUTPUT_A
    R_MOTOR = OUTPUT_D
    GYRO_PORT = INPUT_4
    IS_INVERTED = False
    DEGREES_PER_INCH = 53.0
else:
    L_MOTOR = OUTPUT_A
    R_MOTOR = OUTPUT_D
    GYRO_PORT = INPUT_2
    IS_INVERTED = False
    DEGREES_PER_INCH = 43.0 + 1 - 1


class WideWheel(Wheel):
    def __init__(self):
        Wheel.__init__(self, 68.8, 36)


tank_drive = MoveTank(L_MOTOR, R_MOTOR)
tank_diff = MoveDifferential(L_MOTOR, R_MOTOR, WideWheel, 146.75)


def show(text):
    #disp.text_grid(t, x=5, y=5)
    disp.clear()
    # 0 = left, 80 = far right?
    # align="center" doesn't seem to work
    lr = 0
    down = 20
    #down = 50
    disp.draw.text((lr, down), text, font=f, align="left")
    disp.update()


def follow_line(speed=10, want_rli=65, edge="right"):
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_D)
    while True:
        rli = color_left.reflected_light_intensity
        t = "rli: {}".format(rli)
        show(t)
        diff = rli - want_rli
        # bigger adjustment for faster speed?
        #factor = speed / 100.0
        # light difference is +/- 40 but
        # motor difference should be small.
        factor = 0.05
        bias = diff * factor
        if edge == "left":
            bias = -bias
        m1.on(speed - bias)
        m2.on(speed + bias)
        time.sleep(0.01)


def follow_line_left():
    follow_line(speed=10, want_rli=65, edge="left")


def follow_line_right():
    follow_line(speed=10, want_rli=65, edge="right")

# Needs 2 color sensors!!!


def align_black(speed=10, black_thresh=20):
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_D)
    m1.on(speed)
    m2.on(speed)
    left_running = True
    right_running = True
    while left_running or right_running:
        rli_left = color_left.reflected_light_intensity
        rli_right = color_right.reflected_light_intensity
        t = "rli: {} {}".format(rli_left, rli_right)
        show(t)
        if rli_left <= black_thresh:
            m1.off()
            left_running = False
        if rli_right <= black_thresh:
            m2.off()
            right_running = False


def align_forever():
    while True:
        s.beep()
        #follow_line(speed=20, edge="left")
        align_black(speed=10)
        s.beep()
        while not b.any():
            rli_left = color_left.reflected_light_intensity
            rli_right = color_right.reflected_light_intensity
            t = "rli: {} {}".format(rli_left, rli_right)
            show(t)
            time.sleep(0.1)


def display_rfi():
    while not b.any():
        rli = color_left.reflected_light_intensity
        t = "rli: {}".format(rli)
        show(t)
        logging.info(t)
        time.sleep(0.1)


def beep_and_wait_for_button():
    # Don't do anything else
    s.beep()
    while not b.any():
        time.sleep(0.1)
    s.beep()


def motor_test():
    m = LargeMotor(OUTPUT_D)
    m.on_for_rotations(SpeedPercent(75), 5)
    # hoi bois!!!!


def gyro_test():
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_D)
    m1.on(20)
    m2.on(-20)
    gyro.wait_until_angle_changed_by(90)
    m1.off()
    m2.off()


def long_gyro_test():
    s.beep()
    show("push a button")
    #b.wait_for_pressed([Button.enter, Button.right])
    #b.wait_for_bump([Button.enter, Button.right])
    b.wait_for_pressed(["enter"])
    s.beep()
    gyro_test()
    # motor_test()
    # for i in range(50):
    # Loop until button pressed, displaying gyro angle
    while not b.any():
        ang = gyro.angle
        t = "Angle: {}".format(ang)
        show(t)
        logging.info(t)
        time.sleep(0.1)


# gyro: gyro sensor
# deg: degrees to turn (positive is clockwise)
# speed: motor speed 0-100 (must be positive)
def turn_degrees(gyro, deg, speed=20):
    gyro.mode = GyroSensor.MODE_GYRO_ANG

    if abs(deg) <= 1:
        return

    if deg > 0:
        lspeed = speed
    else:
        lspeed = -speed
    turn_about_center = True
    if turn_about_center:
        rspeed = -lspeed
    else:
        rspeed = 0
    time.sleep(0.5)
    gyro.reset()

    start = gyro.value()
    log.info("gyro inital value: {}".format(start))
    log.info("gyro in mode: {}".format(gyro.mode))
    # log.info("gyro inital value after mode set: {}".format(gyro.value))

    tank_drive.on(lspeed, rspeed)
    log.info("turning {}: {} {}".format(deg, lspeed, rspeed))

    gyro.wait_until_angle_changed_by(abs(deg))
    tank_drive.off()
    time.sleep(0.5)

    final = gyro.value()

    log.info("gyro final value: {}".format(final))
    log.info("stopping")

    # Go back the amount we missed or went over.
    correction = final - deg
    log.info("correction: {}".format(correction))

    if abs(correction) <= 1:
        pass
    else:
        if deg > 0:
            lspeed = 5
        else:
            lspeed = -5
        turn_about_center = True
        if turn_about_center:
            rspeed = -lspeed
        else:
            rspeed = 0

        tank_drive.on(rspeed, lspeed)
        log.info("C: turning {}: {} {}".format(deg, lspeed, rspeed))
        log.info("C: gyro in mode: {}".format(gyro.mode))
        log.info("C: gyro inital value: {}".format(gyro.value()))
        gyro.mode = GyroSensor.MODE_GYRO_ANG
        log.info("C: gyro inital value after mode set: {}".format(gyro.value()))

        gyro.wait_until_angle_changed_by(abs(correction)-1)
        tank_drive.off()
        time.sleep(0.5)
        log.info("C: gyro final value: {}".format(gyro.value()))


def inches_to_mill(inches):
    return 25.4 * inches


def move_turn():
    tank_drive.on_for_seconds(SpeedPercent(-50), SpeedPercent(-50), 1)
    tank_drive.on_for_seconds(SpeedPercent(-50), SpeedPercent(50), 0.65)


def drive_inches(distance, speed=50):
    # wheel diameter is 43.2mm (?)
    circum_inches = 4.32 * 3.14 / 2.54
    rotations = distance / circum_inches
    speed = -speed if IS_INVERTED else speed
    #tank_drive.on_for_rotations(SpeedPercent(speed), SpeedPercent(speed), rotations)
    tank_diff.on_for_distance(SpeedPercent(
        speed), inches_to_mill(distance))


def mission_2_crane(gyro):
    drive_inches(12)
    # 6 inches out 6 inches over
    turn_degrees(gyro, 45)
    drive_inches(7*1.414)
    turn_degrees(gyro, -45)
    drive_inches(9)
    drive_inches(6, -20)
    turn_degrees(gyro, -45)
    drive_inches(3*1.414)
    turn_degrees(gyro, 45)
    drive_inches(4)


def mission_12_dandb(gyro):

    drive_inches(.5)
    #turn_degrees(gyro, -72, 15)
    tank_diff.turn_right(15, 69)
    drive_inches(51, 30)

    # turn_degrees(gyro, 180, 7)

    # turn_degrees(gyro, -85, 35)

    # turn_degrees(gyro, -85, 35)

    # drive_inches(26)


def mission_tan_blocks(gyro):
    # drive_inches(36, 40)
    # drive_inches(-36, 80)
    drive_inches(2, 20)
    tank_diff.turn_right(20, 63)
    drive_inches(43, 46)


def mission_red_blocks(gyro):
    """
    Setup:
    Line up robot from wall to the 5th from the right hashmark.
    Line up blocks so it looks like a rectangle.
    Two pieces of LEGO block will be sticking up on oppisite ends.
    Place an upgrade on the furthest LEGO block sticking up.
    Make sure that the blocks are lined up on the left side of the attachment.
    Your good to go!
    """

    drive_inches(9, 20)
    tank_diff.turn_right(15, 85)
    drive_inches(22.5, 30)
    drive_inches(-37, 80)
    tank_diff.turn_left(80, 70)


done = False
wait_for = None
choice = 2

progs = [
    ("m: 2 crane", mission_2_crane),
    ("m: 12 dandb", mission_12_dandb),
    ("m: red blocks", mission_red_blocks),
    ("m: tan blocks", mission_tan_blocks),
]


def change(changed_buttons):
    global done
    global choice
    # changed_buttons is a list of
    # tuples of changed button names and their states.
    logging.info('These buttons changed state: ' + str(changed_buttons))
    if wait_for is not None and wait_for in changed_buttons:
        logging.info('You pressed the done button')
        done = True
    else:
        done = False
    if ("up", True) in changed_buttons:
        choice -= 1
        if choice < 0:
            choice = len(progs) - 1
    elif ("down", True) in changed_buttons:
        choice += 1
        if choice >= len(progs):
            choice = 0
    logging.info('Done is: ' + str(done))
    s.beep()
    return done

# Set callback from b.process()


def run_program():
    # This loop checks button states
    # continuously and calls appropriate event handlers
    global done
    global wait_for
    done = False
    wait_for = ("enter", True)
    logging.info("Waiting for enter button.")
    while not done:
        #ang = gyro.angle
        ang = 0
        rli_left = color_left.reflected_light_intensity
        rli_right = color_right.reflected_light_intensity
        t = "rli: {} {}".format(rli_left, rli_right)
        t = "Ang: {}\nrli: {} {}\nP: {}\nWaiting for l button".format(
            ang, rli_left, rli_right, progs[choice][0])
        show(t)
        b.process()
        time.sleep(0.1)

    logging.info("And done.")
    logging.info("Running {}".format(progs[choice][0]))
    progs[choice][1](gyro)
    s.beep()


if __name__ == "__main__":
    # Resets to 0, does not fix drift
    gyro.reset()
    b.on_change = change
    s.beep()
    s.beep()
    s.beep()
    while True:
        run_program()

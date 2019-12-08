#!/usr/bin/env micropython

#                                                                         Hello World!!!!!

import logging
import inspect
from time   import sleep
from random import randint
from math import floor, pi

from ev3dev2.motor import *
from ev3dev2.wheel import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor import *
from ev3dev2.button import *
from ev3dev2.led import Leds


b = Button()
s = Sound()

# logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)5s %(filename)s: %(message)s")
log = logging.getLogger(__name__)

# Connect two large motors on output ports B and C:
#motors = [LargeMotor(address) for address in (R_MOTOR, L_MOTOR)]

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
    DEGREES_PER_INCH = 43.0 * 2 # guess
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
# for cal uncomet the line below and ajust 4 param in movediff see dox for more details
#tank_diff.on_arc_left(35, 200, 2*200*pi)
#exit(0)
#tank_drive.on_for_seconds(SpeedPercent(-50), SpeedPercent(50), 0.64)
#tank_drive.on_for_seconds(SpeedPercent(45), SpeedPercent(-45), 1)

#DriveSquare

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
    tank_drive.on_for_distance(SpeedPercent(speed), SpeedPercent(speed), inches_to_mill(distance))

def drive_inches_accel(distance, speed=50):
    # wheel diameter is 43.2mm (?)
    circum_inches = 4.32 * 3.14 / 2.54
    rotations = distance / circum_inches
    #speed = -speed

    rot_left = rotations
    nrot = 0.0
    rot_incr = 0.25
    sp_incr = 5.0
    sp = sp_incr
    while nrot < rot_left:        
        tank_drive.on_for_rotations(SpeedPercent(sp), SpeedPercent(sp), rot_incr,brake=False)
        nrot += rot_incr
        sp += sp_incr
        if sp > speed:
            sp = speed

def square():
    log.info("square")
    for _ in range(0, 4):
        move_turn()

def setup_gyro():
    s = GyroSensor(GYRO_PORT)
    s.mode = GyroSensor.MODE_GYRO_CAL
    s.reset() #fixed in 2.3.3 kernel https://github.com/ev3dev/ev3dev/issues/1236
    return s


def move_test():
    sensor = setup_gyro()
    
    
    log.info(sensor.rate)
    time.sleep(0.5)
    
    STUD_MM = 8

    # test with a robot that:
    # - uses the standard wheels known as EV3Tire
    # - wheels are 16 studs apart
    mdiff = MoveDifferential(L_MOTOR, R_MOTOR, EV3Tire, 16 * STUD_MM)

    # Rotate 90 degrees clockwise
    mdiff.turn_right(SpeedRPM(40), 90)

    # Drive forward 500 mm
    mdiff.on_for_distance(SpeedRPM(40), inches_to_mill(3))

    # Drive in arc to the right along an imaginary circle of radius 150 mm.
    # Drive for 700 mm around this imaginary circle.
    mdiff.on_arc_right(SpeedRPM(80), 150, 700)



    # Enable odometry
    log.info(dir(MoveDifferential))

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

    

def drive_inches(dist, speed=20):
    # Slightly smaller than 43
    degrees = dist * DEGREES_PER_INCH
    tank_drive.on_for_degrees(speed, speed, degrees)


def turn_test():
    sensor = setup_gyro()
    
    #log.info("Rate: {}".format(sensor.rate))
    log.info("Rate: " + str(sensor.rate))
    time.sleep(0.5)

    STUD_MM = 8

    # test with a robot that:
    # - uses the standard wheels known as EV3Tire
    # - wheels are 16 studs apart
    log.info("driving")
    mdiff = MoveDifferential(L_MOTOR, R_MOTOR, EV3Tire, 17.8 * STUD_MM)

    # Rotate 90 degrees clockwise
    #log.info("turning")
    #mdiff.turn_right(SpeedRPM(40), 180)

    #sensor.reset()
    
    #sensor.mode = GyroSensor.MODE_TILT_ANG


    log.info("gyro turning left")
    turn_degrees(sensor, 270, speed=34.5374)

    log.info("gyro turning right")
    turn_degrees(sensor, -270, speed=34.5374)

def drive_test():
    sensor = setup_gyro()
    
    #log.info("Rate: {}".format(sensor.rate))
    log.info("Rate: " + str(sensor.rate))
    time.sleep(0.5)

    STUD_MM = 8

    # test with a robot that:
    # - uses the standard wheels known as EV3Tire
    # - wheels are 16 studs apart
    log.info("driving")
    mdiff = MoveDifferential(L_MOTOR, R_MOTOR, EV3Tire, 17.8 * STUD_MM)
    if IS_INVERTED:
        mdiff.set_polarity(LargeMotor.POLARITY_INVERSED)
   # Drive forward 500 mm
    speed = 40
    mdiff.on_for_distance(SpeedRPM(speed), inches_to_mill(10), block = True)
    return 
    log.info(mdiff.motors[L_MOTOR].state)
    start_angle = sensor.angle
    max_adjust_rpm = 80
    max_angle_adjust = 90
    while mdiff.is_running:
        current_diff = abs(sensor.angle - start_angle)
        speed_diff = floor(speed - current_diff / max_angle_adjust * max_adjust_rpm)
        if sensor.angle < 0:
            #mdiff.left_motor.speed_sp = 0
            mdiff.left_motor.speed_sp = SpeedRPM(speed_diff).to_native_units(mdiff.left_motor)
            
            mdiff.left_motor.run_to_rel_pos()
            log.info("speeddiff "+ str(speed_diff))
        # mdiff.max_speed.spe

def setup():
    pass

def old():
    sensor = setup_gyro()
    sensor.mode = GyroSensor.MODE_GYRO_ANG

    tank_drive.on(-20, 20)
    sensor.wait_until_angle_changed_by(270)
    log.info("stopping")
    tank_drive.off()
    # YOUSHALLNOTPASS!!

    log.info("gyro turning right")
    tank_drive.on(+20, -20)
    sensor.wait_until_angle_changed_by(270)
    log.info("stopping")
    tank_drive.off()

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
    drive_inches(36, 40)
    drive_inches(-36, 80)


def mission_red_blocks(gyro):
    
    """ 
    Setup:
    Line up robot from wall to the 6th hashmark.
    Line up blocks so it looks like a rectangle.
    Two pieces of LEGO block will be sticking up on oppisite ends.
    Place an upgrade on the furthest LEGO block sticking up.
    Make sure that the blocks are lined up on the left side of the attachment.
    Your good to go!
    """

    drive_inches(9)
    tank_diff.turn_right(15, 90)
    drive_inches(19, 30)
    drive_inches(-31, 25)
    tank_diff.turn_left(15, 90)    

if __name__ == "__main_XXX__":

    #square()
    #drive_inches_accel(11, 100)
    log.info("starting")

    

    move_test()
    tank = MoveTank(L_MOTOR, R_MOTOR)
    tank.cs = ColorSensor()
    if IS_INVERTED: 
        tank.set_polarity(LargeMotor.POLARITY_INVERSED)
    try:
        # Follow the line for 4500ms
        tank.follow_line(
            kp=11.3, ki=0.05, kd=3.2,
            speed=SpeedPercent(30),
            white=40,
            follow_left_edge=True,
            follow_for=follow_for_ms,
            ms=2500
        )
    except Exception:
        tank.stop()
        raise

done = False
wait_for = None
choice = 0
progs = [
    ("mission 2 crane", mission_2_crane),
    ("mission 12 dandb", mission_12_dandb),
    ("mission red blocks", mission_red_blocks),
    ("mission tan blocks", mission_tan_blocks),
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
    if ("up", True) in changed_buttons:
        choice -= 1
        if choice < 0:
            choice = len(progs) - 1
    elif ("down", True) in changed_buttons:
        choice += 1
        if choice >= len(progs):
            choice = 0
    logging.info('Done is: ' + str(done))
    # will also beep if release button
    s.beep()

# Set callback from b.process()
b.on_change = change

def run_program(gyro):
    # This loop checks button states
    # continuously and calls appropriate event handlers
    global done
    global wait_for
    done = False
    wait_for = ("enter", True)
    logging.info("Waiting for enter button.")
    while not done:
        ang = gyro.angle
        rli_left = color_left.reflected_light_intensity
        rli_right = color_right.reflected_light_intensity
        t = "Angle: {}\nrli: {} {}\nProg {}: {}\nWaiting for enter".format(ang, rli_left, rli_right, choice, progs[choice][0])
        show(t)
        b.process()
        time.sleep(0.1)

    logging.info("And done.")
    logging.info("Running {}".format(progs[choice][0]))
    progs[choice][1](gyro)
    s.beep()

if __name__ == "__main__":

    log.info("starting")

    s.beep()
    b.wait_for_pressed(["enter"])

    gyro = setup_gyro()
    gyro.mode = GyroSensor.MODE_GYRO_ANG
    # mission_2_crane(gyro)
    #mission_12_dandb(gyro)
    #mission_red_blocks(gyro)
    #mission_tan_blocks(gyro)

    s.beep()
    s.beep()
    s.beep()
    while True:
        run_program(gyro)

    # drive_test()
    # old()
    if False:
        log.info("angle {}".format(gyro.angle))
        turn_degrees(gyro, 270, 20)
        log.info("angle {}".format(gyro.angle))

    if False:
        drive_inches(3 * 18)

    if False: 
        m = MediumMotor(OUTPUT_C)
        #m.on_for_degrees(20, 900)
        m.on_for_rotations(100, 10)
    
    if False:
        sp = -80
        #tank_drive.on_for_seconds(sp, sp, 2)
        #tank_diff.on_for_seconds(sp, sp, 2)
        tank_diff.on_for_distance(sp, 10*10*5)

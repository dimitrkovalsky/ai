# ----------
# Part Five
#
# This time, the sensor measurements from the runaway Traxbot will be VERY 
# noisy (about twice the target's stepsize). You will use this noisy stream
# of measurements to localize and catch the target.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3 and 4. Again, try to catch the target in as few steps as possible.


from robot import *
from math import *
from matrix import *
import random
import turtle

animate = True

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER1 = [[],[]]
        OTHER = (measurements, hunter_positions, hunter_headings, OTHER1) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings, OTHER1= OTHER # now I can always refer to these variables

    stepsAhead = 9
    step = len(OTHER[0])
    nextPos, OTHER = estimate_next_pos(target_measurement, stepsAhead - step % stepsAhead -1, OTHER)

    heading_to_target = get_heading(hunter_position, nextPos)
    headingError = angle_trunc(heading_to_target - hunter_heading)
    distError = sqrt((nextPos[0] - hunter_position[0])**2 + (nextPos[1] - hunter_position[1])**2)
    turning =  headingError
    distance = distError
    return turning, distance, OTHER

def estimate_next_pos(measurement, stepsAhead, OTHER):

    if len(OTHER[3][0]) < 5:
        OTHER[3][0].append(measurement[0])
        OTHER[3][1].append(measurement[1])
        xy_estimate = measurement
        R = sqrt(measurement[0]**2 + measurement[1]**2)
        curtheta = atan2(measurement[1], measurement[0])
        center = [0, 0]
    else:
        OTHER[3][0].append(measurement[0])
        OTHER[3][1].append(measurement[1])

        x_m = sum(OTHER[3][0]) / len(OTHER[3][0])
        y_m = sum(OTHER[3][1]) / len(OTHER[3][1])

        u = []
        v = []
        for i in range(len(OTHER[3][0])):
            u.append(OTHER[3][0][i] - x_m)
            v.append(OTHER[3][1][i] - y_m)

        Suv = []
        Suu = []
        Svv = []
        Suuv = []
        Suvv = []
        Suuu = []
        Svvv = []

        for i in range(len(OTHER[3][0])):
            Suv.append(u[i]*v[i])
            Suu.append(u[i]*u[i])
            Svv.append(v[i]*v[i])
            Suuv.append(u[i]*u[i]*v[i])
            Suvv.append(u[i]*v[i]*v[i])
            Suuu.append(u[i]*u[i]*u[i])
            Svvv.append(v[i]*v[i]*v[i])

        A = matrix([[sum(Suu), sum(Suv)], [sum(Suv), sum(Svv)]])
        B = matrix([[sum(Suuu) + sum(Suvv)], [sum(Svvv) + sum(Suuv)]])
        C = A.inverse() * B
        center = [C.value[0][0] / 2.0 + x_m, C.value[1][0] / 2.0 + y_m]

        res = []
        for i in range(len(OTHER[3][0])):
            res.append(sqrt((OTHER[3][0][i]-center[0])**2 + (OTHER[3][1][i]-center[1])**2))
        R = sum(res) / len(res)


        theta = []
        meantheta = 0
        for i in range(len(OTHER[3][0])):
            theta.append(atan2(OTHER[3][1][i] - center[1], OTHER[3][0][i] - center[0]))
            if i > 0:
                diff = theta[i] - theta[i - 1]
                if diff > pi:
                    diff -= 2 * pi
                elif diff < -pi:
                    diff += 2 * pi
                meantheta += diff
        meantheta /= (len(theta) - 1)

        starttheta = 0
        for i in range(len(theta)):
            start = (theta[i] - meantheta * i) % pi
            if theta[0] < 0:
                start -= pi
            starttheta += start
        starttheta /= len(theta)

#        curtheta = starttheta + len(theta) * meantheta
        curtheta = starttheta + (len(theta) + stepsAhead) * meantheta
        xy_estimate = (center[0] + R * cos(curtheta), center[1] + R * sin(curtheta))

        print "i, center, R, meantheta, starttheta: ", i, center, R, meantheta, starttheta

    return xy_estimate, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.98 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # animation
    if animate:
        window = turtle.Screen()
        window.bgcolor('white')
        size_multiplier= 10.0  #change Size of animation
        robot1 = turtle.Turtle()
        robot1.shape('turtle')
        robot1.color('green')
        robot1.resizemode('user')
        robot1.shapesize(0.3, 0.3, 0.3)
        robot2 = turtle.Turtle()
        robot2.shape('circle')
        robot2.color('red')
        robot2.resizemode('user')
        robot2.shapesize(0.3, 0.3, 0.3)
        robot1.penup()
        robot2.penup()

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # animation
        if animate:
            robot1.setheading(hunter_bot.heading*180/pi)
            robot1.goto(hunter_position[0]*size_multiplier, hunter_position[1]*size_multiplier-200)
            robot2.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-200)
            robot1.stamp()
            robot2.stamp()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught



def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = angle_trunc(heading_to_target - hunter_heading)
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 2e-1*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, next_move)

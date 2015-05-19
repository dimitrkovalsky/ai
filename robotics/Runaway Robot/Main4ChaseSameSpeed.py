# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot. 
# But this time, your speed will be about the same as the runaway bot. 
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time. 
#
# ----------
# GRADING
# 
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
import random
import turtle



animate = True

def estimate_next_pos(measurement, OTHER=None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    # identity matrix
    I = matrix([[1., 0., 0., 0., 0.],
                [0., 1., 0., 0., 0.],
                [0., 0., 1., 0., 0.],
                [0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 1.]])

    #motion update matrix
    H = matrix([[1., 0., 0., 0., 0.],
                [0., 1., 0., 0., 0.]])

    #measurement noise
    R = matrix([[measurement_noise, 0.],
                [0., measurement_noise]])

    z = matrix([[measurement[0]],
                [measurement[1]]])

    u = matrix([[0.],
                [0.],
                [0.],
                [0.],
                [0.]])

    if OTHER is not None and 'X' not in OTHER:
        last_measurement = OTHER['last_measurement']

        angle = atan2(measurement[0] - last_measurement[0], measurement[1] - last_measurement[1])
        if 'last_angle' not in OTHER:
            OTHER['last_angle'] = angle
            xy_estimate = [1., 1.]
            OTHER['last_measurement'] = measurement
            return xy_estimate, OTHER
        else:
            turning_angle = angle - OTHER['last_angle']

    elif OTHER is None:
        OTHER = {'last_measurement': measurement}
        return [1.,1.], OTHER


    if 'X' in OTHER:
        X = OTHER['X']
        P = OTHER['P']

    else:
        print 'here!!'
        X = matrix([[measurement[0]],
                    [measurement[1]],
                    [1.],
                    [turning_angle],
                    [1.]])
        #convariance matrix
        P = matrix([[measurement_noise, 0., 0., 0., 0.],
                    [0., measurement_noise, 0., 0., 0.],
                    [0., 0., measurement_noise, 0., 0.],
                    [0., 0., 0., measurement_noise, 0.],
                    [0., 0., 0., 0., measurement_noise]
        ])

    #measurement update
    Y = z - (H * X)
    S = H * P * H.transpose() + R
    K = P * H.transpose() * S.inverse()

    X = X + (K * Y)
    P = (I - (K * H)) * P

    #Prediction
    x = X.value[0][0]
    y = X.value[1][0]
    angle = X.value[2][0]
    turning_angle = X.value[3][0]
    distance = X.value[4][0]



    new_X = [
        [x + distance * sin(angle+turning_angle)],
        [y + distance * cos(angle+turning_angle)],
        [angle+turning_angle],
        [turning_angle],
        [distance],
    ]

    update_row0 = [
        1.,
        0.,
        distance * cos(angle+turning_angle),
        distance * cos(angle+turning_angle),
        sin(angle+turning_angle),
    ]

    update_row1 = [
        0.,
        1.,
        -distance * sin(turning_angle+angle),
        -distance * sin(turning_angle+angle),
        cos(angle+turning_angle)
    ]

    updated_X = [
        update_row0,
        update_row1,
        [0., 0., 1., 1., 0.],
        [0., 0., 0., 1., 0.],
        [0., 0., 0., 0., 1.],
    ]

    A = matrix(updated_X)

    P = A * P * A.transpose()

    X = matrix(new_X)

    xy_estimate = [X.value[0][0], X.value[1][0]]
    #OTHER = {'X': X, 'P': P}
    OTHER['X'] = X
    OTHER['P'] = P

    # You must return xy_estimate (x, y), and OTHER (even if it is None)
    # in this order for grading purposes.
    #X.show()

    #print '----------------------------------------------'
    return xy_estimate, OTHER

def find_ambush_point(hunter_position, robotpos, max_distance, OTHER):
    print 'Planning ambush..'

    OTHER['chase_counter'] = 0
    target_steps = 1
    next_xy = robotpos

    X = OTHER['X']
    P = OTHER['P']
    X_copy = matrix([[v for v in c] for c in X.value])
    P_copy = matrix([[v for v in c] for c in P.value])

    OTHER2 = {'X': X_copy, 'P': P_copy}

    while True:
        if distance_between(hunter_position, next_xy)/max_distance < target_steps:
            next_xy2, OTHER2 = estimate_next_pos(next_xy, OTHER2)
            next_xy = [(next_xy[0] + next_xy2[0])/2, (next_xy[1] + next_xy2[1])/2]
            break
        else:
            next_xy, OTHER2 = estimate_next_pos(next_xy, OTHER2)
            target_steps += 1

    return next_xy


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    next_xy, OTHER = estimate_next_pos(target_measurement, OTHER)

    if OTHER.get('chased', False):
        ambush_point = find_ambush_point(hunter_position, next_xy, max_distance, OTHER)
        OTHER['chased'] = False
        OTHER['ambush'] = ambush_point


    elif able_to_chase(next_xy, hunter_position, max_distance):
        print 'Chasing...'
        angle = atan2(next_xy[1]-hunter_position[1], next_xy[0]-hunter_position[0])
        desired_heading = angle_trunc(angle - hunter_heading)
        distance_to_move = distance_between(next_xy, hunter_position)

        OTHER['chased'] = True
        return desired_heading, distance_to_move, OTHER


    if ('P' not in OTHER or OTHER['P'].value[0][0] > .1 or OTHER['P'].value[1][1] > .1) and 'ambush' not in OTHER:
        print 'target too far away. moving closer to target'
        destination = next_xy
    elif 'ambush' not in OTHER:
        destination = find_ambush_point(hunter_position, next_xy, max_distance, OTHER)
        OTHER['ambush'] = destination
    else:
        print 'moving towards ambush point...'
        destination = OTHER['ambush']

    angle = atan2(destination[1]-hunter_position[1], destination[0]-hunter_position[0])
    desired_heading = angle_trunc(angle - hunter_heading)
    distance_to_move = distance_between(destination, hunter_position)
    return desired_heading, distance_to_move, OTHER

def able_to_chase(hunter_p, robot_p, max_distance):
    return distance_between(hunter_p, robot_p) < max_distance


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
    #        robot1.stamp()
    #        robot2.stamp()

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
measurement_noise = .05*target.distance
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

print demo_grading(hunter, target, next_move)

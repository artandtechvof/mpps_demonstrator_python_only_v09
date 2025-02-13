
CollisionDetection = False
RandomDestination = True # random object or object based on own ID
HIL_Simulation = False    # run code on
FirstClaimDestination = True # True: wait for destination area te be cleared.
                             # False: turn towards destination even if it is occupied
UseFleetmanager = False
Full_auto = False
ID = 4

DEBUG = False

# Read analog input pins
# pins.analog_read_pin(AnalogPin.P0)
# pins.analog_read_pin(AnalogPin.P1)

UseHusky = True
DisplayDelay = 200

def showText(txt):
    #if not HIL_Simulation:
    if UseHusky:
        txt = str(ID) + ' ' + txt
        huskylens.clear_osd()
        huskylens.write_osd(txt, 30, 30)
        if HIL_Simulation: basic.pause(DisplayDelay)


######################################
serial.redirect_to_usb()
serial.write_string("I am alive")
if not HIL_Simulation or UseHusky:
    huskylens.init_i2c()
    huskylens.init_mode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
    showText("BOOTING")
basic.pause(1000)
######################################
# number of robots
numrobots = 1
# number of 'plastic' object types
numobjects = 4 # should be an even number for now
numtargets = numobjects
# buffer zones for wating robots
# 1 robot = 1 waiting
# 2 robot = 1 waiting, 1 on storage location
# 3 robot = 3 waiting, 1 left, 1 right 1 central
# 4 robot = 5 waiting, 2 left, 2 right, 1 central
# 5 robot = 7 waiting, 3 left, 3 right, 1 central
# 6 robot = 9 waiting, 4 left, 4 right, 1 central
# 7 robot = 11 waiting, 5 left, 5 right, 1 central
# 8 robots = 13 waiting, 6 left, 6 right, 1 central
if numrobots == 1:
    numwaitingareas = 5 # adjust to demo
else:
    numwaitingareas = max(3,(numrobots-1)*2 -1) # 1 central waiting area offset to the front
numstorage = 1 # try increasing this in future scenarios, attack waste-pile from multiple locations
# total number of locations where robots can go
numlocations = numwaitingareas + numstorage + numtargets + numobjects


# constants
IDLE = 0
DRIVEBACKWARD = 1
FINDTARGET = 2
GOTOTARGET = 3
FINDOBJECT = 4
GRABOBJECT = 5

LOCATION_t = 0 # robot is looking for location
OBJECT_t = 1   # robot is looking for object

OPENED = 1
CLOSED = 0

AHEAD = 0
LEFT  = 1
RIGHT = 2

MAXSPEED = 22
MINSPEED = 15
MINYAWSPEED = 8
MAXYAWSPEED = 14
BACKUPSPEED = -20
BACKUPDISTANCE = 20

OBJECT_THRESHOLD = 0 # minimum distance to object, before closing grippers

state_names = ['IDLE', 'DRIVEBACKWARD', 'FINDTARGET', 'GOTOTARGET', 'FINDOBJECT', 'GRABOBJECT']
direction_names = ['AHEAD', 'LEFT', 'RIGHT']
cmd_names = ['STATUS_UPDATE','CLAIM_DESTINATION','RELEASE_POSITION','RELEASE_ROUTE','ROBOT_START','ROBOT_STOP']
rply_names = ['NOK','OK','ACK']

# commands
NO_CMD = -1
POSITION_UPDATE = 10
STATUS_UPDATE = 0
CLAIM_DESTINATION = 1
RELEASE_POSITION = 2
RELEASE_ROUTE = 3
ROBOT_START = 4
ROBOT_STOP = 5
NOK = 0
OK = 1
ACK = 2

#initialize variables related to huskylens
t_start = 0
t_last = 0
dt_AI = 0
inview = 0
x = 0
y = 0
w = 0
h = 0
newdata = False
inview_count = 0
RadioTxPending = False
tagsInView = [0]
frame_count = 0
buffer_xywh = [-1,-1,-1,-1]
buffer_tagsInView = [0]

tagID = 1
# tagIDs = [1 tm 6]
# tuning per robot, mainly sotrage area
if ID == 1:
    tagSizes = [0,  88, 90, 90, 90, 88,  87,  88, 88, 88, 88,  55, 55, 55, 55] # 5x waiting, 1 central, 4 targets, 4 objects
if ID == 2:
    tagSizes = [0,  88, 90, 90, 90, 83,  87,  88, 88, 88, 88,  55, 55, 55, 55] # 5x waiting, 1 central, 4 targets, 4 objects
if ID == 3:
    tagSizes = [0,  88, 90, 90, 90, 86,  87,  88, 88, 88, 88,  55, 55, 55, 55] # 5x waiting, 1 central, 4 targets, 4 objects
if ID == 4:
    tagSizes = [0,  88, 90, 90, 90, 86,  87,  88, 88, 88, 88,  55, 55, 55, 55] # 5x waiting, 1 central, 4 targets, 4 objects

tagSize = tagSizes[tagID]

zeroes = [0]
while i < len(tagSizes):
    zeroes.append(0)
    i += 1
tagsInView = zeroes
#start_searching = 0


#---------------------------------------------------------------------------------------------------------
# helper function to open and close grippers, will be different on actual robot
#---------------------------------------------------------------------------------------------------------
def open_gripper():
    opengripper = True
    maqueen.servo_run(maqueen.Servos.S1, 85)
    return opengripper


def close_gripper():
    closegripper = True
    maqueen.servo_run(maqueen.Servos.S1, 105)
    return closegripper

def actuate_motors():
    if HIL_Simulation: return True

    left_speed = (robot.speed + robot.yaw_speed)
    right_speed = (robot.speed + -robot.yaw_speed)
    #serial.write_line(str(left_speed)+" left speed")
    #serial.write_line(str(right_speed)+" right speed")
    # set speed for motors here
    if left_speed >=0:
        maqueen.motor_run(maqueen.Motors.M1, maqueen.Dir.CW, left_speed)
    else:
        maqueen.motor_run(maqueen.Motors.M1, maqueen.Dir.CCW, -left_speed)
    if right_speed >=0:
       maqueen.motor_run(maqueen.Motors.M2, maqueen.Dir.CW, right_speed)
    else:
        maqueen.motor_run(maqueen.Motors.M2, maqueen.Dir.CCW, -right_speed)

    return True

def get_time():
    return input.running_time()

#---------------------------------------------------------------------------------------------------------
# base class of robot, can also be used on micro:Maqueen robot.
# Robot() class, should be same for offline Simulation and actual physical demo robot
#---------------------------------------------------------------------------------------------------------
class Robot:
    def __init__(self, loc=1, dst=1, _id=1, _obstacle=15):
        # who are we
        self.id = _id
        # ego location and destination nr
        self.location = loc
        self.destination = dst
        self.target = dst
        self.targettype = LOCATION_t
        self.objecttype = None
        self.obstacle = _obstacle
        # states
        self.state = IDLE
        self.gripper = CLOSED
        self.speed = 0
        self.yaw_speed = 0
        # params
        self.active = False
        self.run_enable = False
        self.steering_only = False # allowed to change heading but not driving
        self.target_direction = LEFT
        self.target_inview = False  # target is in view of camera
        self.target_locked = False # target is straight ahead
        self.backup_distance = 0
        self.distance_to_target = 0
        self.backup_starttime = 0
        self.looking_for_object = 0
        self.route_length = 0
        self.route_reported = False
        self.obstacle_detected = False
        self.cmd = 0
        self.reply = 0

    def gripper_open(self):
        if open_gripper():
            self.gripper = OPENED
        pass

    def gripper_close(self):
        if close_gripper():
            self.gripper = CLOSED
        pass

#---------------------------------------------------------------------------------------------------------
# pre allocate robot instance as global
#---------------------------------------------------------------------------------------------------------
robot = Robot( loc=ID, dst=ID, _id=ID)


#---------------------------------------------------------------------------------------------------------
# this function will be based on camera on actual robot
#---------------------------------------------------------------------------------------------------------
def calc_heading(_robot=Robot()):

    #angle error in pixels
    # x = position of tag in camera view, 320 wide, 160 is center
    if HIL_Simulation:
        basic.pause(100)
        return 0
        
    angle_err = 160-x
    
    return angle_err


#---------------------------------------------------------------------------------------------------------
# this function will be based on camera on actual robot
#---------------------------------------------------------------------------------------------------------
def calc_distance(_robot=Robot()):
    '''
    if not _robot.state == DRIVEBACKWARD:
        dist_err = Math.sqrt(((_robot.position[0] - _robot.dest[0]))**2 + ((_robot.position[1] - _robot.dest[1]))**2)
    else:
        dist_err = Math.sqrt(((_robot.position[0] - _robot.startposition[0]))**2 + ((_robot.position[1] - _robot.startposition[1]))**2)
    '''
    tagSize = tagSizes[_robot.target]
    tagID = _robot.target
    
    if HIL_Simulation:
        if not _robot.state == DRIVEBACKWARD:
            distance_measured = _robot.distance_to_target - _robot.speed # drive towards target
        else:
            distance_measured = _robot.distance_to_target + abs(_robot.speed) # drive away from target
    else :
        distance_measured = tagSize - h  # height of tag - height of tag measured
        #serial.write_line('h='+h+'tagsize:'+tagSize) # soort timeout invoeren
    
    distance_measured = max(distance_measured,0) # distance cannot be negative

    return distance_measured
    

#---------------------------------------------------------------------------------------------------------
# look for target and aim robot towards it, different in offline Simulation vs actual robot
#---------------------------------------------------------------------------------------------------------
def find_target(_robot=Robot()):
    # We are driving towards a location
    if _robot.targettype == LOCATION_t:
        _robot.target = _robot.destination
    # We are looking for and driving to an object
    if _robot.targettype == OBJECT_t:
        # start looking for the object
        _robot.target = obj[_robot.objecttype] # obj location nr will become new target

    _robot.steering_only = True
    # set preffered turning direction?
    # actuate drive and steering

    _dist = move_robot(_robot)
    return _dist

#---------------------------------------------------------------------------------------------------------
#function to search target
#---------------------------------------------------------------------------------------------------------
def pulse(_dir=LEFT,_robot=Robot()):
    #pulsing behaviour
    pulse_duration = 700  # Duration of one complete pulse cycle
    active_ratio = 0.2  # Ratio of time the robot is actively turning within a pulse
    base_speed = 15
    speed_multiplier = 1.25

    if robot.state == FINDOBJECT:
        speed_multiplier = 1.2
        if robot.looking_for_object == 0:
            _robot.looking_for_object = input.running_time()
        else:
            time_looking = input.running_time() - _robot.looking_for_object
            if time_looking < 2000:
                 _dir = LEFT
            else: _dir = RIGHT




    #if start_searching == 0: # als
    #    start_searching = input.running_time()
    
    #calculate searching time
    #searching_time = input.running_time() - start_searching
    # Calculate the phase of the pulse (0 to 1)
    pulse_phase = (input.running_time() % pulse_duration) / pulse_duration

    #calculate if we are in the active part of the pulse
    is_active_pulse = pulse_phase < active_ratio

    if is_active_pulse:
        # During active pulse: turn with increased speed
        turn_speed = base_speed * speed_multiplier
    else:
        # During inactive pulse: stand still
        turn_speed = base_speed

    if _dir == LEFT: turn_speed = -turn_speed

    return turn_speed


    #if payload:
    #    speed_left *= 1.15
    #    speed_right *= 1.15



    #if payload:
    #    speed_left *= 1.15
    #    speed_right *= 1.15

#---------------------------------------------------------------------------------------------------------
# actual motion control, aim for target, with or without speed
#---------------------------------------------------------------------------------------------------------
def move_robot(_robot=Robot()):

    #check if robot is allowed to move
    dist_err = 999 # default distance error
    if not _robot.run_enable:
            _robot.speed = 0
            _robot.yaw_speed = 0

    #check if target is in sight
    if (_robot.target in tagsInView) or HIL_Simulation or (_robot.target in buffer_tagsInView) :
            _robot.target_inview = True
                                              
    else:
        # add timeout before target is no longer in view???
        _robot.target_inview = False
        _robot.target_locked = False


    #if target is not in view turn around to look for target, use pulsed turning to speed up process
    if not _robot.target_inview:
        _robot.speed = 0
        _robot.yaw_speed = pulse(_robot.target_direction,_robot)
    # if target is in view
    else:
        # heading / steering control
        angle_err = calc_heading(_robot) # how many pixels are we off centre
        dist_err = calc_distance(_robot) # how many pixels are we away from target
        _robot.distance_to_target = dist_err # store on instance of robot, for logistical and rouet handling

        # heading control, simple P-control
        Kp = 1.0
        _robot.yaw_speed = -angle_err * Kp # simple P-control of heading angle
        if _robot.target_locked:
            _robot.yaw_speed = max(-MAXYAWSPEED, min(_robot.yaw_speed, MAXYAWSPEED))
        else:
            _robot.yaw_speed = max(-MAXYAWSPEED, min(_robot.yaw_speed, MAXYAWSPEED))

        # speed control, start driving when target is almost straight ahead of robot
        if abs(angle_err) < 20:
            _robot.yaw_speed = 0
            _robot.target_locked = True
            # speed control, simple P-control
            Kp = 2.0
            _robot.speed = min(dist_err * Kp, MAXSPEED)
            # drive slow when approaching an object
            if _robot.state == GRABOBJECT: _robot.speed = min(MINSPEED, _robot.speed * Kp)
            
        else:
            _robot.target_locked = False

        # steering only when aiming for target
        if _robot.steering_only:
            _robot.speed = 0

        # if obstacle detected stop robot
        if CollisionDetection and collision_detected(_robot):
            _robot.speed = 0
            _robot.yaw_speed = 0

        # adjust turning speed depening on driving or turning when standing still
        if _robot.speed > 0 :
            # while driving reduce maximum turning speed, for smoother driving
            _robot.yaw_speed = max(-MINYAWSPEED/4, min(_robot.yaw_speed, MINYAWSPEED/4))

        else:
            # turn while standing still, use minimum turning speed
            if  _robot.yaw_speed > 0:
                if _robot.yaw_speed < MAXYAWSPEED: _robot.yaw_speed = MAXYAWSPEED
            if  _robot.yaw_speed < 0:
                if _robot.yaw_speed > -MAXYAWSPEED: _robot.yaw_speed = -MAXYAWSPEED

    actuate_motors()
    return dist_err


#---------------------------------------------------------------------------------------------------------
# actively stop robot
#---------------------------------------------------------------------------------------------------------
def stop_robot(_robot=Robot()):
    _robot.speed = 0
    _robot.yaw_speed = 0
    actuate_motors()

#---------------------------------------------------------------------------------------------------------
# drive backward to clear storage or target areas
#---------------------------------------------------------------------------------------------------------
def reverse_robot(_robot=Robot()):
    dist_err = calc_distance(_robot)
    # drive straight backward, without steering
    _robot.yaw_speed = 0
    # have we driven far enough backward? distance is trigger, could also be timed reverse driving
    if (input.running_time() - _robot.backup_starttime) < 750: # 0.75 seconds
        _robot.looking_for_object = 0
    #if dist_err < _robot.backup_distance:
        _robot.speed = BACKUPSPEED
        _finished = False
    else:
        _finished = True
        _robot.speed = 0
    actuate_motors()
    return _finished


#---------------------------------------------------------------------------------------------------------
# run robot main code (similar to real-time implementation)
#---------------------------------------------------------------------------------------------------------
def do_robot(_robot=Robot()):
    # statemachine
    if _robot.state == IDLE:
        _robot.run_enable = False
        _robot.speed = 0
        _robot.yaw_speed = 0
        _robot.cmd = POSITION_UPDATE
        stop_robot(_robot)
        if _robot.active:
            # we are allowed to drive
            _robot.run_enable = True
            # request next destination, if we cary an object we are going to an output stream area
            result = next_destination( _robot.destination, _robot.objecttype)
            _robot.destination = result[0]
            if _robot.location != waitingarea[3]:
                _robot.target_direction = result[1]
            _robot.target = _robot.destination
            # if we are in the waiting area, we do not need to back up, otherwise, backup before driving
            if HIL_Simulation:
                _robot.distance_to_target = 90
                _robot.route_length = _robot.distance_to_target
            # maybe  try to claim the location here. With QR code another robot might be in the way of seeing the QR code
            # when the robot has reased its starting position (claim released) the QR code should be visible
            if _robot.location in waitingarea:
                _robot.backup_distance = 0
                _robot.state = FINDTARGET  # directly start aiming for next destination
                #serial.write_line("state find target")
                showText("FINDTARGET: " + str(_robot.target) + ' ' + direction_names[_robot.target_direction])
                # for HIL_Simulation only
                if HIL_Simulation:
                    _robot.distance_to_target = 90
                    _robot.route_length = _robot.distance_to_target

            else:
                # backup distance
                _robot.backup_distance = BACKUPDISTANCE
                #serial.write_line("state drive backwards")
                showText("DRIVEBACKWARD")
                _robot.state = DRIVEBACKWARD  # drive backwards first
                _robot.backup_starttime = get_time() # input.running_time()
                # if wa are at target area, weh open gripper and drive backward to leave object there
                if _robot.location in target[1:numtargets+1]:
                    _robot.gripper_open()
                    _robot.objecttype = None # we leave the object here
                    pass
                #elif _robot.location in storagearea:
                # randomly assign new object to location where last object was taken from
                #    nxt_object = randint(1, numobjects+1)
                #    waste_object[_robot.objecttype].color = colorlist[nxt_object]  # we leave the object here
                #    waste_object[_robot.objecttype].objecttype = nxt_object
                #    pass
                else:
                    # all other areas we drive backward with grippers closed
                    _robot.gripper_close()
                    pass
                # for HIL_Simulation only
                if HIL_Simulation: _robot.distance_to_target = 0
        pass
    elif _robot.state == DRIVEBACKWARD:
        # drive backwards
        if reverse_robot(_robot):
            _robot.state = FINDTARGET
            showText("FINDTARGET: " + str(_robot.target) + ' ' + direction_names[_robot.target_direction])
            _robot.gripper_close()
            # for HIL_Simulation only
            if HIL_Simulation:
                _robot.distance_to_target = 90
                _robot.route_length = _robot.distance_to_target
        pass
    elif _robot.state == FINDTARGET:
        # find target by rotating robot, but do not drive
        _proceed = False
        if FirstClaimDestination:
             if UseFleetmanager:
                 _robot.cmd = CLAIM_DESTINATION
                 if _robot.reply == OK:
                     _robot.reply = 0 # reset for next command
                     _proceed = True
                     _robot.cmd = STATUS_UPDATE
                 pass
             else:
                if claim_destination(_robot.location, _robot.destination, _robot.id)[1] == _robot.id:
                    _proceed = True
        else:
            _proceed = True

        if _proceed:
            _robot.targettype = LOCATION_t
            _dist_to_travel = find_target(_robot)
            # when we have found the target, we are going to drive towards it
            if _robot.target_locked:
                # we have found the target, re-check if it is clear to drive there
                #if CollisionDetection or claim_destination(_robot.location, _robot.destination, _robot.id)[1] == _robot.id:
                if not FirstClaimDestination:
                    _proceed = False
                    if CollisionDetection:
                        _proceed = True
                    elif UseFleetmanager:
                        _robot.cmd = CLAIM_DESTINATION
                        if _robot.reply == OK:
                            _robot.reply = 0 # reset for next command
                            _proceed = True
                            _robot.cmd = STATUS_UPDATE
                        pass
                    elif claim_destination(_robot.location, _robot.destination, _robot.id)[1] == _robot.id:
                            _proceed = True
                if _proceed:
                    showText("GOTOTARGET: " + str(_robot.target))
                    _robot.state = GOTOTARGET
                    _robot.route_reported = False
                    _robot.route_length = _dist_to_travel
                    _robot.cmd = STATUS_UPDATE
        pass
    elif _robot.state == GOTOTARGET:
        # make sure we keep claim on target area
        _robot.steering_only = False
        _dist_to_travel = move_robot(_robot)
        _proceed = False
        _robot.cmd = STATUS_UPDATE
        # report route haflway, only onetime reporting
        if _dist_to_travel < _robot.route_length/2 and not _robot.route_reported:
            if UseFleetmanager:
                _robot.route_reported = True
                pass # do not report release position
                '''
                _robot.cmd = RELEASE_POSITION
                if _robot.reply == ACK:
                    _robot.reply = 0 # reset for next command
                    _robot.route_reported = True
                    _robot.cmd = STATUS_UPDATE
                pass
                '''
            else:
                release_position(_robot.location)
                _robot.route_reported = True
        _robot.distance_to_target = _dist_to_travel
        # if we approach storage, open gripper before reaching destination
        if _robot.destination in storagearea:
            if _dist_to_travel == 0:
                _robot.state = FINDOBJECT
                # assign an object to look for
                if RandomDestination:
                    #_robot.objecttype = randint(1, numobjects) # a random object is assigned to robot
                    # better would be to grab an object and see what we picked
                    yield_(20) # yield an wait for an object to appear in view
                    showText("Detecting object: ")
                    while _robot.objecttype is None:
                        _robot.objecttype = find_random_object()
                        yield_(20) # yield an wait for an object to appear in view
                    showText("Object found: ")
                else:
                    _robot.objecttype = _robot.id  # robot ID determines object types to be retrieved
                # switch state and start looking for object
                showText("FINDOBJECT: " + str(_robot.objecttype))
                pass
        elif _dist_to_travel <= 0+5:
            # target reached, release route
            _proceed = False
            if UseFleetmanager:
                _robot.cmd = RELEASE_ROUTE
                if DEBUG: serial.write_line("release route gototarget")
                if DEBUG: serial.write_line("reply: " + str(_robot.reply))
                if _robot.reply == ACK:
                    _robot.reply = 0 # reset for next command
                    _proceed = True
                    _robot.cmd = STATUS_UPDATE
                pass
            else:
                release_route(_robot.location, _robot.destination)
                _proceed = True
            # update position
            if _proceed:
                _robot.location = _robot.destination
                _robot.state = IDLE
                _robot.cmd = STATUS_UPDATE
                showText("IDLE")
        pass
    elif _robot.state == FINDOBJECT:
        # Finding object could be done by sweeping left and reight until object is found
        _robot.steering_only = True
        # if we approcah storage, open gripper before reaching destination
        _robot.targettype = OBJECT_t
        find_target(_robot)
        if _robot.target_locked:
            _robot.steering_only = False
                                                     
            _robot.gripper_open()
            _robot.state = GRABOBJECT
            showText("GRABOBJECT: " + str(_robot.objecttype))
            
    elif _robot.state == GRABOBJECT:
        _dist_to_travel = move_robot(_robot)
        if _dist_to_travel <= OBJECT_THRESHOLD+5:
                                                      
            _robot.gripper_close()
            _proceed = False
            # target reached, release route
            if UseFleetmanager:
                _robot.cmd = RELEASE_ROUTE
                if DEBUG: serial.write_line("release route grabobject")
                if _robot.reply == ACK:
                    _robot.reply = 0 # reset for next command
                    _proceed = True
                    _robot.cmd = STATUS_UPDATE
                pass
            else:
                release_route(_robot.location, _robot.destination)
                _proceed = True
            # update position
            if _proceed:
                _robot.location = _robot.destination
                # switch to idle to start next route
                _robot.state = IDLE
                showText("IDLE")
        pass


#---------------------------------------------------------------------------------------------------------
# collision detection, can be based on camera or sensor on robot
#---------------------------------------------------------------------------------------------------------
def collision_detected(_robot=Robot()):
    #
    # Needs implementing on robot
    #
    _robot.obstacle_detected = False
    obstacle_detected = False
    return obstacle_detected



#---------------------------------------------------------------------------------------------------------
#
#  ROUTE HANDLING BELOW
#
#---------------------------------------------------------------------------------------------------------

#---------------------------------------------------------------------------------------------------------
# locations id's
#---------------------------------------------------------------------------------------------------------

# ids of waiting areas
waitingarea = [0]
for i in range(numwaitingareas):
    waitingarea.append(i+1)
# ids of storage areas
storagearea = [0]
for i in range(numwaitingareas, numwaitingareas+numstorage):
    storagearea.append(i+1)
# ids of target areas
target = [0]
for i in range(numwaitingareas+numstorage, numwaitingareas+numstorage+numtargets):
    target.append(i+1)
# ids of objects
obj = [0]
for i in range(numwaitingareas+numstorage+numtargets, numwaitingareas+numstorage+numtargets+numobjects):
    obj.append(i+1)

#occupation list of all locations
#occupying = [0]  * (numlocations +1) -> does not work in Makecode
# for compatibility reasons wit MakeCode on robot, do it like this
occupying = [0]
for i in range(numlocations +1):
    occupying.append(0)

locationnames = ['None']
for i in range(1,numwaitingareas+1):
    locationnames.append(str('waitingarea ') + str(i))
locationnames.append(str('storage 1'))
for i in range(1,numobjects+1):
    locationnames.append(str('targetarea ') + str(i))
for i in range(numobjects+1):
    locationnames.append(str('object ') + str(i+1))

#---------------------------------------------------------------------------------------------------------
# instances of routes, using location id's
#---------------------------------------------------------------------------------------------------------
routes = [(0,0)]
for i in range(50):
    routes.append((0,0))

# route automatic define from location, to location, use location numbers instead of coordinates for easier update
for i in range(numwaitingareas):
    routes[i] = (waitingarea[1+i], storagearea[1])
offset = numwaitingareas
for i in range(numobjects):
    routes[i+offset] = (storagearea[1], target[1+i])
offset += i+1
for i in range(numobjects):
    if i < numobjects/2:
        routes[i+offset] = (target[i+1], waitingarea[1])
    else:
        routes[i + offset] = (target[i+1], waitingarea[len(waitingarea)-1]) # waitingarea[-1] is not compatibel with Makecode
offset += i+1
idx = 0
for i in range(1, numwaitingareas):
    if i < numwaitingareas/2:
        routes[offset+i-1] = (waitingarea[i], waitingarea[i+1]) # shift from left to middle
        idx = i
    else:
        routes[offset+i-1] = (waitingarea[len(waitingarea)-1]-i+idx+1, waitingarea[len(waitingarea)-1]-i+idx) # shift from right to middle


# potentially crossing route combinations for additional clearance check
# crossing routes:
# - all routes from storage to targets in combination with all targets to waitingarea
# - combination have to be all on left side or right side, right and left never cross (for now)
crossing_routes_ids = [((0,0),(0,0))]
crossing_routes = [(0,0)]
for i in range(len(routes)):
    if routes[i][0] == storagearea[1]:
        for j in range(len(routes)):
            if routes[j][1] == waitingarea[1]:
                if routes[j][0]<routes[i][1]:
                    if routes[i][1] in target[1:int(numtargets/2)+1]:
                        crossing_routes_ids.append((routes[i],routes[j]))
                        crossing_routes.append((i, j))
            if routes[j][1] == waitingarea[len(waitingarea)-1]:
                if routes[j][0] < routes[i][1]:
                    if routes[i][1] in target[int(numtargets/2)+1:numtargets+1]:
                        crossing_routes_ids.append((routes[i], routes[j]))
                        crossing_routes.append((i, j))

active_routes = [False]
for i in range(len(routes) +1):
    active_routes.append(False)
crossed_routes = [False]
for i in range(len(routes) +1):
    crossed_routes.append(False)


#---------------------------------------------------------------------------------------------------------
# check what next destination will be, this is the route scheduler, might be local or remote implemented
#---------------------------------------------------------------------------------------------------------
def next_destination(_location, objecttype = None):
    nxt_dest = _location # if there is no new location found, return current position as next destination
    target_dir = AHEAD
    # if we pick up an object, object nr tels us where to go
    if (_location in storagearea) and (objecttype is not None):
        # were need the collected object go to?
        nxt_dest = target[objecttype]
        # is target location left or rigth of us?
        if nxt_dest in target[1:int(numtargets/2)+1]:
            target_dir = LEFT
        elif nxt_dest in target[int(numtargets/2)+1:numtargets+1]:
            target_dir = RIGHT
    # if we come from a target to the left, goto left most waiting area
    elif _location in target[1:int(numtargets/2)+1]:
        nxt_dest = waitingarea[1]
        target_dir = LEFT
    # if we come from a taget to the right, goto right most waiting area
    elif _location in target[int(numtargets/2)+1:numtargets+1]:
        nxt_dest = waitingarea[len(waitingarea)-1]
        target_dir = RIGHT
    # if we are in the waiting area, move toward the center wating area, before going back to the storage
    elif _location in waitingarea:
        # from waiting to storeage, unless we have to shift in the waiting area
        nxt_dest = storagearea[1]
        target_dir = LEFT
        for i in range(1,int(numwaitingareas/2)+1):
            # from left to middle
            if _location == waitingarea[i]:
                nxt_dest = waitingarea[i+1]
                target_dir = LEFT
            # from right to middle
            if _location == waitingarea[len(waitingarea)-1]-i+1:
                nxt_dest = waitingarea[len(waitingarea)-1]-i
                target_dir = RIGHT
    return nxt_dest, target_dir


#---------------------------------------------------------------------------------------------------------
# try to claim destination, if succesfull, returns True and own ID, if unsuccesfull returns ID that occupies the destination
#---------------------------------------------------------------------------------------------------------
def claim_destination(_loc=0, _dest=0, _id=0):
    claimed = 0
    occupied = occupying[_dest]
    if occupied == 0 and _id > 0:
        occupying[_dest] = _id
        claimed_id = _id
        claimed = 1
    else:
        claimed_id = occupying[_dest]
    if claimed_id == _id > 0:
        if check_routes(_loc, _dest, _id):
            occupying[_dest] = _id
            #release_position(_loc) # this assumes that you start driving imediatly after claim of location
        else:
            claimed_id = 0
    return claimed, claimed_id


#---------------------------------------------------------------------------------------------------------
# check if a destination is occupied, return ID that occupies destination
#---------------------------------------------------------------------------------------------------------
def check_occupied(_dst):
    claimed, occupied = claim_destination(_dest=_dst)
    return occupied


#---------------------------------------------------------------------------------------------------------
# release location when you have left it
#---------------------------------------------------------------------------------------------------------
def release_position(_location):
    occupying[_location] = 0
    return True


#---------------------------------------------------------------------------------------------------------
# release a route when we finished driving it
#---------------------------------------------------------------------------------------------------------
def release_route(_loc,_dest):
    _route = (_loc,_dest)
    #if _route in [r for r in routes]: # not compatible with makecode
    _found = False
    for i in range(len(routes)):
        if _route == routes[i]: _found = True
    if _found:
        # get route nr
        _route_nr = routes.index(_route)
        active_routes[_route_nr] = False
    # check if route is actually released before returning -> need for remote implementation
    while active_routes[_route_nr]:
        # keep on trying to release the route
        pass



#---------------------------------------------------------------------------------------------------------
# check if routes are crossing, claim route if crossing path is free
#---------------------------------------------------------------------------------------------------------
def check_routes(_loc, _dest, _id):
    if CollisionDetection: return True
    proceed = True
    # check start location of route
    # is route valid and in route list]
    _route = (_loc,_dest)
    #if _route in [r for r in routes]: # not compatible with makecode
    _found = False
    for i in range(len(routes)):
        if _route == routes[i] and routes[i] != (0, 0):
            _found = True
    if _found:
        # get route nr
        _route_nr = routes.index(_route)
        #_r1 = _route_nr in [r[0] for r in crossing_routes]  # not compatible with makecode
        #_r2 = _route_nr in [r[1] for r in crossing_routes] # not compatible with makecode
        _r1 = False
        _r2 = False
        for i in range(1,len(crossing_routes)):
            if _route_nr == crossing_routes[i][0]: _r1 = True
            if _route_nr == crossing_routes[i][1]: _r2 = True
        if _r1 or _r2:
            proceed = False
            for i in range(1,len(crossing_routes)):
                if _route_nr == crossing_routes[i][0]:
                    if not active_routes[crossing_routes[i][1]]:
                        active_routes[_route_nr] = True
                        crossed_routes[_route_nr] = False
                        proceed = True
                    else:
                        crossed_routes[_route_nr] = True
                if _route_nr == crossing_routes[i][1]:
                    if not active_routes[crossing_routes[i][0]]:
                        active_routes[_route_nr] = True
                        crossed_routes[_route_nr] = False
                        proceed = True
                    else:
                        crossed_routes[_route_nr] = True

    return proceed



#---------------------------------------------------------------------------------------------------------
# buttons and switches
#---------------------------------------------------------------------------------------------------------

def on_button_pressed_a():
    global stopping, robot, DEBUG, Full_auto
    stopping = False
    robot.active = True
    Full_auto = True
    if DEBUG: serial.write_line('button pressed')
    showText("ACTIVE")
    pass
input.on_button_pressed(Button.A, on_button_pressed_a)

'''
def on_button_pressed_b():
    global stopping
    stopping = True
    if DEBUG: serial.write_line('reset')
    pass
input.on_button_pressed(Button.B, on_button_pressed_b)
'''

def leading_zeros(_num, _digits):
    _str = '0'
    _num = Math.round(_num)
    if _num < 0: _num += 256
    if _digits == 3:
        if _num < 10 :
            _str = '00' + str(_num)
        elif _num < 100:
            _str = '0' + str(_num)
        elif _num < 1000:
            _str = str(_num)
    if _digits == 2:
            if _num < 10 :
                _str = '0' + str(_num)
            elif _num < 100:
                _str = str(_num)
    return _str

latsprint = input.running_time()

def print_data():
    global robot, DEBUG, latsprint
    if DEBUG:
        #serial.write_value("# ", control.device_serial_number())
        t = input.running_time()
        serial.write_value("t : ",t/1000)
        serial.write_value("dt: ", (t-latsprint)/1000)
        latsprint = t
        serial.write_line("speed "+robot.speed)
        serial.write_line("yaw speed "+robot.yaw_speed)
        serial.write_line("distance target"+robot.distance_to_target)
        serial.write_value("state", robot.state)
        serial.write_line(state_names[robot.state])
        serial.write_line(str("target " ) + str(robot.target) + str("  ") + locationnames[robot.target])
        serial.write_value("distance", robot.distance_to_target)
        serial.write_value("speed", robot.speed)
        _cmd = 0
        _chk = (robot.id + _cmd + robot.state + robot.location + robot.destination + robot.target + robot.distance_to_target) % 8
        _str = (leading_zeros(robot.id,2) + str(_cmd) + str(robot.state) +
                    leading_zeros(robot.location,2) + leading_zeros(robot.destination,2)  + leading_zeros(robot.target,2) +
                    leading_zeros(robot.distance_to_target,3) + leading_zeros(robot.speed,3) + str(_chk) + "\r\n")
        serial.write_line(_str)
        serial.write_line('--')
    pass
'''
loops.every_interval(500, print_data)
'''

'''
def on_serial_received():
    global stopping, robot
    data = serial.read_line()
    if data == 'a':
        stopping = False
        robot.active = True
        serial.write_line('remotley started')
    if data == 'b':
        stopping = True
        serial.write_line('remotely reset')
    pass
    basic.pause(100) # yield for other processes
serial.on_data_received(serial.delimiters(Delimiters.NEW_LINE), on_serial_received)
'''

def on_radio_received(receivedString):
    global robot, stopping, DEBUG, RadioTxPending, WaitingForReply
    # cmd reply
    # 0123456
    # 1,1,1,1
    # optimised without string split, and earlier reject of message
    # radio_rx_data = receivedString
    if receivedString[0] == str(robot.id):
        #if DEBUG: serial.write_line("this is for us")
        if DEBUG: serial.write_line("rx:" + receivedString)
        #data = radio_rx_data.split(',')
        #for d in range (len(data)):
        #    if DEBUG: serial.write_line(data[d])
        #_id = int(data[0])
        #if _id == robot.id:
        # this message is send for us
        #_cmd = int(data[1])
        _cmd = int(receivedString[2])
        if _cmd == 0:
            # send status report or pending command
            #robot.cmd = STATUS_UPDATE
            pass
        elif _cmd == 1:
            # NOK/OK claim destination reply
            # data[2] = NOK/OK, data[3] = claimed id
            #if int(data[3]) == robot.id:
            if int(receivedString[6]) == robot.id:
                robot.reply = OK
                robot.cmd = 0 # no more commands
            pass
        elif _cmd == 2:
            # ACK releaseposition
            #if int(data[2]) == 2:
            if int(receivedString[4]) == 2:
                robot.reply = ACK
                robot.cmd = 0
            pass
        elif _cmd == 3:
            # ACK releaseroute
            #if int(data[2]) == 2:
            if int(receivedString[4]) == 2:
                robot.reply = ACK
                robot.cmd = 0
            pass
        elif _cmd == 4:
            # START driving
            stopping = False
            robot.active = True
            robot.backup_starttime = get_time()
            pass
        elif _cmd == 5:
            # STOP driving
            stopping = True
            robot.active = False
            pass

        if DEBUG: serial.write_line("reply: " + str(robot.reply))
        RadioTxPending = False
        WaitingForReply = False
        #radio_transmit(robot)
        #if DEBUG: serial.write_line("sending radio")
        yield_(5) # yield for other processes
    pass
radio.on_received_string(on_radio_received)


def radio_transmit(_robot=Robot()):
    _cmd = _robot.cmd
    _chk = (_robot.id + _cmd + _robot.state + _robot.location + _robot.destination + _robot.target + _robot.distance_to_target + _robot.speed) % 8
    _str = (leading_zeros(robot.id,2) + str(_cmd) + str(robot.state) +
                leading_zeros(robot.location,2) + leading_zeros(robot.destination,2)  + leading_zeros(robot.target,2) +
                leading_zeros(robot.distance_to_target,3) + leading_zeros(robot.speed,3) + str(_chk) + "\r\n")
    # send 2 times to make sure the message came trough
    radio.send_string(_str)
    yield_(100)
    radio.send_string(_str)

    #radio.send_string("id:" + str(robot.id))
    #radio.send_string(",state:" + str(robot.state))
    #radio.send_string(str(",loc:" ) + str(robot.location))
    #radio.send_string(str(",des:" ) + str(robot.destination))
    #radio.send_string(str(",tar:" ) + str(robot.target))
    #radio.send_string("\r\n" )
    #radio.send_string(str("target " ) + str(robot.target) + str("  ") + locationnames[robot.target] + '\r\n')
#loops.every_interval(250, on_every_interval)

def yield_(_t=50):
    basic.pause(_t)


#---------------------------------------------------------------------------------------------------------
# actual main loop
#---------------------------------------------------------------------------------------------------------
stopping = False
radio.set_group(1)
close_gripper()

RadioTxTime = input.running_time()
WaitingForReply = False

# function for getting data from huskylens and function for frame buffer
#---------------------------------------------------------------------------------------------------------
def get_frame_data(inview):
    global tagsInView, obstacle
    for tag in range(inview):
        tagsInView.append(huskylens.readBox_ss(tag + 1, Content3.ID))
    if huskylens.is_appear(robot.target, HUSKYLENSResultType_t.HUSKYLENS_RESULT_BLOCK):
    #volgensmij zit er een bug in robot.target waarbij die soms de verkeerde qr code uitleest als er meerdere  in beeld zijn
    #de waarde robot.target is een nummer van target die die zoekt
    #bij de functie husylens.reade_box() moet de volgorde van tag ingevoerd worden die hij ziet(volgensmij)
        #serial.write_line("target "+str(robot.target))
        #serial.write_line(str(tag)+' inview')
        x = huskylens.reade_box(robot.target, Content1.X_CENTER)
        y = huskylens.reade_box(robot.target, Content1.Y_CENTER)
        w = huskylens.reade_box(robot.target, Content1.WIDTH)
        h = huskylens.reade_box(robot.target, Content1.HEIGHT)
       
    else:
        x,y,w,h = -1,-1,-1,-1

    return x,y,w,h
    
def data_buffer(x,y,w,h,tagsInView,reset):
        global frame_count, buffer_xywh, buffer_tagsInView
        # check if buffer needs to be reset
        if frame_count > 3:
            reset = True
        # replace buffer data
        if reset:
            frame_count = 0
            buffer_xywh = [x,y,w,h]
            buffer_tagsInView = tagsInView
        #count frames
        else:
            frame_count = frame_count + 1
        # return buffer data
        return buffer_xywh,buffer_tagsInView

def find_random_object():
    global tagsInView, obj
    #return 10
    if len(tagsInView) > 0:
        #for tg in range(len(tagsInView)):
        #    serial.write_line('tag_id: ' + str(tagsInView[tg]))
        for obj_tag in range(1, len(obj)):
            if obj[obj_tag] in tagsInView:
                return obj_tag
    return None    

#---------------------------------------------------------------------------------------------------------
# actual main loop
#---------------------------------------------------------------------------------------------------------
stopping = False
radio.set_group(1)
close_gripper()
obstacle = False
obstacle_count = 0


def on_forever():

    global stopping, HIL_Simulation, robot, newdata, Full_auto, obstacle
    if Full_auto:
        robot.active = True
        stopping = False
        if obstacle:
            stopping = True
    if not stopping and robot.active:
        if (robot.cmd == POSITION_UPDATE):
            robot.cmd = STATUS_UPDATE
            radio_transmit(robot) # send message
            robot.cmd = NO_CMD
            robot.active = False # wait for cleareance by fleet manager
            stop_robot(robot)
            stopping = True
        if newdata:
            newdata = False
            do_robot(robot)

    if HIL_Simulation: basic.pause(50)
    #print_data() -> background loop
    if stopping:
        robot.active = False
        stop_robot(robot)
        #robot.location = 1
        #robot.destination = 1
        #robot.target = 1
        #robot.objecttype = None
        #robot.state = IDLE
    pass
basic.forever(on_forever)

def on_in_background():
    global inview, tagsInView, x, y, w, h, newdata, obstacle, obstacle_count
    # approx 56 ms -> ~ 18 fps

    while 1:
        #get new frame data
        huskylens.request()
        inview = huskylens.get_box(HUSKYLENSResultType_t.HUSKYLENS_RESULT_BLOCK)
        tagsInView = [0]

        # check if any of the objects is another robot (collision detection)
        if obstacle:
            # we have seen an obstacle, keep a count how many frames we have seen this obstacle last before it disappered
            obstacle_count += 1
        if obstacle_count >=5:
            # if we have seen it more than 5 frames ago, we assume the object has disappeared
            obstacle = False
        if huskylens.is_appear(robot.obstacle, HUSKYLENSResultType_t.HUSKYLENS_RESULT_BLOCK):
            obstacle = True
            # as long as we have confirmed visual identification, reset the frame count since last we have seen the obstacle
            obstacle_count = 0
        
        # if there is a tag in view, refresh buffer.
        if inview > 0:
            x,y,w,h = get_frame_data(inview)
            data_buffer(x,y,w,h,tagsInView,True)

        # if there is no tag in view use latest buffer data
        else:
            box,tagsInView = data_buffer(x,y,w,h,tagsInView,False)
            x,y,w,h=box[0],box[1],box[2],box[3]

        newdata = True

                          
control.in_background(on_in_background)

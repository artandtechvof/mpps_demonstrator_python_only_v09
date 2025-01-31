let numwaitingareas: number;
let tagSizes: number[];
let i: number;
let CollisionDetection = false
let RandomDestination = true
//  random object or object based on own ID
let HIL_Simulation = false
//  run code on
let FirstClaimDestination = true
//  True: wait for destination area te be cleared.
//  False: turn towards destination even if it is occupied
let UseFleetmanager = false
let Full_auto = false
let ID = 4
let DEBUG = false
let UseHusky = true
let DisplayDelay = 200
function showText(txt: string) {
    // if not HIL_Simulation:
    if (UseHusky) {
        txt = "" + ID + " " + txt
        huskylens.clearOSD()
        huskylens.writeOSD(txt, 30, 30)
        if (HIL_Simulation) {
            basic.pause(DisplayDelay)
        }
        
    }
    
}

// #####################################
serial.redirectToUSB()
serial.writeString("I am alive")
if (!HIL_Simulation || UseHusky) {
    huskylens.initI2c()
    huskylens.initMode(protocolAlgorithm.ALGORITHM_TAG_RECOGNITION)
    showText("BOOTING")
}

basic.pause(1000)
// #####################################
//  number of robots
let numrobots = 1
//  number of 'plastic' object types
let numobjects = 4
//  should be an even number for now
let numtargets = numobjects
//  buffer zones for wating robots
//  1 robot = 1 waiting
//  2 robot = 1 waiting, 1 on storage location
//  3 robot = 3 waiting, 1 left, 1 right 1 central
//  4 robot = 5 waiting, 2 left, 2 right, 1 central
//  5 robot = 7 waiting, 3 left, 3 right, 1 central
//  6 robot = 9 waiting, 4 left, 4 right, 1 central
//  7 robot = 11 waiting, 5 left, 5 right, 1 central
//  8 robots = 13 waiting, 6 left, 6 right, 1 central
if (numrobots == 1) {
    numwaitingareas = 5
} else {
    //  adjust to demo
    numwaitingareas = Math.max(3, (numrobots - 1) * 2 - 1)
}

//  1 central waiting area offset to the front
let numstorage = 1
//  try increasing this in future scenarios, attack waste-pile from multiple locations
//  total number of locations where robots can go
let numlocations = numwaitingareas + numstorage + numtargets + numobjects
//  constants
let IDLE = 0
let DRIVEBACKWARD = 1
let FINDTARGET = 2
let GOTOTARGET = 3
let FINDOBJECT = 4
let GRABOBJECT = 5
let LOCATION_t = 0
//  robot is looking for location
let OBJECT_t = 1
//  robot is looking for object
let OPENED = 1
let CLOSED = 0
let AHEAD = 0
let LEFT = 1
let RIGHT = 2
let MAXSPEED = 22
let MINSPEED = 15
let MINYAWSPEED = 8
let MAXYAWSPEED = 14
let BACKUPSPEED = -20
let BACKUPDISTANCE = 20
let OBJECT_THRESHOLD = 0
//  minimum distance to object, before closing grippers
let state_names = ["IDLE", "DRIVEBACKWARD", "FINDTARGET", "GOTOTARGET", "FINDOBJECT", "GRABOBJECT"]
let direction_names = ["AHEAD", "LEFT", "RIGHT"]
let cmd_names = ["STATUS_UPDATE", "CLAIM_DESTINATION", "RELEASE_POSITION", "RELEASE_ROUTE", "ROBOT_START", "ROBOT_STOP"]
let rply_names = ["NOK", "OK", "ACK"]
//  commands
let NO_CMD = -1
let POSITION_UPDATE = 10
let STATUS_UPDATE = 0
let CLAIM_DESTINATION = 1
let RELEASE_POSITION = 2
let RELEASE_ROUTE = 3
let ROBOT_START = 4
let ROBOT_STOP = 5
let NOK = 0
let OK = 1
let ACK = 2
// initialize variables related to huskylens
let t_start = 0
let t_last = 0
let dt_AI = 0
let inview = 0
let x = 0
let y = 0
let w = 0
let h = 0
let newdata = false
let inview_count = 0
let RadioTxPending = false
let tagsInView = [0]
let frame_count = 0
let buffer_xywh = [-1, -1, -1, -1]
let buffer_tagsInView = [0]
let tagID = 1
//  tagIDs = [1 tm 6]
//  tuning per robot, mainly sotrage area
if (ID == 1) {
    tagSizes = [0, 88, 90, 90, 90, 88, 87, 88, 88, 88, 88, 55, 55, 55, 55]
}

//  5x waiting, 1 central, 4 targets, 4 objects
if (ID == 2) {
    tagSizes = [0, 88, 90, 90, 90, 83, 87, 88, 88, 88, 88, 55, 55, 55, 55]
}

//  5x waiting, 1 central, 4 targets, 4 objects
if (ID == 3) {
    tagSizes = [0, 88, 90, 90, 90, 86, 87, 88, 88, 88, 88, 55, 55, 55, 55]
}

//  5x waiting, 1 central, 4 targets, 4 objects
if (ID == 4) {
    tagSizes = [0, 88, 90, 90, 90, 86, 87, 88, 88, 88, 88, 55, 55, 55, 55]
}

//  5x waiting, 1 central, 4 targets, 4 objects
let tagSize = tagSizes[tagID]
let zeroes = [0]
while (i < tagSizes.length) {
    zeroes.push(0)
    i += 1
}
tagsInView = zeroes
// start_searching = 0
// ---------------------------------------------------------------------------------------------------------
//  helper function to open and close grippers, will be different on actual robot
// ---------------------------------------------------------------------------------------------------------
function open_gripper(): boolean {
    let opengripper = true
    maqueen.servoRun(maqueen.Servos.S1, 85)
    return opengripper
}

function close_gripper(): boolean {
    let closegripper = true
    maqueen.servoRun(maqueen.Servos.S1, 105)
    return closegripper
}

function actuate_motors(): boolean {
    if (HIL_Simulation) {
        return true
    }
    
    let left_speed = robot.speed + robot.yaw_speed
    let right_speed = robot.speed + -robot.yaw_speed
    // serial.write_line(str(left_speed)+" left speed")
    // serial.write_line(str(right_speed)+" right speed")
    //  set speed for motors here
    if (left_speed >= 0) {
        maqueen.motorRun(maqueen.Motors.M1, maqueen.Dir.CW, left_speed)
    } else {
        maqueen.motorRun(maqueen.Motors.M1, maqueen.Dir.CCW, -left_speed)
    }
    
    if (right_speed >= 0) {
        maqueen.motorRun(maqueen.Motors.M2, maqueen.Dir.CW, right_speed)
    } else {
        maqueen.motorRun(maqueen.Motors.M2, maqueen.Dir.CCW, -right_speed)
    }
    
    return true
}

function get_time(): number {
    return input.runningTime()
}

// ---------------------------------------------------------------------------------------------------------
//  base class of robot, can also be used on micro:Maqueen robot.
//  Robot() class, should be same for offline Simulation and actual physical demo robot
// ---------------------------------------------------------------------------------------------------------
class Robot {
    id: number
    location: number
    destination: number
    target: number
    targettype: number
    objecttype: number
    state: number
    gripper: number
    speed: number
    yaw_speed: number
    active: boolean
    run_enable: boolean
    steering_only: boolean
    target_direction: number
    target_inview: boolean
    target_locked: boolean
    backup_distance: number
    distance_to_target: number
    backup_starttime: number
    looking_for_object: number
    route_length: number
    route_reported: boolean
    obstacle_detected: boolean
    cmd: number
    reply: number
    constructor(loc: number = 1, dst: number = 1, _id: number = 1) {
        //  who are we
        this.id = _id
        //  ego location and destination nr
        this.location = loc
        this.destination = dst
        this.target = dst
        this.targettype = LOCATION_t
        this.objecttype = null
        //  states
        this.state = IDLE
        this.gripper = CLOSED
        this.speed = 0
        this.yaw_speed = 0
        //  params
        this.active = false
        this.run_enable = false
        this.steering_only = false
        //  allowed to change heading but not driving
        this.target_direction = LEFT
        this.target_inview = false
        //  target is in view of camera
        this.target_locked = false
        //  target is straight ahead
        this.backup_distance = 0
        this.distance_to_target = 0
        this.backup_starttime = 0
        this.looking_for_object = 0
        this.route_length = 0
        this.route_reported = false
        this.obstacle_detected = false
        this.cmd = 0
        this.reply = 0
    }
    
    public gripper_open() {
        if (open_gripper()) {
            this.gripper = OPENED
        }
        
        
    }
    
    public gripper_close() {
        if (close_gripper()) {
            this.gripper = CLOSED
        }
        
        
    }
    
}

// ---------------------------------------------------------------------------------------------------------
//  pre allocate robot instance as global
// ---------------------------------------------------------------------------------------------------------
let robot = new Robot(ID, ID, ID)
// ---------------------------------------------------------------------------------------------------------
//  this function will be based on camera on actual robot
// ---------------------------------------------------------------------------------------------------------
function calc_heading(_robot: Robot = new Robot()): number {
    // angle error in pixels
    //  x = position of tag in camera view, 320 wide, 160 is center
    if (HIL_Simulation) {
        basic.pause(100)
        return 0
    }
    
    let angle_err = 160 - x
    return angle_err
}

// ---------------------------------------------------------------------------------------------------------
//  this function will be based on camera on actual robot
// ---------------------------------------------------------------------------------------------------------
function calc_distance(_robot: Robot = new Robot()): number {
    let distance_measured: number;
    /** 
    if not _robot.state == DRIVEBACKWARD:
        dist_err = Math.sqrt(((_robot.position[0] - _robot.dest[0]))**2 + ((_robot.position[1] - _robot.dest[1]))**2)
    else:
        dist_err = Math.sqrt(((_robot.position[0] - _robot.startposition[0]))**2 + ((_robot.position[1] - _robot.startposition[1]))**2)
    
 */
    let tagSize = tagSizes[_robot.target]
    let tagID = _robot.target
    if (HIL_Simulation) {
        if (!(_robot.state == DRIVEBACKWARD)) {
            distance_measured = _robot.distance_to_target - _robot.speed
        } else {
            //  drive towards target
            distance_measured = _robot.distance_to_target + Math.abs(_robot.speed)
        }
        
    } else {
        //  drive away from target
        distance_measured = tagSize - h
    }
    
    //  height of tag - height of tag measured
    // serial.write_line('h='+h+'tagsize:'+tagSize) # soort timeout invoeren
    distance_measured = Math.max(distance_measured, 0)
    //  distance cannot be negative
    return distance_measured
}

// ---------------------------------------------------------------------------------------------------------
//  look for target and aim robot towards it, different in offline Simulation vs actual robot
// ---------------------------------------------------------------------------------------------------------
function find_target(_robot: Robot = new Robot()): number {
    //  We are driving towards a location
    if (_robot.targettype == LOCATION_t) {
        _robot.target = _robot.destination
    }
    
    //  We are looking for and driving to an object
    if (_robot.targettype == OBJECT_t) {
        //  start looking for the object
        _robot.target = obj[_robot.objecttype]
    }
    
    //  obj location nr will become new target
    _robot.steering_only = true
    //  set preffered turning direction?
    //  actuate drive and steering
    let _dist = move_robot(_robot)
    return _dist
}

// ---------------------------------------------------------------------------------------------------------
// function to search target
// ---------------------------------------------------------------------------------------------------------
function pulse(_dir: number = LEFT, _robot: Robot = new Robot()): number {
    let time_looking: number;
    let turn_speed: number;
    // pulsing behaviour
    let pulse_duration = 700
    //  Duration of one complete pulse cycle
    let active_ratio = 0.2
    //  Ratio of time the robot is actively turning within a pulse
    let base_speed = 15
    let speed_multiplier = 1.25
    if (robot.state == FINDOBJECT) {
        speed_multiplier = 1.2
        if (robot.looking_for_object == 0) {
            _robot.looking_for_object = input.runningTime()
        } else {
            time_looking = input.runningTime() - _robot.looking_for_object
            if (time_looking < 2000) {
                _dir = LEFT
            } else {
                _dir = RIGHT
            }
            
        }
        
    }
    
    // if start_searching == 0: # als
    //     start_searching = input.running_time()
    // calculate searching time
    // searching_time = input.running_time() - start_searching
    //  Calculate the phase of the pulse (0 to 1)
    let pulse_phase = input.runningTime() % pulse_duration / pulse_duration
    // calculate if we are in the active part of the pulse
    let is_active_pulse = pulse_phase < active_ratio
    if (is_active_pulse) {
        //  During active pulse: turn with increased speed
        turn_speed = base_speed * speed_multiplier
    } else {
        //  During inactive pulse: stand still
        turn_speed = base_speed
    }
    
    if (_dir == LEFT) {
        turn_speed = -turn_speed
    }
    
    return turn_speed
}

// if payload:
//     speed_left *= 1.15
//     speed_right *= 1.15
// if payload:
//     speed_left *= 1.15
//     speed_right *= 1.15
// ---------------------------------------------------------------------------------------------------------
//  actual motion control, aim for target, with or without speed
// ---------------------------------------------------------------------------------------------------------
function move_robot(_robot: Robot = new Robot()): number {
    let angle_err: number;
    let Kp: number;
    // check if robot is allowed to move
    let dist_err = 999
    //  default distance error
    if (!_robot.run_enable) {
        _robot.speed = 0
        _robot.yaw_speed = 0
    }
    
    // check if target is in sight
    if (tagsInView.indexOf(_robot.target) >= 0 || HIL_Simulation || buffer_tagsInView.indexOf(_robot.target) >= 0) {
        _robot.target_inview = true
    } else {
        //  add timeout before target is no longer in view???
        _robot.target_inview = false
        _robot.target_locked = false
    }
    
    // if target is not in view turn around to look for target, use pulsed turning to speed up process
    if (!_robot.target_inview) {
        _robot.speed = 0
        _robot.yaw_speed = pulse(_robot.target_direction, _robot)
    } else {
        //  if target is in view
        //  heading / steering control
        angle_err = calc_heading(_robot)
        //  how many pixels are we off centre
        dist_err = calc_distance(_robot)
        //  how many pixels are we away from target
        _robot.distance_to_target = dist_err
        //  store on instance of robot, for logistical and rouet handling
        //  heading control, simple P-control
        Kp = 1.0
        _robot.yaw_speed = -angle_err * Kp
        //  simple P-control of heading angle
        if (_robot.target_locked) {
            _robot.yaw_speed = Math.max(-MAXYAWSPEED, Math.min(_robot.yaw_speed, MAXYAWSPEED))
        } else {
            _robot.yaw_speed = Math.max(-MAXYAWSPEED, Math.min(_robot.yaw_speed, MAXYAWSPEED))
        }
        
        //  speed control, start driving when target is almost straight ahead of robot
        if (Math.abs(angle_err) < 20) {
            _robot.yaw_speed = 0
            _robot.target_locked = true
            //  speed control, simple P-control
            Kp = 2.0
            _robot.speed = Math.min(dist_err * Kp, MAXSPEED)
            //  drive slow when approaching an object
            if (_robot.state == GRABOBJECT) {
                _robot.speed = Math.min(MINSPEED, _robot.speed * Kp)
            }
            
        } else {
            _robot.target_locked = false
        }
        
        //  steering only when aiming for target
        if (_robot.steering_only) {
            _robot.speed = 0
        }
        
        //  if obstacle detected stop robot
        if (CollisionDetection && collision_detected(_robot)) {
            _robot.speed = 0
            _robot.yaw_speed = 0
        }
        
        //  adjust turning speed depening on driving or turning when standing still
        if (_robot.speed > 0) {
            //  while driving reduce maximum turning speed, for smoother driving
            _robot.yaw_speed = Math.max(-MINYAWSPEED / 4, Math.min(_robot.yaw_speed, MINYAWSPEED / 4))
        } else {
            //  turn while standing still, use minimum turning speed
            if (_robot.yaw_speed > 0) {
                if (_robot.yaw_speed < MAXYAWSPEED) {
                    _robot.yaw_speed = MAXYAWSPEED
                }
                
            }
            
            if (_robot.yaw_speed < 0) {
                if (_robot.yaw_speed > -MAXYAWSPEED) {
                    _robot.yaw_speed = -MAXYAWSPEED
                }
                
            }
            
        }
        
    }
    
    actuate_motors()
    return dist_err
}

// ---------------------------------------------------------------------------------------------------------
//  actively stop robot
// ---------------------------------------------------------------------------------------------------------
function stop_robot(_robot: Robot = new Robot()) {
    _robot.speed = 0
    _robot.yaw_speed = 0
    actuate_motors()
}

// ---------------------------------------------------------------------------------------------------------
//  drive backward to clear storage or target areas
// ---------------------------------------------------------------------------------------------------------
function reverse_robot(_robot: Robot = new Robot()): boolean {
    let _finished: boolean;
    let dist_err = calc_distance(_robot)
    //  drive straight backward, without steering
    _robot.yaw_speed = 0
    //  have we driven far enough backward? distance is trigger, could also be timed reverse driving
    if (input.runningTime() - _robot.backup_starttime < 750) {
        //  0.75 seconds
        _robot.looking_for_object = 0
        // if dist_err < _robot.backup_distance:
        _robot.speed = BACKUPSPEED
        _finished = false
    } else {
        _finished = true
        _robot.speed = 0
    }
    
    actuate_motors()
    return _finished
}

// ---------------------------------------------------------------------------------------------------------
//  run robot main code (similar to real-time implementation)
// ---------------------------------------------------------------------------------------------------------
function do_robot(_robot: Robot = new Robot()) {
    let result: number[];
    let _proceed: boolean;
    let _dist_to_travel: number;
    //  statemachine
    if (_robot.state == IDLE) {
        _robot.run_enable = false
        _robot.speed = 0
        _robot.yaw_speed = 0
        _robot.cmd = POSITION_UPDATE
        stop_robot(_robot)
        if (_robot.active) {
            //  we are allowed to drive
            _robot.run_enable = true
            //  request next destination, if we cary an object we are going to an output stream area
            result = next_destination(_robot.destination, _robot.objecttype)
            _robot.destination = result[0]
            if (_robot.location != waitingarea[3]) {
                _robot.target_direction = result[1]
            }
            
            _robot.target = _robot.destination
            //  if we are in the waiting area, we do not need to back up, otherwise, backup before driving
            if (HIL_Simulation) {
                _robot.distance_to_target = 90
                _robot.route_length = _robot.distance_to_target
            }
            
            //  maybe  try to claim the location here. With QR code another robot might be in the way of seeing the QR code
            //  when the robot has reased its starting position (claim released) the QR code should be visible
            if (waitingarea.indexOf(_robot.location) >= 0) {
                _robot.backup_distance = 0
                _robot.state = FINDTARGET
                //  directly start aiming for next destination
                // serial.write_line("state find target")
                showText("FINDTARGET: " + ("" + _robot.target) + " " + direction_names[_robot.target_direction])
                //  for HIL_Simulation only
                if (HIL_Simulation) {
                    _robot.distance_to_target = 90
                    _robot.route_length = _robot.distance_to_target
                }
                
            } else {
                //  backup distance
                _robot.backup_distance = BACKUPDISTANCE
                // serial.write_line("state drive backwards")
                showText("DRIVEBACKWARD")
                _robot.state = DRIVEBACKWARD
                //  drive backwards first
                _robot.backup_starttime = get_time()
                //  input.running_time()
                //  if wa are at target area, weh open gripper and drive backward to leave object there
                if (target.slice(1, numtargets + 1).indexOf(_robot.location) >= 0) {
                    _robot.gripper_open()
                    _robot.objecttype = null
                    //  we leave the object here
                    
                } else {
                    // elif _robot.location in storagearea:
                    //  randomly assign new object to location where last object was taken from
                    //     nxt_object = randint(1, numobjects+1)
                    //     waste_object[_robot.objecttype].color = colorlist[nxt_object]  # we leave the object here
                    //     waste_object[_robot.objecttype].objecttype = nxt_object
                    //     pass
                    //  all other areas we drive backward with grippers closed
                    _robot.gripper_close()
                    
                }
                
                //  for HIL_Simulation only
                if (HIL_Simulation) {
                    _robot.distance_to_target = 0
                }
                
            }
            
        }
        
        
    } else if (_robot.state == DRIVEBACKWARD) {
        //  drive backwards
        if (reverse_robot(_robot)) {
            _robot.state = FINDTARGET
            showText("FINDTARGET: " + ("" + _robot.target) + " " + direction_names[_robot.target_direction])
            _robot.gripper_close()
            //  for HIL_Simulation only
            if (HIL_Simulation) {
                _robot.distance_to_target = 90
                _robot.route_length = _robot.distance_to_target
            }
            
        }
        
        
    } else if (_robot.state == FINDTARGET) {
        //  find target by rotating robot, but do not drive
        _proceed = false
        if (FirstClaimDestination) {
            if (UseFleetmanager) {
                _robot.cmd = CLAIM_DESTINATION
                if (_robot.reply == OK) {
                    _robot.reply = 0
                    //  reset for next command
                    _proceed = true
                    _robot.cmd = STATUS_UPDATE
                }
                
                
            } else if (claim_destination(_robot.location, _robot.destination, _robot.id)[1] == _robot.id) {
                _proceed = true
            }
            
        } else {
            _proceed = true
        }
        
        if (_proceed) {
            _robot.targettype = LOCATION_t
            _dist_to_travel = find_target(_robot)
            //  when we have found the target, we are going to drive towards it
            if (_robot.target_locked) {
                //  we have found the target, re-check if it is clear to drive there
                // if CollisionDetection or claim_destination(_robot.location, _robot.destination, _robot.id)[1] == _robot.id:
                if (!FirstClaimDestination) {
                    _proceed = false
                    if (CollisionDetection) {
                        _proceed = true
                    } else if (UseFleetmanager) {
                        _robot.cmd = CLAIM_DESTINATION
                        if (_robot.reply == OK) {
                            _robot.reply = 0
                            //  reset for next command
                            _proceed = true
                            _robot.cmd = STATUS_UPDATE
                        }
                        
                        
                    } else if (claim_destination(_robot.location, _robot.destination, _robot.id)[1] == _robot.id) {
                        _proceed = true
                    }
                    
                }
                
                if (_proceed) {
                    showText("GOTOTARGET: " + ("" + _robot.target))
                    _robot.state = GOTOTARGET
                    _robot.route_reported = false
                    _robot.route_length = _dist_to_travel
                    _robot.cmd = STATUS_UPDATE
                }
                
            }
            
        }
        
        
    } else if (_robot.state == GOTOTARGET) {
        //  make sure we keep claim on target area
        _robot.steering_only = false
        _dist_to_travel = move_robot(_robot)
        _proceed = false
        _robot.cmd = STATUS_UPDATE
        //  report route haflway, only onetime reporting
        if (_dist_to_travel < _robot.route_length / 2 && !_robot.route_reported) {
            if (UseFleetmanager) {
                _robot.route_reported = true
                
                //  do not report release position
                /** 
                _robot.cmd = RELEASE_POSITION
                if _robot.reply == ACK:
                    _robot.reply = 0 # reset for next command
                    _robot.route_reported = True
                    _robot.cmd = STATUS_UPDATE
                pass
                
 */
            } else {
                release_position(_robot.location)
                _robot.route_reported = true
            }
            
        }
        
        _robot.distance_to_target = _dist_to_travel
        //  if we approach storage, open gripper before reaching destination
        if (storagearea.indexOf(_robot.destination) >= 0) {
            if (_dist_to_travel == 0) {
                _robot.state = FINDOBJECT
                //  assign an object to look for
                if (RandomDestination) {
                    _robot.objecttype = randint(1, numobjects)
                } else {
                    //  a random object is assigned to robot
                    _robot.objecttype = _robot.id
                }
                
                //  robot ID determines object types to be retrieved
                //  switch state and start looking for object
                showText("FINDOBJECT: " + ("" + _robot.objecttype))
                
            }
            
        } else if (_dist_to_travel <= 0 + 5) {
            //  target reached, release route
            _proceed = false
            if (UseFleetmanager) {
                _robot.cmd = RELEASE_ROUTE
                if (DEBUG) {
                    serial.writeLine("release route gototarget")
                }
                
                if (DEBUG) {
                    serial.writeLine("reply: " + ("" + _robot.reply))
                }
                
                if (_robot.reply == ACK) {
                    _robot.reply = 0
                    //  reset for next command
                    _proceed = true
                    _robot.cmd = STATUS_UPDATE
                }
                
                
            } else {
                release_route(_robot.location, _robot.destination)
                _proceed = true
            }
            
            //  update position
            if (_proceed) {
                _robot.location = _robot.destination
                _robot.state = IDLE
                _robot.cmd = STATUS_UPDATE
                showText("IDLE")
            }
            
        }
        
        
    } else if (_robot.state == FINDOBJECT) {
        //  Finding object could be done by sweeping left and reight until object is found
        _robot.steering_only = true
        //  if we approcah storage, open gripper before reaching destination
        _robot.targettype = OBJECT_t
        find_target(_robot)
        if (_robot.target_locked) {
            _robot.steering_only = false
            _robot.gripper_open()
            _robot.state = GRABOBJECT
            showText("GRABOBJECT: " + ("" + _robot.objecttype))
        }
        
    } else if (_robot.state == GRABOBJECT) {
        _dist_to_travel = move_robot(_robot)
        if (_dist_to_travel <= OBJECT_THRESHOLD + 5) {
            _robot.gripper_close()
            _proceed = false
            //  target reached, release route
            if (UseFleetmanager) {
                _robot.cmd = RELEASE_ROUTE
                if (DEBUG) {
                    serial.writeLine("release route grabobject")
                }
                
                if (_robot.reply == ACK) {
                    _robot.reply = 0
                    //  reset for next command
                    _proceed = true
                    _robot.cmd = STATUS_UPDATE
                }
                
                
            } else {
                release_route(_robot.location, _robot.destination)
                _proceed = true
            }
            
            //  update position
            if (_proceed) {
                _robot.location = _robot.destination
                //  switch to idle to start next route
                _robot.state = IDLE
                showText("IDLE")
            }
            
        }
        
        
    }
    
}

// ---------------------------------------------------------------------------------------------------------
//  collision detection, can be based on camera or sensor on robot
// ---------------------------------------------------------------------------------------------------------
function collision_detected(_robot: Robot = new Robot()): boolean {
    // 
    //  Needs implementing on robot
    // 
    _robot.obstacle_detected = false
    let obstacle_detected = false
    return obstacle_detected
}

// ---------------------------------------------------------------------------------------------------------
// 
//   ROUTE HANDLING BELOW
// 
// ---------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------
//  locations id's
// ---------------------------------------------------------------------------------------------------------
//  ids of waiting areas
let waitingarea = [0]
for (i = 0; i < numwaitingareas; i++) {
    waitingarea.push(i + 1)
}
//  ids of storage areas
let storagearea = [0]
for (i = numwaitingareas; i < numwaitingareas + numstorage; i++) {
    storagearea.push(i + 1)
}
//  ids of target areas
let target = [0]
for (i = numwaitingareas + numstorage; i < numwaitingareas + numstorage + numtargets; i++) {
    target.push(i + 1)
}
//  ids of objects
let obj = [0]
for (i = numwaitingareas + numstorage + numtargets; i < numwaitingareas + numstorage + numtargets + numobjects; i++) {
    obj.push(i + 1)
}
// occupation list of all locations
// occupying = [0]  * (numlocations +1) -> does not work in Makecode
//  for compatibility reasons wit MakeCode on robot, do it like this
let occupying = [0]
for (i = 0; i < numlocations + 1; i++) {
    occupying.push(0)
}
let locationnames = ["None"]
for (i = 1; i < numwaitingareas + 1; i++) {
    locationnames.push("" + "waitingarea " + ("" + i))
}
locationnames.push("" + "storage 1")
for (i = 1; i < numobjects + 1; i++) {
    locationnames.push("" + "targetarea " + ("" + i))
}
for (i = 0; i < numobjects + 1; i++) {
    locationnames.push("" + "object " + ("" + (i + 1)))
}
// ---------------------------------------------------------------------------------------------------------
//  instances of routes, using location id's
// ---------------------------------------------------------------------------------------------------------
let routes = [[0, 0]]
for (i = 0; i < 50; i++) {
    routes.push([0, 0])
}
//  route automatic define from location, to location, use location numbers instead of coordinates for easier update
for (i = 0; i < numwaitingareas; i++) {
    routes[i] = [waitingarea[1 + i], storagearea[1]]
}
let offset = numwaitingareas
for (i = 0; i < numobjects; i++) {
    routes[i + offset] = [storagearea[1], target[1 + i]]
}
offset += i + 1
for (i = 0; i < numobjects; i++) {
    if (i < numobjects / 2) {
        routes[i + offset] = [target[i + 1], waitingarea[1]]
    } else {
        routes[i + offset] = [target[i + 1], waitingarea[waitingarea.length - 1]]
    }
    
}
//  waitingarea[-1] is not compatibel with Makecode
offset += i + 1
let idx = 0
for (i = 1; i < numwaitingareas; i++) {
    if (i < numwaitingareas / 2) {
        routes[offset + i - 1] = [waitingarea[i], waitingarea[i + 1]]
        //  shift from left to middle
        idx = i
    } else {
        routes[offset + i - 1] = [waitingarea[waitingarea.length - 1] - i + idx + 1, waitingarea[waitingarea.length - 1] - i + idx]
    }
    
}
//  shift from right to middle
//  potentially crossing route combinations for additional clearance check
//  crossing routes:
//  - all routes from storage to targets in combination with all targets to waitingarea
//  - combination have to be all on left side or right side, right and left never cross (for now)
let crossing_routes_ids = [[[0, 0], [0, 0]]]
let crossing_routes = [[0, 0]]
for (i = 0; i < routes.length; i++) {
    if (routes[i][0] == storagearea[1]) {
        for (let j = 0; j < routes.length; j++) {
            if (routes[j][1] == waitingarea[1]) {
                if (routes[j][0] < routes[i][1]) {
                    if (target.slice(1, Math.trunc(numtargets / 2) + 1).indexOf(routes[i][1]) >= 0) {
                        crossing_routes_ids.push([routes[i], routes[j]])
                        crossing_routes.push([i, j])
                    }
                    
                }
                
            }
            
            if (routes[j][1] == waitingarea[waitingarea.length - 1]) {
                if (routes[j][0] < routes[i][1]) {
                    if (target.slice(Math.trunc(numtargets / 2) + 1, numtargets + 1).indexOf(routes[i][1]) >= 0) {
                        crossing_routes_ids.push([routes[i], routes[j]])
                        crossing_routes.push([i, j])
                    }
                    
                }
                
            }
            
        }
    }
    
}
let active_routes = [false]
for (i = 0; i < routes.length + 1; i++) {
    active_routes.push(false)
}
let crossed_routes = [false]
for (i = 0; i < routes.length + 1; i++) {
    crossed_routes.push(false)
}
// ---------------------------------------------------------------------------------------------------------
//  check what next destination will be, this is the route scheduler, might be local or remote implemented
// ---------------------------------------------------------------------------------------------------------
function next_destination(_location: number, objecttype: number = null): number[] {
    let nxt_dest = _location
    //  if there is no new location found, return current position as next destination
    let target_dir = AHEAD
    //  if we pick up an object, object nr tels us where to go
    if (storagearea.indexOf(_location) >= 0 && objecttype !== null) {
        //  were need the collected object go to?
        nxt_dest = target[objecttype]
        //  is target location left or rigth of us?
        if (target.slice(1, Math.trunc(numtargets / 2) + 1).indexOf(nxt_dest) >= 0) {
            target_dir = LEFT
        } else if (target.slice(Math.trunc(numtargets / 2) + 1, numtargets + 1).indexOf(nxt_dest) >= 0) {
            target_dir = RIGHT
        }
        
    } else if (target.slice(1, Math.trunc(numtargets / 2) + 1).indexOf(_location) >= 0) {
        //  if we come from a target to the left, goto left most waiting area
        nxt_dest = waitingarea[1]
        target_dir = LEFT
    } else if (target.slice(Math.trunc(numtargets / 2) + 1, numtargets + 1).indexOf(_location) >= 0) {
        //  if we come from a taget to the right, goto right most waiting area
        nxt_dest = waitingarea[waitingarea.length - 1]
        target_dir = RIGHT
    } else if (waitingarea.indexOf(_location) >= 0) {
        //  if we are in the waiting area, move toward the center wating area, before going back to the storage
        //  from waiting to storeage, unless we have to shift in the waiting area
        nxt_dest = storagearea[1]
        target_dir = LEFT
        for (let i = 1; i < Math.trunc(numwaitingareas / 2) + 1; i++) {
            //  from left to middle
            if (_location == waitingarea[i]) {
                nxt_dest = waitingarea[i + 1]
                target_dir = LEFT
            }
            
            //  from right to middle
            if (_location == waitingarea[waitingarea.length - 1] - i + 1) {
                nxt_dest = waitingarea[waitingarea.length - 1] - i
                target_dir = RIGHT
            }
            
        }
    }
    
    return [nxt_dest, target_dir]
}

// ---------------------------------------------------------------------------------------------------------
//  try to claim destination, if succesfull, returns True and own ID, if unsuccesfull returns ID that occupies the destination
// ---------------------------------------------------------------------------------------------------------
function claim_destination(_loc: number = 0, _dest: number = 0, _id: number = 0): number[] {
    let claimed_id: number;
    let claimed = 0
    let occupied = occupying[_dest]
    if (occupied == 0 && _id > 0) {
        occupying[_dest] = _id
        claimed_id = _id
        claimed = 1
    } else {
        claimed_id = occupying[_dest]
    }
    
    if (claimed_id == _id && _id > 0) {
        if (check_routes(_loc, _dest, _id)) {
            occupying[_dest] = _id
        } else {
            // release_position(_loc) # this assumes that you start driving imediatly after claim of location
            claimed_id = 0
        }
        
    }
    
    return [claimed, claimed_id]
}

// ---------------------------------------------------------------------------------------------------------
//  check if a destination is occupied, return ID that occupies destination
// ---------------------------------------------------------------------------------------------------------
function check_occupied(_dst: number) {
    let [claimed, occupied] = claim_destination(0, _dst)
    return occupied
}

// ---------------------------------------------------------------------------------------------------------
//  release location when you have left it
// ---------------------------------------------------------------------------------------------------------
function release_position(_location: number): boolean {
    occupying[_location] = 0
    return true
}

// ---------------------------------------------------------------------------------------------------------
//  release a route when we finished driving it
// ---------------------------------------------------------------------------------------------------------
function release_route(_loc: number, _dest: number) {
    let _route_nr: number;
    let _route = [_loc, _dest]
    // if _route in [r for r in routes]: # not compatible with makecode
    let _found = false
    for (let i = 0; i < routes.length; i++) {
        if (_route == routes[i]) {
            _found = true
        }
        
    }
    if (_found) {
        //  get route nr
        _route_nr = _py.py_array_index(routes, _route)
        active_routes[_route_nr] = false
    }
    
    //  check if route is actually released before returning -> need for remote implementation
    while (active_routes[_route_nr]) {
        //  keep on trying to release the route
        
    }
}

// ---------------------------------------------------------------------------------------------------------
//  check if routes are crossing, claim route if crossing path is free
// ---------------------------------------------------------------------------------------------------------
function check_routes(_loc: number, _dest: number, _id: number): boolean {
    let i: number;
    let _route_nr: number;
    let _r1: boolean;
    let _r2: boolean;
    if (CollisionDetection) {
        return true
    }
    
    let proceed = true
    //  check start location of route
    //  is route valid and in route list]
    let _route = [_loc, _dest]
    // if _route in [r for r in routes]: # not compatible with makecode
    let _found = false
    for (i = 0; i < routes.length; i++) {
        if (_route == routes[i] && routes[i] != [0, 0]) {
            _found = true
        }
        
    }
    if (_found) {
        //  get route nr
        _route_nr = _py.py_array_index(routes, _route)
        // _r1 = _route_nr in [r[0] for r in crossing_routes]  # not compatible with makecode
        // _r2 = _route_nr in [r[1] for r in crossing_routes] # not compatible with makecode
        _r1 = false
        _r2 = false
        for (i = 1; i < crossing_routes.length; i++) {
            if (_route_nr == crossing_routes[i][0]) {
                _r1 = true
            }
            
            if (_route_nr == crossing_routes[i][1]) {
                _r2 = true
            }
            
        }
        if (_r1 || _r2) {
            proceed = false
            for (i = 1; i < crossing_routes.length; i++) {
                if (_route_nr == crossing_routes[i][0]) {
                    if (!active_routes[crossing_routes[i][1]]) {
                        active_routes[_route_nr] = true
                        crossed_routes[_route_nr] = false
                        proceed = true
                    } else {
                        crossed_routes[_route_nr] = true
                    }
                    
                }
                
                if (_route_nr == crossing_routes[i][1]) {
                    if (!active_routes[crossing_routes[i][0]]) {
                        active_routes[_route_nr] = true
                        crossed_routes[_route_nr] = false
                        proceed = true
                    } else {
                        crossed_routes[_route_nr] = true
                    }
                    
                }
                
            }
        }
        
    }
    
    return proceed
}

// ---------------------------------------------------------------------------------------------------------
//  buttons and switches
// ---------------------------------------------------------------------------------------------------------
input.onButtonPressed(Button.A, function on_button_pressed_a() {
    
    stopping = false
    robot.active = true
    Full_auto = true
    if (DEBUG) {
        serial.writeLine("button pressed")
    }
    
    showText("ACTIVE")
    
})
/** 
def on_button_pressed_b():
    global stopping
    stopping = True
    if DEBUG: serial.write_line('reset')
    pass
input.on_button_pressed(Button.B, on_button_pressed_b)

 */
function leading_zeros(_num: number, _digits: number): string {
    let _str = "0"
    _num = Math.round(_num)
    if (_num < 0) {
        _num += 256
    }
    
    if (_digits == 3) {
        if (_num < 10) {
            _str = "00" + ("" + _num)
        } else if (_num < 100) {
            _str = "0" + ("" + _num)
        } else if (_num < 1000) {
            _str = "" + _num
        }
        
    }
    
    if (_digits == 2) {
        if (_num < 10) {
            _str = "0" + ("" + _num)
        } else if (_num < 100) {
            _str = "" + _num
        }
        
    }
    
    return _str
}

let latsprint = input.runningTime()
function print_data() {
    let t: number;
    let _cmd: number;
    let _chk: any;
    let _str: string;
    
    if (DEBUG) {
        // serial.write_value("# ", control.device_serial_number())
        t = input.runningTime()
        serial.writeValue("t : ", t / 1000)
        serial.writeValue("dt: ", (t - latsprint) / 1000)
        latsprint = t
        serial.writeLine("speed " + robot.speed)
        serial.writeLine("yaw speed " + robot.yaw_speed)
        serial.writeLine("distance target" + robot.distance_to_target)
        serial.writeValue("state", robot.state)
        serial.writeLine(state_names[robot.state])
        serial.writeLine("" + "target " + ("" + robot.target) + ("" + "  ") + locationnames[robot.target])
        serial.writeValue("distance", robot.distance_to_target)
        serial.writeValue("speed", robot.speed)
        _cmd = 0
        _chk = (robot.id + _cmd + robot.state + robot.location + robot.destination + robot.target + robot.distance_to_target) % 8
        _str = leading_zeros(robot.id, 2) + ("" + _cmd) + ("" + robot.state) + leading_zeros(robot.location, 2) + leading_zeros(robot.destination, 2) + leading_zeros(robot.target, 2) + leading_zeros(robot.distance_to_target, 3) + leading_zeros(robot.speed, 3) + ("" + _chk) + "\r\n"
        serial.writeLine(_str)
        serial.writeLine("--")
    }
    
    
}

/** loops.every_interval(500, print_data) */
/** 
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

 */
radio.onReceivedString(function on_radio_received(receivedString: string) {
    let _cmd: number;
    
    //  cmd reply
    //  0123456
    //  1,1,1,1
    //  optimised without string split, and earlier reject of message
    //  radio_rx_data = receivedString
    if (receivedString[0] == "" + robot.id) {
        // if DEBUG: serial.write_line("this is for us")
        if (DEBUG) {
            serial.writeLine("rx:" + receivedString)
        }
        
        // data = radio_rx_data.split(',')
        // for d in range (len(data)):
        //     if DEBUG: serial.write_line(data[d])
        // _id = int(data[0])
        // if _id == robot.id:
        //  this message is send for us
        // _cmd = int(data[1])
        _cmd = parseInt(receivedString[2])
        if (_cmd == 0) {
            //  send status report or pending command
            // robot.cmd = STATUS_UPDATE
            
        } else if (_cmd == 1) {
            //  NOK/OK claim destination reply
            //  data[2] = NOK/OK, data[3] = claimed id
            // if int(data[3]) == robot.id:
            if (parseInt(receivedString[6]) == robot.id) {
                robot.reply = OK
                robot.cmd = 0
            }
            
            //  no more commands
            
        } else if (_cmd == 2) {
            //  ACK releaseposition
            // if int(data[2]) == 2:
            if (parseInt(receivedString[4]) == 2) {
                robot.reply = ACK
                robot.cmd = 0
            }
            
            
        } else if (_cmd == 3) {
            //  ACK releaseroute
            // if int(data[2]) == 2:
            if (parseInt(receivedString[4]) == 2) {
                robot.reply = ACK
                robot.cmd = 0
            }
            
            
        } else if (_cmd == 4) {
            //  START driving
            stopping = false
            robot.active = true
            robot.backup_starttime = get_time()
            
        } else if (_cmd == 5) {
            //  STOP driving
            stopping = true
            robot.active = false
            
        }
        
        if (DEBUG) {
            serial.writeLine("reply: " + ("" + robot.reply))
        }
        
        RadioTxPending = false
        WaitingForReply = false
        // radio_transmit(robot)
        // if DEBUG: serial.write_line("sending radio")
        yield_(5)
    }
    
    //  yield for other processes
    
})
function radio_transmit(_robot: Robot = new Robot()) {
    let _cmd = _robot.cmd
    let _chk = (_robot.id + _cmd + _robot.state + _robot.location + _robot.destination + _robot.target + _robot.distance_to_target + _robot.speed) % 8
    let _str = leading_zeros(robot.id, 2) + ("" + _cmd) + ("" + robot.state) + leading_zeros(robot.location, 2) + leading_zeros(robot.destination, 2) + leading_zeros(robot.target, 2) + leading_zeros(robot.distance_to_target, 3) + leading_zeros(robot.speed, 3) + ("" + _chk) + "\r\n"
    //  send 2 times to make sure the message came trough
    radio.sendString(_str)
    yield_(100)
    radio.sendString(_str)
}

// radio.send_string("id:" + str(robot.id))
// radio.send_string(",state:" + str(robot.state))
// radio.send_string(str(",loc:" ) + str(robot.location))
// radio.send_string(str(",des:" ) + str(robot.destination))
// radio.send_string(str(",tar:" ) + str(robot.target))
// radio.send_string("\r\n" )
// radio.send_string(str("target " ) + str(robot.target) + str("  ") + locationnames[robot.target] + '\r\n')
// loops.every_interval(250, on_every_interval)
function yield_(_t: number = 50) {
    basic.pause(_t)
}

// ---------------------------------------------------------------------------------------------------------
//  actual main loop
// ---------------------------------------------------------------------------------------------------------
let stopping = false
radio.setGroup(1)
close_gripper()
let RadioTxTime = input.runningTime()
let WaitingForReply = false
//  function for getting data from huskylens and function for frame buffer
// ---------------------------------------------------------------------------------------------------------
function get_frame_data(inview: number): number[] {
    let tag: number;
    let x: number;
    let y: number;
    let w: number;
    let h: number;
    
    for (tag = 0; tag < inview; tag++) {
        tagsInView.push(huskylens.readBox_ss(tag + 1, Content3.ID))
    }
    if (huskylens.isAppear(robot.target, HUSKYLENSResultType_t.HUSKYLENSResultBlock)) {
        // volgensmij zit er een bug in robot.target waarbij die soms de verkeerde qr code uitleest als er meerdere  in beeld zijn
        // de waarde robot.target is een nummer van target die die zoekt
        // bij de functie husylens.reade_box() moet de volgorde van tag ingevoerd worden die hij ziet(volgensmij)
        serial.writeLine("target " + ("" + robot.target))
        serial.writeLine("" + tag + " inview")
        x = huskylens.readeBox(robot.target, Content1.xCenter)
        y = huskylens.readeBox(robot.target, Content1.yCenter)
        w = huskylens.readeBox(robot.target, Content1.width)
        h = huskylens.readeBox(robot.target, Content1.height)
    } else {
        let [x, y, w, h] = [-1, -1, -1, -1]
    }
    
    return [x, y, w, h]
}

function data_buffer(x: number, y: number, w: number, h: number, tagsInView: number[], reset: boolean): number[][] {
    
    //  check if buffer needs to be reset
    if (frame_count > 3) {
        reset = true
    }
    
    //  replace buffer data
    if (reset) {
        frame_count = 0
        buffer_xywh = [x, y, w, h]
        buffer_tagsInView = tagsInView
    } else {
        // count frames
        frame_count = frame_count + 1
    }
    
    //  return buffer data
    return [buffer_xywh, buffer_tagsInView]
}

// ---------------------------------------------------------------------------------------------------------
//  actual main loop
// ---------------------------------------------------------------------------------------------------------
stopping = false
radio.setGroup(1)
close_gripper()
basic.forever(function on_forever() {
    
    if (Full_auto) {
        robot.active = true
        stopping = false
    }
    
    if (!stopping && robot.active) {
        if (robot.cmd == POSITION_UPDATE) {
            robot.cmd = STATUS_UPDATE
            radio_transmit(robot)
            //  send message
            robot.cmd = NO_CMD
            robot.active = false
            //  wait for cleareance by fleet manager
            stop_robot(robot)
            stopping = true
        }
        
        if (newdata) {
            newdata = false
            do_robot(robot)
        }
        
    }
    
    if (HIL_Simulation) {
        basic.pause(50)
    }
    
    // print_data() -> background loop
    if (stopping) {
        robot.active = false
        stop_robot(robot)
    }
    
    // robot.location = 1
    // robot.destination = 1
    // robot.target = 1
    // robot.objecttype = None
    // robot.state = IDLE
    
})
control.inBackground(function on_in_background() {
    
    //  approx 56 ms -> ~ 18 fps
    while (1) {
        // get new frame data
        huskylens.request()
        inview = huskylens.getBox(HUSKYLENSResultType_t.HUSKYLENSResultBlock)
        tagsInView = [0]
        //  if there is a tag in view, refresh buffer.
        if (huskylens.isAppear(robot.target, HUSKYLENSResultType_t.HUSKYLENSResultBlock)) {
            let ___tempvar15 = get_frame_data(inview)
            x = ___tempvar15[0]
            y = ___tempvar15[1]
            w = ___tempvar15[2]
            h = ___tempvar15[3]
            data_buffer(x, y, w, h, tagsInView, true)
        } else {
            //  if there is no tag in view use latest buffer data
            let ___tempvar16 = data_buffer(x, y, w, h, tagsInView, false)
            let box = ___tempvar16[0]
            tagsInView = ___tempvar16[1]
            let ___tempvar17 = [box[0], box[1], box[2], box[3]]
            x = ___tempvar17[0]
            y = ___tempvar17[1]
            w = ___tempvar17[2]
            h = ___tempvar17[3]
        }
        
        newdata = true
    }
})

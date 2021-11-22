from physics import *
from dataManagement import *
from controlMath import *
from navigation import *
from motors import *

import matplotlib.pyplot as plt
import numpy as np

DEG_TO_RAD = np.pi / 180
RAD_DEG = 180 / np.pi

# stolen from stack overflow
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Print iterations progress
def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bcolors.WARNING}{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total:
        print()

rocket = DOF6("rocket")

rocket.dataLogger = dataLogger()

rocket.dataLogger.addDataPoint("time")

rocket.dataLogger.addDataPoint("ori_x_roll")
rocket.dataLogger.addDataPoint("ori_y_pitch")
rocket.dataLogger.addDataPoint("ori_z_yaw")

rocket.dataLogger.addDataPoint("ori_x_roll_sensed")
rocket.dataLogger.addDataPoint("ori_y_pitch_sensed")
rocket.dataLogger.addDataPoint("ori_z_yaw_sensed")

rocket.dataLogger.addDataPoint("reaction_wheel_output_x_roll")
rocket.dataLogger.addDataPoint("actuator_output_y_pitch")
rocket.dataLogger.addDataPoint("actuator_output_z_yaw")

rocket.dataLogger.addDataPoint("ori_x_roll_setpoint")
rocket.dataLogger.addDataPoint("ori_y_pitch_setpoint")
rocket.dataLogger.addDataPoint("ori_z_yaw_setpoint")

rocket.dataLogger.addDataPoint("ori_x_roll_velocity")
rocket.dataLogger.addDataPoint("ori_y_pitch_velocity")
rocket.dataLogger.addDataPoint("ori_z_yaw_velocity")

rocket.dataLogger.addDataPoint("ori_x_roll_velocity_sensed")
rocket.dataLogger.addDataPoint("ori_y_pitch_velocity_sensed")
rocket.dataLogger.addDataPoint("ori_z_yaw_velocity_sensed")

rocket.dataLogger.addDataPoint("ori_x_roll_velocity_sensed_filtered")
rocket.dataLogger.addDataPoint("ori_y_pitch_velocity_sensed_filtered")
rocket.dataLogger.addDataPoint("ori_z_yaw_velocity_sensed_filtered")

# rocket.dataLogger.addDataPoint("ori_x_acceleration")#############
# rocket.dataLogger.addDataPoint("ori_y_acceleration")#############
# rocket.dataLogger.addDataPoint("ori_z_acceleration")#############

rocket.dataLogger.addDataPoint("x_position")
rocket.dataLogger.addDataPoint("y_position")
rocket.dataLogger.addDataPoint("z_position")

rocket.dataLogger.addDataPoint("x_position_sensed")
rocket.dataLogger.addDataPoint("y_position_sensed")
rocket.dataLogger.addDataPoint("z_position_sensed")

rocket.dataLogger.addDataPoint("x_velocity")
rocket.dataLogger.addDataPoint("y_velocity")
rocket.dataLogger.addDataPoint("z_velocity")

rocket.dataLogger.addDataPoint("x_velocity_sensed")
rocket.dataLogger.addDataPoint("y_velocity_sensed")
rocket.dataLogger.addDataPoint("z_velocity_sensed")

rocket.dataLogger.addDataPoint("resultant_velocity")
rocket.dataLogger.addDataPoint("resultant_velocity_sensed")

rocket.dataLogger.addDataPoint("prograde_direction_x_roll")
rocket.dataLogger.addDataPoint("prograde_direction_y_pitch")
rocket.dataLogger.addDataPoint("prograde_direction_z_yaw")

rocket.dataLogger.addDataPoint("retrograde_direction_x_roll")
rocket.dataLogger.addDataPoint("retrograde_direction_y_pitch")
rocket.dataLogger.addDataPoint("retrograde_direction_z_yaw")

rocket.dataLogger.addDataPoint("x_acceleration")
rocket.dataLogger.addDataPoint("y_acceleration")
rocket.dataLogger.addDataPoint("z_acceleration")

rocket.dataLogger.addDataPoint("x_acceleration_sensed")
rocket.dataLogger.addDataPoint("y_acceleration_sensed")
rocket.dataLogger.addDataPoint("z_acceleration_sensed")

rocket.dataLogger.addDataPoint("x_acceleration_sensed_filtered")
rocket.dataLogger.addDataPoint("y_acceleration_sensed_filtered")
rocket.dataLogger.addDataPoint("z_acceleration_sensed_filtered")

rocket.dataLogger.addDataPoint("x_acceleration_inertial")
rocket.dataLogger.addDataPoint("y_acceleration_inertial")
rocket.dataLogger.addDataPoint("z_acceleration_inertial")

rocket.dataLogger.addDataPoint("x_acceleration_inertial_sensed")
rocket.dataLogger.addDataPoint("y_acceleration_inertial_sensed")
rocket.dataLogger.addDataPoint("z_acceleration_inertial_sensed")

rocket.dataLogger.addDataPoint("altitude_barometric")
rocket.dataLogger.addDataPoint("velocity_barometric")

rocket.dataLogger.addDataPoint("resultant_acceleration")
rocket.dataLogger.addDataPoint("resultant_acceleration_sensed")

rocket.dataLogger.addDataPoint("thrust")
rocket.dataLogger.addDataPoint("mass")
rocket.dataLogger.addDataPoint("throttle")

data_list = []

for point in rocket.dataLogger.variableDescriptions:
    data_list.append(point)

plotter = dataVisualiser()
plotter.allDataDescriptions = data_list

rocket.dataLogger.fileName = "data_out.csv"

rocket.dataLogger.initCSV(True, True)

apogee = False

rocket.dryMass = 0.95
motor = rocketMotor(1000)

motor.add_motor(motorType.f15, "ascent")
motor.add_motor(motorType.f15, "landing")

rocket.mass = rocket.dryMass + motor.totalMotorMass

rocket.mmoi = vector3(0.0102, 0.0404203354, 0.0404203354)
rocket.rotation_euler = vector3(0, random.randint(-100, 100) / 100 * DEG_TO_RAD, random.randint(-100, 100) / 100 * DEG_TO_RAD)
rocket.rotaiton_quaternion = Quaternion().eulerToQuaternion(rocket.rotation_euler.x, rocket.rotation_euler.y, rocket.rotation_euler.z)
rocket.gravity = vector3(-9.83, 0.0, 0.0)

def getAngleFromDesiredAcceleration(desired_acceleration, thrust):
    return np.arcsin(desired_acceleration*rocket.mass/thrust)

ori_setpoint = flightPath()
ori_setpoint.loadFlightPath('flightPath.csv')

maxPosition = 0.0
global setpoint
setpoint = vector3(0.0, 0.0, 0.0)

PID_ori_X = PID(0, 0, 0, 0, 0, False)
PID_ori_Y = PID(1.8, 0.2, 0.4, 0, 2, False)
PID_ori_Z = PID(1.8, 0.2, 0.4, 0, 2, False)

PID_position_Y = PID(4, 0, 0.25, 0, 0, False)
PID_position_Z = PID(4, 0, 0.25, 0, 0, False)

fsf_gains_X = np.matrix([22.36, 5.24])
fsf_setpoint_X = np.array([0.0, 0.0])

fsf_gains_Y = np.matrix([22.36, 5.24])
fsf_setpoint_Y = np.array([0.0, 0.0])

fsf_gains_Z = np.matrix([22.36, 5.24])
fsf_setpoint_Z = np.array([0.0, 0.0])

FSF_ori_X = FSF_ori(fsf_gains_X, fsf_setpoint_X, 0)
FSF_ori_Y = FSF_ori(fsf_gains_Y, fsf_setpoint_Y, 0)
FSF_ori_Z = FSF_ori(fsf_gains_Z, fsf_setpoint_Z, 0)

fsf_test_z = FSF_ori_test(22.36, 5.24)
fsf_test_y = FSF_ori_test(22.36, 5.24)

rocket_TVC = TVC()

rocket_TVC.lever = 0.24
rocket_TVC.linkageRatioZ = 10
rocket_TVC.linkageRatioY = 10

rocket_TVC.noiseY = 0.2 * DEG_TO_RAD
rocket_TVC.noiseZ = 0.2 * DEG_TO_RAD

rocket_TVC.servoSpeed = 200

rocket_TVC.maxY = 3
rocket_TVC.maxZ = 3
rocket_TVC.minY = -3
rocket_TVC.minZ = -3

TVC_derolled = vector3( 0.0, 0.0, 0.0 )

rocket_TVC.positionY = random.randint(-100, 100) / 1000 * DEG_TO_RAD
rocket_TVC.positionZ = random.randint(-100, 100) / 1000 * DEG_TO_RAD

rocket_TVC.offsetY = random.randint(-100, 100) / 1000 * DEG_TO_RAD
rocket_TVC.offsetZ = random.randint(-100, 100) / 1000 * DEG_TO_RAD

time = 0.0
missionTime = 0.0
timeStep = 1000
simTime = 90

totalSteps = simTime * timeStep
currentStep = 0

sensorDelay = 1 / 500
lastSensor = 0.0

controlSpeed = 25
controlDelay = 1 / controlSpeed
lastControl = 0.0

CSVSpeed = 50
CSVDelay = 1 / CSVSpeed
lastCSV = 0.0

dt = 1 / timeStep

motor.ignite("ascent", 1.0)

motor.maxIgnitionDelay = 0.78

NAV = NAVController()
IMU = IMU6DOF()
baro = barometer()

baro.readDelay = 1 / 40
IMU.sampleRateAccel = 1 / 500
IMU.sampleRateGyro = 1 / 500

baro.noise = 0.005

IMU.gyroBias = vector3(np.random.normal(0.0, 1, 1)[0] * DEG_TO_RAD, np.random.normal(0.0, 1, 1)[0] * DEG_TO_RAD, np.random.normal(0.0, 1, 1)[0] * DEG_TO_RAD)
IMU.accelBias = vector3(np.random.normal(0.0, 0.6, 1)[0], np.random.normal(0.0, 0.6, 1)[0], np.random.normal(0.0, 0.6, 1)[0])

IMU.gyroNoise = abs(vector3(np.random.normal(0.2, 0.1, 1)[0] * DEG_TO_RAD, np.random.normal(0.2, 0.1, 1)[0] * DEG_TO_RAD, np.random.normal(0.2, 0.1, 1)[0] * DEG_TO_RAD))
IMU.accelNoise = abs(vector3(np.random.normal(0.3, 0.2, 1)[0], np.random.normal(0.3, 0.2, 1)[0], np.random.normal(0.3, 0.2, 1)[0]))

IMU.gyroScale = vector3(500 * DEG_TO_RAD, 500 * DEG_TO_RAD, 500 * DEG_TO_RAD)
IMU.accelScale = vector3(40, 40, 40)

TVC_command = vector3(0.0, 0.0, 0.0)

controlVelocity = False
followSetpoint = True

NAV.orientation_euler = rocket.rotation_euler
NAV.orientation_quat = rocket.rotaiton_quaternion

# user defined functions and variables
global apogee_sensed
apogee_sensed = 0.0
light_alt = 0.0

dv_estimate = 7.5
dv_actual = 0.0
vel_burn_start = 0.0
throttle = 0.0

def readSensors(time, dt):
    
    IMU.readAccel(rocket.accelerationLocal, time)
    IMU.readGyro(rocket.rotationalVelocity, time)

    if time < 1 and IMU.accel.x < 10 and NAV.debiased == False:
        NAV.measureDebias(IMU.accel, IMU.oriRates)
    else:
        NAV.inFlight = True
        NAV.debias()
        # NAV.accelOri(IMU.accel)
    
    if NAV.inFlight:
        NAV.update(IMU.accel, IMU.oriRates, rocket.gravity, dt, time)

    if time > baro.lastRead + baro.readDelay:
        baro.read(rocket.positionInertial.x, time)
        NAV.passBarometerData(baro.altitude, baro.velocity, time)

def logCSV():
    global setpoint
    rocket.dataLogger.recordVariable("time", time)
    rocket.dataLogger.recordVariable("ori_x_roll", rocket.rotation_euler.x * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_y_pitch", rocket.rotation_euler.y * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_z_yaw", rocket.rotation_euler.z * RAD_TO_DEG)

    rocket.dataLogger.recordVariable("ori_x_roll_sensed", NAV.orientation_euler.x * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_y_pitch_sensed", NAV.orientation_euler.y * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_z_yaw_sensed", NAV.orientation_euler.z * RAD_TO_DEG)

    rocket.dataLogger.recordVariable("reaction_wheel_output_x_roll", PID_ori_X.output)
    rocket.dataLogger.recordVariable("actuator_output_y_pitch", rocket_TVC.positionY * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("actuator_output_z_yaw", rocket_TVC.positionZ * RAD_TO_DEG)

    rocket.dataLogger.recordVariable("ori_x_roll_setpoint", setpoint.x * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_y_pitch_setpoint", setpoint.y * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_z_yaw_setpoint", setpoint.z * RAD_TO_DEG)

    rocket.dataLogger.recordVariable("ori_x_roll_velocity", rocket.rotationalVelocity.x * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_y_pitch_velocity", rocket.rotationalVelocity.y * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_z_yaw_velocity", rocket.rotationalVelocity.z * RAD_TO_DEG)

    rocket.dataLogger.recordVariable("ori_x_roll_velocity_sensed", NAV.oriRates.x * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_y_pitch_velocity_sensed", NAV.oriRates.y * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_z_yaw_velocity_sensed", NAV.oriRates.z * RAD_TO_DEG)
    
    rocket.dataLogger.recordVariable("ori_x_roll_velocity_sensed_filtered", NAV.oriRatesFiltered.x * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_y_pitch_velocity_sensed_filtered", NAV.oriRatesFiltered.y * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("ori_z_yaw_velocity_sensed_filtered", NAV.oriRatesFiltered.z * RAD_TO_DEG)

    # rocket.dataLogger.recordVariable("ori_x_acceleration", rocket.rotationalAcceleration.x)####", rocket.)
    # rocket.dataLogger.recordVariable("ori_y_acceleration", rocket.rotationalAcceleration.y)####", rocket.)
    # rocket.dataLogger.recordVariable("ori_z_acceleration", rocket.rotationalAcceleration.z)####", rocket.)

    rocket.dataLogger.recordVariable("x_position", rocket.positionInertial.x)
    rocket.dataLogger.recordVariable("y_position", rocket.positionInertial.y)
    rocket.dataLogger.recordVariable("z_position", rocket.positionInertial.z)

    rocket.dataLogger.recordVariable("x_position_sensed", NAV.positionInertial.x)
    rocket.dataLogger.recordVariable("y_position_sensed", NAV.positionInertial.y)
    rocket.dataLogger.recordVariable("z_position_sensed", NAV.positionInertial.z)

    rocket.dataLogger.recordVariable("x_velocity", rocket.velocityInertial.x)
    rocket.dataLogger.recordVariable("y_velocity", rocket.velocityInertial.y)
    rocket.dataLogger.recordVariable("z_velocity", rocket.velocityInertial.z)

    rocket.dataLogger.recordVariable("x_velocity_sensed", NAV.velocityInertial.x)
    rocket.dataLogger.recordVariable("y_velocity_sensed", NAV.velocityInertial.y)
    rocket.dataLogger.recordVariable("z_velocity_sensed", NAV.velocityInertial.z)

    rocket.dataLogger.recordVariable("resultant_velocity", rocket.velocityInertial.len())
    rocket.dataLogger.recordVariable("resultant_velocity_sensed", NAV.velocityInertial.len())
    
    rocket.dataLogger.recordVariable("prograde_direction_x_roll", rocket.velocityInertial.dir().x * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("prograde_direction_y_pitch", rocket.velocityInertial.dir().y * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("prograde_direction_z_yaw", rocket.velocityInertial.dir().z * RAD_TO_DEG)

    rocket.dataLogger.recordVariable("x_acceleration", rocket.accelerationLocal.x)
    rocket.dataLogger.recordVariable("y_acceleration", rocket.accelerationLocal.y)
    rocket.dataLogger.recordVariable("z_acceleration", rocket.accelerationLocal.z)

    rocket.dataLogger.recordVariable("x_acceleration_sensed", NAV.accelerationLocal.x)
    rocket.dataLogger.recordVariable("y_acceleration_sensed", NAV.accelerationLocal.y)
    rocket.dataLogger.recordVariable("z_acceleration_sensed", NAV.accelerationLocal.z)

    rocket.dataLogger.recordVariable("x_acceleration_sensed_filtered", NAV.accelerationLocalFiltered.x)
    rocket.dataLogger.recordVariable("y_acceleration_sensed_filtered", NAV.accelerationLocalFiltered.y)
    rocket.dataLogger.recordVariable("z_acceleration_sensed_filtered", NAV.accelerationLocalFiltered.z)

    rocket.dataLogger.recordVariable("x_acceleration_inertial", rocket.accelerationInertial.x)
    rocket.dataLogger.recordVariable("y_acceleration_inertial", rocket.accelerationInertial.y)
    rocket.dataLogger.recordVariable("z_acceleration_inertial", rocket.accelerationInertial.z)

    rocket.dataLogger.recordVariable("x_acceleration_inertial_sensed", NAV.accelerationInertial.x)
    rocket.dataLogger.recordVariable("y_acceleration_inertial_sensed", NAV.accelerationInertial.y)
    rocket.dataLogger.recordVariable("z_acceleration_inertial_sensed", NAV.accelerationInertial.z)

    rocket.dataLogger.recordVariable("altitude_barometric", baro.altitude)
    rocket.dataLogger.recordVariable("velocity_barometric", baro.velocity)
    
    rocket.dataLogger.recordVariable("resultant_acceleration", rocket.accelerationInertial.z)
    rocket.dataLogger.recordVariable("resultant_acceleration_sensed", rocket.accelerationInertial.z)

    rocket.dataLogger.recordVariable("thrust", motor.currentThrust)
    rocket.dataLogger.recordVariable("mass", rocket.mass)

    rocket.dataLogger.saveData(False)

global burnAlt
burnAlt = 0.0
def controlLoop():
    global apogee_sensed
    global burnAlt
    global setpoint
    if NAV.velocityInertial.x < 0 and apogee_sensed == 0.0:
        rocket.mass -= 0.025
        burnAlt = 0.68 * NAV.positionInertial.x
        apogee_sensed = NAV.positionInertial.x

    if apogee_sensed > 0 and NAV.positionInertial.x < burnAlt:
        motor.ignite("landing", time)

    setpoint = vector3(0, 0, 0)
    # print(round(rocket.velocityInertial.dir() * RAD_TO_DEG, 2))

    if apogee_sensed > 0 and NAV.positionInertial.x < burnAlt and NAV.velocityInertial.x < -1:
        pgrad = NAV.velocityInertial.dir()
        if pgrad.z > 0:
            pgrad.z -= 180 * DEG_TO_RAD
        else:
            pgrad.z += 180 * DEG_TO_RAD
        if pgrad.y > 0:
            pgrad.y -= 180 * DEG_TO_RAD
        else:
            pgrad.y += 180 * DEG_TO_RAD
        pgrad.y *= -1
        setpoint = pgrad
        # setpoint = vector3(0.0, 0.0, 0.0)
    else:
        setpoint = vector3(0.0, 0.0, 0.0)

    if followSetpoint == True and missionTime < 3:

        if controlVelocity == False:
            ori_setpoint.getCurrentSetpoint(missionTime)
            setpoint = ori_setpoint.currentSetpoint
            setpoint = NAV.orientation_quat.rotateVector(setpoint * DEG_TO_RAD)

        else:

            ori_setpoint.getCurrentSetpoint(missionTime)

            PID_position_Y.setPoint = ori_setpoint.currentSetpoint.y
            PID_position_Z.setPoint = ori_setpoint.currentSetpoint.z

            PID_position_Y.compute(NAV.velocityInertial.y, controlDelay)
            PID_position_Z.compute(NAV.velocityInertial.z, controlDelay)

            setpoint.y = getAngleFromDesiredAcceleration(PID_position_Y.output, IMU.accel.x)
            setpoint.z = getAngleFromDesiredAcceleration(PID_position_Z.output, IMU.accel.x)  

            setpoint.y = clamp(setpoint.y, -0.25, 0.25)
            setpoint.z = clamp(setpoint.z, -0.25, 0.25)          
            
            setpoint = NAV.orientation_quat.rotateVector(setpoint)
    # else:
    #     setpoint = vector3(0.0, 0.0, 0.0)

    # FSF_ori_Y.fsf_setpoint[0] = setpoint.y
    # FSF_ori_Z.fsf_setpoint[0] = setpoint.z

    fsf_test_z.setpoint = setpoint.z
    fsf_test_y.setpoint = setpoint.y

    # FSF_ori_Y.compute(NAV.orientation_euler.y, controlDelay)
    # FSF_ori_Z.compute(NAV.orientation_euler.z, controlDelay)

    fsf_test_y.compute(NAV.orientation_euler.y, NAV.oriRates.y)
    fsf_test_z.compute(NAV.orientation_euler.z, NAV.oriRates.z)

    # TVC_y = calculateAngleFromDesiredTorque(rocket_TVC.lever, NAV.accelerationLocalFiltered.x, rocket.mmoi.y, FSF_ori_Y.output)
    # TVC_z = calculateAngleFromDesiredTorque(rocket_TVC.lever, NAV.accelerationLocalFiltered.x, rocket.mmoi.z, FSF_ori_Z.output)
    
    TVC_y = calculateAngleFromDesiredTorque(rocket_TVC.lever, NAV.accelerationLocalFiltered.x, rocket.mmoi.y, fsf_test_y.getOutput())
    TVC_z = calculateAngleFromDesiredTorque(rocket_TVC.lever, NAV.accelerationLocalFiltered.x, rocket.mmoi.z, fsf_test_z.getOutput())
    
    rotated_tvc = rotate(TVC_y, TVC_z, NAV.orientation_euler.x)
    return vector3(0.0, rotated_tvc.x, rotated_tvc.y)

while time < simTime:
    time += dt

    if rocket.positionInertial.x > 0.01:
        missionTime += dt

    motor.update(time)
    rocket_TVC.calculateForces(motor.currentThrust)

    rocket.mass = rocket.dryMass + motor.totalMotorMass

    # rocket.addTorque(vector3(0.0, np.sin(time/2)*5,np.sin(time/2)*5) * dt)
    rocket.addForce(rocket_TVC.acceleration)
    rocket.addTorque(rocket_TVC.torque)
    rocket.addTorque(rocket.rotationalVelocity * -0.05)
    rocket.addForce(vector3(rocket.velocityInertial.x * -0.025, rocket.velocityInertial.z * -0.025, rocket.velocityInertial.z * -0.025))

    if NAV.accelerationLocal.x > 2:
        rocket_TVC.actuate(TVC_command, dt)
    else:
        rocket_TVC.actuate(vector3(0.0, 0.0, 0.0), dt)

    if time > lastControl + controlDelay and NAV.positionInertial.x > 0.1:
        TVC_command = controlLoop()
        lastControl = time

    if time > lastSensor + sensorDelay:
        readSensors(time, sensorDelay)
        lastSensor = time

    if time > lastCSV + CSVDelay:
        logCSV()
        lastCSV = time

    rocket.update(dt)
    rocket.clear()

    if abs(rocket.positionInertial.x) > maxPosition:
        maxPosition = abs(rocket.positionInertial.x)

    if abs(rocket.positionInertial.y) > maxPosition:
        maxPosition = abs(rocket.positionInertial.y)

    if abs(rocket.positionInertial.z) > maxPosition:
        maxPosition = abs(rocket.positionInertial.z)

    if rocket.velocityInertial.x < 0 and rocket.positionInertial.x > 0.1 and apogee == False:
        apogee = True
        apogee_actual = rocket.positionInertial.x

    if rocket.positionInertial.x <= 0.05 and apogee == True:
        break

print(f'''
{bcolors.WARNING}
------------------------------------------------------
---             FLIGHT STATISTICS                  ---
------------------------------------------------------

{bcolors.WARNING}flight lasted {round(time, 2)} seconds

{bcolors.OKBLUE}apogee: {bcolors.FAIL}{round(apogee_actual, 2)} m{bcolors.OKBLUE} - apogee sensed:{bcolors.FAIL} {round(apogee_actual, 2)} m

{bcolors.OKBLUE}position y sensed: {bcolors.FAIL}{round(NAV.positionInertial.y, 2)} m {bcolors.OKBLUE}- y actual:{bcolors.FAIL} {round(rocket.positionInertial.y, 2)} m
{bcolors.OKBLUE}position z sensed: {bcolors.FAIL}{round(NAV.positionInertial.z, 2)} m {bcolors.OKBLUE}- z actual:{bcolors.FAIL} {round(rocket.positionInertial.z, 2)} m

{bcolors.WARNING}vertical velocity at impact:
{bcolors.OKBLUE}x sensed: {bcolors.FAIL}{round(rocket.velocityInertial.x, 2)} m/s {bcolors.OKBLUE} x actual: {bcolors.FAIL}{round(rocket.velocityInertial.x, 2)} m/s

{bcolors.WARNING}lateral velocities at impact:
{bcolors.OKBLUE}y sensed: {bcolors.FAIL}{round(NAV.velocityInertial.y, 2)} m/s{bcolors.OKBLUE} - y actual: {bcolors.FAIL}{round(rocket.velocityInertial.y, 2)} m/s 
{bcolors.OKBLUE}z sensed: {bcolors.FAIL}{round(NAV.velocityInertial.z, 2)} m/s{bcolors.OKBLUE} - z actual: {bcolors.FAIL}{round(rocket.velocityInertial.z, 2)} m/s 

{bcolors.WARNING}orientation at impact:

{bcolors.OKBLUE}x sensed: {bcolors.FAIL}{round(NAV.orientation_euler.x * RAD_DEG, 2)} deg{bcolors.OKBLUE} -  x actual: {bcolors.FAIL}{round(rocket.rotation_euler.x * RAD_DEG, 2)} deg
{bcolors.OKBLUE}y sensed: {bcolors.FAIL}{round(NAV.orientation_euler.y * RAD_DEG, 2)} deg{bcolors.OKBLUE} -  y actual: {bcolors.FAIL}{round(rocket.rotation_euler.y * RAD_DEG, 2)} deg
{bcolors.OKBLUE}z sensed: {bcolors.FAIL}{round(NAV.orientation_euler.z * RAD_DEG, 2)} deg{bcolors.OKBLUE} -  z actual: {bcolors.FAIL}{round(rocket.rotation_euler.z * RAD_DEG, 2)} deg

{bcolors.WARNING}rotational velocity at impact:

{bcolors.OKBLUE}x sensed: {bcolors.FAIL}{round(NAV.oriRates.x * RAD_DEG, 2)} deg/s {bcolors.OKBLUE}-  x actual: {bcolors.FAIL}{round(rocket.rotationalVelocity.x * RAD_DEG, 2)} deg/s
{bcolors.OKBLUE}y sensed: {bcolors.FAIL}{round(NAV.oriRates.y * RAD_DEG, 2)} deg/s {bcolors.OKBLUE}-  y actual: {bcolors.FAIL}{round(rocket.rotationalVelocity.y * RAD_DEG, 2)} deg/s
{bcolors.OKBLUE}z sensed: {bcolors.FAIL}{round(NAV.oriRates.z * RAD_DEG, 2)} deg/s {bcolors.OKBLUE}-  z actual: {bcolors.FAIL}{round(rocket.rotationalVelocity.z * RAD_DEG, 2)} deg/s

{bcolors.WARNING}gyro noise:
{bcolors.OKBLUE}x: {bcolors.FAIL}{round(IMU.gyroNoise.x * RAD_TO_DEG, 2)} deg/s - {bcolors.OKBLUE}y: {bcolors.FAIL}{round(IMU.gyroNoise.y * RAD_TO_DEG, 2)} deg/s - {bcolors.OKBLUE}z: {bcolors.FAIL}{round(IMU.gyroNoise.z * RAD_TO_DEG, 2)} deg/s

{bcolors.WARNING}accel noise:
{bcolors.OKBLUE}x: {bcolors.FAIL}{round(IMU.accelNoise.x, 2)} m/s - {bcolors.OKBLUE}y: {bcolors.FAIL}{round(IMU.accelNoise.y, 2)} m/s - {bcolors.OKBLUE}z: {bcolors.FAIL}{round(IMU.accelNoise.z, 2)} m/s

{bcolors.WARNING}gyro bias:
{bcolors.OKBLUE}x: {bcolors.FAIL}{round(IMU.gyroBias.x * RAD_TO_DEG, 2)} deg/s - {bcolors.OKBLUE}y: {bcolors.FAIL}{round(IMU.gyroBias.y * RAD_TO_DEG, 2)} deg/s - {bcolors.OKBLUE}z: {bcolors.FAIL}{round(IMU.gyroBias.z * RAD_TO_DEG, 2)} deg/s

{bcolors.WARNING}accel bias:
{bcolors.OKBLUE}x: {bcolors.FAIL}{round(IMU.accelBias.x, 2)} m/s - {bcolors.OKBLUE}y: {bcolors.FAIL}{round(IMU.accelBias.y, 2)} m/s - {bcolors.OKBLUE}z: {bcolors.FAIL}{round(IMU.accelBias.z, 2)} m/s

{bcolors.WARNING}gyro debias:
{bcolors.OKBLUE}x: {bcolors.FAIL}{round(NAV.gyroscopeBias.x * RAD_TO_DEG, 2)} deg/s - {bcolors.OKBLUE}y: {bcolors.FAIL}{round(NAV.gyroscopeBias.y * RAD_TO_DEG, 2)} deg/s - {bcolors.OKBLUE}z: {bcolors.FAIL}{round(NAV.gyroscopeBias.z * RAD_TO_DEG, 2)} deg/s

{bcolors.WARNING}accel debias:
{bcolors.OKBLUE}x: {bcolors.FAIL}{round(NAV.accelBias.x, 2)} m/s - {bcolors.OKBLUE}y: {bcolors.FAIL}{round(NAV.accelBias.y, 2)} m/s - {bcolors.OKBLUE}z: {bcolors.FAIL}{round(NAV.accelBias.z, 2)} m/s

{bcolors.WARNING}------------------------------------------------------
{bcolors.ENDC}''')

if abs(rocket.positionInertial.y) < 1.5 and abs(rocket.positionInertial.z) > 3 and abs(rocket.positionInertial.z) < 5:
    if abs(rocket.velocityInertial.y) < 0.5 and abs(rocket.velocityInertial.z) < 0.5 and abs(rocket.velocityInertial.x) < 7:
        if abs(rocket.rotation_euler.y) < 5 * DEG_TO_RAD and abs(rocket.rotation_euler.z) < 5 * DEG_TO_RAD:

            print(f'''{bcolors.WARNING}------------------------------------------------------
{bcolors.OKBLUE}Rocket landed successfully!''')

inp = input(f'{bcolors.OKBLUE}show graphs?{bcolors.WARNING}\n')

if "Y" in inp.upper():
    
    ori_plot_points = ['time', 
    'ori_x_roll', 'ori_y_pitch', 'ori_z_yaw', 
    'ori_x_roll_sensed', 'ori_y_pitch_sensed', 'ori_z_yaw_sensed', 
    'actuator_output_y_pitch', 'actuator_output_z_yaw',
    'ori_x_roll_setpoint', 'ori_y_pitch_setpoint', 'ori_z_yaw_setpoint']
    # 'prograde_direction_x_roll', 'prograde_direction_y_pitch', 'prograde_direction_z_yaw']

    oriRate_plot_points = ['time', 
    'ori_x_roll_velocity', 'ori_y_pitch_velocity', 'ori_z_yaw_velocity', 
    'ori_x_roll_velocity_sensed', 'ori_y_pitch_velocity_sensed', 'ori_z_yaw_velocity_sensed', 
    'ori_x_roll_velocity_sensed_filtered', 'ori_y_pitch_velocity_sensed_filtered', 'ori_z_yaw_velocity_sensed_filtered']

    pos_plot_points = ['time', 
    'x_position', 'y_position', 'z_position', 
    'x_position_sensed', 'y_position_sensed', 'z_position_sensed']

    vel_plot_points = ['time', 
    'x_velocity', 'y_velocity', 'z_velocity', 
    'x_velocity_sensed', 'y_velocity_sensed', 'z_velocity_sensed']

    accel_plot_points=['time', 
    'x_acceleration', 'y_acceleration', 'z_acceleration', 
    'x_acceleration_sensed', 'y_acceleration_sensed', 'z_acceleration_sensed', 
    'x_acceleration_sensed_filtered', 'y_acceleration_sensed_filtered', 'z_acceleration_sensed_filtered']

    plot_ori = plotter.graph_from_csv(ori_plot_points)
    plot_oriRates = plotter.graph_from_csv(oriRate_plot_points)
    plot_position = plotter.graph_from_csv(pos_plot_points)
    plot_velocity = plotter.graph_from_csv(vel_plot_points)
    plot_accel = plotter.graph_from_csv(accel_plot_points)
    plot_all = plotter.graph_from_csv(data_list)
    
    # 3d position graph

    plt.figure(1)
    
    ax = plt.axes(projection='3d')
    ax.set_xlim3d(-maxPosition, maxPosition)
    ax.set_ylim3d(-maxPosition, maxPosition)
    ax.set_zlim3d(0, maxPosition)

    ax.scatter3D(plot_position[3], plot_position[2], plot_position[1], c=plot_position[3], cmap='Blues')
    ax.scatter3D(plot_position[6], plot_position[5], plot_position[4], c=plot_position[6], cmap='Greens')

    # orientation plot

    plt.figure(2)

    for index, dataPoint in enumerate(plot_ori):
        if index > 0:
            plt.plot(plot_ori[0], dataPoint, label=ori_plot_points[index])
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("various readings (deg)")

    plt.figure(3)

    for index, dataPoint in enumerate(plot_velocity):
        if index > 0:
            plt.plot(plot_velocity[0], dataPoint, label=vel_plot_points[index])
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("various readings m/s")

    plt.figure(4)

    for index, dataPoint in enumerate(plot_position):
        if index > 0:
            plt.plot(plot_position[0], dataPoint, label=pos_plot_points[index])
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("various readings m")

    plt.figure(5)

    for index, dataPoint in enumerate(plot_accel):
        if index > 0:
            plt.plot(plot_accel[0], dataPoint, label=accel_plot_points[index])
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("various readings m/s2")

    
    plt.figure(6)

    for index, dataPoint in enumerate(plot_oriRates):
        if index > 0:
            plt.plot(plot_oriRates[0], dataPoint, label=oriRate_plot_points[index])
    plt.legend()
    plt.xlabel("time")
    plt.ylabel("various readings deg/s")

    plt.show()

print(f'{bcolors.ENDC}')
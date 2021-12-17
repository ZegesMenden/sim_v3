from physics import *
from dataManagement import *
from controlMath import *
from motors import *

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

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

rocket.dryMass = 1.0
motor = rocketMotor(1000)

motor.add_motor(motorType.f15, "ascent")
motor.add_motor(motorType.f15, "landing")

rocket.mass = rocket.dryMass + motor.totalMotorMass

rocket.mmoi = vector3(0.0102, 0.0404203354, 0.0404203354)
rocket.rotation_euler = vector3(0 * DEG_TO_RAD, random.randint(-100, 100) / 200 * DEG_TO_RAD, random.randint(-100, 100) / 200 * DEG_TO_RAD)
rocket.rotation_quaternion = Quaternion().eulerToQuaternion(rocket.rotation_euler.x, rocket.rotation_euler.y, rocket.rotation_euler.z)
rocket.gravity = vector3(-9.83, 0.0, 0.0)

def getAngleFromDesiredAcceleration(desired_acceleration, thrust):
    return np.arcsin(desired_acceleration*rocket.mass/thrust)

ori_setpoint = flightPath()
ori_setpoint.loadFlightPath('flightPath.csv')

maxPosition = 0.0

global setpoint
setpoint = vector3(0.0, 0.0, 0.0)

fsf_test_z = FSF(12.36, 7.58)
fsf_test_y = FSF(12.36, 7.58)

rocket_TVC = TVC()

rocket_TVC.lever = 0.24
rocket_TVC.linkageRatio.z = 10
rocket_TVC.linkageRatio.y = 10

rocket_TVC.noise.y = 0.05 * DEG_TO_RAD
rocket_TVC.noise.z = 0.05 * DEG_TO_RAD

rocket_TVC.servoSpeed = 200

rocket_TVC.max.y = 3
rocket_TVC.max.z = 3
rocket_TVC.min.y = -3
rocket_TVC.min.z = -3

TVC_derolled = vector3( 0.0, 0.0, 0.0 )

rocket_TVC.position.y = random.randint(-100, 100) / 5000 * DEG_TO_RAD
rocket_TVC.position.z = random.randint(-100, 100) / 5000 * DEG_TO_RAD

rocket_TVC.offset.y = random.randint(-100, 100) / 5000 * DEG_TO_RAD
rocket_TVC.offset.z = random.randint(-100, 100) / 5000 * DEG_TO_RAD

time = 0.0
missionTime = 0.0
timeStep = 5000
simTime = 90

totalSteps = simTime * timeStep
currentStep = 0

sensorDelay = 1 / 500
lastSensor = 0.0

controlSpeed = 40
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

IMU.gyroBias = vector3(np.random.normal(0.0, 0.5, 1)[0] * DEG_TO_RAD, np.random.normal(0.0, 0.5, 1)[0] * DEG_TO_RAD, np.random.normal(0.0, 0.5, 1)[0] * DEG_TO_RAD)
IMU.accelBias = vector3(np.random.normal(0.0, 0.6, 1)[0], np.random.normal(0.0, 0.6, 1)[0], np.random.normal(0.0, 0.6, 1)[0])

IMU.gyroNoise = vector3(0.5 * DEG_TO_RAD, 0.5 * DEG_TO_RAD, 0.5 * DEG_TO_RAD)
IMU.accelNoise = abs(vector3(np.random.normal(0.2, 0.1, 1)[0], np.random.normal(0.2, 0.1, 1)[0], np.random.normal(0.2, 0.1, 1)[0]))

IMU.gyroScale = vector3(1000 * DEG_TO_RAD, 1000 * DEG_TO_RAD, 1000 * DEG_TO_RAD)
IMU.accelScale = vector3(40, 40, 40)

NAV.oriKF.set_gains(vector3(1,1,1), vector3(2,2,2))
NAV.accelKF.set_gains(vector3(4, 4, 4), vector3(8, 8, 8))

TVC_command = vector3(0.0, 0.0, 0.0)

controlVelocity = False
followSetpoint = False

NAV.orientation_euler = vector3(rocket.rotation_euler.x, rocket.rotation_euler.y + (random.randint(-100, 100) / 1000) * DEG_TO_RAD, rocket.rotation_euler.z + (random.randint(-100, 100) / 1000) * DEG_TO_RAD)
NAV.orientation_quat = NAV.orientation_quat.eulerToQuaternion(NAV.orientation_euler.x, NAV.orientation_euler.y, NAV.orientation_euler.z)

# user defined functions and variables
global apogee_sensed
apogee_sensed = 0.0
light_alt = 0.0

dv_estimate = 7.5
dv_actual = 0.0
vel_burn_start = 0.0
throttle = 0.0

def readSensors(time, dt):
    
    localRotation = rocket.rotation_quaternion.conj().rotateVector(rocket.rotationalVelocity)
    localAcceleration = rocket.rotation_quaternion.conj().rotateVector(rocket.accelerationInertial)
    IMU.readAccel(localAcceleration, time)
    IMU.readGyro(localRotation, time)

    if time < 0.9:
        NAV.measureDebias(IMU.accel, IMU.oriRates)
    else:
        NAV.debias()
        # NAV.accelOri(NAV.accelerationLocal)
    
    # if NAV.inFlight == False:
    #     NAV.oriKF.set_initial_value(vector3(0, 0, 0))
    #     NAV.accelKF.set_initial_value(vector3(0, 0, 0))
    if NAV.debiased:
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

    rocket.dataLogger.recordVariable("reaction_wheel_output_x_roll", 0.0)
    rocket.dataLogger.recordVariable("actuator_output_y_pitch", rocket_TVC.position.y * RAD_TO_DEG)
    rocket.dataLogger.recordVariable("actuator_output_z_yaw", rocket_TVC.position.z * RAD_TO_DEG)

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
global retroPeak
retroPeak = 0.0

def controlLoop():

    global apogee_sensed
    global burnAlt
    global setpoint
    global retroPeak

    # setpoint = vector3(0.0, 0.0, 0.0)

    if NAV.velocityInertial.x < 0 and apogee_sensed == 0.0:
        rocket.dryMass -= 0.025
        burnAlt = 0.66 * NAV.positionInertial.x
        apogee_sensed = NAV.positionInertial.x

    if apogee_sensed > 0 and NAV.positionInertial.x < burnAlt:
        motor.ignite("landing", time)

    if apogee_sensed > 0 and NAV.accelerationLocal.x > 25 and retroPeak == 0.0:
        retroPeak = time

    # setpoint = vector3(0, 0, 0)
    # print(round(rocket.velocityInertial.dir() * RAD_TO_DEG, 2))

    # if missionTime > 1 and missionTime < 1.5:
    #     setpoint = vector3(0, 5 * DEG_TO_RAD, 0)
    # if missionTime > 1.5 and missionTime < 2:
    #     setpoint = vector3(0, 0, 0)

    if missionTime < 4:
        ori_setpoint.getCurrentSetpoint(missionTime)
        setpoint = ori_setpoint.currentSetpoint * DEG_TO_RAD
    elif apogee_sensed > 0 and NAV.positionInertial.x < burnAlt and time < retroPeak + 1.5:
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
        pgrad.x = 0.0
        pgrad *= 0.5

        # setpoint = NAV.orientation_quat.rotateVector(pgrad)
        setpoint = pgrad
        setpoint.x = 0.0
    else:
        setpoint = vector3(0, 0, 0)

    # if time > retroPeak + 1 and NAV.positionInertial.x < burnAlt:
    #     setpoint = vector3(0, 0, 0)

    fsf_test_y.setpoint = setpoint.y
    fsf_test_z.setpoint = setpoint.z

    fsf_test_y.compute(NAV.orientation_euler.y, NAV.oriRates.y)
    fsf_test_z.compute(NAV.orientation_euler.z, NAV.oriRates.z)

    TVC_y = getAngleFromDesiredAcceleration(fsf_test_y.getOutput(), NAV.accelerationLocal.x)
    TVC_z = getAngleFromDesiredAcceleration(fsf_test_z.getOutput(), NAV.accelerationLocal.x)
    
    cr = np.cos(-NAV.orientation_euler.x)
    sr = np.sin(-NAV.orientation_euler.x)

    tvcy = TVC_y * cr - TVC_z * sr
    tvcz = TVC_y * sr + TVC_z * cr
    
    return vector3(0.0, tvcy, tvcz)

while time < simTime:
    time += dt

    if rocket.positionInertial.x > 0.01:
        missionTime += dt

    motor.update(time)
    rocket_TVC.calculateForces(motor.currentThrust)

    rocket.mass = rocket.dryMass + motor.totalMotorMass

    rocket.addLocalForce(rocket_TVC.acceleration)
    rocket.addLocalTorque(rocket_TVC.torque)
    rocket.addGlobalTorque(rocket.rotationalVelocity * -0.05)
    rocket.addGlobalForce(vector3(rocket.velocityInertial.x * -0.025, rocket.velocityInertial.z * -0.025, rocket.velocityInertial.z * -0.025))

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

    if rocket.positionInertial.x <= 0.05 and apogee == True and motor.currentThrust <= 0.1:
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

if abs(rocket.positionInertial.z) < 8 and abs(rocket.positionInertial.y) < 8:
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

    fig = plt.figure(7)

    ax = p3.Axes3D(fig)

    ax.set_xlim3d(-maxPosition, maxPosition)
    ax.set_ylim3d(-maxPosition, maxPosition)
    ax.set_zlim3d(0, maxPosition)
    
    # numDataPoints = len(plot_position)
    
    # def func(num, dataset, line):
    #     line.set_data([dataset[2], dataset[1]])
    #     line.set_3d_properties(dataset[0])
    #     return line
    

    # line = plt.plot(plot_position[3], plot_position[2], plot_position[1], lw=2, c='g')[0]

    # line_ani = animation.FuncAnimation(fig, func, frames=numDataPoints, fargs=(plot_position, line), interval=50, blit=False)

    def func(num, dataSet, line):
        # NOTE: there is no .set_data() for 3 dim data...
        xcur = dataSet[2][num] - 1
        numpog = 0
        for index, x in enumerate(dataSet[2]):
            if x >= xcur-0.1 and x <= xcur -0.1:
                numpog = index
        line.set_data(dataSet[0:2, num-20:num])    
        line.set_3d_properties(dataSet[2, num-20:num])    
        return line

    # # THE DATA POINTS
    t = np.array(plot_position[1]) # This would be the z-axis ('t' means time here)
    x = np.array(plot_position[2])
    y = np.array(plot_position[3])
    # dataSet = np.array([plot_position[3], plot_position[2], plot_position[1]])
    # numDataPoints = len(plot_position[0])
    dataSet = np.array([x, y, t])

    numDataPoints = len(t)
    print(numDataPoints)
    # NOTE: Can't pass empty arrays into 3d version of plot()
    line = plt.plot(dataSet[0], dataSet[1], dataSet[2], lw=2, c='g')[0] # For line plot
    
    # AXES PROPERTIES]
    # ax.set_xlim3d([limit0, limit1])
    ax.set_xlabel('y')
    ax.set_ylabel('z')
    ax.set_zlabel('x')
    ax.set_title('Trajectory of electron for E vector along [120]')
    
    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, func, frames=numDataPoints, fargs=(dataSet,line), interval=(time/numDataPoints), blit=False)
    # line_ani.save(r'AnimationNew.mp4')
    plt.show()

print(f'{bcolors.ENDC}')
from physics import *
from dataVisualisation import *
from dataManagement import *
from controlMath import *
from navigation import *
from motors import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

DEG_TO_RAD = np.pi / 180
RAD_DEG = 180 / np.pi

data_list = ['time','ori_x_roll','ori_y_pitch','ori_z_yaw','ori_x_roll_sensed','ori_y_pitch_sensed','ori_z_yaw_sensed','reaction_wheel_output_x_roll','actuator_output_y_pitch','actuator_output_z_yaw','reaction_wheel_output_x_roll_p','actuator_output_y_pitch_p','actuator_output_z_yaw_p','reaction_wheel_output_x_roll_i','actuator_output_y_pitch_i','actuator_output_z_yaw_i','reaction_wheel_output_x_roll_d','actuator_output_y_pitch_d','actuator_output_z_yaw_d','ori_x_roll_setpoint','ori_y_pitch_setpoint','ori_z_yaw_setpoint','ori_x_roll_velocity','ori_y_pitch_velocity','ori_z_yaw_velocity','ori_x_roll_velocity_sensed','ori_y_pitch_velocity_sensed','ori_z_yaw_velocity_sensed','x_position','y_position','z_position','x_position_sensed','y_position_sensed','z_position_sensed','x_velocity','y_velocity','z_velocity','x_velocity_sensed','y_velocity_sensed','z_velocity_sensed','resultant_velocity','resultant_velocity_sensed','x_acceleration','y_acceleration','z_acceleration','x_acceleration_sensed','y_acceleration_sensed','z_acceleration_sensed','x_acceleration_inertial','y_acceleration_inertial','z_acceleration_inertial','x_acceleration__inertial_sensed','y_acceleration__inertial_sensed','z_acceleration__inertial_sensed','resultant_acceleration','resultant_acceleration_sensed','thrust','mass']

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
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total:
        print()

posPlot = dataVisualiser()
posPlot.allDataDescriptions = data_list

oriPlot = dataVisualiser()
oriPlot.allDataDescriptions = data_list

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

rocket.dataLogger.addDataPoint("reaction_wheel_output_x_roll_p")
rocket.dataLogger.addDataPoint("actuator_output_y_pitch_p")
rocket.dataLogger.addDataPoint("actuator_output_z_yaw_p")

rocket.dataLogger.addDataPoint("reaction_wheel_output_x_roll_i")
rocket.dataLogger.addDataPoint("actuator_output_y_pitch_i")
rocket.dataLogger.addDataPoint("actuator_output_z_yaw_i")

rocket.dataLogger.addDataPoint("reaction_wheel_output_x_roll_d")
rocket.dataLogger.addDataPoint("actuator_output_y_pitch_d")
rocket.dataLogger.addDataPoint("actuator_output_z_yaw_d")

rocket.dataLogger.addDataPoint("ori_x_roll_setpoint")
rocket.dataLogger.addDataPoint("ori_y_pitch_setpoint")
rocket.dataLogger.addDataPoint("ori_z_yaw_setpoint")

rocket.dataLogger.addDataPoint("ori_x_roll_velocity")
rocket.dataLogger.addDataPoint("ori_y_pitch_velocity")
rocket.dataLogger.addDataPoint("ori_z_yaw_velocity")

rocket.dataLogger.addDataPoint("ori_x_roll_velocity_sensed")
rocket.dataLogger.addDataPoint("ori_y_pitch_velocity_sensed")
rocket.dataLogger.addDataPoint("ori_z_yaw_velocity_sensed")

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

rocket.dataLogger.addDataPoint("x_acceleration")
rocket.dataLogger.addDataPoint("y_acceleration")
rocket.dataLogger.addDataPoint("z_acceleration")

rocket.dataLogger.addDataPoint("x_acceleration_sensed")
rocket.dataLogger.addDataPoint("y_acceleration_sensed")
rocket.dataLogger.addDataPoint("z_acceleration_sensed")

rocket.dataLogger.addDataPoint("x_acceleration_inertial")
rocket.dataLogger.addDataPoint("y_acceleration_inertial")
rocket.dataLogger.addDataPoint("z_acceleration_inertial")

rocket.dataLogger.addDataPoint("x_acceleration__inertial_sensed")
rocket.dataLogger.addDataPoint("y_acceleration__inertial_sensed")
rocket.dataLogger.addDataPoint("z_acceleration__inertial_sensed")

rocket.dataLogger.addDataPoint("resultant_acceleration")
rocket.dataLogger.addDataPoint("resultant_acceleration_sensed")

rocket.dataLogger.addDataPoint("thrust")
rocket.dataLogger.addDataPoint("mass")

rocket.dataLogger.fileName = "data_out.csv"

rocket.dataLogger.initCSV(True, True)

rocket.dryMass = 0.6
motor = rocketMotor(1000)

motor.add_motor(motorType.e12, "ascent")
rocket.mass = rocket.dryMass + motor.totalMotorMass

rocket.mmoi = vector3(0.0602, 0.0404203354, 0.0404203354)
rocket.rotation_euler = vector3(0, random.randint(-100, 100) / 50 * DEG_TO_RAD, random.randint(-100, 100) / 50 * DEG_TO_RAD)
rocket.rotaiton_quaternion = Quaternion().eulerToQuaternion(rocket.rotation_euler.x, rocket.rotation_euler.y, rocket.rotation_euler.z)
rocket.gravity = vector3(-9.83, 0.0, 0.0)

maxPosition = 0.0

PID_ori_X = PID(0, 0, 0, 0, 0, False)
PID_ori_Y = PID(2, 0.2, 0.4, 0, 2, False)
PID_ori_Z = PID(2, 0.2, 0.4, 0, 2, False)

PID_position_Y = PID(5, 5, 0, 1, 999, True)
PID_position_Z = PID(5, 5, 0, 1, 999, True)

fsf_gains_Y = np.matrix([1, 2, 0.0, 0.0])
fsf_setpoint_Y = np.array([5.0, 0.0, 0.0, 0.0])

fsf_gains_Z = np.matrix([1, 2, 0.0, 0.0])
fsf_setpoint_Z = np.array([5.0, 0.0, 0.0, 0.0])

FSF_ori_Y = FSF(fsf_gains_Y, fsf_setpoint_Y, 0.01)
FSF_ori_Z = FSF(fsf_gains_Z, fsf_setpoint_Z, 0.01)

rocket_TVC = TVC()

rocket_TVC.lever = 0.24
rocket_TVC.linkageRatioZ = 4
rocket_TVC.linkageRatioY = 4

rocket_TVC.noiseY = 0
rocket_TVC.noiseZ = 0

rocket_TVC.servoSpeed = 250

rocket_TVC.maxY = 5
rocket_TVC.maxZ = 5
rocket_TVC.minY = -5
rocket_TVC.minZ = -5

TVC_derolled = vector3( 0.0, 0.0, 0.0 )

tvc_y = 0.0
tvc_z = 0.0

IMU = Sensor()

IMU.gyroscopeNoise = vector3(0.2, 0.2, 0.2)
IMU.gyroscopeBias = vector3(random.randint(40, 100) / 100 * positive_or_negative() * 0, random.randint(40, 100) / 100 * positive_or_negative() * 0, random.randint(40, 100) / 100 * positive_or_negative() * 0)

IMU.accelerometerNoise = vector3(0.4, 0.4, 0.4)
IMU.accelerometerOffset = vector3((random.randint(40, 100) / 100 * positive_or_negative()) * 0.05, (random.randint(40, 100) / 100 * positive_or_negative()) * 0.05, (random.randint(40, 100) / 100 * positive_or_negative()) * 0.05)

IMU.orientation_quat = rocket.rotaiton_quaternion

apogee = False

time = 0.0
timeStep = 1000
simTime = 30

totalSteps = simTime * timeStep
currentStep = 0
sensorDelay = 2 / timeStep
lastSensor = 0.0

PIDSpeed = 50
PIDDelay = 1 / PIDSpeed
lastPID = 0.0

CSVSpeed = 100
CSVDelay = 1 / CSVSpeed
lastCSV = 0.0

dt = 1 / timeStep

motor.ignite("ascent", time)

setpoint = 0

while time < simTime:
    printProgressBar(currentStep, totalSteps, prefix="% complete (time based)")
    currentStep += 1
    time += dt

    # if time > 0.5 and time < 1:
    #     setpoint += 10 * dt
    #     PID_ori_Z.setSetpoint(setpoint)
    #     PID_ori_Y.setSetpoint(setpoint)

    motor.update(time)
    rocket_TVC.calculateForces(motor.currentThrust)

    rocket.mass = rocket.dryMass + motor.totalMotorMass

    rocket.addForce(rocket_TVC.acceleration)
    rocket.addTorque(rocket_TVC.torque)
    rocket.addTorque(rocket.rotationalVelocity * -0.05)

    rocket_TVC.actuate(vector3(0.0, TVC_derolled.x, TVC_derolled.y ), dt)

    IMU.update(rocket.accelerationLocal, rocket.rotationalVelocity, rocket.gravity, dt)

    if time > lastPID + PIDDelay and rocket.accelerationLocal.x > 4:

        ### uncomment for position control PID

        # PID_position_Y.compute(IMU.positionInertial.y, PIDDelay)
        # PID_position_Z.compute(IMU.positionInertial.z, PIDDelay)

        # PID_ori_Y.setSetpoint(-PID_position_Y.output)
        # PID_ori_Z.setSetpoint(PID_position_Z.output)

        ### uncomment for PID

        # PID_ori_Y.compute(rocket.rotation_euler.y * RAD_TO_DEG, PIDDelay)
        # PID_ori_Z.compute(rocket.rotation_euler.z * RAD_TO_DEG, PIDDelay)

        ### uncommend for direct output PID

        # tvc_y = PID_ori_Y.output * DEG_TO_RAD
        # tvc_z = PID_ori_Z.output * DEG_TO_RAD

        ### uncomment for torque based PID

        # tvc_y = calculateAngleFromDesiredTorque(rocket_TVC.lever, IMU.accelerationLocal.x, rocket.mmoi.y, PID_ori_Y.output)
        # tvc_z = calculateAngleFromDesiredTorque(rocket_TVC.lever, IMU.accelerationLocal.x, rocket.mmoi.z, PID_ori_Z.output)

        ### uncomment for FSF

        FSF_ori_Y.compute(0.0, IMU.orientation_euler.y * RAD_DEG, PIDDelay)
        FSF_ori_Z.compute(0.0, IMU.orientation_euler.z * RAD_DEG, PIDDelay)

        tvc_y = calculateAngleFromDesiredTorque(rocket_TVC.lever, IMU.accelerationLocal.x, rocket.mmoi.y, FSF_ori_Y.output)
        tvc_z = calculateAngleFromDesiredTorque(rocket_TVC.lever, IMU.accelerationLocal.x, rocket.mmoi.z, FSF_ori_Z.output)

        TVC_derolled = rotate(tvc_y, tvc_z, -IMU.orientation_euler.x)

        lastPID = time
    # else:
    #     TVC_derolled = vector3(0.0, 0.0, 0.0)

    if time > lastCSV + CSVDelay:

        rocket.dataLogger.recordVariable("time", time)
        rocket.dataLogger.recordVariable("ori_x_roll", rocket.rotation_euler.x * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("ori_y_pitch", rocket.rotation_euler.y * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("ori_z_yaw", rocket.rotation_euler.z * RAD_TO_DEG)

        rocket.dataLogger.recordVariable("ori_x_roll_sensed", IMU.orientation_euler.x * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("ori_y_pitch_sensed", IMU.orientation_euler.y * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("ori_z_yaw_sensed", IMU.orientation_euler.z * RAD_TO_DEG)

        rocket.dataLogger.recordVariable("reaction_wheel_output_x_roll", PID_ori_X.output)
        rocket.dataLogger.recordVariable("actuator_output_y_pitch", rocket_TVC.positionY * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("actuator_output_z_yaw", rocket_TVC.positionZ * RAD_TO_DEG)

        rocket.dataLogger.recordVariable("reaction_wheel_output_x_roll_p", PID_ori_X.proportional)
        rocket.dataLogger.recordVariable("actuator_output_y_pitch_p", PID_ori_Y.proportional)
        rocket.dataLogger.recordVariable("actuator_output_z_yaw_p", PID_ori_Z.proportional)

        rocket.dataLogger.recordVariable("reaction_wheel_output_x_roll_i", PID_ori_X.integral)
        rocket.dataLogger.recordVariable("actuator_output_y_pitch_i", PID_ori_Y.integral)
        rocket.dataLogger.recordVariable("actuator_output_z_yaw_i", PID_ori_Z.integral)

        rocket.dataLogger.recordVariable("reaction_wheel_output_x_roll_d", -PID_ori_X.derivitive)
        rocket.dataLogger.recordVariable("actuator_output_y_pitch_d", -PID_ori_Y.derivitive)
        rocket.dataLogger.recordVariable("actuator_output_z_yaw_d", -PID_ori_Z.derivitive)

        rocket.dataLogger.recordVariable("ori_x_roll_setpoint", PID_ori_X.setPoint)
        rocket.dataLogger.recordVariable("ori_y_pitch_setpoint", PID_ori_Y.setPoint)
        rocket.dataLogger.recordVariable("ori_z_yaw_setpoint", PID_ori_Z.setPoint)

        rocket.dataLogger.recordVariable("ori_x_roll_velocity", rocket.rotationalVelocity.x * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("ori_y_pitch_velocity", rocket.rotationalVelocity.y * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("ori_z_yaw_velocity", rocket.rotationalVelocity.z * RAD_TO_DEG)

        rocket.dataLogger.recordVariable("ori_x_roll_velocity_sensed", IMU.oriRates.x * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("ori_y_pitch_velocity_sensed", IMU.oriRates.y * RAD_TO_DEG)
        rocket.dataLogger.recordVariable("ori_z_yaw_velocity_sensed", IMU.oriRates.z * RAD_TO_DEG)

        # rocket.dataLogger.recordVariable("ori_x_acceleration", rocket.rotationalAcceleration.x)####", rocket.)
        # rocket.dataLogger.recordVariable("ori_y_acceleration", rocket.rotationalAcceleration.y)####", rocket.)
        # rocket.dataLogger.recordVariable("ori_z_acceleration", rocket.rotationalAcceleration.z)####", rocket.)

        rocket.dataLogger.recordVariable("x_position", rocket.positionInertial.x)
        rocket.dataLogger.recordVariable("y_position", rocket.positionInertial.y)
        rocket.dataLogger.recordVariable("z_position", rocket.positionInertial.z)

        rocket.dataLogger.recordVariable("x_position_sensed", IMU.positionInertial.x)
        rocket.dataLogger.recordVariable("y_position_sensed", IMU.positionInertial.y)
        rocket.dataLogger.recordVariable("z_position_sensed", IMU.positionInertial.z)

        rocket.dataLogger.recordVariable("x_velocity", rocket.velocityInertial.x)
        rocket.dataLogger.recordVariable("y_velocity", rocket.velocityInertial.y)
        rocket.dataLogger.recordVariable("z_velocity", rocket.velocityInertial.z)

        rocket.dataLogger.recordVariable("x_velocity_sensed", IMU.velocityInertial.x)
        rocket.dataLogger.recordVariable("y_velocity_sensed", IMU.velocityInertial.y)
        rocket.dataLogger.recordVariable("z_velocity_sensed", IMU.velocityInertial.z)

        rocket.dataLogger.recordVariable("resultant_velocity", rocket.velocityInertial.y)
        rocket.dataLogger.recordVariable("resultant_velocity_sensed", rocket.velocityInertial.y)

        rocket.dataLogger.recordVariable("x_acceleration", rocket.accelerationLocal.x)
        rocket.dataLogger.recordVariable("y_acceleration", rocket.accelerationLocal.y)
        rocket.dataLogger.recordVariable("z_acceleration", rocket.accelerationLocal.z)

        rocket.dataLogger.recordVariable("x_acceleration_sensed", IMU.accelerationLocal.x)
        rocket.dataLogger.recordVariable("y_acceleration_sensed", IMU.accelerationLocal.y)
        rocket.dataLogger.recordVariable("z_acceleration_sensed", IMU.accelerationLocal.z)

        rocket.dataLogger.recordVariable("x_acceleration_inertial", rocket.accelerationInertial.x)
        rocket.dataLogger.recordVariable("y_acceleration_inertial", rocket.accelerationInertial.y)
        rocket.dataLogger.recordVariable("z_acceleration_inertial", rocket.accelerationInertial.z)

        rocket.dataLogger.recordVariable("x_acceleration__inertial_sensed", IMU.accelerationInertial.x)
        rocket.dataLogger.recordVariable("y_acceleration__inertial_sensed", IMU.accelerationInertial.y)
        rocket.dataLogger.recordVariable("z_acceleration__inertial_sensed", IMU.accelerationInertial.z)

        rocket.dataLogger.recordVariable("resultant_acceleration", rocket.accelerationInertial.z)
        rocket.dataLogger.recordVariable("resultant_acceleration_sensed", rocket.accelerationInertial.z)

        rocket.dataLogger.recordVariable("thrust", motor.currentThrust)
        rocket.dataLogger.recordVariable("mass", rocket.mass)

        rocket.dataLogger.saveData(False)

        lastCSV = time

    # # rocket.rotation_euler.x = 0.0
    # rocket.rotaiton_quaternion = Quaternion().eulerToQuaternion(0.0, rocket.rotation_euler.y, rocket.rotation_euler.z)

    rocket.update(dt)

    if abs(rocket.positionInertial.x) > maxPosition:
        maxPosition = abs(rocket.positionInertial.x)

    if abs(rocket.positionInertial.y) > maxPosition:
        maxPosition = abs(rocket.positionInertial.y)

    if abs(rocket.positionInertial.z) > maxPosition:
        maxPosition = abs(rocket.positionInertial.z)

    if rocket.velocityInertial.x < 0:
        apogee = True

    if rocket.positionInertial.x <= 0 and apogee == True:
        break

plot_position = posPlot.graph_from_csv(['time', 'x_position', 'y_position', 'z_position', 'x_position_sensed', 'y_position_sensed', 'z_position_sensed'])
# 'ori_x_roll_sensed', 'ori_y_pitch_sensed', 'ori_z_yaw_sensed',
ori_plot_points = ['time', 'ori_x_roll', 'ori_y_pitch', 'ori_z_yaw', 'actuator_output_y_pitch', 'actuator_output_z_yaw'] #, 'actuator_output_y_pitch_p', 'actuator_output_y_pitch_i', 'actuator_output_y_pitch_d', 'actuator_output_z_yaw_p', 'actuator_output_z_yaw_i', 'actuator_output_z_yaw_d' ]
plot_ori = oriPlot.graph_from_csv(ori_plot_points)
plt.figure(1)
ax = plt.axes(projection='3d')
ax.set_xlim3d(-maxPosition, maxPosition)
ax.set_ylim3d(-maxPosition, maxPosition)
ax.set_zlim3d(0, maxPosition)
# ax.plot3D(plot_position[3], plot_position[2], plot_position[1], 'green')

ax.scatter3D(plot_position[3], plot_position[2], plot_position[1], c=plot_position[3], cmap='Blues')
ax.scatter3D(plot_position[6], plot_position[5], plot_position[4], c=plot_position[6], cmap='Greens')

plt.figure(2)

for index, dataPoint in enumerate(plot_ori):
    if index > 0:
        plt.plot(plot_ori[0], dataPoint, label=ori_plot_points[index])
plt.legend()
plt.xlabel("time")
plt.ylabel("various readings (deg)")

plt.show()
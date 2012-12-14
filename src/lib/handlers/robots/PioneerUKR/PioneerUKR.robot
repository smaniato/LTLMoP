RobotName: # Robot Name
Pioneer_for_UKR

Type: # Robot type
PioneerUKR

ActuatorHandler: # Robot default actuator handler with default argument values
pioneerActuator()

DriveHandler: # Robot default drive handler with default argument values
differentialDrive(d=0.3)

InitHandler: # Robot default init handler with default argument values
pioneerInit(host="10.0.0.96", map_port=11008)

LocomotionCommandHandler: # Robot locomotion command actuator handler with default argument values
pioneerLocomotionCommand()

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
viconPose(host='10.0.0.102',port=800,x_VICON_name="spider06:spider06 <t-X>",y_VICON_name="spider06:spider06 <t-Y>",theta_VICON_name="spider06:spider06 <a-Z>")

SensorHandler: # Robot default sensor handler with default argument values
pioneerSensor()


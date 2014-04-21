DriveHandler: # Robot default drive handler with default argument values
differentialDrive()

InitHandler: # Robot default init handler with default argument values
SegwayInit()

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
SegwayLocomotionCommand()

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
viconPose(host='10.0.0.102',port=800,x_VICON_name="Nao:Nao <t-X>",y_VICON_name="Nao:Nao <t-Y>",theta_VICON_name="Nao:Nao <a-Z>")

RobotName: # Robot Name
BlueSegway

Type: # Robot type
Segway

SensorHandler: # Robot default sensor handler with default argument values
SegwaySensor()

ActuatorHandler: # Robot default actuator handler with default argument values
SegwayActuator()

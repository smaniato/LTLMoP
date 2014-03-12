RobotName: # Robot Name
Johnny5

Type: # Robot type
Johnny5

DriveHandler: # Robot default drive handler with default argument values
differentialDrive()

InitHandler: # Robot default init handler with default argument values
Johnny5Init(comPort='/dev/tty.usbserial-A600eIiI')

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
Johnny5LocomotionCommand()

MotionControlHandler: # Robot default motion control handler with default argument values
vectorController()

PoseHandler: # Robot default pose handler with default argument values
viconPose(host="10.0.0.102",port=800,x_VICON_name="Johnny5-fixed:Johnny5 <t-X>",y_VICON_name="Johnny5-fixed:Johnny5 <t-Y>",theta_VICON_name="Johnny5-fixed:Johnny5 <a-Z>")

SensorHandler: # Robot default sensor handler with default argument values
Johnny5Sensor()

ActuatorHandler: # Robot default actuator handler with default argument values
Johnny5Actuator()

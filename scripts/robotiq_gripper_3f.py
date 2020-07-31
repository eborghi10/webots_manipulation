import rospy

from sensor_msgs.msg import JointState

from controller import Motor, PositionSensor


class Robotiq3F(object):
  '''
    https://cyberbotics.com/doc/guide/gripper-actuators

    https://cyberbotics.com/doc/reference/motor
    https://cyberbotics.com/doc/reference/positionsensor

        Motor Name	          Position Sensor Name
    palm_finger_1_joint	    palm_finger_1_joint_sensor
    finger_1_joint_1	      finger_1_joint_1_sensor
    finger_1_joint_2	      finger_1_joint_2_sensor
    finger_1_joint_3	      finger_1_joint_3_sensor
    palm_finger_2_joint	    palm_finger_2_joint_sensor
    finger_2_joint_1	      finger_2_joint_1_sensor
    finger_2_joint_2	      finger_2_joint_2_sensor
    finger_2_joint_3	      finger_2_joint_3_sensor
    finger_middle_joint_1	  finger_middle_joint_1_sensor
    finger_middle_joint_2	  finger_middle_joint_2_sensor
    finger_middle_joint_3	  finger_middle_joint_3_sensor
  '''

  def __init__(self, robot, timestep):
    self.fingers = dict()
    self.set_finger(robot, 'finger_1', timestep, hasPalm=True)
    self.set_finger(robot, 'finger_2', timestep, hasPalm=True)
    self.set_finger(robot, 'finger_middle', timestep, hasPalm=False)

    self.js_pub = rospy.Publisher('robotiq_hands/left_hand/joint_states', JointState, queue_size=1)
    self.joint_names = [
      'finger_1_joint_1',
      'finger_1_joint_2',
      'finger_1_joint_3',
      'finger_2_joint_1',
      'finger_2_joint_2',
      'finger_2_joint_3',
      'finger_middle_joint_1',
      'finger_middle_joint_2',
      'finger_middle_joint_3',
      'palm_finger_1_joint',
      'palm_finger_2_joint',
    ]

    self.last_time = rospy.get_rostime()
    self.prev_pos = [0] * len(self.joint_names)

    self.js_msg = JointState()
    self.js_msg.name = self.joint_names
    self.js_msg.position = [0] * len(self.joint_names)
    self.js_msg.velocity = [0] * len(self.joint_names)
    self.js_msg.effort = [0] * len(self.joint_names)
    rospy.Timer(rospy.Duration(1/20.), self.js_callback)

  def set_finger(self, robot, name, timestep, hasPalm=False):
    finger = self.Finger(
        robot.getMotor('{}_joint_1'.format(name)),
        robot.getMotor('{}_joint_2'.format(name)),
        robot.getMotor('{}_joint_3'.format(name)),
        robot.getPositionSensor('{}_joint_1_sensor'.format(name)),
        robot.getPositionSensor('{}_joint_2_sensor'.format(name)),
        robot.getPositionSensor('{}_joint_3_sensor'.format(name)),
    )
    finger.enable_sensors(timestep)
    # Storing finger
    self.fingers[name] = finger

  def open(self):
    for finger in self.fingers:
      self.fingers[finger].open()

  def close(self):
    for finger in self.fingers:
      self.fingers[finger].close()

  def js_callback(self, event):
    curr_time = rospy.get_rostime()
    time_diff = (curr_time - self.last_time).to_sec()
    for k,v in enumerate(self.fingers):
      for index in xrange(3):
        curr_position = self.fingers[v].get_position(index)
        self.js_msg.position[k*3+index] = curr_position
        self.js_msg.velocity[k*3+index] = \
          ((curr_position - self.prev_pos[k*3+index]) / time_diff) if time_diff > 0 else 0.0
        self.prev_pos[k*3+index] = curr_position
    self.js_msg.header.stamp = curr_time
    self.js_pub.publish(self.js_msg)
    self.last_time = curr_time

##########################################################

  class Finger():
    def __init__(self, joint1, joint2, joint3, sensor1, sensor2, sensor3):
      self.joint_sensor = [
        Robotiq3F.JointSensor(joint1, sensor1),
        Robotiq3F.JointSensor(joint2, sensor2),
        Robotiq3F.JointSensor(joint3, sensor3),
      ]

    def enable_sensors(self, timestep):
      for sensor in self.joint_sensor:
        sensor.enable_sensor(timestep)

    def open(self):
      for motor in self.joint_sensor:
        motor.set_position(0.0)

    def close(self):
      for motor in self.joint_sensor:
        motor.set_position(0.85)

    def get_position(self, index):
      return self.joint_sensor[index].get_position()

  class JointSensor(object):

    def __init__(self, joint, sensor):
      self.joint = joint
      self.sensor = sensor

    def enable_sensor(self, timestep):
      self.sensor.enable(timestep)

    def set_position(self, position):
      self.joint.setPosition(position)

    def get_position(self):
      return self.sensor.getValue()

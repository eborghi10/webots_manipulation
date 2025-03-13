from controller import Robot

robot = Robot()

camera = robot.getDevice("camera")
camera.enable(10)

while robot.step(32) != -1:
    print("Hello World!")

from Robot import Robot

robot = Robot()
print("robot created")
terminate = False

while not terminate:
    terminate = bool(input("Terminate? (1/0)"))
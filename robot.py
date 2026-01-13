from positional_agent.robot import Robot

r = Robot()

r.nav.move(0.5)

while True:
  r._robot.sleep(2)
#   random_pos = [np.random.randint(1000, 2500), np.random.randint(-2500, -1000)]
#   print("Looking at:", random_pos)
#   print("Current pos:", r.nav.position, "Rotation:", r.nav.rotation)
#   r.nav.look_at(random_pos, speed=0.5)
#   # r.nav.move_to(random_pos, speed=0.5)

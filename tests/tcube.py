import time

from dirigo import units
from dirigo_thorlabs_stage.dcservo import PRM1Z8


rstage = PRM1Z8(
    serial_number="83838244",
    controller_kind="tcube"
)

# Test the Dirigo RotationStage API
dev_info = rstage.device_info
print(dev_info)

p1 = rstage.position
print("Current position:", p1.with_unit("deg", 4))
v1 = rstage.max_velocity
print("Max velocity:", v1.with_unit("deg/s"))

target_p = p1 + units.Angle("20 deg")
rstage.move_to(target_p)
time.sleep(0.5)
while rstage.moving:
    print("Stage is in motion")
    time.sleep(0.5)

p2 = rstage.position
print("Moved to:", p2.with_unit("deg",4), "; expected:", target_p.with_unit("deg",4))

# # With faster velocity & acceleration
# rstage.max_velocity = units.AngularVelocity("20 deg/s")
# rstage.acceleration = units.AngularAcceleration("20 deg/s^2")
# v2 = rstage.max_velocity
# print("Max velocity:", v2.with_unit("deg/s"))

# target_p = p2 + units.Angle("20 deg")
# rstage.move_to(target_p)
# time.sleep(0.5)
# while rstage.moving:
#     print("Stage is in motion")
#     time.sleep(0.5)

# p3 = rstage.position
# print("Moved to:", p3.with_unit("deg",4), "; expected:", target_p.with_unit("deg",4))


# Try constant velocity move
mv = units.AngularVelocity("-5 deg/s")
rstage.move_velocity(mv)
for _ in range(8):
    time.sleep(1)
    print("Current position:", rstage.position.with_unit("deg",4))
rstage.stop()

vr = rstage.max_velocity
print("Max velocity:", vr.with_unit("deg/s"))


rstage.close()
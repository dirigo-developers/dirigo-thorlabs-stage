import time

from dirigo import units
from dirigo.plugins.beam_attenuators import HalfWavePlateAttenuator
from dirigo_thorlabs_stage.dcservo import PRM1Z8


rstage = PRM1Z8(
    serial_number="83838244",
    controller_kind="tcube"
)

beam = HalfWavePlateAttenuator(
    min_transmission_position=units.Angle(0),
    rotation_stage=rstage
)

beam.set_fraction(0.5)
time.sleep(2)

f = beam.fraction

print(f"Current beam transmission fraction: {f :0.3f}, expected: 0.5")
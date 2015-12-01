# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1
test, 1

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
basicsim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
slurp_hospital.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
new_target, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p3
r5 = p2
r6 = p1
r1 = p6
closet = p12
r3 = p4
hall_w = p13, p14
mail_room = p7
r2 = p5
lounge = p8
hall_n = p15, p16
others = 
hall_c = p11

Spec: # Specification in structured English
Group targets is r1

Add to targets if and only if you are sensing new_target
Do resynthesize if and only if you are sensing new_target

If you are not activating resynthesize then visit all targets
If you are activating resynthesize then stay there


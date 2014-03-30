# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
take_footage, 1

CompileOptions:
convexify: False
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
SimulatedDipolar

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
TakeFootage.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
low_battery, 0
hd_full, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p3
r2 = p2
r3 = p1
Upload = p4
Charge = p5
others = 

Spec: # Specification in structured English
Group patrol is r1,r3

if you are not sensing hd_full then visit all patrol
do take_footage if and only if you are in r2
if you are sensing hd_full then go to Upload

# MAKE UPLOAD FACE DOWN
# REMOVE CONSTRAINT ON CHARGING STATION


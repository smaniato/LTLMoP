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
Segway

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
ConstSimple.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
low_battery, 1
hd_full, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
ChargingStation = p4
UploadStation = p1
AreaOfInterest = p5
others = 
UnconstrainedR2 = p2
UnconstrainedR1 = p3

Spec: # Specification in structured English
Group patrol is UnconstrainedR1, UnconstrainedR2

if you are sensing low_battery then go to ChargingStation
if you are not sensing low_battery and you are sensing hd_full then go to UploadStation
if you are not sensing low_battery and you are not sensing hd_full then visit all patrol

do take_footage if and only if you are in AreaOfInterest


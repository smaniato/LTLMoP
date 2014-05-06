# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
resynthesize, 1

CompileOptions:
convexify: True
parser: nltk
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: True
decompose: True

CurrentConfigName:
Basic

Customs: # List of custom propositions
_add_to_letters

OpenWorld: # List of OpenWorld propositions/correspondences
letters
letters->Offices

RegionFile: # Relative path of region description file
slurp_hospital.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
detectLetters, 1
doResynthesis, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
hallN = p15, p16
r5 = p2
r6 = p1
r1 = p6
closet = p12
r3 = p4
mailRoom = p7
hallC = p11
lounge = p8
r2 = p5
others = 
hallW = p13, p14
r4 = p3

Spec: # Specification in structured English
### Group & Correspondence definitions ###

# Letters that the robot already knows about.
Group Letters is empty

# Regions that each letter needs to end up in.
Group Offices is empty

# Regions to patrol if the robot is not carrying letters.
Group PatrolRooms is mailRoom, hallW, hallN

Letters correspond to Offices

### Mission tasks ###

# Deliver letters to their recipients' offices:
If you are sensing any Letters then go to the corresponding Offices

# If you are not carrying any letters, patrol the building:
If you are not sensing any Letters then visit each PatrolRoom

### Open-world settings ###
If you are sensing detectLetters then add to group Letters

Do resynthesize if and only if you are sensing doResynthesis


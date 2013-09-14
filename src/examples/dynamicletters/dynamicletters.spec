# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
deliver_letter1, 1
resynthesize, 1
deliver_letter2, 1

CompileOptions:
convexify: True
parser: nltk
fastslow: False
decompose: True
use_region_bit_encoding: True

Customs: # List of custom propositions
carrying_letter1
carrying_letter2

RegionFile: # Relative path of region description file
slurp_hospital.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
new_letter, 1
letter1, 1
letter2, 1


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
#### Group definitions ####

# Letters that we can detect
Group recipients is letter1, letter2

# Memory Propositions indicating that we are currently carrying a given letter
Group letterslot is carrying_letter1, carrying_letter2

# Regions that each letter needs to end up in.
Group letterdestination is r1, r2

# Actions that deliver each letter
Group letterdelivery is deliver_letter1, deliver_letter2

robot starts in mail_room with false
environment starts with false

# C(letter_slot) = {letters, letter_delivery}
letterslot correspond to recipients
letterslot correspond to letterdelivery

# C(lettery_delivery) = {letter_destination, letter slot}
letterdelivery correspond to letterdestination
letterdelivery correspond to letterslot


#### Spec-rewriting and resynthesis mechanics ####

# This is only triggered if we see a letter we haven't ever seen before
If you are sensing new_letter then add to group recipients and resynthesize

#### Letter delivery specification ####

# Keep track of what we're carrying
each letterslot is set on mail_room and the corresponding recipients and pick_up and reset on the corresponding letterdelivery

# Deliver only in the destination room, and only if you have that letter
do each letterdelivery if and only if you are in the corresponding letterdestination and you are activating the corresponding letterslot

# No spurious pickups
Do pick_up if and only if you are sensing any recipients

# Our goal is to get rid of all our letters
Infinitely often not any letterslot

# Go back to the mailroom if we have nothing else to do
If you are not activating any letterslot then go to mail_room


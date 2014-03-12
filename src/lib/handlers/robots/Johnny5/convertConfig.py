
"""
    8 lines of data for each servo, 16 servos for Johnny 5,
    so index of data ranges from 1-128.
    In each data set of a servo, only lines 4-8 are of interests:
    
    4: servo value at neutral position
    5: Min servo value
    6: Max servo value
    7: Min servo degree
    8: Max servo degree
    
    Generate a 2D array "config" in the following format:
    
    index: 0         1          2         3         4          5
    #servo(0) Neutral_ servo Min_servo Max_servo Min_degree Max_degree
    #servo(1) ...
    .
    .
    .
    
    .csv file is exported form Sequencer project
    Useful data start from second row
    Column 3:18 shows corresponding servo degree(servo #0-15)
    Column 35:50 shows corresponding servo time(servo #0-15)

    Generate servo commands in format: 
    #'Servo Num' + P'Servo Val' + T'Time in ms' + \r
    
    Between each step, sleep for maximum servo Time in that step sequence
"""

cfg = [data.strip('\r\n') for data in open('ConfigSSC32.cfg')]

# config is a 16x6 array, initialized with all 0
config = [[0 for i in range(5)] for j in range(16)]

for servo_index in range(16):
	config[servo_index] = map(int, cfg[8*servo_index+3:8*servo_index+8])
"""
for i in range(128):
    if i%8==3:
        config[i/8][0] = i/8
        config[i/8][1] = int(cfg[i])
    if i%8==4:
        config[i/8][2] = int(cfg[i])
    if i%8==5:
        config[i/8][3] = int(cfg[i])
    if i%8==6:
        config[i/8][4] = int(cfg[i])
    if i%8==7:
        config[i/8][5] = int(cfg[i])
#i+=1
"""
print(config)

move = [data.strip('\r\n') for data in open('TakeBow.csv')]
# Convert .csv file into 2D array "move"
for i in range(len(move)):
    move[i] = move[i].split(';')
#print(move)

#for i in range(16):
#print("self.johnny5Serial.write('#" + str(i) + " P" + str(config[i][1]) + " T" + str(1000) + " \\r')" )

for i in range(1,len(move)):
    # Servo num in column 3:18, corresponding Time in column 35:50
    for j in range(3,19):
        value = config[j-3][1]+(((float(move[i][j]))-config[j-3][3])/(config[j-3][4]-config[j-3][3])*(config[j-3][2]-config[j-3][1]))
        print("self.johnny5Serial.write('#" + str(j-3) + " P" + str(int(value)) + " T" + str(move[i][j-3+35]) + " \\r')" )
    # column 35:50 are corresponding servo time in ms
    # [a:b] goes from a to b-1, b not included
    print("time.sleep(" + str(int(max(move[i][35:51]))/1000) +")")







from controller import Robot, Camera, InertialUnit, DistanceSensor, PositionSensor, \
    DistanceSensor, Motor, Gyro, Accelerometer, Display, GPS, Keyboard
import matplotlib.pyplot as plt
import math, numpy, random, cv2
import sys, csv
from mode import mode
#import pathPlaning
from mapping import map,valnya, grid,grid_scale,delta,movementRobot,printmap,cartesianRobot
from imager import get_data_from_camera, lineproc

#initialization
robot = Robot()
timestep = int(robot.getBasicTimeStep())
kb = Keyboard()
kb.enable(timestep)

# activation camera
camera = robot.getDevice('camera')
camera.enable(timestep)
#camera.recognitionEnable(timestep)
width = int(camera.getWidth())
height = int(camera.getHeight())

# getting the position sensors
roki = robot.getDevice('left wheel sensor')
roka = robot.getDevice('right wheel sensor')
roki.enable(timestep)
roka.enable(timestep)

# enable imu
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

#enableGPS
gps = robot.getDevice('gps')
gps.enable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# values for robot in inch
# wheel_radius = 1.6 / 2.0
# wheel_circ = 2 * 3.14 * wheel_radius
# enc_unit = wheel_circ / 6.28

# variable
init = [0, 0]
goal = [3, 3]
RotEnc = [0, 0]
motor_posisi_awal = [0, 0]
#spekRobot
rRoda = 2.05
enc_unit = (2 * 3.18 * rRoda) / 6.28
max_speed = 4
d = 2.28
d_mid = d / 2.0
#Posisi Awal
x_init = 0
y_init = 0
koreksi = 0
pose = [0,0,0]
#jalan = True
simpanData = 0
putar = 0
maju = 0
xxx=0 #flag
#posisi sekarang
x = 0
y = 0
i = 0 
#posisi_sebelumnya
x_mobo_prev = 0
y_mobo_prev = 0
#exportDataVariable
xname = 1
extension = str('.jpg')


def get_direction():
    # maju = 1, mundur=2, kanan=3, kiri=4
    # jelaskan magsud dari bobot :
    imu_val = (imu.getRollPitchYaw()[2] * 180) / 3.14159
    if (imu_val <= -135 and imu_val >= -180) or (135 <= imu_val <= 180):
        dir = "North"
        bobot = [[25, 75, 100, 75], [1, 4, 3, 2]]
    elif imu_val <= -45 and imu_val > -135:
        dir = "West"
        bobot = [[75, 25, 75, 100], [3, 1, 4, 2]]
    elif 45 <= imu_val <= 135:
        dir = "East"
        bobot = [[75, 100, 75, 25], [4, 2, 3, 1]]
    elif (-45 < imu_val <= 0) or (0 <= imu_val < 45):
        dir = "South"
        bobot = [[100, 75, 25, 75], [2, 3, 1, 4]]
    return imu_val, bobot, dir
def cm(meters):
    return round(meters * 100)
def get_sen_jarak():
    # returns left, front, right sensors in inches
    return cm(depan.getValue()), cm(kanan.getValue()), cm(kiri.getValue())
def get_motor_pos():
    return roki.getValue(), roka.getValue()
def robot_movement(RotEnc, RotEnc_new, enc_unit, pose):
    motor_shift = [0,0]
    jarak = [0,0]
    for i in range(len(RotEnc_new)):  # menghitung jarak pergeseran robot
        motor_shift[i] = RotEnc_new[i] - RotEnc[i]  # menghitung pergeseran rotaty
        jarak[i] = motor_shift[i] * enc_unit  # menghitung jarak pergeseran robot
    v = (jarak[0] + jarak[1]) / 2
    w = (jarak[0] - jarak[1]) / 52
    pose[0] += v * math.cos(pose[2])
    pose[1] += v * math.sin(pose[2])
    pose[2] += w
    return pose

def getMidPoint(camImage):
    #convert to grayscale, gaussian blur filter, and threshold
    camImage = cv2.GaussianBlur(camImage,(9,9),cv2.BORDER_DEFAULT)
    #cv2.imwrite("../thresh.jpg",camImage)
    camImage = cv2.threshold(camImage,100,255,cv2.THRESH_BINARY_INV)      
    #erode for noise elimination,dilate to restore some eroded parts of image

def printdata(saveImage,ImageProc,saveRealPos,saveCSV):
    values=[]
    print(koreksi)
    rho = 0
    theta = 0
    values.append([filename,koreksi])
    if saveImage == 1:
        camera.saveImage(loc_save,10)
    if saveRealPos == 1:
        val_gps = gps.getValues()
        values.append(val_gps[0])
        values.append(val_gps[1]) 
    if ImageProc == 1:
        valuesCam,rho,theta = get_data_from_camera(robot,camera)
        #print(get_data_from_camera(robot,camera))
        values.append(valuesCam)
        print(values)
    if saveCSV == 1:
        file = open('data.csv','a')
        writer = csv.writer(file)
        writer.writerow(values)
        file.close()
    return rho,theta
    

print(xname)
print(mode)
        
while robot.step(timestep) != -1:
    filename = str(str(xname)+extension)
    loc_save = str("img/"+filename)
    counter = 0
    if mode == 0:
        if kb.getKey()==315:
            printdata(1,1,1,1)
            print("get data")
            print(xname)
            xname=xname+1
            while counter!=10000000:
                counter+=1
    
    else :
        #jalan = True
        if x == goal[0] and y == goal[1]:
            print("sampai")
            print("selesai",selesai)
            if not selesai :
                rho,theta = printdata(0,1,1,1)
                koreksi,motor = lineproc(mode,rho,theta,movementRobot[i-1][6],koreksi)
                leftMotor.setVelocity(motor[0])
                rightMotor.setVelocity(motor[1])
            if koreksi == 0 :
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)
                #Flag
                if not selesai :
                    printdata(1,1,1,1)
                    selesai = True
                    print("selesai", selesai)
                    sys.exit(0)
            print("koreksi ",koreksi)
            #saveExperimentData()
            
                
        else:
            selesai = False
            arah = get_direction()
            arahRobot = arah[0]
            # kalkulasi pergerakan robot
            RotEnc_new = get_motor_pos()
            pose = robot_movement(RotEnc, RotEnc_new, enc_unit, pose)
            RotEnc = RotEnc_new
            
            if koreksi == 1:
                print("koreksi maju")
                rho,theta = printdata(1,1,1,1)
                koreksi,motor = lineproc(mode,rho,theta,movementRobot[i-1][6],koreksi)
            elif koreksi == 2:
                print("koreksi mundur")
                rho,theta = printdata(1,1,1,1)
                koreksi,motor = lineproc(mode,rho,theta,movementRobot[i-1][6],koreksi)
            elif koreksi == 0 :
                #cek arah robot apakah sesuai
                if arahRobot >= movementRobot[i][1] - 0.5 and arahRobot <= movementRobot[i][1] + 0.5:    
                    simpanData = 0
                    #flag
                    if not maju :
                        print("maju")
                        putar = True
                        maju = True
                        printdata(1,1,1,1)
                    motor=[3,3]     
                    
                    # kalkulasi pergerakan robot
                    #RotEnc_new = get_motor_pos()
                    #pose = robot_movement(RotEnc, RotEnc_new, enc_unit, pose)
                    #RotEnc = RotEnc_new
                    
                    #update posisi mobo
                    x_mobo = math.floor(abs(pose[0]) / grid_scale) - x_mobo_prev
                    y_mobo = math.floor(abs(pose[1]) / grid_scale) - y_mobo_prev
                    
                
                    #jika robot pindah grid
                    if (x_mobo != 0):
                        #Update Robot Posisi Grid
                        x_mobo_prev = x_mobo + x_mobo_prev
                        y_mobo_prev = y_mobo + y_mobo_prev
                        x += (x_mobo * movementRobot[i][4]) + (y_mobo * movementRobot[i][4])
                        y += (x_mobo * movementRobot[i][5]) + (y_mobo * movementRobot[i][5])
                        #x += (x_mobo * cartesianRobot(movementRobot[i][1])[0]) + (y_mobo * cartesianRobot(movementRobot[i][1])[0])
                        #y += (x_mobo * cartesianRobot(movementRobot[i][1])[1]) + (y_mobo * cartesianRobot(movementRobot[i][1])[1])
        
                        map[x_init][y_init] = ' '
                        map[x][y] = '#'
                        print('updatemap')
                        x_init = x
                        y_init = y
                        i += 1
                        #print('pergerakan ke i= ', i)
                        print(pose)
                        printmap(map)
                        #reset pose untuk grid baru
                        pose = [0, 0, 0]
                        x_mobo_prev = 0
                        y_mobo_prev = 0
                        rho,theta = printdata(0,1,0,0)
                        koreksi,motor = lineproc(mode,rho,theta,movementRobot[i-1][6],koreksi)
                        if koreksi == 0:
                            print("tidak ada koreksi")
                            rho,theta = printdata(1,1,1,1)
                    
                #arah tidak sesuai PUTAR sampai arah sesuai
                else :
                    possible_left  = 180 + movementRobot[i][1] - arahRobot
                    possible_right = 180 - movementRobot[i][1] + arahRobot
                    
                    if (possible_right < possible_left): #belokKiri
                        motor = [-0.5,0.5]
                        sign = "belok kiri"
                    if(possible_right > possible_left): #belokKanan
                        motor = [0.5,-0.5]
                        sign = "belok kanan"
                    
                    #reset posisi    
                    #pose = [0, 0, 0]
                    #x_mobo_prev = 0
                    
                    #flag
                    if not putar :
                        print("putar")
                        putar = True
                        maju = False
                        print("L = ",possible_left)
                        print("R = ",possible_right)    
                        print(sign)
            
                                
            leftMotor.setVelocity(motor[0])
            rightMotor.setVelocity(motor[1])
                   
        xname=xname+1

import numpy,cv2

def get_data_from_camera(robot,camera):
    values=[]
    rho = []
    theta = []
    """
    Take an image from the camera device and prepare it for OpenCV processing:
    - convert data type,
    - convert to RGB format (from BGRA), and
    - rotate & flip to match the actual image.
    """
    img = camera.getImageArray()
    img = numpy.asarray(img, dtype=numpy.uint8)
    #img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.flip(cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE),1)
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imwrite("../img1.jpg",img)
    #return cv2.flip(img, 1)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = numpy.array([0, 93, 0])
    upper = numpy.array([225, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    #cv2.imshow("hsv",mask)
    #cv2.waitKey()
    #output = cv2.bitwise_and(img,img, mask= mask)
    #bluring or not
    edges = cv2.Canny(mask,10,150)
    lines = cv2.HoughLines(edges, 1, numpy.pi / 180, 80, None, 0, 0)
    print(lines)
    if lines is not None:
        for i in range(0, len(lines)):
            rho.append(lines[i][0][0])
            theta.append(lines[i][0][1])
            values.append([lines[i][0][0],lines[i][0][1]])
    return values,rho,theta

def lineproc(rho,theta,val,mode) :
    move = [0,0]
    koreksi = 0
    
    if mode == 2:
        if len(rho) or len(theta) :
            rhoVal = max(rho) - min(rho)
            print("rho selisih ",rhoVal)
            print("rho di map  ",val)
            if rhoVal<val :
                print("koreksi posisi")
                move = [0.5,0.5]
                koreksi = 1
        else :
            print("gak ada koreksi")
            koreksi = 0
          
    return koreksi,move
        

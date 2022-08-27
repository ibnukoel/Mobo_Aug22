import numpy,cv2

def get_data_from_camera(robot,camera):
    values=[]
    """
    Take an image from the camera device and prepare it for OpenCV processing:
    - convert data type,
    - convert to RGB format (from BGRA), and
    - rotate & flip to match the actual image.
    """
    img = camera.getImageArray()
    img = numpy.asarray(img, dtype=numpy.uint8)
    img = cv2.flip(cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE),1)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = numpy.array([0, 93, 0])
    upper = numpy.array([225, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    edges = cv2.Canny(mask,10,150)
    lines = cv2.HoughLines(edges, 1, numpy.pi / 180, 80, None, 0, 0)
    print(lines)
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            values.append([rho,theta])
    return values 

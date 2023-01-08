import qualipy
import cv2 as cv

import cv2
import numpy


def LAPV(img):
    """Implements the Variance of Laplacian (LAP4) focus measure
    operator. Measures the amount of edges present in the image.
    :param img: the image the measure is applied to
    :type img: numpy.ndarray
    :returns: numpy.float32 -- the degree of focus
    """
    return numpy.std(cv2.Laplacian(img, cv2.CV_64F)) ** 2

img = cv.imread("1b.jpg")
img2 = cv.imread("2b.jpg")
#img3 = cv.imread("3b.jpg")
img4 = cv.imread("7b.jpg")

print("1 ",LAPV(img))
print("2 ",LAPV(img2))
#print("3 ",LAPV(img3))
print("4 ",LAPV(img4))

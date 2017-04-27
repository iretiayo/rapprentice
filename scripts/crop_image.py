import cv2
import sys

def main():
	filename = '../demo_files0/rgb2.jpg'
	img = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
	cv2.imshow('select area',img)
	cv2.waitKey()
	cv2.destroyAllWindows()

main()
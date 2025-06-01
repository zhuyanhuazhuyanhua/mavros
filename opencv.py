import cv2
import pzybar.pyzbar as pyzbar

image = cv2.imread('qrcode.png')

decodedObjects = pyzbar.decode(image)

for obj in decodedObjects:
    print("Data:", obj.data.decode('utf-8'))
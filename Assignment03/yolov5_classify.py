import torch
import cv2


def yolo_classify(model, img):
    bboxes = model(img)
    print(bboxes.xywhn[0])


if __name__ == '__main__':
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='weights/best.pt')
    img = cv2.imread('data/test/2022-05-11_19_37_24.536665.jpg')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    yolo_classify(model, img)
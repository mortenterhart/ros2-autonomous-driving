import torch


def save_model(weights_file):
    model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_file)
    torch.save(model, './models/yolov5.pt')


if __name__ == '__main__':
    save_model('./weights/best.pt')
    print('Saved model with custom weights in ./weights/best.pt to ./models/yolov5.pt')

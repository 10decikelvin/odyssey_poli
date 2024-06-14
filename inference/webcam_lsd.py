import cv2
import tensorflow as tf
from utils import find_lines
import argparse
import cv2

SCORE_THRESH = 0.15 # 0 to 1, 0 is most responsive
DIST_THRESH = 20.0 # 0 to 20, how far away lines are detected from origin

parser = argparse.ArgumentParser('webcam inference')
parser.add_argument('--path', default='lsd.tflite', type=str, help='path to model')
parser.add_argument('--size', default=320, type=int, choices=[512, 320], help='input size')
args = parser.parse_args()

interpreter = tf.lite.Interpreter(model_path=args.path)

interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

cap = cv2.VideoCapture(0)
while True:
    DETECTED = False
    ret, frame = cap.read()
    frame = frame[400:-120]
    output = frame.copy()

    try:
        lines = find_lines(
            output, 
            interpreter, 
            input_details, output_details, 
            input_shape=[args.size, args.size], 
            score_thr=SCORE_THRESH, dist_thr=DIST_THRESH
        )

        for line in lines:
            x_start, y_start, x_end, y_end = [int(val) for val in line]
            cv2.line(output, (x_start, y_start), (x_end, y_end), [0,255,255], 10)
            
        cv2.imshow('frame', frame)
        cv2.imshow("output", output)
    except:
        print("No lines detected")
        pass

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
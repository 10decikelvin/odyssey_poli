import numpy as np
import cv2

def find_lines(image, interpreter, input_details, output_details, input_shape=[512, 512], score_thr=0.10, dist_thr=20.0):
    h, w, _ = image.shape
    h_ratio, w_ratio = h / input_shape[0], w / input_shape[1]

    resized_image = np.concatenate([
        cv2.resize(image, tuple(input_shape), interpolation=cv2.INTER_AREA), 
        np.ones([*input_shape, 1])
    ], axis=-1)
    
    batch_image = np.expand_dims(resized_image, axis=0).astype('float32')
    interpreter.set_tensor(input_details[0]['index'], batch_image)
    interpreter.invoke()

    pts, pts_score, vmap = [interpreter.get_tensor(detail['index'])[0] for detail in output_details]
    start, end = vmap[:,:,:2], vmap[:,:,2:]
    dist_map = np.sqrt(np.sum((start - end) ** 2, axis=-1))

    segments_list = [
        [x + disp_x_start, y + disp_y_start, x + disp_x_end, y + disp_y_end]
        for center, score in zip(pts, pts_score)
        if score > score_thr and dist_map[tuple(center)] > dist_thr
        for y, x in [center]
        for disp_x_start, disp_y_start, disp_x_end, disp_y_end in [vmap[y, x, :]]
    ]

    lines = 2 * np.array(segments_list)
    lines[:, [0, 2]] *= w_ratio
    lines[:, [1, 3]] *= h_ratio

    return lines
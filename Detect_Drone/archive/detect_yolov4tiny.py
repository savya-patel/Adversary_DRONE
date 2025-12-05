#!/usr/bin/env python3
"""
Lightweight YOLOv4-tiny inference script using OpenCV DNN.

Usage examples:
  # Run webcam (0) with default weights/cfg in repo
  python detect_yolov4tiny.py --source 0

  # Run video or image
  python detect_yolov4tiny.py --source /path/to/video.mp4 --weights ../yolov4tiny/yolov4-tiny-custom_best.weights --cfg ../yolov4tiny/yolov4-tiny-custom.cfg --names ../yolov4tiny/obj.names

Notes:
- This uses OpenCV's DNN module (supports CUDA if OpenCV was built with CUDA).
- It is intentionally small and easy to adapt to your project.
"""

import argparse
import time
from pathlib import Path
import cv2
import numpy as np


def load_names(names_path: Path):
    if not names_path.exists():
        return [str(i) for i in range(80)]
    with open(names_path, 'r') as f:
        return [ln.strip() for ln in f.readlines() if ln.strip()]


def make_net(cfg: Path, weights: Path, device: str):
    net = cv2.dnn.readNetFromDarknet(str(cfg), str(weights))
    # Prefer CUDA if requested and available
    if device.lower().startswith('cuda'):
        try:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            print('Using OpenCV DNN CUDA backend')
        except Exception as e:
            print('CUDA backend requested but not available in this OpenCV build:', e)
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    else:
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    return net


def detect_frame(net, names, frame, conf_thresh=0.25, nms_thresh=0.45, input_size=416):
    h0, w0 = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (input_size, input_size), swapRB=True, crop=False)
    net.setInput(blob)
    layer_names = net.getLayerNames()
    out_names = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    layer_outputs = net.forward(out_names)

    boxes = []
    confidences = []
    class_ids = []

    for output in layer_outputs:
        for detection in output:
            scores = detection[5:]
            class_id = int(np.argmax(scores))
            conf = float(scores[class_id]) * float(detection[4]) if len(scores) > 0 else float(detection[4])
            if conf > conf_thresh:
                cx = int(detection[0] * w0)
                cy = int(detection[1] * h0)
                w = int(detection[2] * w0)
                h = int(detection[3] * h0)
                x = int(cx - w/2)
                y = int(cy - h/2)
                boxes.append([x, y, w, h])
                confidences.append(conf)
                class_ids.append(class_id)

    idxs = cv2.dnn.NMSBoxes(boxes, confidences, conf_thresh, nms_thresh)
    results = []
    if len(idxs) > 0:
        for i in idxs.flatten():
            x, y, w, h = boxes[i]
            results.append((class_ids[i], confidences[i], (x, y, x+w, y+h)))
    return results


def draw_results(frame, results, names):
    for cls, conf, (x1, y1, x2, y2) in results:
        label = names[cls] if cls < len(names) else str(cls)
        color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        text = f'{label} {conf:.2f}'
        cv2.putText(frame, text, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--weights', type=str, default='../yolov4tiny/yolov4-tiny-custom_best.weights')
    p.add_argument('--cfg', type=str, default='../yolov4tiny/yolov4-tiny-custom.cfg')
    p.add_argument('--names', type=str, default='../yolov4tiny/obj.names')
    p.add_argument('--source', type=str, default='0', help='camera index, video file, or image')
    p.add_argument('--conf', type=float, default=0.25)
    p.add_argument('--nms', type=float, default=0.45)
    p.add_argument('--size', type=int, default=416, help='network input size (416 or 608)')
    p.add_argument('--device', type=str, default='cpu', help='cpu or cuda')
    p.add_argument('--view', action='store_true', help='show window')
    p.add_argument('--save', type=str, default='', help='path to save output video (mp4)')
    return p.parse_args()


def main():
    args = parse_args()
    weights = Path(args.weights).expanduser()
    cfg = Path(args.cfg).expanduser()
    names = Path(args.names).expanduser()
    source = args.source

    if source.isnumeric():
        source = int(source)

    class_names = load_names(names)
    net = make_net(cfg, weights, args.device)

    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        # try reading image
        img = cv2.imread(str(source)) if isinstance(source, (str, Path)) else None
        if img is None:
            raise SystemExit(f'Could not open source: {args.source}')
        frames = [img]
    else:
        frames = None

    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    try:
        while True:
            if frames is None:
                ret, frame = cap.read()
                if not ret:
                    break
            else:
                if not frames:
                    break
                frame = frames.pop(0)

            t0 = time.time()
            results = detect_frame(net, class_names, frame, conf_thresh=args.conf, nms_thresh=args.nms, input_size=args.size)
            t1 = time.time()
            fps = 1.0 / (t1 - t0) if (t1 - t0) > 0 else 0.0
            draw_results(frame, results, class_names)
            cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            if args.view:
                cv2.imshow('yolov4-tiny', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if args.save:
                if writer is None:
                    h, w = frame.shape[:2]
                    writer = cv2.VideoWriter(args.save, fourcc, 20, (w, h))
                writer.write(frame)

            # For single-image input, frames list empties and loop exits
            if frames is not None and not frames:
                break

    finally:
        if writer:
            writer.release()
        if frames is None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

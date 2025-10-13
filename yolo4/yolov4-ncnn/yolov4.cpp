#include "net.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>

#define YOLOV4_TINY 1

ncnn::Net yolov4;
int target_size;
const float mean_vals[3] = {0, 0, 0};
const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};

const char* class_names[] = {
    "background", "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
    "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush"
};

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

static int detect_yolov4(const cv::Mat& bgr, std::vector<Object>& objects)
{
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR, bgr.cols, bgr.rows, target_size, target_size);

    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = yolov4.create_extractor();
    ex.input("data", in);

    ncnn::Mat out;
    ex.extract("output", out);

    objects.clear();
    for (int i = 0; i < out.h; i++)
    {
        const float* values = out.row(i);

        Object object;
        object.label = values[0];
        object.prob = values[1];
        object.rect.x = values[2] * img_w;
        object.rect.y = values[3] * img_h;
        object.rect.width = values[4] * img_w - object.rect.x;
        object.rect.height = values[5] * img_h - object.rect.y;

        objects.push_back(object);
    }

    return 0;
}

static void draw_objects(cv::Mat& bgr, const std::vector<Object>& objects)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        const Object& obj = objects[i];

        cv::rectangle(bgr, obj.rect, cv::Scalar(255, 0, 0));

        char text[256];
        sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > bgr.cols)
            x = bgr.cols - label_size.width;

        cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), -1);

        cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
}

int main(int argc, char** argv)
{
    // --- 1. LOAD THE MODEL ---
#if YOLOV4_TINY
    yolov4.load_param("yolov4-tiny-opt.param");
    yolov4.load_model("yolov4-tiny-opt.bin");
    target_size = 416;
#else
    yolov4.load_param("yolov4-opt.param");
    yolov4.load_model("yolov4-opt.bin");
    target_size = 608;
#endif

    // --- 2. OPEN THE CAMERA ---
    cv::VideoCapture cap(0); // Use 0 for the default camera
    if (!cap.isOpened())
    {
        fprintf(stderr, "ERROR: Could not open camera\n");
        return -1;
    }

    cv::Mat frame;
    while (true)
    {
        // --- 3. CAPTURE A FRAME ---
        cap.read(frame);
        if (frame.empty())
        {
            fprintf(stderr, "ERROR: Captured empty frame\n");
            break;
        }

        // --- 4. DETECT AND DRAW OBJECTS ---
        std::vector<Object> objects;
        detect_yolov4(frame, objects);
        draw_objects(frame, objects);

        // --- 5. DISPLAY THE FRAME ---
        cv::imshow("YOLOv4 Live Detection", frame);

        // --- 6. EXIT ON 'q' KEY PRESS ---
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}

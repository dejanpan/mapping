/* This file was inspired by RGBDSLAM.
  */

//#ifdef USE_SIFT_GPU
#include "RGBD_registration/sift_gpu_wrapper.h"
#include <GL/gl.h>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include "RGBD_registration/parameter_server.h"

using namespace cv;

SiftGPUWrapper* SiftGPUWrapper::instance = NULL;

SiftGPUWrapper::SiftGPUWrapper() {
    error = false;
    siftgpu = new SiftGPU();

#if defined(SIFT_GPU_MODE) and SIFT_GPU_MODE == 1
    char method[] = {"-cuda"};
#elif defined(SIFT_GPU_MODE) and SIFT_GPU_MODE == 2
    char method[] = {"-glsl"};
#endif

    //sprintf(method, "%s", ParameterServer::instance()->get<bool>("cuda_available") ? "-cuda" : "-glsl");
    int max_features = ParameterServer::instance()->get<int>("max_keypoints");
    char max_feat_char[10];
    sprintf(max_feat_char, "%d", max_features);
    //ROS_INFO("Max_feat_char %s", max_feat_char);
    char subpixelKey[] = {"-s"};
    char subpixelValue[] = {"1"};
    char max_flag[] = {"-tc2"};
    //char verbosity[] = {"-v0"};//nothing but errors
    //char * argv[] = {method, "-t", "10", subpixelKey, subpixelValue, max_flag, max_feat_char};
    char first_octave[] = {"-fo"};
    char first_octave_val[] = {"-1"};
    char * argv[] = {method,  subpixelKey, subpixelValue, max_flag, max_feat_char, first_octave, first_octave_val};//, "-t", "0.005"};
    siftgpu->ParseParam(7, argv);

    if (siftgpu->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED) {
        ROS_ERROR("Can't create OpenGL context! SiftGPU cannot be used.");
        error = true;
    }

    data = (unsigned char*) malloc(imageWidth * imageHeight);

    isMatcherInitialized = false;
}

SiftGPUWrapper::~SiftGPUWrapper() {
    delete siftgpu;
    if (isMatcherInitialized) delete matcher;
    instance = NULL;
    if (data != NULL) {
        free(data);
        data = NULL;
    }
}

void SiftGPUWrapper::destroyInstance() {
    delete instance;
}
SiftGPUWrapper* SiftGPUWrapper::getInstance() {
    if (instance == NULL) {
        ROS_INFO("Create Instance");
        instance = new SiftGPUWrapper();
    }
    return instance;
}

void SiftGPUWrapper::detect(const cv::Mat& image, cv::vector<cv::KeyPoint>& keypoints, std::vector<float>& descriptors, const Mat& mask) const {
    if (error) {
        keypoints.clear();
        ROS_FATAL("SiftGPU cannot be used. Detection of keypoints failed");
    }

    //get image
    cvMatToSiftGPU(image, data);

    int num_features = 0;
    SiftGPU::SiftKeypoint* keys = 0;

    ROS_DEBUG("SIFTGPU: cols: %d, rows: %d", image.cols, image.rows);
    if (siftgpu->RunSIFT(image.cols, image.rows, data, GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
        num_features = siftgpu->GetFeatureNum();
        ROS_INFO("Number of features found: %i", num_features);
        keys = new SiftGPU::SiftKeypoint[num_features];
        descriptors.resize(128 * num_features);
        //descriptors = new float[128 * num_features];
        siftgpu->GetFeatureVector(&keys[0], &descriptors[0]);
    } else {
        ROS_WARN("SIFTGPU->RunSIFT() failed!");
    }

    //copy to opencv structure
    keypoints.clear();
    for (int i = 0; i < num_features; ++i) {
        KeyPoint key(keys[i].x, keys[i].y, keys[i].s, keys[i].o);
        keypoints.push_back(key);
    }

    //	FILE *fp = fopen("bla.pgm", "w");
    //	WritePGM(fp, data, image.cols, image.rows);
    //	fclose(fp);

}

int SiftGPUWrapper::match(
        const std::vector<float>& descriptors1,
        int num1,
        const std::vector<float>& descriptors2,
        int num2,
        std::vector<cv::DMatch>* matches) {
    if (!isMatcherInitialized)
        initializeMatcher();

    float sumDistances = 0;

    matcher->SetDescriptors(0, num1, &descriptors1[0]);
    matcher->SetDescriptors(1, num2, &descriptors2[0]);

    int (*match_buf)[2] = new int[num1][2];
    int number = matcher->GetSiftMatch(num1, match_buf, 0.9, 0.9);

    if (matches->size() != 0) {
        ROS_WARN("Clearing matches vector!");
        matches->clear();
    }

    cv::DMatch match;
    int counter = 0;

    for (int i = 0; i < number; i++) {
        match.queryIdx = match_buf[i][0];
        match.trainIdx = match_buf[i][1];

        //only use matches with indices != 0 (opengl context problem may sometimes happen)
        if (match.queryIdx == 0 || match.trainIdx == 0) {
            counter++;
        }

        if (counter > 0.5 * number) {
            matches->clear();
            sumDistances = 0;
            ROS_ERROR("Matches bad due to context error");
            break;
        }

        float sum = 0;
        for (int j = 0; j < 128; j++) {
            float a = descriptors1[match.queryIdx * 128 + j] - descriptors2[match.trainIdx * 128 + j];
            sum += a * a;
        }

        match.distance = sqrt(sum);
        sumDistances += match.distance;
        matches->push_back(match);
        ROS_DEBUG("Matched Features %d and %d with distance of %f. Sum: %f", match.queryIdx, match.trainIdx, match.distance, sumDistances);
    }

    delete[] match_buf;

    return sumDistances;
}

void SiftGPUWrapper::initializeMatcher() {
    matcher = CreateNewSiftMatchGPU(4096);
    if (!matcher->VerifyContextGL()) {
        ROS_FATAL("Can't create OpenGL context! SiftGPU Matcher cannot be used.");
        error = true;
        return;
    }
    ROS_INFO("matcher - ok");
    isMatcherInitialized = true;
}

void SiftGPUWrapper::cvMatToSiftGPU(const Mat& image,
        unsigned char* siftImage) const {
    Mat tmp;
    image.convertTo(tmp, CV_8U);
    for (int y = 0; y < tmp.rows; ++y) {
        for (int x = 0; x < tmp.cols; ++x) {
            siftImage[y * tmp.cols + x] = tmp.at<unsigned char> (y, x);
        }
    }
}

void SiftGPUWrapper::writePGM(FILE *fp, unsigned char* data, int width, int height)
{
    int val;
    fprintf(fp, "P5\n%d %d\n255\n", width, height);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            val = (int) (/*255.0 */data[y * width + x]);
            if (x == 0) val = 255;
            if (y == 0) val = 255;
            fputc(MAX(0, MIN(255, val)), fp);
        }
    }
}

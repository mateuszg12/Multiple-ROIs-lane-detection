#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

#include <include/def.hpp>
#include <include/ids.hpp>
#include <include/lanedetector.hpp>
#include <include/stoplights.hpp>
#include <include/sharedmemory.hpp>
#include <pthread.h>

#ifdef FUNCTION_TIMING
#define INIT_TIMER auto start1 = std::chrono::high_resolution_clock::now();
#define START_TIMER  start1 = std::chrono::high_resolution_clock::now();
#define STOP_TIMER(name)  std::cout << "RUNTIME of " << name << ": " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-start1).count() << " us " << std::endl;
#else
#define INIT_TIMER
#define START_TIMER
#define STOP_TIMER(name)
#endif

#ifdef FPS_COUNT
#define INIT_TIMER2 auto start2 = std::chrono::high_resolution_clock::now();
#define START_TIMER2  start2 = std::chrono::high_resolution_clock::now();
#define TIMER2_DIFF ((double)(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()-start2).count()))
#endif

// Custom handlers
IDS ids;
LaneDetector lanedetector;
StopLightDetector lightDetector;
SharedMemory shm_lane_points(50002, 5000);
SharedMemory shm_usb_to_send(50003, 32);
SharedMemory shm_watchdog(50004, 32);

pthread_cond_t algorithm_signal = PTHREAD_COND_INITIALIZER;
pthread_mutex_t algorithm_signal_mutex = PTHREAD_MUTEX_INITIALIZER;

// Frame for cv::Size info
cv::Mat frame_ref(CAM_RES_Y, CAM_RES_X, CV_8UC1);

// Functions declarations
void update_trackbar(int, void*);
void update_params_1(int, void*);
void *camera_thread(void *);

int main()
{
#ifdef FUNCTION_TIMING
    INIT_TIMER
#endif
#ifdef FPS_COUNT
    INIT_TIMER2
#endif
    // Threshold variables
    int thresh_h_low = 30;
    int thresh_h_high = 36;
    int thresh_l_low = 0;
    int thresh_l_high = 220;
    int thresh_s_low = 200;
    int thresh_s_high = 255;
    int thresh_hist = 40;
    int thresh_stop = 50;

#ifndef VID_MODE
#ifndef IDS_MODE
    // Camera init
    cv::VideoCapture camera;
    camera.open(CAMERA_INDEX, cv::CAP_V4L2);

    // Check if camera is opened propely
    if (!camera.isOpened())
    {
        std::cout << "Error could not open camera on port: " << CAMERA_INDEX << std::endl
                  << "Closing app!" << std::endl;

        camera.release();
        return 0;
    }
    else
    {
        // Set resolution
        camera.set(cv::CAP_PROP_FRAME_WIDTH, CAM_RES_X);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_RES_Y);

        // Set coding type of frames (Kurokesu)
        camera.set(cv::CAP_PROP_CONVERT_RGB, true);
    }
#else
    ids.create_trackbars();
    pthread_mutex_init(&algorithm_signal_mutex,NULL);
    ids.initialize_camera();
    //ids.setting_auto_params();
    //ids.change_params();

    int x = 0;
    pthread_t frame_thread;
    if (pthread_create(&frame_thread, NULL, camera_thread, &x))
    {
        std::cout<<"Error creating thread"<<std::endl;
    }
    else
    {
         std::cout << "Thread started" << std::endl;
    }
#endif
#else
#ifndef IDS_MODE
    // Video read init
    cv::VideoCapture video("/home/mateusz/Videos/sample_selfie/output_1.mp4");
    // Check
    if(!video.isOpened())
    {
        std::cout << "Error opening video stream or file!"
                  << "Closing app!" << std::endl;
        return 0;
    }
#endif
#endif

    int metoda = 1;
    int typ = 1;
    int blok_i = 25;
    int blok = blok_i*2+1;
    int c = 15;

    // Init shm
    shm_lane_points.init();
    shm_usb_to_send.init();
    shm_watchdog.init();

    // Declarations of cv::Mat
    cv::Mat frame(CAM_RES_Y, CAM_RES_X, CV_8UC3);
    cv::Mat frame_hls(CAM_RES_Y, CAM_RES_X, CV_8UC3);
    cv::Mat frame_blur(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_bird(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    std::vector<cv::Mat> frame_split_vec(3);
    cv::Mat frame_h(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_l(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_s(CAM_RES_Y, CAM_RES_X, CV_8UC1);
#ifdef ADAPTIVE_MODE
    cv::Mat frame_l_adapt(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_s_adapt(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_l_morph(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_s_morph(CAM_RES_Y, CAM_RES_X, CV_8UC1);
#else
    cv::Mat frame_binary_h(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_binary_l(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_binary_s(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_and(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_morph(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_sobel(CAM_RES_Y, CAM_RES_X, CV_8UC1);
#endif
    cv::Mat frame_data(CAM_RES_Y, CAM_RES_X, CV_8UC3);
    cv::Mat frame_bird_inverse(CAM_RES_Y, CAM_RES_X, CV_8UC1);
    cv::Mat frame_settings(1, 580, CV_8UC1);

#ifdef STOP_L_MODE
    //cv::Mats used in stoplight algorythm
    cv::Mat old_frame(CAM_RES_Y,CAM_RES_X,CV_8UC1);
    cv::Mat display;
    cv::Mat diffrence(CAM_RES_Y,CAM_RES_X,CV_8UC1);

#endif

#ifdef TEST_SHM
    cv::Mat frame_test_shm(CAM_RES_Y, CAM_RES_X, CV_8UC3);
#endif

// Creations of windows
#ifdef DEBUG_MODE
    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Camera", 0, 0);
    cv::namedWindow("Bird Eye", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Bird Eye", 705, 0);

#ifndef ADAPTIVE_MODE
    cv::namedWindow("Morph and", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Morph and", 1300, 0);
    cv::namedWindow("Sobel", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Sobel", 0, 410);
#else
    cv::namedWindow("L morph", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("L morph", 1340, 410);
    cv::namedWindow("S morph", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("S morph", 0, 0);
#endif

    cv::namedWindow("Data inverse", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Data inverse", 705, 410);
    cv::namedWindow("Settings", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Settings", 0, 410);

#ifdef VERBOSE_MODE
#ifdef ADAPTIVE_MODE
    cv::namedWindow("Blur", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Blur", 1300, 100);
    cv::namedWindow("L", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("L", 1300, 200);
    cv::namedWindow("S", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("S", 1300, 300);
    cv::namedWindow("Binary h", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Binary h", 1300, 400);
    cv::namedWindow("Binary s", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Binary s", 1300, 500);
    cv::namedWindow("AND", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("AND", 1300, 600);
#else
    cv::namedWindow("H", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("H", 1300, 200);
    cv::namedWindow("S", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("S", 1300, 300);
#endif
    cv::namedWindow("Data", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("Data", 1300, 700);
#endif

#ifdef TEST_SHM
    cv::namedWindow("SHM TEST", cv::WINDOW_AUTOSIZE);
    cv::moveWindow("SHM TEST", 1300, 700);
#endif
#endif

    // Bird Eye first calculation
    lanedetector.calculate_bird_var(frame_ref);

    // Detection areas, count from the bottom to the top of the cv::Mat
    DetectionArea area_left_line[DETECTION_NUMBER];

    //Init def values
    area_left_line[0].init(cv::Point(320, 320));
    area_left_line[1].init(cv::Point(320, 270));
    area_left_line[2].init(cv::Point(320, 220));
    area_left_line[3].init(cv::Point(320, 170));

    // Detected points
    std::vector<cv::Point> left_points;
    std::vector<cv::Point> right_points;
    std::vector<cv::Point> cones_points;

    // Scene variables
    bool reset = false;
    bool red_light_visible = true;
    bool green_light_visible = false;

#ifdef DEBUG_MODE
    // UI variables
    char keypressed;
#endif

#ifdef DEBUG_MODE
    // Trackbars
    cv::createTrackbar("f", "Settings", &lanedetector.f_i, 1000, update_trackbar);
    cv::createTrackbar("dst", "Settings", &lanedetector.dist_i, 1000, update_trackbar);
    cv::createTrackbar("alpha", "Settings", &lanedetector.alpha_i, 200, update_trackbar);
    cv::createTrackbar("l low", "Settings", &thresh_l_low, 255, NULL);
    cv::createTrackbar("l high", "Settings", &thresh_l_high, 255, NULL);
    cv::createTrackbar("s low", "Settings", &thresh_s_low, 255, NULL);
    cv::createTrackbar("s high", "Settings", &thresh_s_high, 255, NULL);
    cv::createTrackbar("t hist", "Settings", &thresh_hist, 255, NULL);
    cv::createTrackbar("t stop", "Settings", &thresh_stop, 255, NULL);
    cv::createTrackbar("method", "Settings", &metoda, 1, NULL);
    cv::createTrackbar("inv", "Settings", &typ, 1, NULL);
    cv::createTrackbar("bl size", "Settings", &blok_i, 100, NULL);
    cv::createTrackbar("C", "Settings", &c, 255, NULL);
#endif

    ids.update_params();

#ifdef FPS_COUNT
    // FPS
    struct timespec start, end;
    unsigned int frame_count = 0;
    float seconds = 0;
    float fps = 0;
#endif

START:

    // Pre-scan
    // Get new frame
#ifndef VID_MODE
#ifndef IDS_MODE
    camera >> frame;
#else
    pthread_cond_wait(&algorithm_signal, &algorithm_signal_mutex);
    pthread_mutex_lock(&ids.frame_mutex);
    ids.ids_frame.copyTo(frame);
    pthread_mutex_unlock(&ids.frame_mutex);
#endif
#else
//video >> frame;
#endif
/*...*/ STOP_TIMER("Get new frame")

    // Blur image
    //cv::GaussianBlur(frame, frame_blur, cv::Size(3,3), 0, 0);

/*...*/ START_TIMER
    // Change perspective
    lanedetector.bird_eye(frame, frame_bird);
/*...*/ STOP_TIMER("Bird eye")

/*...*/ START_TIMER
    // Obtain HLS color space image
    cv::cvtColor(frame_bird, frame_hls, cv::COLOR_BGR2HSV);
/*...*/ STOP_TIMER("Color space change")

/*...*/ START_TIMER
    // Split channels
    cv::split(frame_hls, frame_split_vec);
    //frame_h = frame_split_vec[0];
    frame_l = frame_split_vec[1];
    frame_s = frame_split_vec[2];
/*...*/ STOP_TIMER("Channels split")

#ifdef ADAPTIVE_MODE
/*...*/ START_TIMER
//        cv::adaptiveThreshold(frame_l, frame_l_adapt, 255, metoda, typ, blok, c);
    cv::adaptiveThreshold(frame_l, frame_s_adapt, 255, metoda, typ, blok, c);
/*...*/ STOP_TIMER("Adaptive threshold")

/*...*/ START_TIMER
//        lanedetector.opening(frame_l_adapt, frame_l_morph);
    lanedetector.opening(frame_s_adapt, frame_s_morph);
/*...*/ STOP_TIMER("Morph opening")

/*...*/ START_TIMER
    // Calculate new ROI's positions
    for(int i = 0; i < DETECTION_NUMBER; i++)
    {
        // Take new ROI based on previous frame
        area_left_line[i].update(frame_s_morph);

        // Detect position of the line relative to the ROI
        area_left_line[i].find_center(thresh_hist);

        // Corect position of ROI if outside cv::Mat
        area_left_line[i].check_boundaries();
    }
/*...*/ STOP_TIMER("ROI's positioning")

/*...*/ START_TIMER
    // Pull back lost ROI's
    lanedetector.correct_ROIs(area_left_line, frame_s_morph);
/*...*/ STOP_TIMER("Check for lost ROI's")

/*...*/ START_TIMER
    // Check for stop line
    lanedetector.search_stop_line(area_left_line, frame_s_morph, thresh_stop);
/*...*/ STOP_TIMER("Search STOP line")
#else
    // Binarize channels
    //lanedetector.binarize(frame_h, frame_binary_h, thresh_h_low, thresh_h_high);
    lanedetector.binarize(frame_l, frame_binary_l, thresh_l_low, thresh_l_high);
    cv::bitwise_not(frame_binary_l, frame_binary_l);
    lanedetector.binarize(frame_s, frame_binary_s, thresh_s_low, thresh_s_high);

    // Logical conjunction of H and S channel
    //cv::bitwise_and(frame_binary_h, frame_binary_s, frame_and);

    // Opening operation
    //lanedetector.opening(frame_and, frame_morph);

    // Obtain gradient points
    //lanedetector.apply_sobel(frame_morph, frame_sobel);

    // Calculate new ROI's positions
    for(int i = 0; i < DETECTION_NUMBER; i++)
    {
        // Take new ROI based on previous frame
        area_left_line[i].update(frame_binary_s);

        // Detect position of the line relative to the ROI
        area_left_line[i].find_center(thresh_hist);

        // Corect position of ROI if outside cv::Mat
        area_left_line[i].check_boundaries();
    }

    // Pull back lost ROI's
    //lanedetector.correct_ROIs(area_left_line, frame_morph);
#endif
#ifdef DEBUG_MODE
    // Draw detection
    lanedetector.draw_data(frame_bird, area_left_line, NULL);

    // Inverse data to original camera perspective
    lanedetector.bird_eye_inverse(frame_bird, frame_bird_inverse);
#endif


    // Pack data for SHM
    lanedetector.pack_data(area_left_line, NULL, left_points, right_points);

    // Push output data to SHM
    shm_lane_points.push_lane_data(left_points, right_points, cones_points);
    shm_usb_to_send.push_scene_data(reset, red_light_visible, green_light_visible, lanedetector.stop_detected, lanedetector.stop_distance);

    // Set standard ROI size
    for(int i = 0; i < DETECTION_NUMBER; i++)
        area_left_line[i].roi_x = ROI_X;
    // End of first scan

    //
    static int denom=0;

#ifdef STOP_L_MODE
    for(int i = 0; i < 15; i++)
    {
#ifndef VID_MODE
#ifndef IDS_MODE
        camera >> frame;
#else
        pthread_cond_wait(&algorithm_signal, &algorithm_signal_mutex);
        pthread_mutex_lock(&ids.frame_mutex);
        ids.ids_frame.copyTo(frame);
        pthread_mutex_unlock(&ids.frame_mutex);
#endif
#else
        video >> frame;
#endif
        lightDetector.prepare_first_image(frame,old_frame,lightDetector.roi_number);
    }

    while(true)
    {
#ifndef VID_MODE
#ifndef IDS_MODE
        camera >> frame;
#else
        pthread_cond_wait(&algorithm_signal, &algorithm_signal_mutex);
        pthread_mutex_lock(&ids.frame_mutex);
        ids.ids_frame.copyTo(frame);
        pthread_mutex_unlock(&ids.frame_mutex);
#endif
#else
        video >> frame;
#endif

#ifdef VERBOSE_MODE
        lightDetector.test_roi(frame,display);
#endif
        lightDetector.find_start(frame,diffrence,old_frame,lightDetector.roi_number);

#ifdef DEBUG_MODE
        if (++denom>5)
        {
            denom = 0;
            cv::imshow("Frame", frame);
            cv::imshow("Light detection", diffrence);
#ifdef VERBOSE_MODE
            cv::imshow("ROI", display);
#endif
            keypressed = (char)cv::waitKey(FRAME_TIME);

            if(keypressed == 27)
                    break;
        }
#endif

        if (lightDetector.start_light == true)
        {
                //std::cout<<"START"<<std::endl;
                red_light_visible = false;
                green_light_visible = true;
                break;
        }
        //else
                //std::cout<<"WAIT"<<std::endl;
    }
#endif

     // ========================Main loop ==================================
#ifndef VID_MODE
    while(true)
#else
    for(int i = 1; i < video.get(cv::CAP_PROP_FRAME_COUNT); )
#endif
    {
#ifdef FPS_COUNT
        // FPS
        if(frame_count == 0)
        {
            clock_gettime(CLOCK_MONOTONIC, &start);
        }
#endif
/*...*/ START_TIMER
        // Get new frame
#ifndef VID_MODE
#ifndef IDS_MODE
        camera >> frame;
#else
        pthread_cond_wait(&algorithm_signal, &algorithm_signal_mutex);
        pthread_mutex_lock(&ids.frame_mutex);
        ids.ids_frame.copyTo(frame);
        pthread_mutex_unlock(&ids.frame_mutex);
#endif
#else
    //video >> frame;
#endif
/*...*/ STOP_TIMER("Get new frame")

        // Blur image
        //cv::GaussianBlur(frame, frame_blur, cv::Size(3,3), 0, 0);

/*...*/ START_TIMER
        // Change perspective
        lanedetector.bird_eye(frame, frame_bird);
/*...*/ STOP_TIMER("Bird eye")

/*...*/ START_TIMER
        // Obtain HLS color space image
        cv::cvtColor(frame_bird, frame_hls, cv::COLOR_BGR2HSV);
/*...*/ STOP_TIMER("Color space change")

/*...*/ START_TIMER
        // Split channels
        cv::split(frame_hls, frame_split_vec);
        //frame_h = frame_split_vec[0];
        frame_l = frame_split_vec[1];
        frame_s = frame_split_vec[2];
/*...*/ STOP_TIMER("Channels split")

#ifdef ADAPTIVE_MODE
/*...*/ START_TIMER
//        cv::adaptiveThreshold(frame_l, frame_l_adapt, 255, metoda, typ, blok, c);
        cv::adaptiveThreshold(frame_l, frame_s_adapt, 255, metoda, typ, blok, c);
/*...*/ STOP_TIMER("Adaptive threshold")

/*...*/ START_TIMER
//        lanedetector.opening(frame_l_adapt, frame_l_morph);
        lanedetector.opening(frame_s_adapt, frame_s_morph);
/*...*/ STOP_TIMER("Morph opening")

/*...*/ START_TIMER
        // Calculate new ROI's positions
        for(int i = 0; i < DETECTION_NUMBER; i++)
        {
            // Take new ROI based on previous frame
            area_left_line[i].update(frame_s_morph);

            // Detect position of the line relative to the ROI
            area_left_line[i].find_center(thresh_hist);

            // Corect position of ROI if outside cv::Mat
            area_left_line[i].check_boundaries();
        }
/*...*/ STOP_TIMER("ROI's positioning")

/*...*/ START_TIMER
        // Pull back lost ROI's
        lanedetector.correct_ROIs(area_left_line, frame_s_morph);
/*...*/ STOP_TIMER("Check for lost ROI's")

/*...*/ START_TIMER
        // Check for stop line
        lanedetector.search_stop_line(area_left_line, frame_s_morph, thresh_stop);
/*...*/ STOP_TIMER("Search STOP line")
#else
        // Binarize channels
        //lanedetector.binarize(frame_h, frame_binary_h, thresh_h_low, thresh_h_high);
        lanedetector.binarize(frame_l, frame_binary_l, thresh_l_low, thresh_l_high);
        cv::bitwise_not(frame_binary_l, frame_binary_l);
        lanedetector.binarize(frame_s, frame_binary_s, thresh_s_low, thresh_s_high);

        // Logical conjunction of H and S channel
        //cv::bitwise_and(frame_binary_h, frame_binary_s, frame_and);

        // Opening operation
        //lanedetector.opening(frame_and, frame_morph);

        // Obtain gradient points
        //lanedetector.apply_sobel(frame_morph, frame_sobel);

        // Calculate new ROI's positions
        for(int i = 0; i < DETECTION_NUMBER; i++)
        {
            // Take new ROI based on previous frame
            area_left_line[i].update(frame_binary_s);

            // Detect position of the line relative to the ROI
            area_left_line[i].find_center(thresh_hist);

            // Corect position of ROI if outside cv::Mat
            area_left_line[i].check_boundaries();
        }

        // Pull back lost ROI's
        //lanedetector.correct_ROIs(area_left_line, frame_morph);
#endif
#ifdef DEBUG_MODE
        // Draw detection
        lanedetector.draw_data(frame_bird, area_left_line, NULL);

        // Inverse data to original camera perspective
        lanedetector.bird_eye_inverse(frame_bird, frame_bird_inverse);
#endif

/*...*/ START_TIMER
        // Pack data for SHM
        lanedetector.pack_data(area_left_line, NULL, left_points, right_points);
/*...*/ STOP_TIMER("Pack data for SHM")

/*...*/ START_TIMER
        // Push output data to SHM
        shm_lane_points.push_lane_data(left_points, right_points, cones_points);
        shm_usb_to_send.push_scene_data(reset, red_light_visible, green_light_visible, lanedetector.stop_detected, lanedetector.stop_distance);
/*...*/ STOP_TIMER("Push data to SHM")

#ifdef TEST_SHM
        shm_lane_points.pull_lane_data(frame_test_shm);
        shm_usb_to_send.pull_scene_data();
#endif

        //
        blok = blok_i*2+1;

#ifdef DEBUG_MODE
        // Show frames
#ifdef IDS_MODE
        if (++denom>5)
        {
            denom = 0;
#endif
            cv::imshow("Camera", frame);
            cv::imshow("Bird Eye", frame_bird);

#ifdef ADAPTIVE_MODE
            cv::imshow("L morph", frame_l_morph);
            cv::imshow("S morph", frame_s_morph);
#else
        //cv::imshow("Morph and", frame_morph);
        //cv::imshow("Sobel", frame_sobel);
#endif
            cv::imshow("Data inverse", frame_bird_inverse);
            cv::imshow("Settings", frame_settings);

            // Get input from user
            keypressed = (char)cv::waitKey(FRAME_TIME);

#ifdef IDS_MODE
        }
#endif

#ifdef VERBOSE_MODE
#ifdef ADAPTIVE_MODE
        cv::imshow("Blur", frame_blur);
        cv::imshow("H", frame_h);
        cv::imshow("S", frame_s);
#else
        //cv::imshow("H", frame_h);
        //cv::imshow("L", frame_l);
        cv::imshow("Binary l", frame_binary_l);
        cv::imshow("Binary s", frame_binary_s);
        //cv::imshow("AND", frame_and);
#endif
        //cv::imshow("Data", frame_data);
#endif
#ifdef TEST_SHM
        cv::imshow("SHM TEST", frame_test_shm);
        frame_test_shm = cv::Mat::zeros(CAM_RES_Y, CAM_RES_X, CV_8UC3);
#endif
        // Clear cv::Mat's
        frame_data = cv::Mat::zeros(CAM_RES_Y, CAM_RES_X, CV_8UC3);

        // Reset from Futaba
        //std::cout << "RESET: " << shm_usb_to_send.shared_variable[1] << std::endl; //++++++++++++++++++++++++++++++
        if(shm_usb_to_send.shared_variable[1] > 0)
        {
#ifdef STOP_L_MODE
            lightDetector.start_light = false;
            red_light_visible = true;
            green_light_visible = false;
#endif
            // Reset
            for(int i = 0; i < DETECTION_NUMBER; i++)
            {
                area_left_line[i].reset();
            }

            // Pre scan
            // Get new frame
#ifndef VID_MODE
#ifndef IDS_MODE
        camera >> frame;
#else
        pthread_cond_wait(&algorithm_signal, &algorithm_signal_mutex);
        pthread_mutex_lock(&ids.frame_mutex);
        ids.ids_frame.copyTo(frame);
        pthread_mutex_unlock(&ids.frame_mutex);
#endif
#else
        video >> frame;
#endif
            // Change perspective
            //cv::GaussianBlur(frame, frame_blur, cv::Size(3,3), 0, 0);
            lanedetector.bird_eye(frame, frame_bird);

            // Obtain HLS color space image
            cv::cvtColor(frame_bird, frame_hls, cv::COLOR_BGR2HLS);

            // Split channels
            cv::split(frame_hls, frame_split_vec);
            frame_h = frame_split_vec[0];
            frame_l = frame_split_vec[1];
            frame_s = frame_split_vec[2];

#ifdef ADAPTIVE_MODE
            cv::adaptiveThreshold(frame_l, frame_l_adapt, 255, metoda, typ, blok, c);
            cv::adaptiveThreshold(frame_s, frame_s_adapt, 255, metoda, typ, blok, c);

            lanedetector.opening(frame_l_adapt, frame_l_morph);
            lanedetector.opening(frame_s_adapt, frame_s_morph);

            // Calculate new ROI's positions
            for(int i = 0; i < DETECTION_NUMBER; i++)
            {
                // Take new ROI based on previous frame
                area_left_line[i].update(frame_s_morph);

                // Detect position of the line relative to the ROI
                area_left_line[i].find_center(thresh_hist);

                // Corect position of ROI if outside cv::Mat
                area_left_line[i].check_boundaries();
            }

            // Pull back lost ROI's
            //lanedetector.correct_ROIs(area_left_line, frame_s_morph);
#else
            // Binarize channels
            lanedetector.binarize(frame_h, frame_binary_h, thresh_h_low, thresh_h_high);
            lanedetector.binarize(frame_s, frame_binary_s, thresh_s_low, thresh_s_high);

            // Logical conjunction of H and S channel
            cv::bitwise_and(frame_binary_h, frame_binary_s, frame_and);

            // Opening operation
            lanedetector.opening(frame_and, frame_morph);

            // Obtain gradient points
            lanedetector.apply_sobel(frame_morph, frame_sobel);

            // Calculate new ROI's positions
            for(int i = 0; i < DETECTION_NUMBER; i++)
            {
                // Take new ROI based on previous frame
                area_left_line[i].update(frame_morph);

                // Detect position of the line relative to the ROI
                area_left_line[i].find_center(thresh_hist);

                // Corect position of ROI if outside cv::Mat
                area_left_line[i].check_boundaries();
            }

            // Pull back lost ROI's
            //lanedetector.correct_ROIs(area_left_line, frame_morph);
#endif
#ifdef DEBUG_MODE
            // Draw detection
            //lanedetector.draw_data(frame_bird, area_left_line, NULL);

            // Inverse data to original camera perspective
            lanedetector.bird_eye_inverse(frame_bird, frame_bird_inverse);
#endif

            // Pack data for SHM
            lanedetector.pack_data(area_left_line, NULL, left_points, right_points);

            // Push output data to SHM
            shm_lane_points.push_lane_data(left_points, right_points, cones_points);
            shm_usb_to_send.push_scene_data(reset, red_light_visible, green_light_visible, lanedetector.stop_detected, lanedetector.stop_distance);


            // Set standard ROI size
            for(int i = 0; i < DETECTION_NUMBER; i++)
                area_left_line[i].roi_x = ROI_X;
            // End of first scan
        }

        // Process given input
        if( keypressed == 27 )
            break;
        switch(keypressed)
        {
        // Reset ROI positions
        case 'r':
        {
            // Reset
            for(int i = 0; i < DETECTION_NUMBER; i++)
            {
                area_left_line[i].reset();
            }

            // Pre scan
            // Get new frame
        #ifndef VID_MODE
        #ifndef IDS_MODE
        camera >> frame;
        #else
        pthread_cond_wait(&algorithm_signal, &algorithm_signal_mutex);
        pthread_mutex_lock(&ids.frame_mutex);
        ids.ids_frame.copyTo(frame);
        pthread_mutex_unlock(&ids.frame_mutex);
        #endif
        #else
        video >> frame;
        #endif
            // Change perspective
            //cv::GaussianBlur(frame, frame_blur, cv::Size(3,3), 0, 0);
            lanedetector.bird_eye(frame, frame_bird);

            // Obtain HLS color space image
            cv::cvtColor(frame_bird, frame_hls, cv::COLOR_BGR2HLS);

            // Split channels
            cv::split(frame_hls, frame_split_vec);
            frame_h = frame_split_vec[0];
            frame_l = frame_split_vec[1];
            frame_s = frame_split_vec[2];

        #ifdef ADAPTIVE_MODE
            cv::adaptiveThreshold(frame_l, frame_l_adapt, 255, metoda, typ, blok, c);
            cv::adaptiveThreshold(frame_s, frame_s_adapt, 255, metoda, typ, blok, c);

            lanedetector.opening(frame_l_adapt, frame_l_morph);
            lanedetector.opening(frame_s_adapt, frame_s_morph);

            // Calculate new ROI's positions
            for(int i = 0; i < DETECTION_NUMBER; i++)
            {
                // Take new ROI based on previous frame
                area_left_line[i].update(frame_s_morph);

                // Detect position of the line relative to the ROI
                area_left_line[i].find_center(thresh_hist);

                // Corect position of ROI if outside cv::Mat
                area_left_line[i].check_boundaries();
            }

            // Pull back lost ROI's
            //lanedetector.correct_ROIs(area_left_line, frame_s_morph);
        #else
            // Binarize channels
            lanedetector.binarize(frame_h, frame_binary_h, thresh_h_low, thresh_h_high);
            lanedetector.binarize(frame_s, frame_binary_s, thresh_s_low, thresh_s_high);

            // Logical conjunction of H and S channel
            cv::bitwise_and(frame_binary_h, frame_binary_s, frame_and);

            // Opening operation
            lanedetector.opening(frame_and, frame_morph);

            // Obtain gradient points
            lanedetector.apply_sobel(frame_morph, frame_sobel);

            // Calculate new ROI's positions
            for(int i = 0; i < DETECTION_NUMBER; i++)
            {
                // Take new ROI based on previous frame
                area_left_line[i].update(frame_morph);

                // Detect position of the line relative to the ROI
                area_left_line[i].find_center(thresh_hist);

                // Corect position of ROI if outside cv::Mat
                area_left_line[i].check_boundaries();
            }

            // Pull back lost ROI's
            //lanedetector.correct_ROIs(area_left_line, frame_morph);
        #endif
        #ifdef DEBUG_MODE
            // Draw detection
            //lanedetector.draw_data(frame_bird, area_left_line, NULL);

            // Inverse data to original camera perspective
            lanedetector.bird_eye_inverse(frame_bird, frame_bird_inverse);
        #endif

            // Pack data for SHM
            lanedetector.pack_data(area_left_line, NULL, left_points, right_points);

            // Push output data to SHM
            shm_lane_points.push_lane_data(left_points, right_points, cones_points);
            shm_usb_to_send.push_scene_data(reset, red_light_visible, green_light_visible, lanedetector.stop_detected, lanedetector.stop_distance);


            // Set standard ROI size
            for(int i = 0; i < DETECTION_NUMBER; i++)
                area_left_line[i].roi_x = ROI_X;
            // End of first scan

            break;
        }
        // Change side on which line is searched
        case 'c':
        {

            break;
        }
        case 'q':
        {
            lightDetector.start_light = false;
            red_light_visible = true;
            green_light_visible = false;

            goto START;
        }
        }
#endif

#ifdef FPS_COUNT
        // FPS
        if(frame_count > FPS_AMOUNT)
        {
            frame_count = 0;
            clock_gettime(CLOCK_MONOTONIC, &end);
            seconds = (end.tv_sec - start.tv_sec);
            fps  =  1 / (seconds / FPS_AMOUNT);

            std::cout <<"FPS: " << fps << std::endl;
        }
        else
        {
            frame_count++;
        }
#endif
    }

    // Clean up
#ifndef VID_MODE
#ifndef IDS_MODE
    camera.release();
#else
    ids.exit();
#endif
#else
    video.release();
#endif
    shm_lane_points.close();
    shm_usb_to_send.close();
    return 0;
}

void update_trackbar(int, void*)
{
    lanedetector.calculate_bird_var(frame_ref);
}

void update_params_1(int, void*)
{
    ids.update_params();
}


void *camera_thread(void*)
{
    while(true)
    {
        ids.frame_loop();
    }
    return NULL;
}

// Creating in debug mode trackbars
void IDS::create_trackbars(void)
{
    cvNamedWindow("ids", 1);
    cv::createTrackbar("Pixel", "ids", &pixelclock_slider, 40, update_params_1);
    cv::createTrackbar("Exposure", "ids", &exposure_slider, 500, update_params_1);
    cv::createTrackbar("FPS", "ids", &fps_slider, 87, update_params_1);
    cv::createTrackbar("Master", "ids", &Master_GAIN_Factor, 300, update_params_1);
    cv::setTrackbarMin("Master", "ids", 100);
    cv::createTrackbar("Green", "ids", &Green_GAIN_Factor, 300, update_params_1);
    cv::setTrackbarMin("Green", "ids",100);
    cv::createTrackbar("Red", "ids", &Red_GAIN_Factor, 300, update_params_1);
    cv::setTrackbarMin("Red", "ids", 100);
    cv::createTrackbar("Blue", "ids", &Blue_GAIN_Factor, 300, update_params_1);
    cv::setTrackbarMin("Blue", "ids", 100);
    cv::createTrackbar("Sharpness", "ids", &sharpness_slider, 9, update_params_1);
    cv::setTrackbarMin("Sharpness", "ids", 0);
    cv::createTrackbar("Gamma", "ids", &Gamma, 300, update_params_1);
}

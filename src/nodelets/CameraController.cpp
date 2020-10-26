/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include <pluginlib/class_list_macros.h>
#include <fstream>
#include "flir_adk_ethernet/CameraController.h"

PLUGINLIB_EXPORT_CLASS(flir_adk_ethernet::CameraController, nodelet::Nodelet)

using namespace cv;
using namespace flir_adk_ethernet;

CameraController::CameraController(ros::Duration timeout) : BaseCameraController(), _timeout(timeout), _last_capture_time(ros::Time::now())
{
}

CameraController::~CameraController()
{
}

void CameraController::setupFramePublish() {
    pnh.param<float>("frame_rate", _frameRate, 60.0);
    ROS_INFO("flir_adk_ethernet - Got frame rate: %f.", _frameRate);

    capture_timer = nh.createTimer(ros::Duration(1.0 / _frameRate),
        boost::bind(&CameraController::captureAndPublish, this, _1));
    _last_capture_time = ros::Time::now(); // pretend last capture was at init, this allows us to handle startup failures
    _last_camera_stamp = ros::Time(0, 0);  // init to something, we only want to see that this changes
    _post_init = true;
}

void CameraController::captureAndPublish(const ros::TimerEvent &evt)
{
    /*
     * The capture state machine below is designed to handle several different types of capture failures
     * 1. Capture is not successful for length of timeout
     * 2. So many capture failures in row that a new frame is never aquired in a whole timeout period
     * 3. Camera timestamps stop being updated for an entire timeout period
     *
     * When a capture failure occurs, the node is shut down. A launchfile can set respawn to true to allow the node to restart on failure.
     * TODO: integrate camera-capture and device-finding restart capabilities into the node itself
    */
    ros::Time now(ros::Time::now());
    ros::Time camera_stamp;

    // try to get the latest image
    bool capture_success = publishImage(ros::Time::now());
    if (capture_success) {
        camera_stamp = ros::Time(_camera->getActualTimestamp() * 1e-9);
        if (_post_init) {
            // TODO: camera timestamps should be synchronized to os clock on start of capture right?
            ROS_INFO("initial capture delay was %f seconds", (ros::Time::now() - camera_stamp).toSec());
            _post_init = false;
        }
    }
    // check if we have a timeout
    if ((now - _last_capture_time) > _timeout) {
        // We have not received a new frame, after timeout we should just kill the node so that it can be restarted
        // TODO: Add restart on camera disconnect/failure capability
        ROS_ERROR("flir_adk_ethernet - Stopped receiving frames from camera after %f seconds, exiting", _timeout.toSec());
        ros::shutdown();
    }
    // update image time tracking
    if (capture_success) {
        // only update the capture time when we really get a new image
        if (camera_stamp != _last_camera_stamp) {
            _last_capture_time = now;
        }
        _last_camera_stamp = camera_stamp;
    }
}

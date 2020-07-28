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

// TODO: should we give last capture time a head start to account for the intial delay of capture?
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
    _last_capture_time = ros::Time::now();
    _post_init = true;
}

void CameraController::captureAndPublish(const ros::TimerEvent &evt)
{
    ros::Time stamp(_camera->getActualTimestamp() * 1e-9);
    if (stamp >= _last_capture_time || _post_init) {
        if (publishImage(stamp)) {
            if (_post_init) {
                ROS_INFO("initial capture delay was %f seconds", (ros::Time::now() - stamp).toSec());
            }
            _last_capture_time = stamp;
            _post_init = false;
        }
    } else if ((ros::Time::now() - _last_capture_time) > _timeout) {
        // We have not received a new frame, after timeout we should just kill the node so that it can be restarted
        // TODO: Add restart on camera disconnect/failure capability
        ROS_ERROR("flir_adk_ethernet - Stopped receiving frames from camera after %f seconds, exiting", _timeout.toSec());
        ros::shutdown();
    }
}

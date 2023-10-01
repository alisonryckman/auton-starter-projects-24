#include "perception.hpp"
#include "mrover/StarterProjectTag.h"

// ROS Headers, ros namespace
#include <cfloat>
#include <cmath>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <ros/init.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "starter_project_perception"); // Our node name (See: http://wiki.ros.org/Nodes)

    [[maybe_unused]] mrover::Perception perception;

    // "spin" blocks until our node dies
    ROS_INFO("init");
    // It listens for new messages and calls our subscribed functions with them
    ros::spin();

    return EXIT_SUCCESS;
}

namespace mrover {

    Perception::Perception() : mNodeHandle{}, mImageTransport{mNodeHandle} {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = mImageTransport.subscribe("camera/right/image", 1, &Perception::imageCallback, this);

        ROS_ERROR("index counter %d \n", 1);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

        mTagDetectorParams = cv::aruco::DetectorParameters::create();
        mTagDictionary = cv::aruco::getPredefinedDictionary(0);
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& image) {
        ROS_INFO("index counter %d \n", 0);
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat cvImage{static_cast<int>(image->height), static_cast<int>(image->width),
                        CV_8UC3, const_cast<uint8_t*>(image->data.data())};
        // Detect tags in the image pixels
        findTagsInImage(cvImage, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(cvImage, mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions
        tags.clear(); // Clear old tags in output vector
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);
        ROS_INFO("index counter %d \n", 0);
        //Get the number of tags in image
        int numTagsDetected = mTagIds.size();


        for (int i = 0; i < numTagsDetected; i++) {
            StarterProjectTag msg;
            float closeness = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);
            msg.closenessMetric = closeness;
            msg.xTagCenterPixel = center.first;
            msg.yTagCenterPixel = center.second;
            msg.tagId = mTagIds[i];
            tags.push_back(msg);
            ROS_INFO("index counter %d \n", i);
        }
        // TODO: implement me!
    }

    StarterProjectTag Perception::selectTag(cv::Mat const& image, std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        int centerY = image.rows / 2;
        int centerX = image.cols / 2;

        int closeIndex = 0;
        float closeLength = FLT_MAX;
        for (int i = 0; i < tags.size(); i++) {
            float distX = (float) centerX - tags[i].xTagCenterPixel;
            float distY = (float) centerY - tags[i].yTagCenterPixel;
            float dist = std::sqrt(std::pow(distX, 2) + std::pow(distY, 2));
            if (dist < closeLength) {
                closeIndex = i;
                closeLength = dist;
            }
        }

        return tags[closeIndex];
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: implement me!
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag
        float tlX = tagCorners[0].x;
        float tlY = tagCorners[0].y;

        float trX = tagCorners[1].x;
        float trY = tagCorners[1].y;

        float distX = tlX - trX;
        float distY = trY - trY;

        float dist = std::sqrt(std::pow(distX, 2) + std::pow(distY, 2));

        float metric = dist / (float) image.rows;

        // TODO: implement me!
        return metric;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float sumX = 0;
        float sumY = 0;

        for (cv::Point2f corner: tagCorners) {
            sumX += corner.x;
            sumY += corner.y;
        }

        sumX /= 4;
        sumY /= 4;

        std::pair<float, float> center(sumX, sumY);
        return center;
    }

} // namespace mrover
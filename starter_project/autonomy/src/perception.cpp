#include "perception.hpp"
#include "mrover/StarterProjectTag.h"

// ROS Headers, ros namespace
#include <opencv2/aruco.hpp>
#include <ros/init.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "starter_project_perception"); // Our node name (See: http://wiki.ros.org/Nodes)

    [[maybe_unused]] mrover::Perception perception;

    // "spin" blocks until our node dies
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

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

        mTagDetectorParams = cv::aruco::DetectorParameters::create();
        mTagDictionary = cv::aruco::getPredefinedDictionary(0);
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& image) {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat cvImage{static_cast<int>(image->height), static_cast<int>(image->width),
                        CV_8UC3, const_cast<uint8_t*>(image->data.data())};
        // Detect tags in the image pixels
        findTagsInImage(cvImage, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: implement me!
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);
        for (int i = 0; i < static_cast<int>(mTagCorners.size()); i++) {
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);
            StarterProjectTag tag;
            tag.tagId = mTagIds[i];
            // tag.xTagCenterPixel = center.first - static_cast<float>(image.cols) / 2;
            tag.xTagCenterPixel = 45;
            tag.yTagCenterPixel = center.second - static_cast<float>(image.rows) / 2;
            tag.closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            std::cout << tag.closenessMetric << tag.xTagCenterPixel << tag.yTagCenterPixel << '\n';
            tags.push_back(tag);
        }
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float minDist = std::numeric_limits<float>::max();
        StarterProjectTag selectedTag;
        for (auto tag : tags) {
            float currDist = tag.xTagCenterPixel * tag.xTagCenterPixel + tag.yTagCenterPixel * tag.yTagCenterPixel;
            if (currDist < minDist) {
                minDist = currDist;
                selectedTag = tag;
            }
        }
        return selectedTag;
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: implement me!
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me!
        std::pair<float, float> topLeft, topRight, botLeft, botRight;
        topLeft = std::make_pair(tagCorners[0].y, tagCorners[0].x);
        topRight = std::make_pair(tagCorners[1].y, tagCorners[1].x);
        botRight = std::make_pair(tagCorners[2].y, tagCorners[2].x);
        botLeft = std::make_pair(tagCorners[3].y, tagCorners[3].x);

        float top = (topRight.first - topLeft.first) * (topRight.first - topLeft.first) + (topRight.second - topLeft.second) * (topRight.second - topLeft.second);
        float left = (botLeft.first - topLeft.first) * (botLeft.first - topLeft.first) + (botLeft.second - topLeft.second) * (botLeft.second - topLeft.second);
        float bottom = (botRight.first - botLeft.first) * (botRight.first - botLeft.first) + (botRight.second - botRight.second) * (botRight.second - botRight.second);
        float right = (botRight.first - topRight.first) * (botRight.first - topRight.first) + (botRight.second - topRight.second) * (botRight.second - topRight.second);

        return top + left + bottom + right;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float midVertical = (tagCorners[0].y + tagCorners[3].y) / 2;
        float midHorizontal = (tagCorners[0].x + tagCorners[1].x) / 2;
        return std::make_pair(midHorizontal, midVertical);
    }

} // namespace mrover

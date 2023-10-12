#include "perception.hpp"

// ROS Headers, ros namespace
#include <arm_neon.h>
#include <cmath>
#include <cstddef>
#include <opencv2/aruco.hpp>
#include <ros/init.h>
#include <utility>

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
        // mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

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

        int tagId;
        float xTagCenterPixel;
        float yTagCenterPixel;
        float closenessMetric;

        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);

        for (size_t i = 0; i < tags.size(); ++i) {
            tags[i].tagId = mTagIds[i];
            tags[i].xTagCenterPixel = getCenterFromTagCorners(mTagCorners[i]).first;
            tags[i].yTagCenterPixel = getCenterFromTagCorners(mTagCorners[i]).second;
            tags[i].closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
        }
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        return {};
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: implement me!
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        int image_area = image.rows * image.cols;
        float x_1 = (tagCorners[0].x + tagCorners[3].x) / 2, x_2 = (tagCorners[1].x + tagCorners[2].x) / 2;
        float y_1 = (tagCorners[0].y + tagCorners[1].y) / 2, y_2 = (tagCorners[3].y + tagCorners[2].y) / 2;
        float tag_area = abs(x_1 - x_2) * abs(y_1 - y_2);

        return tag_area / (float) image_area;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        float x_center = 0;
        float y_center = 0;

        for (const cv::Point2f& corner : tagCorners) {
            x_center += corner.x;
            y_center += corner.y;
        }

        return std::make_pair(x_center / 4, y_center / 4);
    }

} // namespace mrover

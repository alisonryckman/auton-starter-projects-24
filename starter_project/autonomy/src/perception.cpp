#include "perception.hpp"
#include "mrover/StarterProjectTag.h"

// ROS Headers, ros namespace
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

    Perception::Perception() : mNodeHandle{} {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = mNodeHandle.subscribe("camera/left/image", 1, &Perception::imageCallback, this);

        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = mNodeHandle.advertise<StarterProjectTag>("tag", 1);

        mTagDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();
        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& imageMessage) {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat image{static_cast<int>(imageMessage->height), static_cast<int>(imageMessage->width),
                      CV_8UC3, const_cast<uint8_t*>(imageMessage->data.data())};
        // Detect tags in the image pixels
        findTagsInImage(image, mTags);
        // Select the tag that is closest to the middle of the screen
        StarterProjectTag tag = selectTag(image, mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);

        int numTagsDetected = (int) mTagIds.size();
        StarterProjectTag msg;

        for (int i = 0; i < numTagsDetected; i++) {
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);
            float closeness = getClosenessMetricFromTagCorners(image, mTagCorners[i]);

            msg.tagId = i;
            msg.xTagCenterPixel = center.first;
            msg.yTagCenterPixel = center.second;
            msg.closenessMetric = closeness;
            tags.push_back(msg);
        }
    }

    StarterProjectTag Perception::selectTag(cv::Mat const& image, std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)

        if (!tags.empty()) {
            if (tags.size() == 1) {
                return tags[0];
            }
            int ans = 0;
            double minn = pow((pow(tags[0].xTagCenterPixel, 2) - pow(tags[0].yTagCenterPixel, 2)), 0.5);
            for (int i = 1; i < tags.size(); i++) {
                double curr = pow((pow(tags[i].xTagCenterPixel, 2) - pow(tags[i].yTagCenterPixel, 2)), 0.5);
                if (curr < minn) {
                    minn = curr;
                    ans = i;
                }
            }
            return tags[ans];
        } else {
            StarterProjectTag tag;
            tag.tagId = -1;
            return tag;
        }
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        float xWidth = tagCorners[1].x - tagCorners[0].x;
        float yHeight = tagCorners[2].y - tagCorners[1].y;
        float tag_area = xWidth * yHeight;

        int camera_width = image.rows;  //width of image
        int camera_height = image.cols; //height of image
        int camera_area = camera_width * camera_height;

        return 1 - (tag_area / (float) camera_area);
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float width = abs(tagCorners[1].x - tagCorners[0].x);
        float height = abs(tagCorners[2].y - tagCorners[1].y);
        return {width / 2, height / 2};
    }

} // namespace mrover

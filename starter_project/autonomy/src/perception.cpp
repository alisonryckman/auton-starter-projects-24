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
        StarterProjectTag tag = selectTag(mTags, cvImage);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: implement me!
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds);
        for (int i = 0; i < mTagCorners.size(); i++) {
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);
            StarterProjectTag tag;
            tag.tagId = mTagIds[i];
            tag.xTagCenterPixel = (center.first - ((float)image.cols / 2.f)) / (float)image.cols;
            tag.yTagCenterPixel = (center.second - (float)image.rows / 2.f) / (float)image.rows;
            tag.closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            tags.push_back(tag);
        }
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags, cv::Mat const& image) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        if (tags.empty()) {
            StarterProjectTag tag;
            tag.tagId = -1;
            return tag;
        }
        std::pair<float, float> center = {image.cols / 2.0, image.rows / 2.0};
        int closestInd = 0;
        double minDist = sqrt(pow(tags[0].xTagCenterPixel - center.first, 2) + pow(tags[0].yTagCenterPixel - center.second, 2));
        for (int i = 1; i < tags.size(); i++) {
            double dist = sqrt(pow(tags[0].xTagCenterPixel - center.first, 2) + pow(tags[0].yTagCenterPixel - center.second, 2));
            if (dist < minDist) {
                closestInd = i;
                minDist = dist;
            }
        }
        return tags[closestInd];
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: implement me!
        mTagPublisher.publish(tag);
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // When the tag is close, it appears big - corners are far apart
        // find the distance between the corners (pythagorean theorem)
        float dist = sqrt((tagCorners[2].x - tagCorners[0].x) * (tagCorners[2].x - tagCorners[0].x) + 
                          (tagCorners[2].y - tagCorners[0].y) * (tagCorners[2].y - tagCorners[0].y));
        
        auto screenDiagDist = (float)(sqrt(image.rows * image.rows + image.cols * image.cols));
        // return 1 - dist/screenDiagDist;
        return exp(-10 * dist / screenDiagDist);
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        float centerX = (tagCorners[0].x + tagCorners[1].x + tagCorners[2].x + tagCorners[3].x) / 4.0f;
        float centerY = (tagCorners[0].y + tagCorners[1].y + tagCorners[2].y + tagCorners[3].y) / 4.0f;
        return {centerX, centerY};
    }

} // namespace mrover

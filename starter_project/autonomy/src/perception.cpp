#include "perception.hpp"
#include "mrover/StarterProjectTag.h"

// ROS Headers, ros namespace
#include <cstdint>
#include <opencv2/core/matx.hpp>
#include <cmath>
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
        StarterProjectTag tag = selectTag(cvImage, mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    void Perception::findTagsInImage(cv::Mat const& image, std::vector<StarterProjectTag>& tags) { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, mTagIds, and mTagDetectorParams member variables already defined!
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: implement me!
        std::vector<std::vector<cv::Point2f>> rejectedCandidates;
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams, rejectedCandidates);
        StarterProjectTag tag;
        std::pair<float, float> xy;
        float closeness;

        for (int i=0; i < mTagIds.size(); i++){
            xy = getCenterFromTagCorners(image, mTagCorners[i]);
            closeness = getClosenessMetricFromTagCorners(image, mTagCorners[i]);

            tag.xTagCenterPixel = xy.first;
            tag.yTagCenterPixel = xy.second;
            tag.closenessMetric = closeness;
            tag.tagId = mTagIds[i];

            tags.push_back(tag);
        }
    }

    StarterProjectTag Perception::selectTag(cv::Mat const& image, std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        StarterProjectTag closest_tag;
        float closest_distance;
        double x_dis, y_dis, distance;

        for (int i=0; i<tags.size(); i++){
            x_dis = abs(tags[i].xTagCenterPixel);
            y_dis = abs(tags[i].yTagCenterPixel);
            distance = pow(pow(x_dis, 2) + pow(y_dis, 2), 0.5);
            if (i==0) {
                closest_distance = distance;
                closest_tag = tags[i];
            }
            else if (distance < closest_distance) {
                closest_distance = distance;
                closest_tag = tags[i];
            }
        }

        return closest_tag;
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: implement me!
        mTagPublisher.publish(tag);

    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // Distance based on area; Setting Full screen = 0
        int screen_width = image.cols;
        int screen_height = image.rows;
        double screen_diag = pow(pow(screen_width, 2) + pow(screen_height, 2), 0.5);

        double tag_width = (tagCorners[1].x - tagCorners[0].x);
        double tag_height = tagCorners[1].y - tagCorners[2].y;
        double tag_diag = pow(pow(tag_width, 2) + pow(tag_height, 2), 0.5);
        
        // TODO: implement me!
        return 1 - (tag_diag / screen_diag);
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        std::pair<float, float> center;
        center.first = (tagCorners[0].x + tagCorners[1].x)/2 - image.cols/2;
        center.second = (tagCorners[1].y + tagCorners[2].y)/2 - image.rows/2;


        return center;
    }

} // namespace mrover

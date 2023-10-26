#include "perception.hpp"
#include "mrover/StarterProjectTag.h"

// ROS Headers, ros namespace
#include <cmath>
#include <ros/init.h>
#include <vector>

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


        // Populate tag then push to tags using for loop, loop through one tag at a time

        for (int i = 0; i < mTagIds.size(); i++) {

            StarterProjectTag tag;

            std::pair<float, float> tagCenter = getCenterFromTagCorners(mTagCorners[i]);

            // Change pair into reference from middle

            tagCenter.first = (tagCenter.first - float(image.cols / 2.0));

            tagCenter.second = (tagCenter.second) - float(image.cols / 2.0);

            tag.tagId = mTagIds[i];

            tag.xTagCenterPixel = tagCenter.first / float(image.cols);

            tag.yTagCenterPixel = tagCenter.second / float(image.rows);

            tag.closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);

            tags.push_back(tag);
        }
    }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) { // NOLINT(*-convert-member-functions-to-static)
                                                                                          // TODO: implement me!

        //  Select tag by center of image

        // Get center of image
        // find distances from image to cto all tags using  root((y2-y1(^2)) + (x2-x1)^2)
        // Find minimum then return the tag

        int selectedIndex = 0;

        float minDist = 10000.0;

        if (!tags.empty()) {
            if (tags.size() == 1) {
                return tags[0];
            }

            for (int j = 0; j < tags.size(); j++) {
                float tagDist = sqrt(float(pow((tags[j].xTagCenterPixel), 2) - pow(tags[j].yTagCenterPixel, 2)));

                if (tagDist < minDist) {
                    minDist = tagDist;
                    selectedIndex = j;
                }
            }

            StarterProjectTag selectedTag = tags[selectedIndex];

            return {selectedTag};
        }

        else {
            StarterProjectTag tag;
            tag.tagId = -1;
            return tag;
        }
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

        float diff_x = tagCorners[1].x - tagCorners[0].x;
        float diff_y = abs(tagCorners[1].y - tagCorners[2].y);


        // the closer the tag center is to the image center, the less the distance


        // Get output from getCenterFromTagCorners

        float closenessMetric = 1 - abs(diff_x * diff_y / float(image.cols * image.rows));

        // between 0 and 1


        return closenessMetric;
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!

        float diff_x = tagCorners[1].x - tagCorners[0].x;
        float diff_y = abs(tagCorners[1].y - tagCorners[2].y);

        return {diff_x / 2, diff_y / 2};
    }

} // namespace mrover

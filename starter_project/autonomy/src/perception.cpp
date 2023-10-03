#include "perception.hpp"

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

        Perception::image_height = 0;
        Perception::image_width = 0;
    }

    void Perception::imageCallback(sensor_msgs::ImageConstPtr const& image) {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat cvImage{static_cast<int>(image->height), static_cast<int>(image->width),
                        CV_8UC3, const_cast<uint8_t*>(image->data.data())};
        // Detect tags in the image pixels

        Perception::image_width = image->width;
        Perception::image_height = image->height;

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
        // cv::aruco::ArucoDetector detector = cv::aruco::ArucoDetector(mTagDictionary, mTagDetectorParams);

        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds, mTagDetectorParams);

        std::vector<float> closeness_metrics;
        std::vector<std::pair<float, float>> centers;

        for (std::vector<cv::Point2f> &corner_set : mTagCorners) {
            closeness_metrics.push_back(getClosenessMetricFromTagCorners(image, corner_set));
            centers.push_back(getCenterFromTagCorners(corner_set));
        }

        for (size_t i = 0; i < closeness_metrics.size(); i++) {
            StarterProjectTag tag;

            tag.tagId = mTagIds[i];
            tag.xTagCenterPixel = centers[i].first;
            tag.yTagCenterPixel = centers[i].second;
            tag.closenessMetric = closeness_metrics[i];

            tags.push_back(tag);
        }


    }

    // double Perception::get_center_offset_magnitude_sq(float x, float y) const {
    //     double x_offset = x - (double(Perception::image_width) / 2.0);
    //     double y_offset = y - (double(Perception::image_height) / 2.0);

    //     return x_offset*x_offset + y_offset*y_offset;
    // }

    StarterProjectTag Perception::selectTag(std::vector<StarterProjectTag> const& tags) const { // NOLINT(*-convert-member-functions-to-static)
        if (tags.empty())
            return {};

        StarterProjectTag best = tags[0];
        double best_dist = best.xTagCenterPixel*best.xTagCenterPixel + best.yTagCenterPixel*best.yTagCenterPixel;

        for (int i = 1; i < tags.size(); i++) {
            if ((tags[i].xTagCenterPixel*tags[i].xTagCenterPixel + tags[i].yTagCenterPixel*tags[i].yTagCenterPixel) < best_dist) {
                best_dist = tags[i].xTagCenterPixel*tags[i].xTagCenterPixel + tags[i].yTagCenterPixel*tags[i].yTagCenterPixel;
                best = tags[i];
            }
        }

        return best;
    }

    void Perception::publishTag(StarterProjectTag const& tag) {
        // TODO: implement me!
        Perception::mTagPublisher.publish(tag);
        
    }

    float Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me!
        //return percent of screen taken off by corners
        //assume the tag is a square?
        int total_area = image.rows * image.cols;
        
        float corner_area = abs((tagCorners[1].x - tagCorners[0].x) * (tagCorners[0].y - tagCorners[3].y));

        return 1 - float((1.0*corner_area) / (1.0*total_area));
    }

    std::pair<float, float> Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) const { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float center_x = float(tagCorners[0].x + tagCorners[1].x) / float(2.0);
        float center_y = float(tagCorners[0].y + tagCorners[3].y) / float(2.0);

        center_x = float(Perception::image_width) - center_x;
        center_y = float(Perception::image_height) - center_y;

        std::pair<float, float> ret;
        ret.first = center_x;
        ret.second = center_y;

        return ret;
    }

} // namespace mrover

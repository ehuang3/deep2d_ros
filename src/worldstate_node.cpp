#include <ros/ros.h>
#include <apriltags/AprilTagDetections.h>
#include <deep2d_ros/WorldState.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

using namespace std;

class DetectionsCallback {
public:
    vector<int> mIdsRed;
    vector<int> mIdsBlue;
    vector<int> mIdsYellow;
    vector<int> mIdsGreen;
    vector<int> mDetectedRed;
    vector<int> mDetectedBlue;
    vector<int> mDetectedYellow;
    vector<int> mDetectedGreen;
    int mExpectedRed;
    int mExpectedBlue;
    int mExpectedYellow;
    int mExpectedGreen;
    ros::Publisher* mPub;

    Eigen::VectorXd  mState;
    Eigen::VectorXi  mFresh;

    DetectionsCallback(int red, int blue, int yellow, int green)
        : mExpectedRed(red),
          mExpectedBlue(blue),
          mExpectedYellow(yellow),
          mExpectedGreen(green) {
        // Add IDs.
        mIdsRed.push_back(11);
        mIdsRed.push_back(29);
        mIdsRed.push_back(31);
        mIdsRed.push_back(32);
        mIdsRed.push_back(5);
        mIdsRed.push_back(21);
        mIdsRed.push_back(14);
        mIdsRed.push_back(0);
        mIdsRed.push_back(30);
        mIdsRed.push_back(9);
        mIdsBlue.push_back(8);
        mIdsBlue.push_back(15);
        mIdsBlue.push_back(17);
        mIdsBlue.push_back(24);
        mIdsBlue.push_back(22);
        mIdsBlue.push_back(7);
        mIdsBlue.push_back(13);
        mIdsBlue.push_back(33);
        mIdsBlue.push_back(6);
        mIdsGreen.push_back(16);
        mIdsGreen.push_back(20);
        mIdsGreen.push_back(27);
        mIdsGreen.push_back(34);
        mIdsGreen.push_back(1);
        mIdsGreen.push_back(2);
        mIdsGreen.push_back(3);
        mIdsGreen.push_back(18);
        mIdsYellow.push_back(10);
        mIdsYellow.push_back(12);
        mIdsYellow.push_back(25);
        mIdsYellow.push_back(26);
        mIdsYellow.push_back(23);
        mIdsYellow.push_back(19);
        mIdsYellow.push_back(28);
        mIdsYellow.push_back(4);
        // 
        mDetectedRed.clear();
        mDetectedBlue.clear();
        mDetectedYellow.clear();
        mDetectedGreen.clear();
        // Create state vector.
        mState.resize(3 * NumBlocks());
        mState.setZero();
        for (int i = 0; i < NumBlocks(); i++) {
            mState[3*i+0] =    0; //mm
            mState[3*i+1] = -485; //mm
        }
        mFresh.resize(NumBlocks());
        mFresh.setZero();
    }

    int NumBlocks() {
        return mExpectedRed + mExpectedBlue + mExpectedYellow + mExpectedGreen;
    }

    int FindIndex(int id) {
        int index = 0;
        if (Expected(&index, id, mIdsRed, mDetectedRed, mExpectedRed)) {
            return index;
        }
        if (Expected(&index, id, mIdsBlue, mDetectedBlue, mExpectedBlue)) {
            return index;
        }
        if (Expected(&index, id, mIdsYellow, mDetectedYellow, mExpectedYellow)) {
            return index;
        }
        if (Expected(&index, id, mIdsGreen, mDetectedGreen, mExpectedGreen)) {
            return index;
        }
        return -1;
    }

    bool Expected(int* index, int id, vector<int>& ids, vector<int>& detected, int expected) {
        // Check if we already have seen this id.
        for (int i = 0; i < detected.size(); i++) {
            if (id == detected[i]) {
                return true;
            }
            (*index)++;
        }
        // Check if this id belongs to this group.
        bool match = false;
        for (int i = 0; i < ids.size(); i++) {
            match |= ids[i] == id;
        }
        if (!match) {
            return false;
        }
        // If not seen but match, add id.
        if (detected.size() < expected) {
            cout << "Adding " << id << " to detected." << endl;
            detected.push_back(id);
            return true;
        }
        // Otherwise, bad id.
        return false;
    }

    void callback0(const apriltags::AprilTagDetections::ConstPtr& msg) {
        // q: [-80.86, -428.67, 7.27, -0.00602, -0.00219, 0.04996, 0.99873]
        geometry_msgs::Pose camera_pose;
        // camera_pose.position.x =  -82.59;
        camera_pose.position.x =  -81.946462;
        // camera_pose.position.y = -431.50;
        camera_pose.position.y = -437.698947;
        camera_pose.position.z =    5.51;
        camera_pose.orientation.w = -0.00072;
        camera_pose.orientation.x =  0.00231;
        camera_pose.orientation.y =  0.00669;
        camera_pose.orientation.z =  0.99997;

        callback(msg, camera_pose);
    }

    void callback1(const apriltags::AprilTagDetections::ConstPtr& msg) {
        // q: [72.78, -431.72, 5.45, 0.00002, -0.00890, 0.04842, 0.99879]
        geometry_msgs::Pose camera_pose;
        camera_pose.position.x =   70.16;
        camera_pose.position.y = -430.08;
        camera_pose.position.z =    3.70;
        camera_pose.orientation.w =  0.00149;
        camera_pose.orientation.x = -0.00497;
        camera_pose.orientation.y =  0.00152;
        camera_pose.orientation.z =  0.99999;

        callback(msg, camera_pose);
    }

    void callback2(const apriltags::AprilTagDetections::ConstPtr& msg) {
        // q: [-82.61, -589.37, 3.99, -0.00016, -0.00429, 0.00207, 0.99999]
        geometry_msgs::Pose camera_pose;
        camera_pose.position.x =  -82.61;
        camera_pose.position.y = -589.37;
        camera_pose.position.z =    3.99;
        camera_pose.orientation.w = -0.00016;
        camera_pose.orientation.x = -0.00429;
        camera_pose.orientation.y =  0.00207;
        camera_pose.orientation.z =  0.99999;

        callback(msg, camera_pose);
    }

    void callback3(const apriltags::AprilTagDetections::ConstPtr& msg) {
        // q: [72.38, -590.43, 4.20, 0.00183, -0.00449, -0.00158, 0.99999]
        geometry_msgs::Pose camera_pose;
        // camera_pose.position.x =   72.38;
        camera_pose.position.x =   70.31;
        camera_pose.position.y = -590.43;
        camera_pose.position.z =    4.20;
        camera_pose.orientation.w =  0.00183;
        camera_pose.orientation.x = -0.00449;
        camera_pose.orientation.y = -0.00158;
        camera_pose.orientation.z =  0.99999;

        callback(msg, camera_pose);
    }

    void callback(const apriltags::AprilTagDetections::ConstPtr& msg, 
                  const geometry_msgs::Pose& camera_pose) {
        // 
        Eigen::Isometry3d tf_camera;
        tf::poseMsgToEigen(camera_pose, tf_camera);
        // tf_camera.linear().setIdentity();
        // tf_camera.linear().row(0) *= -1;
        // tf_camera.linear().row(1) *= -1;
        Eigen::Isometry3d tf_inv = tf_camera.inverse();

        // cout << "tf" << endl;
        // cout << tf_inv.matrix() << endl;
        // cout << tf_camera.matrix() << endl;

        // 
        for (int i = 0; i < msg->detections.size(); i++) {
            const apriltags::AprilTagDetection& d = msg->detections[i];
            int index = FindIndex(d.id);

            if (index == -1) {
                cout << "Got bad tag: " << d.id << endl;
                continue;
            }

            Eigen::Vector3d position;
            tf::pointMsgToEigen(d.pose.position, position);
            position *= 1000;
            position = tf_inv * position;
            // position = -position + tf_camera.translation();
            
            Eigen::Quaterniond quat;
            tf::quaternionMsgToEigen(d.pose.orientation, quat);
            Eigen::Matrix3d rot = quat.toRotationMatrix();
            std::atan2(rot(1,0), rot(0,0));

            mState[3*index+0] = position.x(); // to mm
            mState[3*index+1] = position.y(); // to mm
            // mState[3*index+2] = quat.angularDistance(Eigen::Quaterniond::Identity());
            mState[3*index+2] = std::atan2(rot(1,0), rot(0,0));

            mFresh[index] = 1;
        }

        if (mFresh.sum() == NumBlocks()) {
            deep2d_ros::WorldState worldstate;
            worldstate.state.resize(mState.size());
            for (int i = 0; i < mState.size(); i++) {
                worldstate.state[i] = mState[i];
            }
            mPub->publish(worldstate);
            mFresh.setZero();
            static int fresh_count = 0;
            cout << "Fresh " << fresh_count++ << endl;
        }
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "worldstate_node");
    ros::NodeHandle n;

    DetectionsCallback detections(9, 9, 0, 6);

    ros::Subscriber sub0 = n.subscribe("/elp_cam_0/apriltags/detections", 10, &DetectionsCallback::callback0, &detections);
    ros::Subscriber sub1 = n.subscribe("/elp_cam_1/apriltags/detections", 10, &DetectionsCallback::callback1, &detections);
    ros::Subscriber sub2 = n.subscribe("/elp_cam_2/apriltags/detections", 10, &DetectionsCallback::callback2, &detections);
    ros::Subscriber sub3 = n.subscribe("/elp_cam_3/apriltags/detections", 10, &DetectionsCallback::callback3, &detections);
    ros::Publisher pub = n.advertise<deep2d_ros::WorldState>("/worldstate", 10);
    detections.mPub = &pub;

    // ros::Rate r(3);
    // while (true) {
    //     apriltags::AprilTagDetectionsConstPtr msg;
    //     msg = ros::topic::waitForMessage<apriltags::AprilTagDetections>("/elp_cam_0/apriltags/detections", n);
    //     detections.callback0(msg);
    //     msg = ros::topic::waitForMessage<apriltags::AprilTagDetections>("/elp_cam_1/apriltags/detections", n);
    //     detections.callback1(msg);
    //     msg = ros::topic::waitForMessage<apriltags::AprilTagDetections>("/elp_cam_2/apriltags/detections", n);
    //     detections.callback2(msg);
    //     msg = ros::topic::waitForMessage<apriltags::AprilTagDetections>("/elp_cam_3/apriltags/detections", n);
    //     detections.callback3(msg);
    //     ros::spinOnce();
    //     r.sleep();
    // }

    ros::spin();
}
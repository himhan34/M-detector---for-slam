#include <omp.h> // OpenMP를 사용하기 위한 헤더
#include <mutex> // 뮤텍스를 사용하기 위한 헤더
#include <math.h> // 수학 함수를 사용하기 위한 헤더
#include <thread> // 멀티스레딩을 사용하기 위한 헤더
#include <fstream> // 파일 입출력을 위한 헤더
#include <iostream> // 표준 입출력을 위한 헤더
#include <csignal> // 신호 처리를 위한 헤더
#include <unistd.h> // 유닉스 시스템 호출을 위한 헤더
#include <Python.h> // Python API를 사용하기 위한 헤더
// #include <so3_math.h> // SO(3) 수학 헤더 (주석 처리됨)
#include <ros/ros.h> // ROS 기본 헤더
#include <Eigen/Core> // Eigen 라이브러리의 코어 기능을 사용하기 위한 헤더
#include <types.h> // 사용자 정의 타입을 포함한 헤더
#include <m-detector/DynObjFilter.h> // 동적 객체 필터 헤더
#include <nav_msgs/Odometry.h> // 오도메트리 메시지를 사용하기 위한 헤더
#include <nav_msgs/Path.h> // 경로 메시지를 사용하기 위한 헤더
#include <visualization_msgs/Marker.h> // 시각화 마커 메시지를 사용하기 위한 헤더
#include <pcl_conversions/pcl_conversions.h> // PCL과 ROS 메시지 간 변환을 위한 헤더
#include <pcl/point_cloud.h> // PCL 포인트 클라우드 클래스를 사용하기 위한 헤더
#include <pcl/point_types.h> // PCL 포인트 타입을 사용하기 위한 헤더
#include <pcl/filters/voxel_grid.h> // Voxel Grid 필터를 사용하기 위한 헤더
#include <pcl/io/pcd_io.h> // PCD 파일 입출력을 위한 헤더
#include <sensor_msgs/PointCloud2.h> // 포인트 클라우드 메시지를 사용하기 위한 헤더
#include <tf/transform_datatypes.h> // TF 변환 데이터 타입을 사용하기 위한 헤더
#include <tf/transform_broadcaster.h> // TF 브로드캐스터를 사용하기 위한 헤더
#include <geometry_msgs/Vector3.h> // 3D 벡터 메시지를 사용하기 위한 헤더
#include <pcl/filters/random_sample.h> // 랜덤 샘플 필터를 사용하기 위한 헤더
#include <unistd.h> // 유닉스 시스템 호출을 위한 헤더 (중복됨)
#include <dirent.h> // 디렉터리 관리를 위한 헤더
#include <iomanip> // 입출력 포맷 설정을 위한 헤더
#include <livox_ros_driver/CustomMsg.h> // Livox LIDAR 메시지를 사용하기 위한 헤더

using namespace std; // 표준 네임스페이스를 사용합니다

// 마지막 포인트 클라우드를 저장하는 변수
pcl::PointCloud<pcl::PointXYZINormal> lastcloud;
PointCloudXYZI::Ptr last_pc(new PointCloudXYZI()); // 마지막 포인트 클라우드를 위한 포인터

// ROS 퍼블리셔를 선언합니다
ros::Publisher pub_pointcloud, pub_marker, pub_iou_view, pub_static, pub_dynamic;
ros::Publisher pub_tp, pub_fp, pub_fn, pub_tn; // 진짜 양성, 거짓 양성, 거짓 음성, 진짜 음성 퍼블리셔

// 포인트 클라우드 토픽과 프레임 ID를 설정합니다
string points_topic = "/velodyne_points_revise";
string frame_id = "camera_init"; // 초기 프레임 ID

// 폴더 경로를 저장할 문자열 변수
string pc_folder, label_folder, pred_folder, iou_file;

// 총 진짜 양성, 거짓 음성, 거짓 양성의 개수를 저장하는 변수
int total_tp = 0, total_fn = 0, total_fp = 0;

// 프레임 수와 마이너스 수를 저장하는 변수
int frames = 0, minus_num = 1;


void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in)
{
    // 새로운 포인트 클라우드를 생성하고 ROS 메시지를 PCL 포인트 클라우드로 변환합니다
    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *points_in);

    // 프레임이 설정된 마이너스 수보다 작을 경우 메시지를 퍼블리시합니다
    if (frames < minus_num)
    {
        // 포인트 클라우드를 ROS 메시지로 변환하여 퍼블리시합니다
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_in, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = frame_id;
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);
    }
    else
    {   
        // 프레임 정보를 출력합니다
        cout << "frame: " << frames << endl;

        // 예측 파일의 경로를 생성합니다
        string pred_file = pred_folder;
        stringstream sss;
        sss << setw(6) << setfill('0') << frames - minus_num;
        pred_file += sss.str(); 
        pred_file.append(".label");

        // 예측 파일을 바이너리 모드로 엽니다
        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if (!pred_input.good())
        {
            // 예측 파일을 읽을 수 없으면 오류를 출력하고 종료합니다
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        // 동적 및 정적 객체를 저장할 포인트 클라우드를 생성합니다
        pcl::PointCloud<pcl::PointXYZI>::Ptr points_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_out(new pcl::PointCloud<pcl::PointXYZI>);

        // 평가 지표를 초기화합니다
        int tp = 0, fn = 0, fp = 0, count = 0;
        float iou = 0.0f;

        // 입력 포인트 클라우드의 모든 점에 대해 반복합니다
        for (int i = 0; i < points_in->points.size(); i++) 
        {
            pcl::PointXYZI point;
            point.x = points_in->points[i].x;
            point.y = points_in->points[i].y;
            point.z = points_in->points[i].z;

            // 예측 번호를 읽습니다
            int pred_num;
            pred_input.read((char *)&pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF; // 필요한 부분만 추출합니다

            point.intensity = 0;

            // 예측 번호가 251 이상일 경우 동적 객체로 간주합니다
            if (pred_num >= 251)
            {
                point.intensity = 10;
                iou_out->push_back(point);
                dynamic_out->push_back(point);
            }
            else // 그렇지 않으면 정적 객체로 간주합니다
            {
                point.intensity = 20;
                iou_out->push_back(point);
                static_out->push_back(point);
            }
            
            // 모든 점을 출력 포인트 클라우드에 추가합니다
            points_out->push_back(point);
        }

        // 포인트 클라우드를 ROS 메시지로 변환하여 퍼블리시합니다
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_out, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = frame_id;
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);

        // IOU를 계산하기 위한 포인트 클라우드를 퍼블리시합니다
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*iou_out, pcl_msg);
        pcl_msg.header.frame_id = frame_id;
        pcl_msg.header.stamp = ros::Time::now();
        pub_iou_view.publish(pcl_msg); 

        // 동적 객체 포인트 클라우드를 퍼블리시합니다
        sensor_msgs::PointCloud2 dynamic_msg;
        pcl::toROSMsg(*dynamic_out, dynamic_msg);
        dynamic_msg.header.frame_id = frame_id;
        dynamic_msg.header.stamp = ros::Time::now();
        pub_dynamic.publish(dynamic_msg); 

        // 정적 객체 포인트 클라우드를 퍼블리시합니다
        sensor_msgs::PointCloud2 static_msg;
        pcl::toROSMsg(*static_out, static_msg);
        static_msg.header.frame_id = frame_id;
        static_msg.header.stamp = ros::Time::now();
        pub_static.publish(static_msg);

        // 마커를 생성하여 퍼블리시합니다
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        // 마커 크기와 색상을 설정합니다
        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 255;
        marker.color.a = 1;  
        
        // 마커의 위치를 설정합니다
        geometry_msgs::Pose pose;
        pose.position.x = points_out->points[0].x;
        pose.position.y = points_out->points[0].y;
        pose.position.z = points_out->points[0].z;

        // 마커에 텍스트를 설정합니다
        ostringstream str;
        str << "tp: " << tp << " fn: " << fn << " fp: " << fp << " count: " << count << " iou: " << iou;
        marker.text = str.str();
        marker.pose = pose;
        pub_marker.publish(marker);
    }
    
    frames++; // 프레임을 증가시킵니다
    // pred_input.seekg(0, std::ios::beg); // 예측 입력의 시작으로 돌아갑니다 (주석 처리됨)
}


void AviaPointsCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
    // 새로운 포인트 클라우드를 생성하고 크기를 설정합니다
    PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
    points_in->resize(msg_in->point_num);

    // 포인트 수를 출력합니다
    std::cout << "points size: " << msg_in->point_num << std::endl;
    if (msg_in->point_num == 0) return; // 포인트 수가 0이면 함수 종료

    // 포인트 데이터를 복사합니다
    for (int i = 0; i < msg_in->point_num; i++)
    {
        points_in->points[i].x = msg_in->points[i].x;
        points_in->points[i].y = msg_in->points[i].y;
        points_in->points[i].z = msg_in->points[i].z;
    }

    // 프레임이 설정된 마이너스 수보다 작을 경우 메시지를 퍼블리시합니다
    if (frames < minus_num)
    {
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_in, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = frame_id;
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);
    }
    else
    {
        // 프레임 번호를 출력합니다
        cout << "frame: " << frames << endl;

        // 예측 파일의 경로를 생성합니다
        string pred_file = pred_folder;
        stringstream sss;
        sss << setw(6) << setfill('0') << frames - minus_num;
        pred_file += sss.str();
        pred_file.append(".label");

        // 예측 파일을 바이너리 모드로 엽니다
        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
        if (!pred_input.good())
        {
            // 예측 파일을 읽을 수 없으면 오류를 출력하고 종료합니다
            std::cerr << "Could not read prediction file: " << pred_file << std::endl;
            exit(EXIT_FAILURE);
        }

        // 포인트 클라우드를 저장할 변수들을 생성합니다
        pcl::PointCloud<pcl::PointXYZI>::Ptr points_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_out(new pcl::PointCloud<pcl::PointXYZI>);

        // 평가 지표를 초기화합니다
        int tp = 0, fn = 0, fp = 0, count = 0;
        float iou = 0.0f;

        // 모든 입력 포인트에 대해 반복합니다
        for (int i = 0; i < points_in->points.size(); i++)
        {
            pcl::PointXYZI point;
            point.x = points_in->points[i].x;
            point.y = points_in->points[i].y;
            point.z = points_in->points[i].z;

            // 예측 번호를 읽어옵니다
            int pred_num;
            pred_input.read((char *)&pred_num, sizeof(int));
            pred_num = pred_num & 0xFFFF; // 필요한 부분만 추출합니다

            point.intensity = 0;

            // 예측 번호가 251 이상일 경우 동적 객체로 간주합니다
            if (pred_num >= 251)
            {
                point.intensity = 10;
                iou_out->push_back(point);
                dynamic_out->push_back(point);
            }
            else // 그렇지 않으면 정적 객체로 간주합니다
            {
                point.intensity = 20;
                iou_out->push_back(point);
                static_out->push_back(point);
            }

            // 모든 포인트를 출력 포인트 클라우드에 추가합니다
            points_out->push_back(point);
        }

        // 포인트 클라우드를 ROS 메시지로 변환하여 퍼블리시합니다
        sensor_msgs::PointCloud2 pcl_ros_msg;
        pcl::toROSMsg(*points_out, pcl_ros_msg);
        pcl_ros_msg.header.frame_id = frame_id;
        pcl_ros_msg.header.stamp = ros::Time::now();
        pub_pointcloud.publish(pcl_ros_msg);

        // IOU를 계산하기 위한 포인트 클라우드를 퍼블리시합니다
        sensor_msgs::PointCloud2 pcl_msg;
        pcl::toROSMsg(*iou_out, pcl_msg);
        pcl_msg.header.frame_id = frame_id;
        pcl_msg.header.stamp = ros::Time::now();
        pub_iou_view.publish(pcl_msg);

        // 동적 객체 포인트 클라우드를 퍼블리시합니다
        sensor_msgs::PointCloud2 dynamic_msg;
        pcl::toROSMsg(*dynamic_out, dynamic_msg);
        dynamic_msg.header.frame_id = frame_id;
        dynamic_msg.header.stamp = ros::Time::now();
        pub_dynamic.publish(dynamic_msg);

        // 정적 객체 포인트 클라우드를 퍼블리시합니다
        sensor_msgs::PointCloud2 static_msg;
        pcl::toROSMsg(*static_out, static_msg);
        static_msg.header.frame_id = frame_id;
        static_msg.header.stamp = ros::Time::now();
        pub_static.publish(static_msg);

        // 마커를 생성하여 퍼블리시합니다
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        // 마커 크기와 색상을 설정합니다
        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 255;
        marker.color.a = 1;

        // 마커의 위치를 설정합니다
        geometry_msgs::Pose pose;
        pose.position.x = points_out->points[0].x;
        pose.position.y = points_out->points[0].y;
        pose.position.z = points_out->points[0].z;

        // 마커에 텍스트를 설정합니다
        ostringstream str;
        str << "tp: " << tp << " fn: " << fn << " fp: " << fp << " count: " << count << " iou: " << iou;
        marker.text = str.str();
        marker.pose = pose;
        pub_marker.publish(marker);
    }

    frames++; // 프레임 수를 증가시킵니다
    // pred_input.seekg(0, std::ios::beg); // 예측 입력을 시작 위치로 이동합니다 (주석 처리됨)
}

int main(int argc, char** argv)
{
    // ROS 노드를 초기화합니다
    ros::init(argc, argv, "display_prediction");
    ros::NodeHandle nh;

    // 파라미터를 가져옵니다
    nh.param<string>("dyn_obj/pc_file", pc_folder, "");
    nh.param<string>("dyn_obj/pred_file", pred_folder, "");
    nh.param<string>("dyn_obj/pc_topic", points_topic, "/velodyne_points");
    nh.param<string>("dyn_obj/frame_id", frame_id, "camera_init");

    // 예측 파일의 수를 계산합니다
    int pred_num = 0;
    DIR* pred_dir;
    pred_dir = opendir(pred_folder.c_str());
    struct dirent* pred_ptr;
    while ((pred_ptr = readdir(pred_dir)) != NULL)
    {
        if (pred_ptr->d_name[0] == '.') { continue; } // 숨김 파일을 무시합니다
        pred_num++;
    }
    closedir(pred_dir);

    // 초기 프레임 수를 설정합니다
    minus_num = 0;

    // 퍼블리셔를 설정합니다
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/result_view", 100000);
    pub_marker = nh.advertise<visualization_msgs::Marker>("/m_detector/text_view", 10);
    pub_iou_view = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/iou_view", 100000);
    pub_static = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/std_points", 100000);
    pub_dynamic = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/dyn_points", 100000);

    // 포인트 클라우드 토픽에 대한 구독자를 설정합니다
    ros::Subscriber sub_pcl;
    if (points_topic == "/livox/lidar")
    {
        // Livox LIDAR 토픽에 대한 콜백을 설정합니다
        sub_pcl = nh.subscribe(points_topic, 200000, AviaPointsCallback);
    }
    else
    {
        // 기본 포인트 클라우드 토픽에 대한 콜백을 설정합니다
        sub_pcl = nh.subscribe(points_topic, 200000, PointsCallback);
    }

    // ROS 이벤트 루프를 실행합니다
    ros::spin();
    return 0;
}


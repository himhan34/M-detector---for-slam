#include <ros/ros.h> // ROS 기본 헤더를 포함합니다
#include <omp.h> // OpenMP를 사용하기 위한 헤더를 포함합니다
#include <mutex> // 뮤텍스(상호 배제)를 사용하기 위한 헤더를 포함합니다
#include <math.h> // 수학 함수를 사용하기 위한 헤더를 포함합니다
#include <thread> // 멀티스레딩을 사용하기 위한 헤더를 포함합니다
#include <fstream> // 파일 입출력을 위한 헤더를 포함합니다
#include <iostream> // 표준 입출력을 위한 헤더를 포함합니다
#include <csignal> // 신호 처리를 위한 헤더를 포함합니다
#include <unistd.h> // 유닉스 시스템 호출을 위한 헤더를 포함합니다
#include <Python.h> // Python API를 사용하기 위한 헤더를 포함합니다
#include <ros/ros.h> // ROS 헤더가 중복 포함되었습니다 (중복된 헤더는 제거 가능)
#include <Eigen/Core> // Eigen 라이브러리의 코어 기능을 사용하기 위한 헤더를 포함합니다
#include <types.h> // 사용자 정의 타입이 포함된 헤더를 포함합니다
#include <m-detector/DynObjFilter.h> // 동적 객체 필터 클래스 헤더를 포함합니다
#include <nav_msgs/Odometry.h> // 오도메트리 메시지를 사용하기 위한 헤더를 포함합니다
#include <nav_msgs/Path.h> // 경로 메시지를 사용하기 위한 헤더를 포함합니다
#include <visualization_msgs/Marker.h> // 시각화 마커 메시지를 사용하기 위한 헤더를 포함합니다
#include <pcl_conversions/pcl_conversions.h> // PCL과 ROS 메시지 간 변환을 위한 헤더를 포함합니다
#include <pcl/point_cloud.h> // PCL 포인트 클라우드 클래스를 사용하기 위한 헤더를 포함합니다
#include <pcl/point_types.h> // PCL 포인트 타입을 정의하기 위한 헤더를 포함합니다
#include <pcl/filters/voxel_grid.h> // Voxel Grid 필터를 사용하기 위한 헤더를 포함합니다
#include <pcl/io/pcd_io.h> // PCD 파일 입출력을 위한 헤더를 포함합니다
#include <sensor_msgs/PointCloud2.h> // 포인트 클라우드 메시지를 사용하기 위한 헤더를 포함합니다
#include <tf/transform_datatypes.h> // TF 변환 데이터 타입을 사용하기 위한 헤더를 포함합니다
#include <tf/transform_broadcaster.h> // TF 브로드캐스터를 사용하기 위한 헤더를 포함합니다
#include <geometry_msgs/Vector3.h> // 3D 벡터 메시지를 사용하기 위한 헤더를 포함합니다
#include <pcl/filters/random_sample.h> // 랜덤 샘플 필터를 사용하기 위한 헤더를 포함합니다
#include <Eigen/Eigen> // Eigen 라이브러리의 모든 기능을 사용하기 위한 헤더를 포함합니다
#include <eigen_conversions/eigen_msg.h> // Eigen과 ROS 메시지 간 변환을 위한 헤더를 포함합니다
#include <deque> // 덱 컨테이너를 사용하기 위한 헤더를 포함합니다

// #include "preprocess.h" // 전처리 관련 헤더가 주석 처리되었습니다

using namespace std; // 표준 네임스페이스를 사용합니다

// 동적 객체 필터에 대한 공유 포인터를 생성합니다
shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());

// 현재 회전 행렬을 항등 행렬로 초기화합니다
M3D cur_rot = Eigen::Matrix3d::Identity();
// 현재 위치 벡터를 영벡터로 초기화합니다
V3D cur_pos = Eigen::Vector3d::Zero();

// 전역 변수들을 설정합니다
int QUAD_LAYER_MAX = 1; // 최대 쿼드 레이어 수를 설정합니다
int occlude_windows = 3; // 오클루전 윈도우 수를 설정합니다
int point_index = 0; // 포인트 인덱스를 초기화합니다
float VER_RESOLUTION_MAX = 0.01; // 최대 수직 해상도를 설정합니다
float HOR_RESOLUTION_MAX = 0.01; // 최대 수평 해상도를 설정합니다
float angle_noise = 0.001; // 각도 노이즈를 설정합니다
float angle_occlude = 0.02; // 오클루전 각도를 설정합니다
float dyn_windows_dur = 0.5; // 동적 윈도우 지속 시간을 설정합니다
bool dyn_filter_en = true, dyn_filter_dbg_en = true; // 동적 필터와 디버그 필터의 활성화 여부를 설정합니다
string points_topic, odom_topic; // 포인트와 오도메트리 토픽을 위한 문자열 변수들을 선언합니다
string out_folder, out_folder_origin; // 출력 폴더 경로를 위한 문자열 변수들을 선언합니다
double lidar_end_time = 0; // 라이다 종료 시간을 초기화합니다
int dataset = 0; // 데이터셋 번호를 초기화합니다
int cur_frame = 0; // 현재 프레임 번호를 초기화합니다

// 덱 컨테이너를 사용하여 데이터 버퍼를 설정합니다
deque<M3D> buffer_rots; // 회전 행렬 버퍼를 설정합니다
deque<V3D> buffer_poss; // 위치 벡터 버퍼를 설정합니다
deque<double> buffer_times; // 시간 버퍼를 설정합니다
deque<boost::shared_ptr<PointCloudXYZI>> buffer_pcs; // 포인트 클라우드 버퍼를 설정합니다

// ROS 퍼블리셔를 선언합니다
ros::Publisher pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std; // 동적 포인트 클라우드 퍼블리셔를 선언합니다

void OdomCallback(const nav_msgs::Odometry &cur_odom) // 오도메트리 콜백 함수입니다
{
    Eigen::Quaterniond cur_q; // 현재 쿼터니언을 저장할 변수를 선언합니다
    geometry_msgs::Quaternion tmp_q; // 임시 쿼터니언 메시지를 선언합니다
    tmp_q = cur_odom.pose.pose.orientation; // 현재 오도메트리의 쿼터니언을 임시 변수에 복사합니다
    tf::quaternionMsgToEigen(tmp_q, cur_q); // 쿼터니언 메시지를 Eigen 형식으로 변환합니다
    cur_rot = cur_q.matrix(); // 현재 회전 행렬을 설정합니다
    cur_pos << cur_odom.pose.pose.position.x, cur_odom.pose.pose.position.y, cur_odom.pose.pose.position.z; // 현재 위치 벡터를 설정합니다
    buffer_rots.push_back(cur_rot); // 회전 행렬을 버퍼에 추가합니다
    buffer_poss.push_back(cur_pos); // 위치 벡터를 버퍼에 추가합니다
    lidar_end_time = cur_odom.header.stamp.toSec(); // 라이다 종료 시간을 설정합니다
    buffer_times.push_back(lidar_end_time); // 시간을 버퍼에 추가합니다
}

void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in) // 포인트 클라우드 콜백 함수입니다
{
    boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI()); // 포인트 클라우드 데이터를 저장할 포인터를 생성합니다
    pcl::fromROSMsg(*msg_in, *feats_undistort); // ROS 메시지를 PCL 포인트 클라우드로 변환합니다
    buffer_pcs.push_back(feats_undistort); // 변환된 포인트 클라우드를 버퍼에 추가합니다
}

void TimerCallback(const ros::TimerEvent& e) // 타이머 콜백 함수입니다
{
    if(buffer_pcs.size() > 0 && buffer_poss.size() > 0 && buffer_rots.size() > 0 && buffer_times.size() > 0) // 버퍼들이 비어 있지 않은 경우
    {
        boost::shared_ptr<PointCloudXYZI> cur_pc = buffer_pcs.at(0); // 현재 포인트 클라우드를 가져옵니다
        buffer_pcs.pop_front(); // 포인트 클라우드를 버퍼에서 제거합니다
        auto cur_rot = buffer_rots.at(0); // 현재 회전 행렬을 가져옵니다
        buffer_rots.pop_front(); // 회전 행렬을 버퍼에서 제거합니다
        auto cur_pos = buffer_poss.at(0); // 현재 위치 벡터를 가져옵니다
        buffer_poss.pop_front(); // 위치 벡터를 버퍼에서 제거합니다
        auto cur_time = buffer_times.at(0); // 현재 시간을 가져옵니다
        buffer_times.pop_front(); // 시간을 버퍼에서 제거합니다

        string file_name = out_folder; // 출력 파일 이름의 기본 경로를 설정합니다
        stringstream ss; // 문자열 스트림을 선언합니다
        ss << setw(6) << setfill('0') << cur_frame; // 프레임 번호를 6자리로 설정하고 0으로 채웁니다
        file_name += ss.str(); // 파일 이름에 프레임 번호를 추가합니다
        file_name.append(".label"); // 파일 확장자를 추가합니다

        string file_name_origin = out_folder_origin; // 원본 출력 파일 경로를 설정합니다
        stringstream sss; // 또 다른 문자열 스트림을 선언합니다
        sss << setw(6) << setfill('0') << cur_frame; // 프레임 번호를 6자리로 설정하고 0으로 채웁니다
        file_name_origin += sss.str(); // 원본 파일 이름에 프레임 번호를 추가합니다
        file_name_origin.append(".label"); // 원본 파일 확장자를 추가합니다

        if(file_name.length() > 15 || file_name_origin.length() > 15) // 파일 이름 길이를 확인합니다
            DynObjFilt->set_path(file_name, file_name_origin); // 파일 경로를 동적 객체 필터에 설정합니다

        DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time); // 동적 객체 필터링을 수행합니다
        DynObjFilt->publish_dyn(pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std, cur_time); // 필터링 결과를 퍼블리시합니다
        cur_frame++; // 현재 프레임 번호를 증가시킵니다
    }
}

int main(int argc, char** argv) // 메인 함수입니다
{
    ros::init(argc, argv, "dynfilter_odom"); // ROS 노드를 초기화합니다
    ros::NodeHandle nh; // 노드 핸들을 생성합니다
    nh.param<string>("dyn_obj/points_topic", points_topic, ""); // 포인트 클라우드 토픽을 설정합니다
    nh.param<string>("dyn_obj/odom_topic", odom_topic, ""); // 오도메트리 토픽을 설정합니다
    nh.param<string>("dyn_obj/out_file", out_folder,""); // 출력 폴더 경로를 설정합니다
    nh.param<string>("dyn_obj/out_file_origin", out_folder_origin, ""); // 원본 출력 폴더 경로를 설정합니다

    DynObjFilt->init(nh); // 동적 객체 필터를 초기화합니다

    /*** ROS subscribe and publisher initialization ***/
    pub_pcl_dyn_extend = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/frame_out", 10000); // 확장된 포인트 클라우드 퍼블리셔를 설정합니다
    pub_pcl_dyn = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/point_out", 100000); // 포인트 클라우드 퍼블리셔를 설정합니다
    pub_pcl_std  = nh.advertise<sensor_msgs::PointCloud2>("/m_detector/std_points", 100000); // 표준 포인트 클라우드 퍼블리셔를 설정합니다
    ros::Subscriber sub_pcl = nh.subscribe(points_topic, 200000, PointsCallback); // 포인트 클라우드 구독자를 설정합니다
    ros::Subscriber sub_odom = nh.subscribe(odom_topic, 200000, OdomCallback); // 오도메트리 구독자를 설정합니다
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), TimerCallback); // 타이머를 설정하여 주기적으로 콜백을 호출합니다

    ros::spin(); // ROS 이벤트 루프를 실행합니다
    return 0; // 프로그램을 종료합니다
}





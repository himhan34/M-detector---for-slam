#include <m-detector/DynObjCluster.h> // DynObjCluster 클래스 정의를 포함하는 헤더 파일
#include <cluster_predict/EA_disk.h> // EA_disk 클래스 정의를 포함하는 헤더 파일
#include <algorithm> // 알고리즘 함수들 사용을 위해 포함
#include <chrono> // 시간 측정 함수들 사용을 위해 포함
#include <execution> // 병렬 실행 정책을 위해 포함

// void DynObjCluster::Init(ros::Publisher &pub_pcl_dyn_extend_in, ros::Publisher &cluster_vis_high_in, ros::Publisher &pub_ground_points_in)
// 주석 처리된 ROS 퍼블리셔 초기화 함수
void DynObjCluster::Init()
{
    // pub_pcl_dyn_extend = pub_pcl_dyn_extend_in; // 퍼블리셔 pub_pcl_dyn_extend를 초기화 (현재 주석 처리됨)
    // cluster_vis_high = cluster_vis_high_in; // 퍼블리셔 cluster_vis_high를 초기화 (현재 주석 처리됨)
    // pub_ground_points = pub_ground_points_in; // 퍼블리셔 pub_ground_points를 초기화 (현재 주석 처리됨)
    
    // xyz_origin 초기화 (기존 값: -20, -20, -20은 주석 처리됨)
    // xyz_origin << -20., -20., -20.;
    
    // maprange 초기화 (기존 값: 40, 40, 40은 주석 처리됨)
    // maprange << 40., 40., 40.;
    
    // xyz_origin 초기화 (새로운 값: -100, -100, -20)
    xyz_origin << -100., -100., -20.;
    
    // maprange 초기화 (새로운 값: 200, 200, 40)
    maprange << 200., 200., 40.;
    
    // 그리드 맵의 XY 방향 크기를 Voxel_revolusion으로 나눈 후 올림하여 결정
    GridMapedgesize_xy = ceil(maprange(0) / Voxel_revolusion);
    
    // 그리드 맵의 Z 방향 크기를 Voxel_revolusion으로 나눈 후 올림하여 결정
    GridMapedgesize_z = ceil(maprange(2) / Voxel_revolusion);
    
    // 전체 그리드 맵의 크기를 XY 및 Z 크기를 곱하여 계산
    GridMapsize = GridMapedgesize_xy * GridMapedgesize_xy * GridMapedgesize_z;
    
    // 초기화 시작 메시지를 출력하고, 전체 그리드 맵 크기를 출력
    std::cout << "clustering init begin, please wait------------" << GridMapsize << std::endl;
    
    // umap 해시맵의 메모리를 GridMapsize 크기로 예약 및 초기화
    umap.reserve(GridMapsize);
    umap.resize(GridMapsize);
    
    // umap_ground 해시맵의 메모리를 GridMapsize 크기로 예약 및 초기화
    umap_ground.reserve(GridMapsize);
    umap_ground.resize(GridMapsize);
    
    // umap_insidebox 해시맵의 메모리를 GridMapsize 크기로 예약 및 초기화
    umap_insidebox.reserve(GridMapsize);
    umap_insidebox.resize(GridMapsize);
    
    // 초기화 완료 메시지를 출력
    std::cout << "clustering init finish------------" << std::endl;
    
    // out_file이 빈 문자열이 아닐 경우 파일을 쓰기 및 바이너리 모드로 열기
    if(out_file != "") out.open(out_file, std::ios::out  | std::ios::binary);
}


void DynObjCluster::Clusterprocess(std::vector<int> &dyn_tag, pcl::PointCloud<PointType> event_point, const pcl::PointCloud<PointType> &raw_point, const std_msgs::Header &header_in, const Eigen::Matrix3d odom_rot_in, const Eigen::Vector3d odom_pos_in)
{
    // 클러스터링 시작 시간을 ros::Time으로 설정
    cluster_begin = ros::Time::now();
    
    // 입력된 헤더, 위치 회전 행렬 및 위치 벡터를 클래스 멤버에 할당
    header = header_in;
    odom_rot = odom_rot_in;
    odom_pos = odom_pos_in;
    
    // 현재 시간 저장
    ros::Time t0 = ros::Time::now();
    
    // 시간 차이를 위한 delta_t 변수 설정
    float delta_t = 0.1;
    
    // 확장된 포인트 클라우드를 위한 객체 생성
    pcl::PointCloud<PointType> extend_points;
    
    // 클린된 포인트 클라우드 포인터 생성 및 event_point를 공유 포인터로 설정
    pcl::PointCloud<PointType>::Ptr cloud_clean_ptr(new pcl::PointCloud<PointType>);
    cloud_clean_ptr = event_point.makeShared();
    
    // 고가 클러스터의 경계 상자를 위한 객체 생성
    bbox_t bbox_high;
    
    // 클러스터링 및 추적 함수 호출
    ClusterAndTrack(dyn_tag, cloud_clean_ptr, pub_pcl_before_high, header, pub_pcl_after_high, cluster_vis_high, predict_path_high, bbox_high, delta_t, raw_point);
    
    // 클러스터링 완료 후 현재 시간 저장
    ros::Time t3 = ros::Time::now();
    
    // 총 소요 시간 계산
    time_total = (ros::Time::now() - t0).toSec();
    
    // 총 시간 평균을 계산하기 위한 시간 지표 증가
    time_ind++;
    time_total_average = time_total_average * (time_ind - 1) / time_ind + time_total / time_ind;
    
    // 현재 프레임 증가
    cur_frame += 1;
}

void DynObjCluster::ClusterAndTrack(std::vector<int> &dyn_tag, pcl::PointCloud<PointType>::Ptr &points_in, ros::Publisher points_in_msg, std_msgs::Header header_in,
                                    ros::Publisher points_out_msg, ros::Publisher cluster_vis, ros::Publisher predict_path, bbox_t &bbox, double delta,
                                    const pcl::PointCloud<PointType> &raw_point)
{
    // 포인트 클라우드를 ROS 메시지로 변환
    sensor_msgs::PointCloud2 pcl4_ros_msg;
    pcl::toROSMsg(*points_in, pcl4_ros_msg);
    
    // 변환된 메시지의 타임스탬프와 프레임 아이디를 설정
    pcl4_ros_msg.header.stamp = header_in.stamp;
    pcl4_ros_msg.header.frame_id = header_in.frame_id;
    
    // 클러스터 인덱스를 저장할 벡터 객체 생성
    std::vector<pcl::PointIndices> cluster_indices;
    
    // 보폭 클러스터를 저장할 2차원 벡터 객체 생성
    std::vector<std::vector<int>> voxel_clusters;
    
    // 현재 시간 저장
    ros::Time t0 = ros::Time::now();
    
    // 사용된 맵 셋을 위한 언오더드 셋 객체 생성
    std::unordered_set<int> used_map_set;
    
    // 보폭 클러스터링 결과를 얻기 위해 함수 호출
    GetClusterResult_voxel(points_in, umap, voxel_clusters, used_map_set);
    
    // 보폭 클러스터 결과를 퍼블리싱하기 위한 함수 호출
    PubClusterResult_voxel(dyn_tag, header_in, bbox, delta, voxel_clusters, raw_point, used_map_set);
}


void DynObjCluster::GetClusterResult(pcl::PointCloud<PointType>::Ptr points_in, std::vector<pcl::PointIndices> &cluster_indices)
{
    // 입력 포인트 클라우드의 크기가 2보다 작으면 클러스터링을 중단
    if (points_in->size() < 2)
    {
        return;
    }

    // KdTree 검색 객체 생성 및 입력 포인트 클라우드 설정
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(points_in);

    // DBSCAN 클러스터링 객체 생성 및 설정
    DBSCANKdtreeCluster<PointType> ec;
    ec.setCorePointMinPts(nn_points_size); // 코어 포인트의 최소 포인트 개수 설정
    ec.setClusterTolerance(nn_points_radius); // 클러스터 허용 반경 설정
    ec.setMinClusterSize(min_cluster_size); // 최소 클러스터 크기 설정
    ec.setMaxClusterSize(max_cluster_size); // 최대 클러스터 크기 설정
    ec.setSearchMethod(tree); // 검색 방법 설정
    ec.setInputCloud(points_in); // 입력 포인트 클라우드 설정

    // 클러스터링 시작 시간 저장
    ros::Time t0 = ros::Time::now();

    // 클러스터링 실행 및 결과 추출
    ec.extract(cluster_indices);
}

void DynObjCluster::GetClusterResult_voxel(pcl::PointCloud<PointType>::Ptr points_in, std::vector<Point_Cloud> &umap_in, std::vector<std::vector<int>> &voxel_clusters, std::unordered_set<int> &used_map_set)
{
    // 클러스터링 시작 시간 저장
    ros::Time t0 = ros::Time::now();

    // 출력 파일이 설정되어 있고 포인트 클라우드의 크기가 2보다 작으면 중단
    if ( (out_file != "") && points_in->size() < 2)
    {   
        out << (ros::Time::now() - t0).toSec() << " "; // 소요 시간을 파일에 기록
        return;
    }

    // VOXEL_CLUSTER 객체 생성 및 설정
    VOXEL_CLUSTER cluster;
    cluster.setInputCloud(*points_in); // 입력 포인트 클라우드 설정
    cluster.setVoxelResolution(Voxel_revolusion, GridMapedgesize_xy, GridMapedgesize_z, xyz_origin); // 보폭 해상도 설정
    cluster.setExtendRange(cluster_extend_pixel); // 확장 범위 설정
    cluster.setMinClusterSize(cluster_min_pixel_number); // 최소 클러스터 크기 설정
    cluster.createVoxelMap(umap_in, used_map_set); // 보폭 맵 생성
    cluster.extract(voxel_clusters); // 클러스터링 결과 추출

    // 출력 파일이 설정되어 있으면 소요 시간을 파일에 기록
    if(out_file != "") out << (ros::Time::now() - t0).toSec() << " ";
}

void DynObjCluster::PubClusterResult_voxel(std::vector<int> &dyn_tag, std_msgs::Header current_header, bbox_t &bbox, double delta,
                                           std::vector<std::vector<int>> &voxel_clusters, const pcl::PointCloud<PointType> &raw_point, std::unordered_set<int> &used_map_set)
{
    // 변수를 선언하고 초기화합니다.
    int j = 0; // 클러스터 인덱스를 초기화합니다.
    pcl::PointCloud<PointType> cluster_points; // 클러스터 점을 저장할 포인트 클라우드를 선언합니다.
    pcl::PointCloud<PointType> true_ground; // 실제 지면 점을 저장할 포인트 클라우드를 선언합니다.
    visualization_msgs::MarkerArray numbers; // 마커 배열을 선언합니다.
    numbers.markers.reserve(200); // 마커 배열의 크기를 200개로 예약합니다.
    cluster_points.reserve(raw_point.size()); // 클러스터 점의 크기를 raw_point의 크기만큼 예약합니다.
    true_ground.reserve(raw_point.size()); // 실제 지면 점의 크기를 raw_point의 크기만큼 예약합니다.
    Eigen::Matrix3f R = odom_rot.cast<float>(); // odom_rot 행렬을 float 타입으로 캐스팅하여 R에 저장합니다.
    Eigen::Vector3f world_z = R.col(2); // R 행렬의 세 번째 열을 world_z에 저장합니다.
    int Grid_size_1d = 3; // 1차원 그리드 크기를 3으로 설정합니다.
    int Grid_size = pow(Grid_size_1d, 3); // 전체 그리드 크기를 3차원으로 계산합니다.

    // 현재 시간을 저장합니다.
    ros::Time t0 = ros::Time::now();

    // 각 보텍스 클러스터에 대해 반복문을 실행합니다.
    for (auto it = voxel_clusters.begin(); it != voxel_clusters.end(); it++, j++)
    {
        Eigen::Vector3f xyz; // xyz 벡터를 선언합니다.
        XYZExtract(*(it->begin()), xyz); // 첫 번째 보텍스의 위치를 xyz에 추출합니다.
        float x_min = xyz(0), x_max = xyz(0); // x의 최소값과 최대값을 초기화합니다.
        float y_min = xyz(1), y_max = xyz(1); // y의 최소값과 최대값을 초기화합니다.
        float z_min = xyz(2), z_max = xyz(2); // z의 최소값과 최대값을 초기화합니다.
        int n = 0; // 점의 개수를 초기화합니다.

        // 현재 클러스터의 모든 보텍스에 대해 반복문을 실행합니다.
        for (auto pit = it->begin(); pit != it->end(); ++pit)
        {
            int voxel = *pit; // 보텍스 값을 가져옵니다.
            umap[voxel].bbox_index = j; // 해당 보텍스의 bbox 인덱스를 설정합니다.
            n = n + umap[voxel].points_num; // 점의 개수를 누적합니다.
            XYZExtract(voxel, xyz); // 보텍스의 위치를 xyz에 추출합니다.

            // x, y, z의 최소값을 갱신합니다.
            if (xyz(0) < x_min)
                x_min = xyz(0);
            if (xyz(1) < y_min)
                y_min = xyz(1);
            if (xyz(2) < z_min)
                z_min = xyz(2);

            // x, y, z의 최대값을 갱신합니다.
            if ((xyz(0) + Voxel_revolusion) > x_max)
                x_max = xyz(0) + Voxel_revolusion;
            if ((xyz(1) + Voxel_revolusion) > y_max)
                y_max = xyz(1) + Voxel_revolusion;
            if ((xyz(2) + Voxel_revolusion) > z_max)
                z_max = xyz(2) + Voxel_revolusion;
        }

        // x, y, z의 크기를 계산합니다.
        float x_size = x_max - x_min;
        float y_size = y_max - y_min;
        float z_size = z_max - z_min;
        
        // 조건을 확인하여 클러스터가 유효한지 검사합니다.
        if (cluster_min_pixel_number == 1 || (x_size > Voxel_revolusion + 0.001f && y_size > Voxel_revolusion + 0.001f) || 
            (x_size > Voxel_revolusion + 0.001f && z_size > Voxel_revolusion + 0.001f) || 
            (y_size > Voxel_revolusion + 0.001f && z_size > Voxel_revolusion + 0.001f))
        {
            // 새로운 포인트 클라우드를 생성하고 bbox에 추가합니다.
            pcl::PointCloud<PointType> clus_pcl;
            bbox.Point_cloud.push_back(clus_pcl);

            // 새로운 포인트 인덱스 벡터를 생성하고 bbox에 추가합니다.
            std::vector<int> new_point_indices;
            bbox.Point_indices.push_back(new_point_indices);

            // 중심 위치를 계산하고 PoseWithCovarianceStamped 메시지를 생성합니다.
            geometry_msgs::PoseWithCovarianceStamped center;
            center.header = current_header; // 헤더 설정
            center.pose.pose.position.x = (x_max + x_min) / 2; // x 중심 위치
            center.pose.pose.position.y = (y_max + y_min) / 2; // y 중심 위치
            center.pose.pose.position.z = (z_max + z_min) / 2; // z 중심 위치

            // 회전 설정 (기본 값)
            center.pose.pose.orientation.x = 0;
            center.pose.pose.orientation.y = 0;
            center.pose.pose.orientation.z = 0;
            center.pose.pose.orientation.w = 1;

            // 공분산 행렬 설정
            center.pose.covariance[0 * 6 + 0] = x_size / 2; // x 크기의 절반
            center.pose.covariance[1 * 6 + 1] = y_size / 2; // y 크기의 절반
            center.pose.covariance[2 * 6 + 2] = z_size / 2; // z 크기의 절반
            center.pose.covariance[3 * 6 + 3] = x_size; // x 크기
            center.pose.covariance[4 * 6 + 4] = y_size; // y 크기
            center.pose.covariance[5 * 6 + 5] = z_size; // z 크기

            // 공분산 행렬에 최대 및 최소 좌표를 추가합니다.
            center.pose.covariance[2 * 6 + 3] = x_max;
            center.pose.covariance[3 * 6 + 4] = y_max;
            center.pose.covariance[4 * 6 + 5] = z_max;
            center.pose.covariance[3 * 6 + 2] = x_min;
            center.pose.covariance[4 * 6 + 3] = y_min;
            center.pose.covariance[5 * 6 + 4] = z_min;

            // 중심 위치를 bbox에 추가합니다.
            bbox.Center.push_back(center);

            // 새로운 포인트 클라우드를 생성하고 bbox에 추가합니다.
            pcl::PointCloud<PointType> new_pcl;
            bbox.Ground_points.push_back(new_pcl);
            bbox.true_ground.push_back(new_pcl);

            // 새로운 집합 및 벡터를 생성하고 bbox에 추가합니다.
            std::unordered_set<int> new_set;
            bbox.Ground_voxels_set.push_back(new_set);
            std::vector<int> new_vec;
            bbox.Ground_voxels_vec.push_back(new_vec);

            // 점의 수를 bbox에 추가합니다.
            bbox.umap_points_num.push_back(n);
        }
        else
        {
            // 유효하지 않은 클러스터인 경우 인덱스를 감소시킵니다.
            j--;

            // 현재 클러스터의 모든 보텍스를 초기화합니다.
            for (auto v = it->begin(); v != it->end(); ++v)
            {
                umap[*v].reset(); // 보텍스를 초기화합니다.
            }
        }
    }


    // 현재 시간을 저장합니다.
    ros::Time t1 = ros::Time::now();

    // 새로운 시간 변수 초기화
    double hash_newtime = 0.0;

    // bbox.Center의 크기만큼 인덱스를 가진 벡터를 생성합니다.
    std::vector<int> index_bbox(bbox.Center.size());
    for (int i = 0; i < bbox.Center.size(); i++)
    {
        index_bbox[i] = i; // 각 인덱스를 초기화합니다.
    }

    // bbox.Center의 크기만큼 unordered_set 벡터를 생성합니다.
    std::vector<std::unordered_set<int>> used_map_set_vec(bbox.Center.size());

    // 병렬 실행을 사용하여 각 bbox의 인덱스에 대해 작업을 수행합니다.
    std::for_each(std::execution::par, index_bbox.begin(), index_bbox.end(), [&](const int &bbox_i)
                  {
        PointType center; // 중심 점을 저장할 변수를 선언합니다.
        float x_size = bbox.Center[bbox_i].pose.covariance[3*6+3]; // x 크기를 가져옵니다.
        float y_size = bbox.Center[bbox_i].pose.covariance[4*6+4]; // y 크기를 가져옵니다.
        float z_size = bbox.Center[bbox_i].pose.covariance[5*6+5]; // z 크기를 가져옵니다.

        // 중심 위치를 설정합니다.
        center.x = bbox.Center[bbox_i].pose.pose.position.x;
        center.y = bbox.Center[bbox_i].pose.pose.position.y;
        center.z = bbox.Center[bbox_i].pose.pose.position.z;

        // 최대 및 최소 좌표를 설정합니다.
        PointType max;
        max.x = bbox.Center[bbox_i].pose.covariance[2*6+3];
        max.y = bbox.Center[bbox_i].pose.covariance[3*6+4];
        max.z = bbox.Center[bbox_i].pose.covariance[4*6+5];

        PointType min;
        min.x = bbox.Center[bbox_i].pose.covariance[3*6+2];
        min.y = bbox.Center[bbox_i].pose.covariance[4*6+3];
        min.z = bbox.Center[bbox_i].pose.covariance[5*6+4];

        // x, y, z 방향의 보텍스 수를 계산합니다.
        int n_x = std::max(1.0f, 1.0f * x_size) / Voxel_revolusion;
        int n_y = std::max(1.0f, 1.0f * y_size) / Voxel_revolusion;
        int n_z = std::max(1.0f, 1.0f * z_size) / Voxel_revolusion;

        // 중심 보텍스의 인덱스를 계산합니다.
        int voxel_center = floor((center.x - xyz_origin(0))/Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z +
                           floor((center.y - xyz_origin(1))/Voxel_revolusion) * GridMapedgesize_z +
                           floor((center.z - xyz_origin(2))/Voxel_revolusion);

        int ii = 0; // x 방향 인덱스 변화 초기화
        Eigen::Vector3f xyz; // 보텍스의 위치를 저장할 변수를 선언합니다.

        // x 방향으로 보텍스를 반복합니다.
        for (int i = 0; i <= 2 * n_x + 1; i++)
        {
            ii += (i % 2 ? 1 : -1) * i; // i가 짝수인지 홀수인지에 따라 ii를 증가 또는 감소시킵니다.
            int jj = 0; // y 방향 인덱스 변화 초기화

            // y 방향으로 보텍스를 반복합니다.
            for (int j = 0; j <= 2 * n_y + 1; j++)
            {
                jj += (j % 2 ? 1 : -1) * j; // j가 짝수인지 홀수인지에 따라 jj를 증가 또는 감소시킵니다.
                int kk = 0; // z 방향 인덱스 변화 초기화

                // z 방향으로 보텍스를 반복합니다.
                for (int k = 0; k <= 2 * n_z + 1; k++)
                {
                    kk += (k % 2 ? 1 : -1) * k; // k가 짝수인지 홀수인지에 따라 kk를 증가 또는 감소시킵니다.
                    
                    // 보텍스 인덱스를 계산합니다.
                    int voxel = voxel_center + ii * GridMapedgesize_xy * GridMapedgesize_z + jj * GridMapedgesize_z + kk;

                    // 보텍스가 범위를 벗어났는지 확인합니다.
                    if (voxel < 0 || voxel > GridMapsize) continue;

                    // 보텍스의 위치를 추출합니다.
                    XYZExtract(voxel, xyz);
                    Eigen::Vector3f voxel_loc(xyz(0) + 0.5f * Voxel_revolusion, xyz(1) + 0.5f * Voxel_revolusion, xyz(2) + 0.5f * Voxel_revolusion);

                    // 보텍스가 지면 위에 있는지 확인하고, 조건에 따라 처리합니다.
                    if (umap[voxel].points_num == 0 && !((voxel_loc(0) > min.x && voxel_loc(0) < max.x) &&
                                                          (voxel_loc(1) > min.y && voxel_loc(1) < max.y) &&
                                                          (voxel_loc(2) > min.z && voxel_loc(2) < max.z)))
                    {
                        // 보텍스를 지면으로 설정합니다.
                        umap_ground[voxel].bbox_index = bbox_i;
                        used_map_set_vec[bbox_i].insert(voxel);
                        bbox.Ground_voxels_set[bbox_i].emplace(voxel);
                        bbox.Ground_voxels_vec[bbox_i].push_back(voxel);
                    }
                    else if (umap[voxel].points_num == 0 && (voxel_loc(0) > min.x && voxel_loc(0) < max.x) &&
                             (voxel_loc(1) > min.y && voxel_loc(1) < max.y) &&
                             (voxel_loc(2) > min.z && voxel_loc(2) < max.z))
                    {
                        // 보텍스를 내부 박스로 설정합니다.
                        umap_insidebox[voxel].bbox_index = bbox_i;
                        used_map_set_vec[bbox_i].insert(voxel);
                    }
                }
            }
        }
    });

        // 각 bbox의 사용된 맵 셋을 병합합니다.
    for (int bbox_i = 0; bbox_i < bbox.Center.size(); bbox_i++)
    {
        used_map_set.merge(used_map_set_vec[bbox_i]); // used_map_set에 각 used_map_set_vec의 요소를 병합합니다.
    }

    // 현재 시간을 저장합니다.
    ros::Time t2 = ros::Time::now();

    // 모든 raw_point에 대해 반복문을 실행합니다.
    for (int ite = 0; ite < raw_point.size(); ite++)
    {
        // 동적 태그가 -1인 경우 건너뜁니다.
        if (dyn_tag[ite] == -1)
            continue;

        // 보텍스 인덱스를 계산합니다.
        int voxel = floor((raw_point[ite].x - xyz_origin(0)) / Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z +
                    floor((raw_point[ite].y - xyz_origin(1)) / Voxel_revolusion) * GridMapedgesize_z +
                    floor((raw_point[ite].z - xyz_origin(2)) / Voxel_revolusion);

        // 보텍스가 범위를 벗어난 경우 건너뜁니다.
        if (voxel < 0 || voxel > GridMapsize)
        {
            continue;
        }

        // 보텍스가 지면 보텍스인 경우 처리합니다.
        if (umap_ground[voxel].bbox_index > -1)
        {
            // 지면 점 클라우드에 raw_point를 추가합니다.
            bbox.Ground_points[umap_ground[voxel].bbox_index].push_back(raw_point[ite]);

            // 보텍스의 점 개수가 0이면 초기화합니다.
            if (umap_ground[voxel].points_num == 0)
            {
                // 새로운 포인트 클라우드와 인덱스 벡터를 생성하고 초기화합니다.
                umap_ground[voxel].cloud.reset(new pcl::PointCloud<PointType>());
                umap_ground[voxel].cloud->reserve(5); // 메모리 예약
                umap_ground[voxel].cloud_index.reset(new std::vector<int>());
                umap_ground[voxel].cloud_index->reserve(5); // 메모리 예약

                // raw_point를 클라우드와 인덱스에 추가합니다.
                umap_ground[voxel].cloud->push_back(raw_point[ite]);
                umap_ground[voxel].cloud_index->push_back(ite);
            }
            else
            {
                // 기존 클라우드와 인덱스에 raw_point를 추가합니다.
                umap_ground[voxel].cloud->push_back(raw_point[ite]);
                umap_ground[voxel].cloud_index->push_back(ite);
            }

            // 보텍스의 점 개수를 증가시킵니다.
            umap_ground[voxel].points_num++;

            // 동적 태그를 0으로 설정합니다.
            dyn_tag[ite] = 0;
        }

                // 보텍스가 동적 객체 보텍스인 경우 처리합니다.
        else if (umap[voxel].points_num > 0 && umap[voxel].bbox_index > -1)
        {
            // raw_point를 tmp로 복사하고 곡률 값을 설정합니다.
            auto tmp = raw_point[ite];
            tmp.curvature = ite;

            // 포인트 클라우드 및 인덱스에 추가합니다.
            bbox.Point_cloud[umap[voxel].bbox_index].push_back(tmp);
            bbox.Point_indices[umap[voxel].bbox_index].push_back(ite);

            // 보텍스의 클라우드에 tmp를 추가합니다.
            umap[voxel].cloud->push_back(tmp);

            // 동적 태그를 1로 설정합니다.
            dyn_tag[ite] = 1;
        }
        // 보텍스가 내부 박스 보텍스인 경우 처리합니다.
        else if (umap_insidebox[voxel].bbox_index > -1)
        {
            // raw_point를 tmp로 복사하고 곡률 값을 설정합니다.
            auto tmp = raw_point[ite];
            tmp.curvature = ite;

            // 포인트 클라우드 및 인덱스에 추가합니다.
            bbox.Point_cloud[umap_insidebox[voxel].bbox_index].push_back(tmp);
            bbox.Point_indices[umap_insidebox[voxel].bbox_index].push_back(ite);

            // 보텍스의 점 개수가 0인 경우 초기화합니다.
            if (umap_insidebox[voxel].points_num == 0)
            {
                // 새로운 포인트 클라우드를 생성하고 초기화합니다.
                umap_insidebox[voxel].cloud.reset(new pcl::PointCloud<PointType>());
                umap_insidebox[voxel].cloud->reserve(5); // 메모리 예약
                umap_insidebox[voxel].cloud->push_back(tmp); // tmp를 클라우드에 추가합니다.
            }
            else
            {
                // 기존 클라우드에 tmp를 추가합니다.
                umap_insidebox[voxel].cloud->push_back(tmp);
            }

            // 보텍스의 점 개수를 증가시킵니다.
            umap_insidebox[voxel].points_num++;

            // 동적 태그를 1로 설정합니다.
            dyn_tag[ite] = 1;
        }
        // 해당하는 보텍스가 없는 경우 동적 태그를 0으로 설정합니다.
        else
        {
            dyn_tag[ite] = 0;
        }
    }

    // 변수 k를 0으로 초기화합니다.
    int k = 0;

    // 현재 시간을 저장합니다.
    ros::Time t3 = ros::Time::now();

    // ground_estimate 및 region_growth에 대한 시간 측정 변수를 생성합니다.
    std::vector<double> ground_estimate_total_time(index_bbox.size(), 0.0);
    std::vector<double> region_growth_time(index_bbox.size(), 0.0);

        // 병렬 실행을 사용하여 각 bbox에 대해 작업을 수행합니다.
    std::for_each(std::execution::par, index_bbox.begin(), index_bbox.end(), [&](const int &k)
    {
        // 중심 위치 및 크기 정보를 가져옵니다.
        geometry_msgs::PoseWithCovarianceStamped center = bbox.Center[k];
        float x_size = center.pose.covariance[3 * 6 + 3];
        float y_size = center.pose.covariance[4 * 6 + 4];
        float z_size = center.pose.covariance[5 * 6 + 5];
        float x_min = center.pose.covariance[3 * 6 + 2];
        float y_min = center.pose.covariance[4 * 6 + 3];
        float z_min = center.pose.covariance[5 * 6 + 4];
        float x_max = center.pose.covariance[2 * 6 + 3];
        float y_max = center.pose.covariance[3 * 6 + 4];
        float z_max = center.pose.covariance[4 * 6 + 5];

        // 지면 정규 벡터 및 평면을 초기화합니다.
        Eigen::Vector3f ground_norm(0.0, 0.0, 0.0);
        Eigen::Vector4f ground_plane;

        // 지면 추정을 수행하고 시간을 측정합니다.
        ros::Time t_ge = ros::Time::now();
        bool ground_detect = ground_estimate(bbox.Ground_points[k], world_z, ground_norm, ground_plane, bbox.true_ground[k], bbox.Ground_voxels_set[k]);
        ground_estimate_total_time[k] = (ros::Time::now() - t_ge).toSec();

        // 지면 정규 벡터를 회전 행렬 R에 설정합니다.
        Eigen::Matrix3f R;
        R.col(0) = ground_norm;

        // 지면이 탐지된 경우 처리합니다.
        if (ground_detect)
        {
            // 영역 확장을 수행하고 시간을 측정합니다.
            ros::Time t_rg = ros::Time::now();
            event_extend(R, ground_detect, bbox, dyn_tag, k);
            region_growth_time[k] = (ros::Time::now() - t_rg).toSec();

            // 지면을 제거합니다.
            ground_remove(ground_plane, bbox.Point_cloud[k], bbox.Point_indices[k], dyn_tag, bbox.true_ground[k], umap);
        }

        // 고립된 점을 제거합니다.
        isolate_remove(bbox.Point_cloud[k], bbox.Point_indices[k], dyn_tag);

        // 신뢰할 수 없는 경우 동적 태그를 0으로 설정합니다.
        if ((float)bbox.umap_points_num[k] / (float)bbox.Point_cloud[k].size() < thrustable_thresold)
        {
            for (int i = 0; i < bbox.Point_indices[k].size(); i++)
            {
                dyn_tag[bbox.Point_indices[k][i]] = 0;
            }
        }
        else
        {
            // 클러스터 점과 실제 지면 점을 누적합니다.
            cluster_points += bbox.Point_cloud[k];
            true_ground += bbox.true_ground[k];
        }
    });

    // 총 지면 추정 시간 및 영역 확장 시간을 계산합니다.
    double total_ground_estimate_total_time = 0.0;
    double total_region_growth_time = 0.0;
    for (int i = 0; i < index_bbox.size(); i++)
    {
        total_ground_estimate_total_time += ground_estimate_total_time[i];
        total_region_growth_time += region_growth_time[i];
    }

    // 결과 시간을 출력합니다.
    if (out_file != "") out << total_ground_estimate_total_time << " ";
    if (out_file != "") out << total_region_growth_time << " ";

    // 클러스터 시각화 (주석 처리)
    // cluster_vis_high.publish(numbers);

    // 사용된 보텍스를 초기화합니다.
    ros::Time t5 = ros::Time::now();
    for (auto ite = used_map_set.begin(); ite != used_map_set.end(); ite++)
    {
        umap[*ite].reset();
        umap_ground[*ite].reset();
        umap_insidebox[*ite].reset();
    }

    // 클러스터 시간을 계산하고 출력합니다.
    double cluster_time = (ros::Time::now() - cluster_begin).toSec() - total_region_growth_time;
    if (out_file != "") out << cluster_time << std::endl;
}

bool DynObjCluster::ground_estimate(const pcl::PointCloud<PointType> &ground_pcl, const Eigen::Vector3f &world_z, Eigen::Vector3f &ground_norm, Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &true_ground, std::unordered_set<int> &extend_pixels)
{
    // ground_pcl의 크기가 0보다 크지 않으면 false를 반환합니다.
    if (!ground_pcl.size() > 0)
        return false;

    // BNUM은 ground_pcl의 크기를 100으로 나눈 값 또는 최소 4 중 더 큰 값으로 설정합니다.
    int BNUM = std::max(4, (int)ground_pcl.size() / 100);

    // 임계값 및 최대 허용 각도를 설정합니다.
    const float thershold = 0.10f;
    const float max_angle_from_body = 30.0f / 57.3f; // 라디안으로 변환

    // 클라우드 포인트들을 분리하여 저장할 객체를 선언합니다.
    pcl::PointCloud<PointType> split_pcl;
    int max_count = 0; // 최대 점 개수를 초기화합니다.
    Eigen::Vector3f max_normvec(0, 0, 0); // 최대 정규 벡터를 초기화합니다.
    pcl::PointCloud<PointType> max_points; // 최대 점 클라우드를 초기화합니다.

    // 모든 ground_pcl의 점에 대해 반복합니다.
    for (int i = 0; i < ground_pcl.size(); i++)
    {
        split_pcl.push_back(ground_pcl[i]); // 점을 split_pcl에 추가합니다.

        // split_pcl의 크기가 BNUM에 도달하면 평면 추정을 시도합니다.
        if (split_pcl.size() == BNUM)
        {
            Eigen::Vector4f plane; // 평면을 저장할 변수를 선언합니다.

            // 평면을 추정하고, 평면의 네 번째 요소가 임계값보다 작으면 처리합니다.
            if (esti_plane(plane, split_pcl) && plane[3] < thershold)
            {
                // 정규 벡터를 계산하고 정규화합니다.
                Eigen::Vector3f normvec = plane.head(3).normalized();

                // 정규 벡터와 world_z의 교차 곱의 크기를 계산하여 허용 각도 내에 있는지 확인합니다.
                if (normvec.cross(world_z).norm() < sin(max_angle_from_body))
                {
                    int count = 0; // 평면에 속하는 점의 개수를 초기화합니다.
                    pcl::PointCloud<PointType> tmp_points; // 임시로 점들을 저장할 클라우드를 선언합니다.

                    // 모든 ground_pcl의 점에 대해 반복합니다.
                    for (int j = 0; j < ground_pcl.size(); j++)
                    {
                        Eigen::Vector3f point;
                        point[0] = ground_pcl[j].x; // x 좌표 설정
                        point[1] = ground_pcl[j].y; // y 좌표 설정
                        point[2] = ground_pcl[j].z; // z 좌표 설정

                        // 점과 평면 사이의 거리를 계산합니다.
                        float dis = fabs(point.dot(plane.head(3)) + 1.0f) / plane.head(3).norm();

                        // 거리가 임계값보다 작으면 점을 tmp_points에 추가하고 개수를 증가시킵니다.
                        if (dis < thershold)
                        {
                            tmp_points.push_back(ground_pcl[j]);
                            count++;
                        }
                    }

                    // 현재 점의 개수가 최대 개수보다 크면 갱신합니다.
                    if (count > max_count)
                    {
                        max_count = count; // 최대 개수를 갱신합니다.
                        max_normvec = normvec; // 최대 정규 벡터를 갱신합니다.
                        ground_plane = plane; // 최대 평면을 갱신합니다.
                        max_points = tmp_points; // 최대 점 클라우드를 갱신합니다.

                        // 최대 점의 개수가 전체 점의 60% 이상이면 반복을 종료합니다.
                        if (max_count > 0.6f * ground_pcl.size())
                            break;
                    }
                }
            }

            // split_pcl을 비웁니다.
            split_pcl.clear();
        }
    }
        // ground_pcl이 비어있지 않고, max_count가 일정 비율 이상이거나 500보다 크면 지면 평면을 재설정합니다.
    if (ground_pcl.size() > 0 && (max_count > 0.2f * ground_pcl.size() || max_count > 500))
    {
        Eigen::Vector4f plane; // 평면을 저장할 변수를 선언합니다.

        // max_points로 평면을 추정하고, 평면의 네 번째 요소가 임계값보다 작으면 처리합니다.
        if (esti_plane(plane, max_points) && plane[3] < thershold)
        {
            // 정규 벡터를 계산하고 정규화합니다.
            Eigen::Vector3f normvec = plane.head(3).normalized();

            // 정규 벡터와 world_z의 교차 곱의 크기를 계산하여 허용 각도 내에 있는지 확인합니다.
            if (normvec.cross(world_z).norm() < sin(max_angle_from_body))
            {
                max_normvec = normvec; // 최대 정규 벡터를 갱신합니다.
                ground_plane = plane; // 지면 평면을 갱신합니다.
            }
        }

        // 모든 ground_pcl의 점에 대해 반복합니다.
        for (int j = 0; j < ground_pcl.size(); j++)
        {
            Eigen::Vector3f point;
            point[0] = ground_pcl[j].x; // x 좌표 설정
            point[1] = ground_pcl[j].y; // y 좌표 설정
            point[2] = ground_pcl[j].z; // z 좌표 설정

            // 점과 지면 평면 사이의 거리를 계산합니다.
            float dis = fabs(point.dot(ground_plane.head(3)) + 1.0f) / ground_plane.head(3).norm();

            // 거리가 임계값보다 작으면 true_ground에 추가하고 extend_pixels에서 제거합니다.
            if (dis < thershold)
            {
                true_ground.push_back(ground_pcl[j]); // 지면 점 클라우드에 추가합니다.

                // 보텍스 인덱스를 계산합니다.
                int voxel = floor((ground_pcl[j].x - xyz_origin(0)) / Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z +
                            floor((ground_pcl[j].y - xyz_origin(1)) / Voxel_revolusion) * GridMapedgesize_z +
                            floor((ground_pcl[j].z - xyz_origin(2)) / Voxel_revolusion);

                extend_pixels.erase(voxel); // extend_pixels에서 보텍스를 제거합니다.
            }
        }

        // 정규 벡터의 z 성분이 음수이면 벡터의 방향을 반대로 설정합니다.
        if (max_normvec[2] < 0)
            max_normvec *= -1;

        // ground_norm을 max_normvec으로 설정합니다.
        ground_norm = max_normvec;
    }

    // ground_norm의 크기가 1에 가까우면 true를 반환합니다.
    if (abs(ground_norm.norm() - 1.0f) < 0.1f)
        return true;
    else
        return false;
}


void DynObjCluster::ground_remove(const Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag, pcl::PointCloud<PointType> &true_ground, std::vector<Point_Cloud> &umap)
{
    // 지면 평면에서의 임계값을 설정합니다.
    const float thershold = 0.10f;

    // 새로운 클러스터 포인트 클라우드와 인덱스를 저장할 객체를 선언합니다.
    pcl::PointCloud<PointType> new_clustet_pcl;
    std::vector<int> new_cluster_pcl_ind;

    // 모든 클러스터 포인트에 대해 반복합니다.
    for (int i = 0; i < cluster_pcl.size(); i++)
    {
        // 포인트를 Eigen 벡터로 변환합니다.
        Eigen::Vector3f point;
        point[0] = cluster_pcl[i].x; // x 좌표 설정
        point[1] = cluster_pcl[i].y; // y 좌표 설정
        point[2] = cluster_pcl[i].z; // z 좌표 설정

        // 점과 지면 평면 사이의 거리를 계산합니다.
        float dis = fabs(point.dot(ground_plane.head(3)) + 1.0f) / ground_plane.head(3).norm();

        // 거리가 임계값보다 크면 새로운 클러스터에 추가합니다.
        if (dis > thershold)
        {
            new_clustet_pcl.push_back(cluster_pcl[i]); // 새로운 클러스터에 점을 추가합니다.
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[i]); // 인덱스도 추가합니다.
        }
        else
        {
            // 거리가 임계값 이내이면 점을 지면 점으로 간주하고 태그를 0으로 설정합니다.
            dyn_tag[cluster_pcl_ind[i]] = 0;
            true_ground.push_back(cluster_pcl[i]); // 지면 점 클라우드에 추가합니다.
        }
    }

    // 클러스터 포인트 클라우드와 인덱스를 업데이트합니다.
    cluster_pcl = new_clustet_pcl;
    cluster_pcl_ind = new_cluster_pcl_ind;
}


void DynObjCluster::isolate_remove(pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag)
{
    // 클러스터 포인트의 크기가 2보다 작으면 함수 종료
    if (cluster_pcl.size() < 2)
    {
        return;
    }

    // 새로운 클러스터 포인트 클라우드와 인덱스를 저장할 객체를 선언합니다.
    pcl::PointCloud<PointType> new_cluster_pcl;
    std::vector<int> new_cluster_pcl_ind;

    // VOXEL_CLUSTER 객체 생성 및 초기화
    VOXEL_CLUSTER cluster;
    std::unordered_map<int, Point_Cloud::Ptr> umap_cluster;
    std::vector<std::vector<int>> voxel_cluster;

    // 클러스터의 입력 포인트 클라우드를 설정합니다.
    cluster.setInputCloud(cluster_pcl);
    cluster.setVoxelResolution(Voxel_revolusion, GridMapedgesize_xy, GridMapedgesize_z, xyz_origin); // 보텍스 해상도를 설정합니다.
    cluster.setExtendRange(cluster_extend_pixel); // 확장 범위를 설정합니다.
    cluster.setMinClusterSize(cluster_min_pixel_number); // 최소 클러스터 크기를 설정합니다.
    cluster.createVoxelMap(umap_cluster); // 보텍스 맵을 생성합니다.
    cluster.extract(voxel_cluster); // 보텍스 클러스터를 추출합니다.

    // 최대 클러스터 인덱스 및 최대 보텍스 수를 찾습니다.
    int max_cluster_ind = 0;
    int max_voxel_num = 0;
    for (int i = 0; i < voxel_cluster.size(); i++)
    {
        if (voxel_cluster[i].size() > max_voxel_num)
        {
            max_cluster_ind = i; // 최대 클러스터 인덱스를 갱신합니다.
            max_voxel_num = voxel_cluster[i].size(); // 최대 보텍스 수를 갱신합니다.
        }
    }

    // 동적 인덱스를 저장할 집합을 선언합니다.
    std::unordered_set<int> dyn_index;

    // 최대 클러스터의 보텍스를 순회합니다.
    for (int i = 0; i < max_voxel_num; i++)
    {
        int voxel = voxel_cluster[max_cluster_ind][i];
        for (int j = 0; j < umap_cluster[voxel]->cloud->size(); j++)
        {
            // 새 클러스터에 포인트와 인덱스를 추가합니다.
            new_cluster_pcl.push_back(umap_cluster[voxel]->cloud->points[j]);
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[umap_cluster[voxel]->cloud_index->at(j)]);
            dyn_index.insert(cluster_pcl_ind[umap_cluster[voxel]->cloud_index->at(j)]); // 동적 인덱스를 삽입합니다.
        }
    }

    // 클러스터 포인트 인덱스를 순회하며 동적 인덱스에 포함되지 않은 경우 태그를 0으로 설정합니다.
    for (int i = 0; i < cluster_pcl_ind.size(); i++)
    {
        if (!dyn_index.count(cluster_pcl_ind[i]))
        {
            dyn_tag[cluster_pcl_ind[i]] = 0;
        }
    }

    // 메모리를 해제하기 위해 umap_cluster를 스왑하여 초기화합니다.
    std::unordered_map<int, Point_Cloud::Ptr>().swap(umap_cluster);

    // 클러스터 포인트 클라우드와 인덱스를 업데이트합니다.
    cluster_pcl = new_cluster_pcl;
    cluster_pcl_ind = new_cluster_pcl_ind;
}

void DynObjCluster::oobb_estimate(const VoxelMap &vmap, const pcl::PointCloud<PointType> &points, Eigen::Vector3f &min_point_obj,
                                  Eigen::Vector3f &max_point_obj, Eigen::Matrix3f &R, const Eigen::Vector3f ground_norm)
{
    // 정수 NMATCH를 5로 설정
    int NMATCH = 5;
    // 링(rings)의 개수를 3으로 설정
    int n = 3; 
    // EA_disk 클래스의 객체를 n을 인자로 생성
    EA_disk disk(n);
    
    // NormVectorMap을 크기가 disk.size인 벡터로 초기화, 각 요소는 Eigen::Vector4f 벡터
    std::vector<std::vector<Eigen::Vector4f>> NormVectorMap(disk.size);
    // PointSizeList를 크기가 disk.size인 벡터로 초기화, 각 요소는 정수 벡터
    std::vector<std::vector<int>> PointSizeList(disk.size);

    // vmap의 모든 요소를 반복
    for (int i = 0; i < vmap.size(); i++)
    {
        // 현재 vmap 요소가 비어있지 않고, NMATCH 이상의 점이 포함된 경우 실행
        if (!vmap[i].empty() && vmap[i].points.size() >= NMATCH)
        {
            // 평면을 나타내는 Eigen::Vector4f plane 선언
            Eigen::Vector4f plane;

            // 평면을 추정하고, 추정이 성공한 경우 실행
            if (esti_plane(plane, vmap[i]))
            {
                // 평면의 첫 세 개 요소를 단위 벡터로 정규화
                plane.head(3) = plane.head(3).normalized();
                
                // 평면의 z 방향 값이 음수이면 방향을 반대로 전환
                if (plane[2] < 0)
                {
                    plane.head(3) *= -1;
                }

                // 구면 좌표를 나타내는 Eigen::Vector2f sphere_coor 선언
                Eigen::Vector2f sphere_coor;
                // 디스크 객체를 사용해 평면의 단위 벡터를 구면 좌표로 변환
                disk.CatesianToSphere(plane.head(3), sphere_coor);

                // 디스크 좌표를 나타내는 Eigen::Vector2f disk_coor 선언
                Eigen::Vector2f disk_coor;
                // 구면 좌표를 디스크 좌표로 변환
                disk.SphereToDisk(sphere_coor, disk_coor);

                // 디스크 좌표의 인덱스를 찾음
                int index = disk.index_find(disk_coor);

                // 인덱스가 특정 조건을 초과하면, 인덱스를 조정하고 방향을 반전
                if (index > pow(2 * (n - 1) + 1, 2) + 4 * n)
                {
                    index = index - 4 * n;
                    plane.head(3) *= -1;
                }

                // NormVectorMap의 해당 인덱스에 평면 추가
                NormVectorMap[index].push_back(plane);
                // PointSizeList의 해당 인덱스에 현재 vmap 크기 추가
                PointSizeList[index].push_back(vmap[i].size());
            }
        }
    }

  // 최대 인덱스와 두 번째 인덱스를 0으로 초기화
int max_ind = 0, sec_ind = 0;
// 최대 보상과 두 번째 보상을 0.0으로 초기화
float max_award = 0.0, sec_award = 0.0;

// NormVectorMap의 모든 요소를 반복
for (int i = 0; i < NormVectorMap.size(); i++)
{
    // 현재 NormVectorMap 요소가 비어있지 않은 경우 실행
    if (!NormVectorMap[i].empty())
    {
        // 현재 보상을 0.0으로 초기화
        float award = 0.0;
        // 해당 NormVectorMap 요소의 모든 항목을 반복하며 보상 계산
        for (int ite = 0; ite < NormVectorMap[i].size(); ite++)
        {
            // 보상에 항목의 점 크기 제곱근을 더해 계산
            award += std::sqrt(PointSizeList;
        }
        // 계산된 보상이 최대 보상보다 큰 경우
        if (award > max_award)
        {
            // 현재 최대 보상을 두 번째 보상에 저장하고 인덱스도 변경
            sec_award = max_award;
            sec_ind = max_ind;
            // 새로운 최대 보상과 인덱스를 설정
            max_award = award;
            max_ind = i;
        }
        // 계산된 보상이 두 번째 보상보다 큰 경우
        else if (award > sec_award)
        {
            // 두 번째 보상과 인덱스를 업데이트
            sec_award = award;
            sec_ind = i;
        }
    }
}

// 주 방향과 보조 방향을 (0,0,0)으로 초기화
Eigen::Vector3f direction_main(0.0f, 0.0f, 0.0f);
Eigen::Vector3f direction_aux(0.0f, 0.0f, 0.0f);

// 최대 보상이 0보다 큰 경우 주 방향을 계산
if (max_award > 0)
{
    // 해당 NormVectorMap 요소의 각 항목을 이용해 주 방향을 계산
    for (int ite = 0; ite < NormVectorMap[max_ind].size(); ite++)
    {
        direction_main = direction_main + NormVectorMap;
    }
    // 주 방향을 정규화
    direction_main.normalize();
}
else
    // 최대 보상이 없는 경우 기본값 (0,0,1) 설정
    direction_main << 0.0, 0.0, 1.0;

// 두 번째 보상이 0보다 큰 경우 보조 방향을 계산
if (sec_award > 0)
{
    // 해당 NormVectorMap 요소의 각 항목을 이용해 보조 방향을 계산
    for (int ite = 0; ite < NormVectorMap[sec_ind].size(); ite++)
    {
        direction_aux = direction_aux + NormVectorMap;
    }
    // 보조 방향을 정규화
    direction_aux.normalize();
}
else
    // 두 번째 보상이 없는 경우 기본값 (1,0,0) 설정
    direction_aux << 1.0, 0.0, 0.0;

// ground_norm이 충분히 작으면 R 행렬을 주 방향과 보조 방향으로 설정
if (ground_norm.norm() < 0.1)
{
    R.col(0) = direction_main;
    R.col(1) = (direction_aux - direction_aux.dot(R.col(0)) * R.col(0)).normalized();
    Eigen::Vector3f world_z(0.0, 0.0, 1.0);

    // 보조 방향이 월드 z축과 평행한 경우 처리
    if (abs(R.col(1).dot(world_z)) > 0.866f)
    {
        R.col(2) = R.col(1);
        R.col(1) = -(R.col(0).cross(R.col(2))).normalized();
    }
    // 주 방향이 월드 z축과 평행한 경우 처리
    else if (abs(R.col(0).dot(world_z)) > 0.866f)
    {
        R.col(1).swap(R.col(0));
        R.col(2) = R.col(1);
        R.col(1) = -(R.col(0).cross(R.col(2))).normalized();
    }
    else
    {
        // 모든 조건을 만족하지 않으면 R의 세 번째 열 계산
        R.col(2) = (R.col(0).cross(R.col(1))).normalized();
    }
}
else
{
    // ground_norm이 충분히 크면 R 행렬을 ground_norm을 기준으로 설정
    R.col(0) = ground_norm;
    if (ground_norm.dot(direction_main) > 0.95)
    {
        // 주 방향이 ground_norm과 너무 비슷하면 보조 방향을 사용
        direction_main = direction_aux;
    }
    R.col(1) = (direction_main - direction_main.dot(R.col(0)) * R.col(0)).normalized();
    R.col(2) = (R.col(0).cross(R.col(1))).normalized();
}

// 첫 번째 점의 좌표를 point_vec으로 설정
Eigen::Vector3f point_vec(points[0].x, points[0].y, points[0].z);
// point_vec을 R에 투영하여 project 계산
Eigen::Vector3f project = (point_vec.transpose() * R).transpose();

// x, y, z의 최소/최대 값을 첫 번째 점으로 초기화
float x_min = project[0], x_max = project[0];
float y_min = project[1], y_max = project[1];
float z_min = project[2], z_max = project[2];

// 모든 점에 대해 반복하며 최소/최대 값 계산
for (int pit = 0; pit < points.size(); pit++)
{
    point_vec << points[pit].x, points[pit].y, points[pit].z;
    project = (point_vec.transpose() * R).transpose();

    if (project[0] < x_min)
        x_min = project[0];
    if (project[1] < y_min)
        y_min = project[1];
    if (project[2] < z_min)
        z_min = project[2];
    if (project[0] > x_max)
        x_max = project[0];
    if (project[1] > y_max)
        y_max = project[1];
    if (project[2] > z_max)
        z_max = project[2];
}

// min_point_obj와 max_point_obj에 최소/최대 값을 설정
max_point_obj << x_max, y_max, z_max;
min_point_obj << x_min, y_min, z_min;

}

void DynObjCluster::event_extend(const Eigen::Matrix3f &R, bool ground_detect,
                                 bbox_t &bbox, std::vector<int> &dyn_tag, const int &bbox_index)
{
    // 주어진 bbox의 Ground_voxels_vec에서 bbox_index에 해당하는 모든 요소를 반복
    for (int i = 0; i < bbox.Ground_voxels_vec[bbox_index].size(); i++)
    {
        // 현재 반복 중인 Ground_voxel의 인덱스를 voxel_cur에 저장
        int voxel_cur = bbox.Ground_voxels_vec[bbox_index][i];
        // Ground_voxels_set에 voxel_cur이 포함되고, 해당 voxel의 포인트 수가 2보다 크면 실행
        if (bbox.Ground_voxels_set[bbox_index].count(voxel_cur) && umap_ground[voxel_cur].points_num > 2)
        {
            // x, y, z 방향으로 인접한 6개의 인덱스를 설정
            int x_ind[6] = {1, -1, 0, 0, 0, 0};
            int y_ind[6] = {0, 0, 1, -1, 0, 0};
            int z_ind[6] = {0, 0, 0, 0, 1, -1};
            
            // 6개의 인접한 방향을 반복
            for (int ind = 0; ind < 6; ind++)
            {
                // 인접한 voxel의 인덱스를 계산
                int voxel_neighbor = voxel_cur + x_ind[ind] * GridMapedgesize_xy * GridMapedgesize_z + y_ind[ind] * GridMapedgesize_z + z_ind[ind];
                
                // voxel_neighbor가 유효한 범위 내에 있는지 확인
                if (voxel_neighbor < 0 || voxel_neighbor > GridMapsize)
                    continue;
                
                // 인접한 voxel이 bbox 내에 있거나 점이 존재하는 경우 실행
                if ((umap_insidebox[voxel_neighbor].bbox_index > -1 && umap_insidebox[voxel_neighbor].points_num > 0 && umap_insidebox[voxel_neighbor].bbox_index == bbox_index) || (umap[voxel_neighbor].points_num > 0 && umap[voxel_neighbor].cloud->size() > 0 && umap[voxel_neighbor].bbox_index == bbox_index))
                {
                    // 현재 voxel의 점들을 복사하여 points에 저장
                    pcl::PointCloud<PointType> points = *(umap_ground[voxel_cur].cloud);
                    // 평면을 나타내는 Eigen::Vector4f plane 선언
                    Eigen::Vector4f plane;
                    
                    // ground_detect가 참인 경우에만 실행
                    if (ground_detect)
                    {
                        // 평면을 추정하고, R 행렬의 첫 번째 열과의 내적이 0.8보다 작은 경우 실행
                        if (esti_plane(plane, points) && abs(R.col(0).dot(plane.head(3).normalized())) < 0.8f)
                        {
                            // 현재 voxel을 bbox에 포함시키고 점 클라우드를 복사
                            umap[voxel_cur].bbox_index = bbox_index;
                            umap[voxel_cur].cloud = umap_ground[voxel_cur].cloud;
                            bbox.Point_cloud[bbox_index] += points;
                            
                            // 해당 voxel의 모든 점 인덱스를 dyn_tag와 bbox에 추가
                            for (int j = 0; j < umap_ground[voxel_cur].cloud_index->size(); j++)
                            {
                                dyn_tag[umap_ground[voxel_cur].cloud_index->at(j)] = 1;
                                bbox.Point_indices[bbox_index].push_back(umap_ground[voxel_cur].cloud_index->at(j));
                            }
                            // 조건을 만족한 경우 반복 종료
                            break;
                        }
                    }
                }
            }
        }
    }
}

bool DynObjCluster::esti_plane(Eigen::Vector4f &pca_result, const pcl::PointCloud<PointType> &point)
{
    // 평면 추정에 사용할 임계값을 설정
    const float threshold = 0.1;
    // 포인트 클라우드의 크기를 가져옴
    int point_size = point.size();
    
    // 포인트 클라우드의 좌표를 저장할 행렬 A와 상수 벡터 b를 선언
    Eigen::Matrix<float, Eigen::Dynamic, 3> A;
    Eigen::Matrix<float, Eigen::Dynamic, 1> b;
    
    // 행렬 A를 point_size x 3 크기로 조정
    A.resize(point_size, 3);
    // 벡터 b를 point_size x 1 크기로 조정하고, 모든 값을 -1로 설정
    b.resize(point_size, 1);
    b.setOnes();
    b *= -1.0f;
    
    // 포인트 클라우드의 각 점을 A에 할당
    for (int j = 0; j < point_size; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }
    
    // A의 QR 분해를 사용해 법선 벡터를 계산
    Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);
    
    // 법선 벡터의 크기를 계산
    float norm = normvec.norm();
    // 평균 거리 초기화
    float average_dis = 0.0;
    
    // 각 점에 대해 평면과의 거리 계산
    for (int j = 0; j < point_size; j++)
    {
        float tmp = fabs(normvec.dot(A.row(j)) + 1.0);
        average_dis += tmp;
        // 거리 값이 임계값보다 크면 평면 추정을 실패로 반환
        if (tmp > threshold)
        {
            return false;
        }
    }
    
    // 평균 거리 값을 정규화하고 최소값을 0.01로 설정
    average_dis = std::max(0.01f, average_dis / point_size / norm);
    
    // 추정된 평면의 결과를 pca_result에 저장
    pca_result(0) = normvec(0);
    pca_result(1) = normvec(1);
    pca_result(2) = normvec(2);
    pca_result(3) = average_dis;
    
    // 평면 추정이 성공했음을 반환
    return true;
}

void DynObjCluster::XYZExtract(const int &position, Eigen::Vector3f &xyz)
{
    // 포지션 값을 복사하여 left에 저장
    int left = position;
    
    // x 좌표를 계산
    xyz(0) = xyz_origin(0) + (float)floor(position / (GridMapedgesize_xy * GridMapedgesize_z)) * Voxel_revolusion;
    
    // left 값을 x 좌표에 해당하는 부분만큼 감소
    left = left - (float)floor(position / (GridMapedgesize_xy * GridMapedgesize_z)) * (GridMapedgesize_xy * GridMapedgesize_z);
    
    // y 좌표를 계산
    xyz(1) = xyz_origin(1) + (float)floor(left / GridMapedgesize_z) * Voxel_revolusion;
    
    // left 값을 y 좌표에 해당하는 부분만큼 감소
    left = left - (float)floor(left / GridMapedgesize_z) * GridMapedgesize_z;
    
    // z 좌표를 계산
    xyz(2) = xyz_origin(2) + left * Voxel_revolusion;
}


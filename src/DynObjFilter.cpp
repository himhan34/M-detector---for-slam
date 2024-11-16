#include <iostream> // 입출력을 위한 표준 라이브러리 포함
#include <vector> // 벡터 자료구조를 사용하기 위한 라이브러리 포함
#include <random> // 랜덤 숫자 생성을 위한 라이브러리 포함
#include <m-detector/DynObjFilter.h> // 동적 객체 필터링 라이브러리를 포함

// #include <algorithm> // 주석 처리된 알고리즘 라이브러리 포함
// #include <chrono> // 주석 처리된 시간 측정 라이브러리 포함
// #include <execution> // 주석 처리된 병렬 실행 라이브러리 포함

#define PI_MATH  (3.14159f) // 매크로 정의로 파이 값 설정



void DynObjFilter::init(ros::NodeHandle& nh) // DynObjFilter의 초기화 함수입니다.
{
    nh.param<double>("dyn_obj/buffer_delay", buffer_delay, 0.1); // 버퍼 지연 시간(buffer_delay)을 설정하고 기본값은 0.1초입니다.
    nh.param<int>("dyn_obj/buffer_size", buffer_size, 300000); // 버퍼 크기(buffer_size)를 설정하고 기본값은 300,000입니다.
    nh.param<int>("dyn_obj/points_num_perframe", points_num_perframe, 150000); // 프레임당 점의 수를 설정하고 기본값은 150,000입니다.
    nh.param<double>("dyn_obj/depth_map_dur", depth_map_dur, 0.2); // 깊이 맵의 지속 시간을 설정하고 기본값은 0.2초입니다.
    nh.param<int>("dyn_obj/max_depth_map_num", max_depth_map_num, 5); // 최대 깊이 맵 수를 설정하고 기본값은 5입니다.
    nh.param<int>("dyn_obj/max_pixel_points", max_pixel_points, 50); // 최대 픽셀 점 수를 설정하고 기본값은 50입니다.
    nh.param<double>("dyn_obj/frame_dur", frame_dur, 0.1); // 프레임 지속 시간을 설정하고 기본값은 0.1초입니다.
    nh.param<int>("dyn_obj/dataset", dataset, 0); // 데이터셋 ID를 설정하고 기본값은 0입니다.
    nh.param<float>("dyn_obj/self_x_f", self_x_f, 0.15f); // 자기 전방 거리(self_x_f)를 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/self_x_b", self_x_b, 0.15f); // 자기 후방 거리(self_x_b)를 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/self_y_l", self_y_l, 0.15f); // 자기 왼쪽 거리(self_y_l)를 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/self_y_r", self_y_r, 0.5f); // 자기 오른쪽 거리(self_y_r)를 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/blind_dis", blind_dis, 0.15f); // 사각 지대 거리(blind_dis)를 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/fov_up", fov_up, 0.15f); // 상단 시야각(fov_up)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/fov_down", fov_down, 0.15f); // 하단 시야각(fov_down)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/fov_cut", fov_cut, 0.15f); // 시야 절단 각도(fov_cut)를 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/fov_left", fov_left, 180.0f); // 왼쪽 시야각(fov_left)을 설정하고 기본값은 180도입니다.
    nh.param<float>("dyn_obj/fov_right", fov_right, -180.0f); // 오른쪽 시야각(fov_right)을 설정하고 기본값은 -180도입니다.
    nh.param<int>("dyn_obj/checkneighbor_range", checkneighbor_range, 1); // 이웃 범위를 확인할 범위(checkneighbor_range)를 설정하고 기본값은 1입니다.
    nh.param<bool>("dyn_obj/stop_object_detect", stop_object_detect, false); // 객체 감지를 멈출지 여부(stop_object_detect)를 설정하고 기본값은 false입니다.
    nh.param<float>("dyn_obj/depth_thr1", depth_thr1, 0.15f); // 첫 번째 깊이 임계값(depth_thr1)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/enter_min_thr1", enter_min_thr1, 0.15f); // 최소 진입 임계값(enter_min_thr1)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/enter_max_thr1", enter_max_thr1, 0.15f); // 최대 진입 임계값(enter_max_thr1)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/map_cons_depth_thr1", map_cons_depth_thr1, 0.5f); // 지도 구성 깊이 임계값(map_cons_depth_thr1)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/map_cons_hor_thr1", map_cons_hor_thr1, 0.01f); // 지도 구성 수평 임계값(map_cons_hor_thr1)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/map_cons_ver_thr1", map_cons_ver_thr1, 0.01f); // 지도 구성 수직 임계값(map_cons_ver_thr1)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/map_cons_hor_dis1", map_cons_hor_dis1, 0.2f); // 지도 구성 수평 거리(map_cons_hor_dis1)를 설정하고 기본값은 0.2입니다.
    nh.param<float>("dyn_obj/map_cons_ver_dis1", map_cons_ver_dis1, 0.1f); // 지도 구성 수직 거리(map_cons_ver_dis1)를 설정하고 기본값은 0.1입니다.
    nh.param<float>("dyn_obj/depth_cons_depth_thr1", depth_cons_depth_thr1, 0.5f); // 깊이 구성 깊이 임계값(depth_cons_depth_thr1)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr1", depth_cons_depth_max_thr1, 0.5f); // 최대 깊이 구성 임계값(depth_cons_depth_max_thr1)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/depth_cons_hor_thr1", depth_cons_hor_thr1, 0.02f); // 깊이 구성 수평 임계값(depth_cons_hor_thr1)을 설정하고 기본값은 0.02입니다.
    nh.param<float>("dyn_obj/depth_cons_ver_thr1", depth_cons_ver_thr1, 0.01f); // 깊이 구성 수직 임계값(depth_cons_ver_thr1)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/enlarge_z_thr1", enlarge_z_thr1, 0.05f); // z축 확대 임계값(enlarge_z_thr1)을 설정하고 기본값은 0.05입니다.
    nh.param<float>("dyn_obj/enlarge_angle", enlarge_angle, 2.0f); // 확대 각도(enlarge_angle)를 설정하고 기본값은 2도입니다.
    nh.param<float>("dyn_obj/enlarge_depth", enlarge_depth, 3.0f); // 깊이 확대 값(enlarge_depth)을 설정하고 기본값은 3입니다.
    nh.param<int>("dyn_obj/occluded_map_thr1", occluded_map_thr1, 3); // 지도 폐색 임계값(occluded_map_thr1)을 설정하고 기본값은 3입니다.
    nh.param<bool>("dyn_obj/case1_interp_en", case1_interp_en, false); // 첫 번째 경우 보간 활성화(case1_interp_en)를 설정하고 기본값은 false입니다.
    nh.param<float>("dyn_obj/k_depth_min_thr1", k_depth_min_thr1, 0.0f); // 최소 깊이 k값(k_depth_min_thr1)을 설정하고 기본값은 0.0입니다.
    nh.param<float>("dyn_obj/d_depth_min_thr1", d_depth_min_thr1, 0.15f); // 최소 깊이 d값(d_depth_min_thr1)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/k_depth_max_thr1", k_depth_max_thr1, 0.0f); // 최대 깊이 k값(k_depth_max_thr1)을 설정하고 기본값은 0.0입니다.
    nh.param<float>("dyn_obj/d_depth_max_thr1", d_depth_max_thr1, 0.15f); // 최대 깊이 d값(d_depth_max_thr1)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/v_min_thr2", v_min_thr2, 0.5f); // 최소 속도 임계값(v_min_thr2)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/acc_thr2", acc_thr2, 1.0f); // 가속 임계값(acc_thr2)을 설정하고 기본값은 1.0입니다.
    nh.param<float>("dyn_obj/map_cons_depth_thr2", map_cons_depth_thr2, 0.15f); // 두 번째 지도 깊이 제약(map_cons_depth_thr2)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/map_cons_hor_thr2", map_cons_hor_thr2, 0.02f); // 두 번째 지도 수평 제약(map_cons_hor_thr2)을 설정하고 기본값은 0.02입니다.
    nh.param<float>("dyn_obj/map_cons_ver_thr2", map_cons_ver_thr2, 0.01f); // 두 번째 지도 수직 제약(map_cons_ver_thr2)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/occ_depth_thr2", occ_depth_thr2, 0.15f); // 두 번째 깊이 폐색 임계값(occ_depth_thr2)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/occ_hor_thr2", occ_hor_thr2, 0.02f); // 두 번째 수평 폐색 임계값(occ_hor_thr2)을 설정하고 기본값은 0.02입니다.
    nh.param<float>("dyn_obj/occ_ver_thr2", occ_ver_thr2, 0.01f); // 두 번째 수직 폐색 임계값(occ_ver_thr2)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/depth_cons_depth_thr2", depth_cons_depth_thr2, 0.5f); // 두 번째 깊이 구성 깊이 임계값(depth_cons_depth_thr2)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr2", depth_cons_depth_max_thr2, 0.5f); // 두 번째 최대 깊이 구성 임계값(depth_cons_depth_max_thr2)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/depth_cons_hor_thr2", depth_cons_hor_thr2, 0.02f); // 두 번째 깊이 구성 수평 임계값(depth_cons_hor_thr2)을 설정하고 기본값은 0.02입니다.
    nh.param<float>("dyn_obj/depth_cons_ver_thr2", depth_cons_ver_thr2, 0.01f); // 두 번째 깊이 구성 수직 임계값(depth_cons_ver_thr2)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/k_depth2", k_depth2, 0.005f); // 깊이 k값(k_depth2)을 설정하고 기본값은 0.005입니다.
    nh.param<int>("dyn_obj/occluded_times_thr2", occluded_times_thr2, 3); // 두 번째 폐색 횟수 임계값(occluded_times_thr2)을 설정하고 기본값은 3입니다.
    nh.param<bool>("dyn_obj/case2_interp_en", case2_interp_en, false); // 두 번째 경우 보간 활성화(case2_interp_en)를 설정하고 기본값은 false입니다.
    nh.param<float>("dyn_obj/k_depth_max_thr2", k_depth_max_thr2, 0.0f); // 두 번째 최대 깊이 k값(k_depth_max_thr2)을 설정하고 기본값은 0.0입니다.
    nh.param<float>("dyn_obj/d_depth_max_thr2", d_depth_max_thr2, 0.15f); // 두 번째 최대 깊이 d값(d_depth_max_thr2)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/v_min_thr3", v_min_thr3, 0.5f); // 세 번째 최소 속도 임계값(v_min_thr3)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/acc_thr3", acc_thr3, 1.0f); // 세 번째 가속 임계값(acc_thr3)을 설정하고 기본값은 1.0입니다.
    nh.param<float>("dyn_obj/map_cons_depth_thr3", map_cons_depth_thr3, 0.15f); // 세 번째 지도 깊이 제약(map_cons_depth_thr3)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/map_cons_hor_thr3", map_cons_hor_thr3, 0.02f); // 세 번째 지도 수평 제약(map_cons_hor_thr3)을 설정하고 기본값은 0.02입니다.
    nh.param<float>("dyn_obj/map_cons_ver_thr3", map_cons_ver_thr3, 0.01f); // 세 번째 지도 수직 제약(map_cons_ver_thr3)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/occ_depth_thr3", occ_depth_thr3, 0.15f); // 세 번째 깊이 폐색 임계값(occ_depth_thr3)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/occ_hor_thr3", occ_hor_thr3, 0.02f); // 세 번째 수평 폐색 임계값(occ_hor_thr3)을 설정하고 기본값은 0.02입니다.
    nh.param<float>("dyn_obj/occ_ver_thr3", occ_ver_thr3, 0.01f); // 세 번째 수직 폐색 임계값(occ_ver_thr3)을 설정하고 기본값은 0.01입니다.

    nh.param<float>("dyn_obj/depth_cons_depth_thr3", depth_cons_depth_thr3, 0.5f); // 세 번째 깊이 구성 깊이 임계값(depth_cons_depth_thr3)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/depth_cons_depth_max_thr3", depth_cons_depth_max_thr3, 0.5f); // 세 번째 최대 깊이 구성 임계값(depth_cons_depth_max_thr3)을 설정하고 기본값은 0.5입니다.
    nh.param<float>("dyn_obj/depth_cons_hor_thr3", depth_cons_hor_thr3, 0.02f); // 세 번째 깊이 구성 수평 임계값(depth_cons_hor_thr3)을 설정하고 기본값은 0.02입니다.
    nh.param<float>("dyn_obj/depth_cons_ver_thr3", depth_cons_ver_thr3, 0.01f); // 세 번째 깊이 구성 수직 임계값(depth_cons_ver_thr3)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/k_depth3", k_depth3, 0.005f); // 깊이 k값(k_depth3)을 설정하고 기본값은 0.005입니다.
    nh.param<int>("dyn_obj/occluding_times_thr3", occluding_times_thr3, 3); // 세 번째 폐색 횟수 임계값(occluding_times_thr3)을 설정하고 기본값은 3입니다.
    nh.param<bool>("dyn_obj/case3_interp_en", case3_interp_en, false); // 세 번째 경우 보간 활성화(case3_interp_en)를 설정하고 기본값은 false입니다.
    nh.param<float>("dyn_obj/k_depth_max_thr3", k_depth_max_thr3, 0.0f); // 세 번째 최대 깊이 k값(k_depth_max_thr3)을 설정하고 기본값은 0.0입니다.
    nh.param<float>("dyn_obj/d_depth_max_thr3", d_depth_max_thr3, 0.15f); // 세 번째 최대 깊이 d값(d_depth_max_thr3)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/interp_hor_thr", interp_hor_thr, 0.01f); // 보간 수평 임계값(interp_hor_thr)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/interp_ver_thr", interp_ver_thr, 0.01f); // 보간 수직 임계값(interp_ver_thr)을 설정하고 기본값은 0.01입니다.
    nh.param<float>("dyn_obj/interp_thr1", interp_thr1, 1.0f); // 첫 번째 보간 임계값(interp_thr1)을 설정하고 기본값은 1.0입니다.
    nh.param<float>("dyn_obj/interp_static_max", interp_static_max, 10.0f); // 보간의 최대 정적 값(interp_static_max)을 설정하고 기본값은 10.0입니다.
    nh.param<float>("dyn_obj/interp_start_depth1", interp_start_depth1, 20.0f); // 첫 번째 보간 시작 깊이(interp_start_depth1)를 설정하고 기본값은 20.0입니다.
    nh.param<float>("dyn_obj/interp_kp1", interp_kp1, 0.1f); // 첫 번째 보간의 kp 값(interp_kp1)을 설정하고 기본값은 0.1입니다.
    nh.param<float>("dyn_obj/interp_kd1", interp_kd1, 1.0f); // 첫 번째 보간의 kd 값(interp_kd1)을 설정하고 기본값은 1.0입니다.
    nh.param<float>("dyn_obj/interp_thr2", interp_thr2, 0.15f); // 두 번째 보간 임계값(interp_thr2)을 설정하고 기본값은 0.15입니다.
    nh.param<float>("dyn_obj/interp_thr3", interp_thr3, 0.15f); // 세 번째 보간 임계값(interp_thr3)을 설정하고 기본값은 0.15입니다.
    nh.param<bool>("dyn_obj/dyn_filter_en", dyn_filter_en, true); // 동적 필터 활성화(dyn_filter_en)를 설정하고 기본값은 true입니다.
    nh.param<bool>("dyn_obj/debug_publish", debug_en, true); // 디버그 출력 활성화(debug_publish)를 설정하고 기본값은 true입니다.
    nh.param<int>("dyn_obj/laserCloudSteadObj_accu_limit", laserCloudSteadObj_accu_limit, 5); // 레이저 클라우드 누적 제한(laserCloudSteadObj_accu_limit)을 설정하고 기본값은 5입니다.
    nh.param<float>("dyn_obj/voxel_filter_size", voxel_filter_size, 0.1f); // 보켈 필터 크기(voxel_filter_size)를 설정하고 기본값은 0.1입니다.
    nh.param<bool>("dyn_obj/cluster_coupled", cluster_coupled, false); // 클러스터 연계 활성화(cluster_coupled)를 설정하고 기본값은 false입니다.
    nh.param<bool>("dyn_obj/cluster_future", cluster_future, false); // 미래 클러스터 활성화(cluster_future)를 설정하고 기본값은 false입니다.
    nh.param<int>("dyn_obj/cluster_extend_pixel", Cluster.cluster_extend_pixel, 2); // 클러스터 확장 픽셀 수(cluster_extend_pixel)를 설정하고 기본값은 2입니다.
    nh.param<int>("dyn_obj/cluster_min_pixel_number", Cluster.cluster_min_pixel_number, 4); // 클러스터 최소 픽셀 수(cluster_min_pixel_number)를 설정하고 기본값은 4입니다.
    nh.param<float>("dyn_obj/cluster_thrustable_thresold", Cluster.thrustable_thresold, 0.3f); // 클러스터 추력 임계값(cluster_thrustable_thresold)을 설정하고 기본값은 0.3입니다.
    nh.param<float>("dyn_obj/cluster_Voxel_revolusion", Cluster.Voxel_revolusion, 0.3f); // 클러스터 보켈 해상도(cluster_Voxel_revolusion)를 설정하고 기본값은 0.3입니다.
    nh.param<bool>("dyn_obj/cluster_debug_en", Cluster.debug_en, false); // 클러스터 디버그 활성화(cluster_debug_en)를 설정하고 기본값은 false입니다.
    nh.param<string>("dyn_obj/cluster_out_file", Cluster.out_file, ""); // 클러스터 출력 파일(cluster_out_file)을 설정하고 기본값은 빈 문자열입니다.
    nh.param<float>("dyn_obj/ver_resolution_max", hor_resolution_max, 0.0025f); // 수직 최대 해상도(ver_resolution_max)를 설정하고 기본값은 0.0025입니다.
    nh.param<float>("dyn_obj/hor_resolution_max", ver_resolution_max, 0.0025f); // 수평 최대 해상도(hor_resolution_max)를 설정하고 기본값은 0.0025입니다.
    nh.param<float>("dyn_obj/buffer_dur", buffer_dur, 0.1f); // 버퍼 지속 시간(buffer_dur)을 설정하고 기본값은 0.1입니다.
    nh.param<int>("dyn_obj/point_index", point_index, 0); // 점 인덱스(point_index)를 설정하고 기본값은 0입니다.
    nh.param<string>("dyn_obj/frame_id", frame_id, "camera_init"); // 프레임 ID(frame_id)를 설정하고 기본값은 "camera_init"입니다.
    nh.param<string>("dyn_obj/time_file", time_file, ""); // 시간 파일 경로(time_file)를 설정하고 기본값은 빈 문자열입니다.
    nh.param<string>("dyn_obj/time_breakdown_file", time_breakdown_file, ""); // 시간 분해 파일 경로(time_breakdown_file)를 설정하고 기본값은 빈 문자열입니다.
    max_ind = floor(3.1415926 * 2 / hor_resolution_max); // 수평 해상도를 기반으로 max_ind를 계산합니다.

    if (pcl_his_list.size() == 0) // Point cloud history list가 비어있는지 확인합니다.
{   
    PointCloudXYZI::Ptr first_frame(new PointCloudXYZI()); // 첫 번째 프레임을 위한 PointCloudXYZI 포인터를 생성합니다.
    first_frame->reserve(400000); // 첫 번째 프레임에 400,000개의 포인트 공간을 예약합니다.
    pcl_his_list.push_back(first_frame); // 첫 번째 프레임을 point cloud history list에 추가합니다.
    laserCloudSteadObj_hist = PointCloudXYZI::Ptr(new PointCloudXYZI()); // SteadObj history point cloud를 초기화합니다.
    laserCloudSteadObj = PointCloudXYZI::Ptr(new PointCloudXYZI()); // SteadObj point cloud를 초기화합니다.
    laserCloudDynObj = PointCloudXYZI::Ptr(new PointCloudXYZI()); // DynObj point cloud를 초기화합니다.
    laserCloudDynObj_world = PointCloudXYZI::Ptr(new PointCloudXYZI()); // DynObj의 world point cloud를 초기화합니다.
    int xy_ind[3] = {-1, 1}; // x와 y의 인덱스를 정의합니다.
    for (int ind_hor = 0; ind_hor < 2*hor_num + 1; ind_hor++) // 수평 방향으로 반복합니다.
    {
        for (int ind_ver = 0; ind_ver < 2*ver_num + 1; ind_ver++) // 수직 방향으로 반복합니다.
        {
            // pos_offset에 수평 및 수직 오프셋을 추가합니다.
            pos_offset.push_back(((ind_hor)/2 + ind_hor%2) * xy_ind[ind_hor%2] * MAX_1D_HALF + 
                                 ((ind_ver)/2 + ind_ver%2) * xy_ind[ind_ver%2]);
        }   
    }
}
	
map_cons_hor_num1 = ceil(map_cons_hor_thr1 / hor_resolution_max); // 첫 번째 수평 지도 구성 번호를 계산합니다.
map_cons_ver_num1 = ceil(map_cons_ver_thr1 / ver_resolution_max); // 첫 번째 수직 지도 구성 번호를 계산합니다.
interp_hor_num = ceil(interp_hor_thr / hor_resolution_max); // 수평 보간 번호를 계산합니다.
interp_ver_num = ceil(interp_ver_thr / ver_resolution_max); // 수직 보간 번호를 계산합니다.
map_cons_hor_num2 = ceil(map_cons_hor_thr2 / hor_resolution_max); // 두 번째 수평 지도 구성 번호를 계산합니다.
map_cons_ver_num2 = ceil(map_cons_ver_thr2 / ver_resolution_max); // 두 번째 수직 지도 구성 번호를 계산합니다.
occ_hor_num2 = ceil(occ_hor_thr2 / hor_resolution_max); // 두 번째 수평 폐색 번호를 계산합니다.
occ_ver_num2 = ceil(occ_ver_thr2 / ver_resolution_max); // 두 번째 수직 폐색 번호를 계산합니다.
depth_cons_hor_num2 = ceil(depth_cons_hor_thr2 / hor_resolution_max); // 두 번째 깊이 수평 구성 번호를 계산합니다.
depth_cons_ver_num2 = ceil(depth_cons_ver_thr2 / ver_resolution_max); // 두 번째 깊이 수직 구성 번호를 계산합니다.
map_cons_hor_num3 = ceil(map_cons_hor_thr3 / hor_resolution_max); // 세 번째 수평 지도 구성 번호를 계산합니다.
map_cons_ver_num3 = ceil(map_cons_ver_thr3 / ver_resolution_max); // 세 번째 수직 지도 구성 번호를 계산합니다.
occ_hor_num3 = ceil(occ_hor_thr3 / hor_resolution_max); // 세 번째 수평 폐색 번호를 계산합니다.
occ_ver_num3 = ceil(occ_ver_thr3 / ver_resolution_max); // 세 번째 수직 폐색 번호를 계산합니다.
depth_cons_hor_num3 = ceil(depth_cons_hor_thr3 / hor_resolution_max); // 세 번째 깊이 수평 구성 번호를 계산합니다.
depth_cons_ver_num3 = ceil(depth_cons_ver_thr3 / ver_resolution_max); // 세 번째 깊이 수직 구성 번호를 계산합니다.
buffer.init(buffer_size); // 버퍼를 초기화합니다.

pixel_fov_up = floor((fov_up / 180.0 * PI_MATH + 0.5 * PI_MATH) / ver_resolution_max); // 상단 시야각의 픽셀 번호를 계산합니다.
pixel_fov_down = floor((fov_down / 180.0 * PI_MATH + 0.5 * PI_MATH) / ver_resolution_max); // 하단 시야각의 픽셀 번호를 계산합니다.
pixel_fov_cut = floor((fov_cut / 180.0 * PI_MATH + 0.5 * PI_MATH) / ver_resolution_max); // 시야 절단의 픽셀 번호를 계산합니다.
pixel_fov_left = floor((fov_left / 180.0 * PI_MATH + PI_MATH) / hor_resolution_max); // 왼쪽 시야각의 픽셀 번호를 계산합니다.
pixel_fov_right = floor((fov_right / 180.0 * PI_MATH + PI_MATH) / hor_resolution_max); // 오른쪽 시야각의 픽셀 번호를 계산합니다.
max_pointers_num = round((max_depth_map_num * depth_map_dur + buffer_delay) / frame_dur) + 1; // 최대 포인터 수를 계산합니다.
point_soph_pointers.reserve(max_pointers_num); // 포인터 공간을 예약합니다.

	for (int i = 0; i < max_pointers_num; i++) // 포인터 배열을 초기화합니다.
{
    point_soph* p = new point_soph[points_num_perframe]; // 포인트 소프 배열을 생성합니다.
    point_soph_pointers.push_back(p); // 배열을 포인터 리스트에 추가합니다.
}

	if(time_file != "") // time_file이 빈 문자열이 아닌 경우
{
    time_out.open(time_file, ios::out); // 시간 파일을 쓰기 모드로 엽니다.
}

	if(time_breakdown_file != "") // time_breakdown_file이 빈 문자열이 아닌 경우
{
    time_breakdown_out.open(time_breakdown_file, ios::out); // 시간 분해 파일을 쓰기 모드로 엽니다.
}    
Cluster.Init(); // 클러스터를 초기화합니다.
}

void  DynObjFilter::filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time)
{
    // omp_get_wtime()를 사용하여 현재 시간을 가져와 변수 t00에 저장합니다.
    double t00 = omp_get_wtime();
    
    // 여러 시간 측정 변수들을 초기화합니다.
    time_search = time_research = time_search_0 = time_build = time_total = time_other0 = 0.0;
    time_interp1 = time_interp2 = 0;
    
    // 빌드 및 탐색에 사용될 카운터 변수들을 초기화합니다.
    int num_build = 0, num_search_0 = 0, num_research = 0;
    
    // 입력 포인트 클라우드가 NULL인 경우 함수를 종료합니다.
    if (feats_undistort == NULL) return;
    
    // 포인트 클라우드의 크기를 size 변수에 저장합니다.
    int size = feats_undistort->points.size();
    
    // 디버그 모드가 활성화된 경우, 히스토리 포인트 클라우드를 초기화합니다.
    if (debug_en)
    {
        laserCloudSteadObj_hist.reset(new PointCloudXYZI());
        laserCloudSteadObj_hist->reserve(20 * size); // 히스토리 포인트 클라우드의 메모리를 할당합니다.
    }
    
    // 원래의 동적 태그 벡터를 초기화하고, 메모리를 예약 및 크기를 조정합니다.
    dyn_tag_origin.clear();
    dyn_tag_origin.reserve(size);
    dyn_tag_origin.resize(size);
    
    // 클러스터된 동적 태그 벡터를 초기화하고, 메모리를 예약 및 크기를 조정합니다.
    dyn_tag_cluster.clear();
    dyn_tag_cluster.reserve(size);
    dyn_tag_cluster.resize(size);
    
    // 동적 객체 포인트 클라우드를 초기화하고 메모리를 예약합니다.
    laserCloudDynObj.reset(new PointCloudXYZI());
    laserCloudDynObj->reserve(size);
    
    // 월드 좌표계의 동적 객체 포인트 클라우드를 초기화하고 메모리를 예약합니다.
    laserCloudDynObj_world.reset(new PointCloudXYZI());
    laserCloudDynObj_world->reserve(size);
    
    // 정적 객체 포인트 클라우드를 초기화하고 메모리를 예약합니다.
    laserCloudSteadObj.reset(new PointCloudXYZI());
    laserCloudSteadObj->reserve(size);
    
    // 클러스터된 동적 객체 포인트 클라우드를 초기화하고 메모리를 예약합니다.
    laserCloudDynObj_clus.reset(new PointCloudXYZI());
    laserCloudDynObj_clus->reserve(size);
    
    // 클러스터된 정적 객체 포인트 클라우드를 초기화하고 메모리를 예약합니다.
    laserCloudSteadObj_clus.reset(new PointCloudXYZI());
    laserCloudSteadObj_clus->reserve(size); 
    
    // 파일 출력을 위한 ofstream 객체를 선언합니다.
    ofstream out;
    ofstream out_origin;
    
    // 기록 상태를 나타내는 불리언 변수들을 초기화합니다.
    bool is_rec = false;
    bool is_rec_origin = false;

        // 경로가 설정되어 있는 경우 파일 스트림을 엽니다.
    if (is_set_path)
    {
        // out_file을 바이너리 모드로 열어 출력 파일 스트림을 설정합니다.
        out.open(out_file, ios::out | ios::binary);
        
        // out_file_origin을 바이너리 모드로 열어 원본 출력 파일 스트림을 설정합니다.
        out_origin.open(out_file_origin, ios::out | ios::binary);
        
        // 출력 스트림이 성공적으로 열렸는지 확인하고, 열렸으면 기록 상태를 true로 설정합니다.
        if (out.is_open()) 
        {
            is_rec = true; // 파일이 열리면 기록 활성화
        }
        
        // 원본 출력 스트림이 성공적으로 열렸는지 확인하고, 열렸으면 원본 기록 상태를 true로 설정합니다.
        if (out_origin.is_open()) 
        {
            is_rec_origin = true; // 원본 파일이 열리면 기록 활성화
        }
    }
    
    // 여러 시간 측정 벡터들을 초기화하고, 메모리를 예약 및 크기를 설정합니다.
    time_test1.reserve(size);
    time_test1.resize(size); // 시간 테스트 1용 벡터를 초기화합니다.
    
    time_test2.reserve(size);
    time_test2.resize(size); // 시간 테스트 2용 벡터를 초기화합니다.
    
    time_test3.reserve(size);
    time_test3.resize(size); // 시간 테스트 3용 벡터를 초기화합니다.
    
    time_occ_check.reserve(size);
    time_occ_check.resize(size); // 점유 체크 시간 벡터를 초기화합니다.
    
    time_map_cons.reserve(size);
    time_map_cons.resize(size); // 맵 구성 시간 벡터를 초기화합니다.
    
    time_proj.reserve(size);
    time_proj.resize(size); // 투영 시간 벡터를 초기화합니다.


    // for 루프를 통해 모든 배열을 초기화합니다.
for(int i = 0; i < size; i++)
{
    // time_test1 배열의 값을 0.0으로 설정합니다.
    time_test1[i] = 0.0;
    // time_test2 배열의 값을 0.0으로 설정합니다.
    time_test2[i] = 0.0;
    // time_test3 배열의 값을 0.0으로 설정합니다.
    time_test3[i] = 0.0;
    // time_occ_check 배열의 값을 0.0으로 설정합니다.
    time_occ_check[i] = 0.0;
    // time_map_cons 배열의 값을 0.0으로 설정합니다.
    time_map_cons[i] = 0.0;
    // time_proj 배열의 값을 0.0으로 설정합니다.
    time_proj[i] = 0.0;
}

// case2_num 변수를 0으로 초기화합니다.
int case2_num = 0;

// 현재 시간을 측정하여 t0에 저장합니다.
double t0 = omp_get_wtime();

// time_case1, time_case2, time_case3 변수를 0으로 초기화합니다.
double time_case1 = 0, time_case2 = 0, time_case3 = 0;  

// PointCloud 객체 raw_points_world를 생성하고 크기를 미리 예약합니다.
pcl::PointCloud<PointType> raw_points_world;
raw_points_world.reserve(size);

// PointCloud 객체의 크기를 size로 조정합니다.
raw_points_world.resize(size);

// 정수 벡터 index를 생성하고 크기를 size로 설정합니다.
std::vector<int> index(size);

// for 루프를 통해 index 배열에 0부터 size-1까지 값을 할당합니다.
for (int i = 0; i < size; i++) {
    // index[i]에 i 값을 할당합니다.
    index[i] = i;
}

// 벡터 포인터 'points_soph*' 생성
vector<point_soph*> points;

// 'size' 만큼의 공간을 미리 예약
points.reserve(size);

// 'size' 크기로 벡터를 크기 조정
points.resize(size);

// 현재 포인터를 할당
point_soph* p = point_soph_pointers[cur_point_soph_pointers];

// 'time_file'이 비어있지 않으면 계산 시간 기록
if(time_file != "") time_out << size << " "; // rec computation time

// 인덱스 벡터의 요소들을 병렬로 순회
std::for_each(std::execution::par, index.begin(), index.end(), [&](const int &i)
{
    // 포인터 'p[i]' 초기화
    p[i].reset();

    // 왜곡되지 않은 특징 포인트 좌표를 로컬 좌표계로 변환
    V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z);

    // 포인트의 강도 값을 가져옴
    int intensity = feats_undistort->points[i].curvature;

    // 글로벌 좌표계로 변환
    V3D p_glob(rot_end * (p_body) + pos_end);

    // 변환된 글로벌 좌표 할당
    p[i].glob = p_glob;

    // 포인트를 정적으로 설정
    p[i].dyn = STATIC;

    // 회전 변환을 할당
    p[i].rot = rot_end.transpose();

    // 이동 변환을 할당
    p[i].transl = pos_end;

    // 스캔 끝 시간 할당
    p[i].time = scan_end_time;

    // 로컬 좌표 할당
    p[i].local = p_body;

    // 강도 값 할당
    p[i].intensity = feats_undistort->points[i].intensity;

    // 데이터셋이 0이고 강도 값이 특정 범위 내에 있으면 왜곡된 것으로 설정
    if(dataset == 0 && fabs(intensity-666) < 10E-4)
    {
        p[i].is_distort = true;
    }

    // 포인트가 유효하지 않은지 검사
    if (InvalidPointCheck(p_body, intensity))
    {
        // 유효하지 않은 것으로 설정
        p[i].dyn = INVALID;
        dyn_tag_origin[i] = 0;
        dyn_tag_cluster[i] = -1;
    }
    // 포인트가 자기 포인트인지 검사
    else if(SelfPointCheck(p_body, p[i].dyn))
    {
        // 유효하지 않은 것으로 설정
        p[i].dyn = INVALID;
        dyn_tag_origin[i] = 0;
    }
    // 첫 번째 경우 검사
    else if (Case1(p[i]))
    {
        // 첫 번째 경우로 설정
        p[i].dyn = CASE1;
        dyn_tag_origin[i] = 1;
    }
    // 두 번째 경우 검사
    else if (Case2(p[i]))
    {
        // 두 번째 경우로 설정
        p[i].dyn = CASE2;
        dyn_tag_origin[i] = 1;
    }
    // 세 번째 경우 검사
    else if(Case3(p[i]))
    {
        // 세 번째 경우로 설정
        p[i].dyn = CASE3;
        dyn_tag_origin[i] = 1;
    }
    // 모든 경우에 해당하지 않으면 기본값 설정
    else
    {
        dyn_tag_origin[i] = 0;
    }

    // 포인터를 벡터에 저장
    points[i] = &p[i];
});
	

// 시간이 기록될 파일이 존재하면 계산 시간을 기록
if(time_file != "") time_out << omp_get_wtime()-t0 << " "; // rec computation time 

// 포인트 수만큼 반복
for(int i = 0; i < size; i++)
{
    // 포인트 타입의 객체 생성 및 로컬 좌표 할당
    PointType po;
    po.x = points[i]->local[0];
    po.y = points[i]->local[1];
    po.z = points[i]->local[2];
    po.intensity = points[i]->intensity;

    // 글로벌 좌표에 해당하는 포인트 객체 생성
    PointType po_w;
    po_w.x = points[i]->glob[0];
    po_w.y = points[i]->glob[1];
    po_w.z = points[i]->glob[2];

    // 로우 포인트를 월드 좌표에 할당
    raw_points_world[i] = po;

    // 포인트의 동적 상태에 따라 처리
    switch(points[i]->dyn)
    {
        // CASE1이면 x축에 값을 설정하고 동적 객체로 추가
        case CASE1:
            po.normal_x = 1;
            laserCloudDynObj->push_back(po);
            laserCloudDynObj_world->push_back(po_w);
            break;

        // CASE2이면 y축에 점유 시간 값을 설정하고 동적 객체로 추가
        case CASE2:
            po.normal_y = points[i]->occu_times;
            laserCloudDynObj->push_back(po);
            laserCloudDynObj_world->push_back(po_w);
            break;

        // CASE3이면 z축에 점유 시간 값을 설정하고 동적 객체로 추가
        case CASE3:
            po.normal_z = points[i]->is_occu_times;
            laserCloudDynObj->push_back(po);
            laserCloudDynObj_world->push_back(po_w);
            break;

        // 기본적으로 정적인 객체로 추가
        default:
            laserCloudSteadObj->push_back(po_w);
    }
}

// 다양한 경우의 포인트 개수를 초기화
int num_1 = 0, num_2 = 0, num_3 = 0, num_inval = 0, num_neag = 0;

// 클러스터링 시작 전 시간 기록
double clus_before = omp_get_wtime(); // rec computation time

// 클러스터링에 대한 헤더 설정
std_msgs::Header header_clus;
header_clus.stamp = ros::Time().fromSec(scan_end_time);
header_clus.frame_id = frame_id;


   if (cluster_coupled || cluster_future) // 클러스터가 결합되었거나 미래에 사용할 클러스터가 있다면
{
    // 동적 태그 클러스터를 처리하는 함수 호출
    Cluster.Clusterprocess(dyn_tag_cluster, *laserCloudDynObj, raw_points_world, header_clus, rot_end, pos_end);
    
    for(int i = 0; i < size; i++) // 포인트들을 반복 처리
    {   
        PointType po; // 새로운 포인트 객체 생성
        po.x = points[i]->glob(0); // x 좌표 설정
        po.y = points[i]->glob(1); // y 좌표 설정
        po.z = points[i]->glob(2); // z 좌표 설정
        po.curvature = i; // 커브 정보 설정
        
        switch (points[i]->dyn) // 포인트의 동적 상태에 따라 분기 처리
        {   
            case CASE1: // 동적 상태가 CASE1일 경우
                if (dyn_tag_cluster[i] == 0) // 클러스터 태그가 0이면
                {
                    points[i]->dyn = STATIC; // 동적 상태를 정적으로 설정
                    points[i]->occu_times = -1; // 점유 시간 초기화
                    points[i]->is_occu_times = -1; // 점유 여부 초기화
                    po.normal_x = 0; // 법선 벡터의 x 값 설정
                    po.normal_y = points[i]->is_occu_times; // 법선 벡터의 y 값 설정
                    po.normal_z = points[i]->occu_times; // 법선 벡터의 z 값 설정
                    po.intensity = (int) (points[i]->local.norm() * 10) + 10; // 강도 값 설정
                    laserCloudSteadObj_clus->push_back(po); // 정적 객체 클러스터에 추가
                    num_neag += 1; // 정적 객체 수 증가
                }
                else // 클러스터 태그가 0이 아닐 경우
                {
                    po.normal_x = 1; // 법선 벡터의 x 값 설정
                    po.normal_y = points[i]->is_occu_times; // 법선 벡터의 y 값 설정
                    po.normal_z = points[i]->occu_times; // 법선 벡터의 z 값 설정
                    laserCloudDynObj_clus->push_back(po); // 동적 객체 클러스터에 추가
                    if(!dyn_filter_en) // 동적 필터가 비활성화된 경우
                    {
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10; // 강도 값 설정
                    }
                    num_1 += 1; // CASE1 객체 수 증가
                }
                break;
                
            case CASE2: // 동적 상태가 CASE2일 경우
                if(dyn_tag_cluster[i] == 0) // 클러스터 태그가 0이면
                {
                    points[i]->dyn = STATIC; // 동적 상태를 정적으로 설정
                    points[i]->occu_times = -1; // 점유 시간 초기화
                    points[i]->is_occu_times = -1; // 점유 여부 초기화
                    po.normal_x = 0; // 법선 벡터의 x 값 설정
                    po.normal_y = points[i]->is_occu_times; // 법선 벡터의 y 값 설정
                    po.normal_z = points[i]->occu_times; // 법선 벡터의 z 값 설정
                    po.intensity = (int) (points[i]->local.norm() * 10) + 10; // 강도 값 설정
                    laserCloudSteadObj_clus->push_back(po); // 정적 객체 클러스터에 추가
                    num_neag += 1; // 정적 객체 수 증가
                }
                else // 클러스터 태그가 0이 아닐 경우
                {
                    po.normal_x = 0; // 법선 벡터의 x 값 설정
                    po.normal_y = points[i]->is_occu_times; // 법선 벡터의 y 값 설정
                    po.normal_z = points[i]->occu_times; // 법선 벡터의 z 값 설정
                    laserCloudDynObj_clus->push_back(po); // 동적 객체 클러스터에 추가
                    if(!dyn_filter_en) // 동적 필터가 비활성화된 경우
                    {
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10; // 강도 값 설정
                    }
                    num_2 += 1; // CASE2 객체 수 증가
                }
                break;
                
            case CASE3: // 동적 상태가 CASE3일 경우
                if(dyn_tag_cluster[i] == 0) // 클러스터 태그가 0이면
                {
                    points[i]->dyn = STATIC; // 동적 상태를 정적으로 설정
                    points[i]->occu_times = -1; // 점유 시간 초기화
                    points[i]->is_occu_times = -1; // 점유 여부 초기화
                    po.normal_x = 0; // 법선 벡터의 x 값 설정
                    po.normal_y = points[i]->is_occu_times; // 법선 벡터의 y 값 설정
                    po.normal_z = points[i]->occu_times; // 법선 벡터의 z 값 설정
                    po.intensity = (int) (points[i]->local.norm() * 10) + 10; // 강도 값 설정
                    laserCloudSteadObj_clus->push_back(po); // 정적 객체 클러스터에 추가
                    num_neag += 1; // 정적 객체 수 증가
                }
                else // 클러스터 태그가 0이 아닐 경우
                {
                    po.normal_x = 0; // 법선 벡터의 x 값 설정
                    po.normal_y = points[i]->is_occu_times; // 법선 벡터의 y 값 설정
                    po.normal_z = points[i]->occu_times; // 법선 벡터의 z 값 설정
                    laserCloudDynObj_clus->push_back(po); // 동적 객체 클러스터에 추가
                    if(!dyn_filter_en) // 동적 필터가 비활성화된 경우
                    {
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10; // 강도 값 설정
                    }
                    num_3 += 1; // CASE3 객체 수 증가
                }
                break;       
                
            case STATIC: // 동적 상태가 STATIC일 경우
                if(dyn_tag_cluster[i] == 1) // 클러스터 태그가 1이면
                {
                    points[i]->dyn = CASE1; // 동적 상태를 CASE1로 설정
                    points[i]->occu_times = -1; // 점유 시간 초기화
                    points[i]->is_occu_times = -1; // 점유 여부 초기화
                    po.normal_x = 0; // 법선 벡터의 x 값 설정
                    po.normal_y = points[i]->is_occu_times; // 법선 벡터의 y 값 설정
                    po.normal_z = points[i]->occu_times; // 법선 벡터의 z 값 설정
                    laserCloudDynObj_clus->push_back(po); // 동적 객체 클러스터에 추가
                    if(!dyn_filter_en) // 동적 필터가 비활성화된 경우
                    {
                        po.intensity = (int) (points[i]->local.norm() * 10) + 10; // 강도 값 설정
                    }
                    num_1 += 1; // CASE1 객체 수 증가
                }
                else // 클러스터 태그가 1이 아닐 경우
                {
                    po.normal_x = 0; // 법선 벡터의 x 값 설정
                    po.normal_y = points[i]->is_occu_times; // 법선 벡터의 y 값 설정
                    po.normal_z = points[i]->occu_times; // 법선 벡터의 z 값 설정
                    po.intensity = (int) (points[i]->local.norm() * 10) + 10; // 강도 값 설정
                    laserCloudSteadObj_clus->push_back(po); // 정적 객체 클러스터에 추가
                    num_neag += 1; // 정적 객체 수 증가
                }
                break;
                
            default: // 유효하지 않은 경우
                num_inval += 1; // 유효하지 않은 객체 수 증가
                break;
        }
    }
}
	


	if(time_file != "") time_out << omp_get_wtime()-clus_before << " "; // 시간 파일이 비어있지 않다면 클러스터 처리 시간 기록
double t3 = omp_get_wtime(); // 현재 시간을 t3에 저장
Points2Buffer(points, index); // 포인트 데이터를 버퍼로 변환
double t4 = omp_get_wtime(); // 현재 시간을 t4에 저장
if(time_file != "") time_out << omp_get_wtime()-t3 << " "; // 변환 시간 기록
Buffer2DepthMap(scan_end_time); // 버퍼 데이터를 깊이 맵으로 변환
if(time_file != "") time_out << omp_get_wtime()-t3 << endl; // 변환 시간 기록

if (cluster_coupled) // 클러스터가 결합된 경우
{   
    for(int i = 0; i < size; i++) // 포인트 수만큼 반복
    {
        if (dyn_tag_cluster[i] == 1) // 동적 태그가 1인 경우
        {
            if(is_rec) // 기록 중일 경우
            {
                int tmp = 251; // 임시 값 251 설정
                out.write((char*)&tmp, sizeof(int)); // 파일에 기록
            }
        } 
        else // 동적 태그가 1이 아닌 경우
        {
            if(is_rec) // 기록 중일 경우
            {
                int tmp = 9; // 임시 값 9 설정
                out.write((char*)&tmp, sizeof(int)); // 파일에 기록
            } 
        } 

        if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2) // 원래 동적 태그가 1 또는 2인 경우
        {
            if(is_rec_origin) // 원본 기록 중일 경우
            {
                int tmp = 251; // 임시 값 251 설정
                out_origin.write((char*)&tmp, sizeof(int)); // 원본 파일에 기록
            }
        } 
        else // 원래 동적 태그가 1 또는 2가 아닌 경우
        {
            if(is_rec_origin) // 원본 기록 중일 경우
            {
                int tmp = 9; // 임시 값 9 설정
                out_origin.write((char*)&tmp, sizeof(int)); // 원본 파일에 기록
            }
        } 
    }
}
else // 클러스터가 결합되지 않은 경우
{
    for(int i = 0; i < size; i++) // 포인트 수만큼 반복
    {
        if (dyn_tag_origin[i] == 1 || dyn_tag_origin[i] == 2) // 원래 동적 태그가 1 또는 2인 경우
        {
            if(is_rec) // 기록 중일 경우
            {
                int tmp = 251; // 임시 값 251 설정
                out.write((char*)&tmp, sizeof(int)); // 파일에 기록
            }
        } 
        else // 원래 동적 태그가 1 또는 2가 아닌 경우
        {
            if(is_rec) // 기록 중일 경우
            {
                int tmp = 9; // 임시 값 9 설정
                out.write((char*)&tmp, sizeof(int)); // 파일에 기록
            }
        } 
    }
}

double total_test1 = 0, total_test2 = 0, total_test3 = 0, total_proj = 0, total_occ = 0, total_map = 0; // 총 시간 변수 초기화
for (int i = 0; i < size; i++) // 포인트 수만큼 반복
{
    total_test1 += time_test1[i]; // 테스트1 시간 누적
    total_test2 += time_test2[i]; // 테스트2 시간 누적
    total_test3 += time_test3[i]; // 테스트3 시간 누적
    total_proj += time_proj[i]; // 투영 시간 누적
    total_occ += time_occ_check[i]; // 점유 확인 시간 누적
    total_map += time_map_cons[i]; // 맵 구성 시간 누적
}

if(time_breakdown_file != "") // 시간 분해 파일이 비어있지 않은 경우
{
    // 누적 시간을 파일에 기록
    time_breakdown_out << total_test1 << " " << total_test2 << " "<< total_test3 << " " << total_proj << " " << total_occ << " " << total_map << endl;
}

	frame_num_for_rec ++; // 기록용 프레임 번호 증가
	cur_point_soph_pointers = (cur_point_soph_pointers + 1) % max_pointers_num; // 포인터 순환 인덱스 갱신
	
	if(is_rec) out.close(); // 기록 파일 닫기
	
	time_total = omp_get_wtime() - t00; // 전체 시간 계산
	time_ind ++; // 시간 인덱스 증가
	// 평균 총 시간을 계산
	time_total_avr = time_total_avr * (time_ind - 1) / time_ind + time_total / time_ind;
}

void DynObjFilter::Points2Buffer(vector<point_soph*> &points, std::vector<int> &index_vector)
{
    // 현재 버퍼의 tail 위치를 cur_tail 변수에 저장
    int cur_tail = buffer.tail;
    
    // 병렬 삽입을 준비하기 위해 버퍼에 삽입할 요소의 개수를 설정
    buffer.push_parallel_prepare(points.size());
    
    // index_vector의 각 요소에 대해 병렬 실행을 수행
    std::for_each(std::execution::par, index_vector.begin(), index_vector.end(), [&](const int &i)
    {   
        // points의 i번째 요소를 cur_tail + i 위치에 병렬로 버퍼에 삽입
        buffer.push_parallel(points[i], cur_tail+i);
    });
}

void DynObjFilter::Buffer2DepthMap(double cur_time)
{
    // 버퍼의 크기를 len에 저장
    int len = buffer.size();

    // 총 합을 계산하기 위한 변수 초기화
    double total_0 = 0.0;
    double total_1 = 0.0;
    double total_2 = 0.0;
    double total_3 = 0.0;
    double t = 0.0;
    int max_point = 0;

    // 버퍼의 각 요소를 순회
    for (int k = 0; k < len; k++)
    {   
        // 버퍼의 가장 앞에 있는 포인트를 가져옴
        point_soph* point = buffer.front();

        // 현재 시간과 포인트의 시간을 비교하여 조건 검사
        if ((cur_time - point->time) >= buffer_delay - frame_dur / 2.0)
        {   
            // depth_map_list가 비어있는 경우
            if(depth_map_list.size() == 0)
            {
                // 최대 depth map 수를 초과하지 않을 때만 새 depth map을 생성
                if(depth_map_list.size() < max_depth_map_num)
                {
                    map_index++;
                    DepthMap::Ptr new_map_pointer(new DepthMap(point->rot, point->transl, point->time, map_index));
                    depth_map_list.push_back(new_map_pointer);
                }
                else
                {
                    // 버퍼의 첫 번째 포인트를 제거하고 다음 반복으로 넘어감
                    buffer.pop();
                    continue;
                }          
            }
            // 마지막 depth map의 시간과 현재 포인트의 시간을 비교하여 조건 검사
            else if((point->time - depth_map_list.back()->time) >= depth_map_dur - frame_dur / 2.0)
            {
                map_index++;
                // depth map 리스트가 최대 크기에 도달한 경우
                if (depth_map_list.size() == max_depth_map_num)
                {   
                    // 가장 오래된 depth map을 새 포인트로 리셋
                    depth_map_list.front()->Reset(point->rot, point->transl, point->time, map_index);
                    DepthMap::Ptr new_map_pointer = depth_map_list.front();
                    depth_map_list.pop_front();
                    depth_map_list.push_back(new_map_pointer);
                }
                // depth map 리스트가 아직 최대 크기에 도달하지 않은 경우
                else if (depth_map_list.size() < max_depth_map_num)
                {   
                    DepthMap::Ptr new_map_pointer(new DepthMap(point->rot, point->transl, point->time, map_index));
                    depth_map_list.push_back(new_map_pointer);
                }
            }

            // 포인트의 상태에 따라 처리
            switch (point->dyn)
            {
                // depth map의 크기가 포인트의 위치보다 작을 때 STATIC 상태 처리
                if(depth_map_list.back()->depth_map.size() <= point->position)
                case STATIC:
                    // 구면 투영을 수행
                    SphericalProjection(*point, depth_map_list.back()->map_index, depth_map_list.back()->project_R, depth_map_list.back()->project_T, *point);
                    
                    // 포인트가 픽셀당 최대 포인트 수를 초과하지 않으면 depth map에 추가
                    if(depth_map_list.back()->depth_map[point->position].size() < max_pixel_points)
                    {
                        depth_map_list.back()->depth_map[point->position].push_back(point);
                        
                        // 최대 깊이를 갱신하는 경우
                        if (point->vec(2) > depth_map_list.back()->max_depth_all[point->position])  
                        {
                            depth_map_list.back()->max_depth_all[point->position] = point->vec(2);
                            depth_map_list.back()->max_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size() - 1;
                        }
                        // 최소 깊이를 갱신하거나 초기 값이 매우 작은 경우
                        if (point->vec(2) < depth_map_list.back()->min_depth_all[point->position] ||\
                            depth_map_list.back()->min_depth_all[point->position] < 10E-5)  
                        {
                            depth_map_list.back()->min_depth_all[point->position] = point->vec(2);
                            depth_map_list.back()->min_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size() - 1;
                        }
                        // STATIC 상태에서 최소 깊이를 갱신하는 경우
                        if (point->vec(2) < depth_map_list.back()->min_depth_static[point->position] ||\
                            depth_map_list.back()->min_depth_static[point->position] < 10E-5)  
                        {
                            depth_map_list.back()->min_depth_static[point->position] = point->vec(2);
                        }
                        // STATIC 상태에서 최대 깊이를 갱신하는 경우
                        if (point->vec(2) > depth_map_list.back()->max_depth_static[point->position])  
                        {
                            depth_map_list.back()->max_depth_static[point->position] = point->vec(2);
                        }                        
                    }
                    break;   

		case CASE1:   
		case CASE2:
		case CASE3:
		    // 포인트를 구면 투영으로 변환
		    SphericalProjection(*point, depth_map_list.back()->map_index, depth_map_list.back()->project_R, depth_map_list.back()->project_T, *point);
		    
		    // 포인트가 픽셀당 최대 포인트 수를 초과하지 않으면 depth map에 추가
		    if(depth_map_list.back()->depth_map[point->position].size() < max_pixel_points)
		    {
		        depth_map_list.back()->depth_map[point->position].push_back(point);
		
		        // 최대 깊이를 갱신하는 경우
		        if (point->vec(2) > depth_map_list.back()->max_depth_all[point->position])  
		        {
		            depth_map_list.back()->max_depth_all[point->position] = point->vec(2);
		            depth_map_list.back()->max_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size() - 1;
		        }
		        // 최소 깊이를 갱신하거나 초기 값이 매우 작은 경우
		        if (point->vec(2) < depth_map_list.back()->min_depth_all[point->position] ||\
		            depth_map_list.back()->min_depth_all[point->position] < 10E-5)  
		        {
		            depth_map_list.back()->min_depth_all[point->position] = point->vec(2);
		            depth_map_list.back()->min_depth_index_all[point->position] = depth_map_list.back()->depth_map[point->position].size() - 1;
		        }
		    }
		    break;
		
		// 기본 case 처리 (아무 작업도 하지 않음)
		default:    
		    break;
		}
		
		// 버퍼의 가장 앞에 있는 포인트를 제거
		buffer.pop();
		}
		else
		{
		    // 조건에 맞지 않는 경우 반복문 종료
		    break;
		}
		}
		
		// 디버깅이 활성화된 경우
		if (debug_en)
		{   
		    // 각 depth map 리스트를 순회
		    for (int i = 0; i < depth_map_list.size(); i++)
		    {
		        // 각 depth map의 포인트를 순회
		        for (int j = 0; j < depth_map_list[i]->depth_map.size(); j++)
		        {
		            // 포인트를 순회하여 정보를 저장
		            for (int k = 0; k < depth_map_list[i]->depth_map[j].size(); k++)
		            {
		                PointType po;
		                point_soph* point = depth_map_list[i]->depth_map[j][k];
		                
		                // 전역 좌표 설정
		                po.x = point->glob(0);
		                po.y = point->glob(1);
		                po.z = point->glob(2);
		
		                // 강도와 곡률 설정
		                po.intensity = point->local(2);
		                po.curvature = point->local(1);
		
		                // 법선 방향 및 동적 상태 설정
		                po.normal_x = point->hor_ind;
		                po.normal_y = point->ver_ind;
		                po.normal_z = point->dyn;
		
		                // 포인트가 STATIC 상태인 경우 laserCloudSteadObj_hist에 추가
		                if(point->dyn == STATIC) laserCloudSteadObj_hist->push_back(po);
		            }
		        }
		    }
		}
	}
void DynObjFilter::SphericalProjection(point_soph &p, int depth_index, const M3D &rot, const V3D &transl, point_soph &p_spherical)
{
    // depth_index를 HASH_PRIM으로 나눈 값으로 벡터의 Z 성분이 일정 크기 이상인지 검사
    if(fabs(p.last_vecs.at(depth_index % HASH_PRIM)[2]) > 10E-5)
    {       
        // 저장된 값을 p_spherical에 복사
        p_spherical.vec = p.last_vecs.at(depth_index % HASH_PRIM);
        p_spherical.hor_ind = p.last_positions.at(depth_index % HASH_PRIM)[0];
        p_spherical.ver_ind = p.last_positions.at(depth_index % HASH_PRIM)[1];
        p_spherical.position = p.last_positions.at(depth_index % HASH_PRIM)[2];     
    }
    else
    {
        // 글로벌 좌표를 회전 변환하고 평행 이동하여 투영된 좌표 계산
        V3D p_proj(rot * (p.glob - transl));
        
        // p_spherical에 투영된 좌표 정보 설정
        p_spherical.GetVec(p_proj, hor_resolution_max, ver_resolution_max);
        
        // p_spherical의 정보를 저장
        p.last_vecs.at(depth_index % HASH_PRIM) = p_spherical.vec;
        p.last_positions.at(depth_index % HASH_PRIM)[0] = p_spherical.hor_ind;
        p.last_positions.at(depth_index % HASH_PRIM)[1] = p_spherical.ver_ind;
        p.last_positions.at(depth_index % HASH_PRIM)[2] = p_spherical.position;
    }
}

bool DynObjFilter::InvalidPointCheck(const V3D &body, const int intensity)
{
    // 포인트가 특정 거리 이내이거나, 데이터셋 조건에 부합하면 유효하지 않은 포인트로 간주
    if ((pow(body(0), 2) + pow(body(1), 2) + pow(body(2), 2)) < blind_dis * blind_dis || 
        (dataset == 1 && fabs(body(0)) < 0.1 && fabs(body(1)) < 1.0) && fabs(body(2)) < 0.1)
    {
        return true;
    } 
    else
    {
        return false;
    }
}

bool DynObjFilter::SelfPointCheck(const V3D &body, const dyn_obj_flg dyn)
{
    // 데이터셋이 0일 때 특정 범위 내의 포인트를 자기 자신으로 인식
    if (dataset == 0)
    {      
        if ((body(0) > -1.2 && body(0) < -0.4 && body(1) > -1.7 && body(1) < -1.0 && body(2) > -0.65 && body(2) < -0.4) || 
            (body(0) > -1.75 && body(0) < -0.85 && body(1) > 1.0 && body(1) < 1.6 && body(2) > -0.75 && body(2) < -0.40) || 
            (body(0) > 1.4 && body(0) < 1.7 && body(1) > -1.3 && body(1) < -0.9 && body(2) > -0.8 && body(2) < -0.6) || 
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > -0.6 && body(1) < -0.45 && body(2) > -1.0 && body(2) < -0.9) || 
            (body(0) > 2.45 && body(0) < 2.6 && body(1) > 0.45 && body(1) < 0.6 && body(2) > -1.0 && body(2) < -0.9))
        {
            return true;
        }
        else
        {
            return false;
        }
    } 
    return false;
}

bool DynObjFilter::CheckVerFoV(const point_soph &p, const DepthMap &map_info)
{
    // 상하 시야 검사 초기화
    bool ver_up = false, ver_down = false;

    // ver_ind부터 아래로 시야 범위를 검사
    for(int i = p.ver_ind; i >= pixel_fov_down; i--)
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(map_info.depth_map[cur_pos].size() > 0)
        {
            ver_down = true;
            break;
        }
    } 

    // ver_ind부터 위로 시야 범위를 검사
    for(int i = p.ver_ind; i <= pixel_fov_up; i++)
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(map_info.depth_map[cur_pos].size() > 0)
        {
            ver_up = true;
            break;
        }
    }   

    // 위와 아래 시야 모두 차단된 경우 false 반환
    if(ver_up && ver_down)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void DynObjFilter::CheckNeighbor(const point_soph &p, const DepthMap &map_info, float &max_depth, float &min_depth)
{   
    // 주변 범위 크기 설정
    int n = checkneighbor_range;
    
    // 주변 포인트를 순회하며 깊이를 확인
    for (int i = -n; i <= n; i++)
    {
        for (int j = -n; j <= n; j++)
        {
            // 현재 위치 계산
            int cur_pos = (p.hor_ind + i) * MAX_1D_HALF + p.ver_ind + j;
            
            // 현재 위치가 범위 내에 있고, 깊이 정보가 존재하는 경우
            if(cur_pos < MAX_2D_N && cur_pos >= 0 && map_info.depth_map[cur_pos].size() > 0)
            {
                // 현재 위치의 최대 깊이와 최소 깊이를 가져옴
                float cur_max_depth = map_info.max_depth_static[cur_pos];
                float cur_min_depth = map_info.min_depth_static[cur_pos];
                
                // 최소 깊이 갱신
                if(min_depth > 10E-5) 
                    min_depth = std::min(cur_min_depth, min_depth);
                else 
                    min_depth = cur_min_depth;
                
                // 최대 깊이 갱신
                if(max_depth > 10E-5) 
                    max_depth = std::max(cur_max_depth, max_depth);
                else 
                    max_depth = cur_max_depth;
            }
        }
    }
}

bool DynObjFilter::Case1(point_soph &p)
{
    // depth_map_list의 크기를 저장
    int depth_map_num = depth_map_list.size();
    int occluded_map = depth_map_num;
    
    // depth_map_list를 역순으로 순회
    for (int i = depth_map_num - 1; i >= 0; i--)
    {   
        // 포인트를 구면 투영
        SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);
        
        // 포인트가 유효한 범위에 있는지 검사
        if (fabs(p.hor_ind) > MAX_1D || fabs(p.ver_ind) > MAX_1D_HALF || p.vec(2) < 0.0f \
            || p.position < 0 || p.position >= MAX_2D_N)
        {
            p.dyn = INVALID;
            continue;
        }
        
        // Case1Enter 검사
        if (Case1Enter(p, *depth_map_list[i]))
        { 
            // 거짓된 거부인지 검사
            if (Case1FalseRejection(p, *depth_map_list[i]))
            {
                occluded_map -= 1;
            }
        }
        else
        {
            occluded_map -= 1;
        }
        
        // occluded_map이 특정 값보다 작으면 false 반환
        if (occluded_map < occluded_map_thr1)
        {   
            return false;
        }
        // occluded_map이 조건을 충족하면 true 반환
        if (occluded_map - i >= occluded_map_thr1)
        {
            return true;
        }
    }
    
    // occluded_map이 특정 값 이상이면 true 반환
    if (occluded_map >= occluded_map_thr1)
    {
        return true;
    }
    return false;  
}

bool DynObjFilter::Case1Enter(const point_soph &p, const DepthMap &map_info)
{
    // 깊이 값 초기화
    float max_depth = 0, min_depth = 0;
    float max_depth_all = 0, min_depth_all = 0;
    
    // 현재 포인트 위치에 깊이 정보가 있으면 깊이 값을 가져옴
    if (map_info.depth_map[p.position].size() > 0)
    {
        max_depth = map_info.max_depth_static[p.position];
        min_depth = map_info.min_depth_static[p.position];
    }
    else 
    {
        // 포인트가 시야 범위 내에 있고 CheckVerFoV가 true인 경우
        if(p.ver_ind <= pixel_fov_up && p.ver_ind > pixel_fov_down && \
           p.hor_ind <= pixel_fov_left && p.hor_ind >= pixel_fov_right && \
           CheckVerFoV(p, map_info))
        {
            CheckNeighbor(p, map_info, max_depth, min_depth);
        }
    }        
    
    // 현재 최소/최대 깊이 임계값 계산
    float cur_min = max(cutoff_value, k_depth_min_thr1 * (p.vec(2) - d_depth_min_thr1)) + enter_min_thr1;
    float cur_max = max(cutoff_value, k_depth_max_thr1 * (p.vec(2) - d_depth_max_thr1)) + enter_max_thr1;
    float cur_depth = depth_thr1;
    
    // 데이터셋이 0이고 포인트가 왜곡된 경우 깊이 값을 확대
    if(dataset == 0 && p.is_distort) 
    {
        cur_min = enlarge_distort * cur_min;
        cur_max = enlarge_distort * cur_max;
        cur_depth = enlarge_distort * cur_depth;
    }
    
    // 깊이 조건을 만족하는지 검사
    if (p.vec(2) < min_depth - cur_max || \
        (min_depth < p.vec(2) - cur_min && max_depth > p.vec(2) + cur_max) || \
        (stop_object_detect && min_depth < 10E-5 && max_depth < 10E-5 && map_info.depth_map[p.position].size() > 0 && p.vec(2) < map_info.max_depth_all[p.position] + 1.0))
    {
        case1_num++;
        return true;
    }
    return false;
}

bool DynObjFilter::Case1FalseRejection(point_soph &p, const DepthMap &map_info)
{
    // 포인트와 DepthMap의 일관성을 검사
    return Case1MapConsistencyCheck(p, map_info, case1_interp_en);
}


bool  DynObjFilter::Case1MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp)
{   
    // 수평 거리의 절반 값 계산 (p.vec(2) 값과 blind_dis 중 큰 값을 사용)
    float hor_half = max(map_cons_hor_dis1/(max(p.vec(2), blind_dis)), map_cons_hor_thr1);
    
    // 수직 거리의 절반 값 계산 (p.vec(2) 값과 blind_dis 중 큰 값을 사용)
    float ver_half = max(map_cons_ver_dis1/(max(p.vec(2), blind_dis)), map_cons_ver_thr1);
    
    // 현재 깊이 임계값 계산
    float cur_map_cons_depth_thr1 = max(cutoff_value, k_depth_max_thr1*(p.vec(2) - d_depth_max_thr1)) + map_cons_depth_thr1;
    
    // 현재 최소 깊이 임계값 계산
    float cur_map_cons_min_thr1 = max(cutoff_value, k_depth_min_thr1*(p.vec(2) - d_depth_min_thr1)) + enter_min_thr1;
    
    // 현재 최대 깊이 임계값 계산
    float cur_map_cons_max_thr1 = max(cutoff_value, k_depth_max_thr1*(p.vec(2) - d_depth_max_thr1)) + enter_max_thr1;

    // 왜곡된 데이터세트일 경우 임계값을 확대
    if(dataset == 0 && p.is_distort) 
    {
        cur_map_cons_depth_thr1 = enlarge_distort*cur_map_cons_depth_thr1;
        cur_map_cons_min_thr1 = enlarge_distort*cur_map_cons_min_thr1;
        cur_map_cons_max_thr1 = enlarge_distort*cur_map_cons_max_thr1;
    }
    
    // 각도가 작은 경우 거리 및 깊이 임계값을 확대
    if (fabs(p.vec(1)) < enlarge_z_thr1 / 57.3)
    {
        hor_half = enlarge_angle * hor_half;
        ver_half = enlarge_angle * ver_half;
        cur_map_cons_depth_thr1 = enlarge_depth * cur_map_cons_depth_thr1;
    }

    // 수평 및 수직 해상도에 따른 개수 계산
    int cur_map_cons_hor_num1 = ceil(hor_half/hor_resolution_max);
    int cur_map_cons_ver_num1 = ceil(ver_half/ver_resolution_max);
    int num = 0;
    point_soph closest_point;
    float closest_dis = 100;

    // 수평 및 수직 인덱스를 기반으로 반복하며 맵 내 포인트 확인
    for (int ind_hor = -cur_map_cons_hor_num1; ind_hor <= cur_map_cons_hor_num1; ind_hor ++)
    {
        for (int ind_ver = -cur_map_cons_ver_num1; ind_ver <= cur_map_cons_ver_num1; ind_ver ++)
        {   
            // 새 위치 계산 (맵의 크기를 초과하지 않도록 제한)
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;

            // 해당 위치의 픽셀 내 포인트 벡터 가져오기
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];                        
            
            // 깊이 범위 조건을 만족하지 않으면 계속
            if (map_info.max_depth_static[pos_new] < p.vec(2) - cur_map_cons_min_thr1 || \
                map_info.min_depth_static[pos_new] > p.vec(2) + cur_map_cons_max_thr1)
            {
                continue;
            }   
            
            // 포인트 벡터 내 모든 포인트 반복
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j];
                
                // 포인트가 정적이며 깊이 및 거리 조건을 만족하는지 확인
                if (point->dyn == STATIC &&\
                  (fabs(p.vec(2)-point->vec(2)) <  cur_map_cons_depth_thr1 ||\
                   ((p.vec(2)-point->vec(2)) >  cur_map_cons_depth_thr1 && (p.vec(2)-point->vec(2)) <  cur_map_cons_min_thr1)) && \
                    fabs(p.vec(0)-point->vec(0)) < hor_half && \
                  fabs(p.vec(1)-point->vec(1)) < ver_half)
                {
                    return true;  // 조건 만족 시 true 반환
                }    
            }         
        }
    }   
    
    // 보간이 필요하고 특정 영역을 벗어난 경우 깊이 보간 계산
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float depth_static = DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
        float cur_interp = interp_thr1;
        
        // 깊이가 시작 깊이보다 크면 보간 값을 증가
        if(p.vec(2) > interp_start_depth1)
            cur_interp += ((p.vec(2) - interp_start_depth1)* interp_kp1 + interp_kd1);

        // 데이터세트가 0일 경우 특정 값 비교 후 결과 반환
        if(dataset == 0 )
        {
            if(fabs(depth_static+1) < 10E-5 || fabs(depth_static+2) < 10E-5)
            {
                return false;  // 값이 특정 범위에 있으면 false 반환
            }
            else 
            {
                if(fabs(depth_static - p.vec(2)) < cur_interp)
                {
                    return true;  // 보간 값이 임계값을 만족하면 true 반환
                } 
            }
        }
        else
        {
            if(fabs(depth_static+1) < 10E-5 || fabs(depth_static+2) < 10E-5)
            {
                return false;  // 값이 특정 범위에 있으면 false 반환
            }
            else 
            {
                if(fabs(depth_static - p.vec(2)) < cur_interp)
                {
                    return true;  // 보간 값이 임계값을 만족하면 true 반환
                } 
            }        
        }     
    }

    // 모든 조건을 만족하지 않으면 false 반환
    return false;
}


float DynObjFilter::DepthInterpolationStatic(point_soph & p, int map_index, const DepthMap2D &depth_map)
{
    // 이전 깊이 보간 값이 유효한지 확인
    if(fabs(p.last_depth_interps.at(map_index - depth_map_list.front()->map_index)) > 10E-4)
    {
        float depth_cal = p.last_depth_interps.at(map_index - depth_map_list.front()->map_index);
        return depth_cal; // 유효한 경우 깊이 값을 반환
    }

    // 세 개의 보간 포인트 초기화
    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    
    // 인접 포인트 벡터를 저장할 벡터 초기화
    vector<V3F> p_neighbors;
    int all_num = 0, static_num = 0, no_bg_num = 0;
    
    // 주어진 수평 및 수직 범위 내 모든 포인트 반복
    for (int ind_hor = -interp_hor_num; ind_hor <= interp_hor_num; ind_hor ++)
    {
        for (int ind_ver = -interp_ver_num; ind_ver <= interp_ver_num; ind_ver ++)
        {   
            // 새 위치 계산 (맵의 크기를 초과하지 않도록 제한)
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            
            // 픽셀 내 포인트 벡터 가져오기
            const vector<point_soph*> & points_in_pixel = depth_map[pos_new];
            
            // 포인트 벡터 내 모든 포인트 반복
            for (int j = 0; j < points_in_pixel.size(); j++)
            {   
                const point_soph* point = points_in_pixel[j];
                
                // 포인트의 시간이 p와 너무 가까우면 계속
                if (fabs(point->time - p.time) < frame_dur)
                {
                    continue;
                }
                
                // 수평 및 수직 차이 계산
                float hor_minus =  point->vec(0) - p.vec(0);
                float ver_minus =  point->vec(1) - p.vec(1);
                
                // 수평 및 수직 차이가 임계값 이내일 경우
                if (fabs(hor_minus) < interp_hor_thr && fabs(ver_minus) < interp_ver_thr)
                {
                    all_num++; // 전체 포인트 개수 증가
                    if(point->dyn == STATIC) 
                    {
                        static_num++; // 정적 포인트 개수 증가
                    }
                    if((point->vec(2) - p.vec(2)) <= interp_static_max && (p.vec(2) - point->vec(2)) < 5.0)
                    {
                        no_bg_num++; // 배경에 속하지 않는 포인트 개수 증가
                    }
                    if(point->dyn == STATIC)
                    {
                        // 정적 포인트인 경우 인접 포인트 벡터에 추가
                        p_neighbors.push_back(point->vec);
                        
                        // p_1 포인트 갱신
                        if (p_1(2) < 0.000001 || fabs(hor_minus) + fabs(ver_minus) < fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1)))
                        {
                            p_1 = point->vec;
                        }
                    }
                }
            }
        }
    }

    // p_1 포인트가 유효하지 않으면 -1 반환
    if (p_1(2) < 10E-5)
    {
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -1;
        return -1;
    }

    // 인접 포인트 크기 가져오기
    int cur_size = p_neighbors.size();
    
    // 세 개의 포인트를 선택해 삼각형 보간 수행
    for(int t_i = 0; t_i < cur_size-2; t_i++)
    {
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2 * (interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha = 0, beta = 0;
        
        // 두 번째 포인트 선택
        for(int i = t_i + 1; i < cur_size - 1; i++)
        {           
            if(fabs(p_neighborsvec(0)) + fabs(p_neighbors  - p)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                float single_fabs = fabs(p_neighbors  - p.vec(fabs(p_neighbors  - p.vec(1));
          if (single_fabs >= min_fabs) continue;
                
                // 세 번째 포인트 선택 및 삼각형 면적 계산
                for(int ii = i + 1; ii < cur_size; ii++)
                {
                    float cur_fabs = fabs(p_neighbors  - p.vec(0)) + fabsghbors  - p.vec(1)) + \
                              fabs(p_neighbors  - p.vec(0)) + fabs(p_neighbo.vec(1));
                    iabs < min_fabs)
                    {
                        float x1 = p_neighbors  - p_1(0);
                        float _neighbors  - p_1(0);
                        float y1 = bors  - p_1(1);
                        float y2 = p_neig - p_1(1);
                        float lower = x1 * y2 y1;
                        
                        // 보간 계수 alpha, beta 계산
                        if(fabs(lower) > 10E-5)
                        {
                            alpha = (x * y2 - y * x2) / lower;
                            beta = -(x * y1 - y * x1) / lower;
                            
                            // alpha, beta가 유효한 범위에 있는 경우
                            if(alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                            {
                                p_3 = p_neighbors[ii];
                                min_fabs = cur_fabs; 
                            }
                        }
                    }
                }  
            }
        }
        
        // 유효한 p_2 또는 p_3이 없으면 계속
        if (p_2(2) < 10E-5 || p_3(2) < 10E-5)
        {
            continue;
        }
        
        // 깊이 보간 값 계산 및 반환
        float depth_cal = (1 - alpha - beta) * p_1(2) + alpha * p_2(2) + beta * p_3(2);
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = depth_cal;
        return depth_cal;
    }

    // 정적 포인트가 충분하지 않으면 -2 반환
    if(static_num > 0 && cur_size < all_num / 2)
    {
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -2;
        return -2;
    }
    else
    {
        p.last_depth_interps.at(map_index - depth_map_list.front()->map_index) = -2;
        return -2;
    }
} // 반환값 -1은 포인트 없음, -2는 삼각형 보간 불가하지만 포인트는 있음


bool DynObjFilter::Case2(point_soph & p)
{   
    // 데이터셋이 0이고 포인트가 왜곡되었다면 거짓을 반환
    if(dataset == 0 && p.is_distort) return false;
    
    // 깊이 맵 리스트의 첫 번째 인덱스를 가져옴
    int first_i = depth_map_list.size();
    first_i -= 1;
    
    // 만약 첫 번째 인덱스가 0보다 작으면 거짓 반환
    if(first_i < 0) return false;
    
    // 포인트를 구면 좌표계로 변환
    point_soph p_spherical = p;   
    SphericalProjection(p, depth_map_list[first_i]->map_index, depth_map_list[first_i]->project_R, depth_map_list[first_i]->project_T, p_spherical);  
    
    // 구면 좌표계 인덱스 값이 범위를 벗어나거나 위치가 유효하지 않다면 처리 중단
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f || \
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        // 동적 객체 상태로 설정하고 거짓 반환
        p.dyn = INVALID;
        return false;
    }
    
    // 현재 차폐된 횟수를 0으로 초기화
    int cur_occ_times = 0;
    
    // Case2Enter 함수로 진입 여부를 확인
    if (Case2Enter(p_spherical, *depth_map_list[first_i]))
    {
        // 맵 일관성 검사를 통과하지 못하면 추가 검사를 수행
        if (!Case2MapConsistencyCheck(p_spherical, *depth_map_list[first_i], case2_interp_en))
        {
            double ti = 0;
            float vi = 0; 
            float min_hor = occ_hor_thr2, min_ver = occ_ver_thr2;
            bool map_cons = true;    
            
            // 수평 및 수직 인덱스를 기준으로 탐색
            for (int ind_hor = -occ_hor_num2; ind_hor <= occ_hor_num2; ind_hor ++)
            {
                for (int ind_ver = -occ_ver_num2; ind_ver <= occ_ver_num2; ind_ver ++)
                {   
                    // 새로운 위치 계산
                    int pos_new = ((p_spherical.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p_spherical.ver_ind +ind_ver)%MAX_1D_HALF);       
                    
                    // 위치가 유효하지 않다면 건너뜀
                    if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
                    
                    // 해당 픽셀의 포인트 벡터를 가져옴
                    const vector<point_soph*> & points_in_pixel = depth_map_list[first_i]->depth_map[pos_new];                    
                    
                    // 깊이가 기준보다 크면 건너뜀
                    if (depth_map_list[first_i]->min_depth_all[pos_new] > p_spherical.vec(2))
                    {
                        continue;
                    }   
                    
                    // 포인트를 차례로 검사하여 차폐 여부 확인
                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const point_soph*  p_occ = points_in_pixel[k];                   
                        
                        // 포인트가 차폐된 경우 및 깊이 일관성 검사를 통과하면 처리
                        if(Case2IsOccluded(p_spherical, *p_occ) && Case2DepthConsistencyCheck(*p_occ, *depth_map_list[first_i]))
                        {                        
                            cur_occ_times = 1;
                            
                            // 차폐 횟수가 기준 이상이면 루프 탈출
                            if(cur_occ_times >= occluded_times_thr2) break;
                            
                            // 시간과 속도를 계산
                            ti = (p_occ->time + p.time)/2;
                            vi = (p_spherical.vec(2) - p_occ->vec(2))/(p.time - p_occ->time);
                            
                            // 차폐 인덱스 및 벡터 설정
                            p.occu_index[0] = depth_map_list[first_i]->map_index;
                            p.occu_index[1] = pos_new;
                            p.occu_index[2] = k;
                            p.occ_vec = p_spherical.vec;
                            p.occu_times = cur_occ_times;
                            
                            // 현재 포인트와 차폐된 포인트 복사
                            point_soph  p0 = p;
                            point_soph p1 = *points_in_pixel[k];                          
                            
                            // 이전 맵들을 검사하기 위한 인덱스 설정
                            int i = depth_map_list.size();
                            i = i - 2;
                            V3D t1, t2;
                            t1.setZero();
                            t2.setZero();
                            
                            // 차례로 맵을 검사하며 일관성 및 차폐 검사
                            while(i >= 0)
                            {                              
                                // 포인트의 차폐 인덱스가 없거나 기준보다 작으면 구면 변환 수행
                                if(p1.occu_index[0] == -1 || p1.occu_index[0] < depth_map_list.front()->map_index)
                                {
                                    SphericalProjection(p1, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p1);
                                    
                                    // 포인트가 차폐되었는지 검색
                                    if(Case2SearchPointOccludingP(p1, *depth_map_list[i]))
                                    {
                                        p1.occ_vec = p1.vec;
                                    }
                                    else
                                    {
                                        break;
                                    }                                   
                                }                                
                                
                                // 인덱스를 업데이트
                                i = p1.occu_index[0] - depth_map_list.front()->map_index;
                                point_soph*  p2 = depth_map_list[i]->depth_map[p1.occu_index[1]][p1.occu_index[2]];                                   
                                
                                // 구면 변환을 다시 수행
                                SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);
                                
                                // 맵 일관성 검사를 통과하지 못하면 루프 중단
                                if(Case2MapConsistencyCheck(p, *depth_map_list[i], case2_interp_en))
                                {
                                    map_cons = false;
                                    break;
                                }
                                
                                // 속도와 시간을 다시 계산
                                float vc = (p1.occ_vec(2) - p2->vec(2))/(p1.time - p2->time);
                                double tc = (p2->time + p1.time)/2;        
                                
                                // 포인트가 차폐되고 속도 검사를 통과하면 차폐 횟수 증가
                                if (Case2IsOccluded(p, *p2) &&\
                                    Case2DepthConsistencyCheck(*p2, *depth_map_list[i]) && Case2VelCheck(vi, vc, ti-tc) )
                                {                            
                                    cur_occ_times += 1;
                                    
                                    // 차폐 횟수가 기준 이상이면 루프 탈출
                                    if(cur_occ_times >= occluded_times_thr2)
                                    {
                                        p.occu_times = cur_occ_times;
                                        return true;
                                    }
                                    
                                    // 전역 위치 설정 및 포인트 복사
                                    t2 = p2->glob;
                                    p1 = *p2;
                                    vi = vc;
                                    ti = tc;
                                }
                                else
                                {
                                    break;
                                }
                                i--;
                            }                       
                        } 
                        if(cur_occ_times >= occluded_times_thr2) break;
                    }
                    if(cur_occ_times >= occluded_times_thr2) break;
                }
                if(cur_occ_times >= occluded_times_thr2) break;
            }
        }
    }
    
    // 차폐 횟수가 기준 이상이면 true 반환
    if (cur_occ_times >= occluded_times_thr2) 
    {
        p.occu_times = cur_occ_times;
        return true;
    }
    
    // 차폐 조건을 만족하지 않으면 false 반환
    return false;
}



bool DynObjFilter::Case2Enter(point_soph & p, const DepthMap &map_info)
{
    // 포인트가 정적인 객체가 아니면 거짓 반환
    if(p.dyn != STATIC)
    {
        return false;
    }
    
    // 최대 깊이 초기화
    float max_depth = 0;
    // 깊이 임계값 계산
    float depth_thr2_final = max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + occ_depth_thr2;
    
    // 깊이 맵에 포인트가 존재하면 처리
    if(map_info.depth_map[p.position].size() > 0)
    {
        // 가장 깊은 포인트를 가져옴
        const point_soph* max_point = map_info.depth_map[p.position][map_info.max_depth_index_all[p.position]];
        max_depth = max_point->vec(2); 
        
        // 시간 차이에 따른 깊이 임계값 업데이트
        float delta_t = (p.time - max_point->time);
        depth_thr2_final = min(depth_thr2_final, v_min_thr2*delta_t);
    }
    
    // 포인트 깊이가 최대 깊이와 임계값을 초과하면 처리
    if(p.vec(2) > max_depth + depth_thr2_final) 
    {
        case2_num ++;
        return true;
    }
    else
    {
        return false;
    }
}

bool DynObjFilter::Case2MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp)
{
    // 현재 수평, 수직, 깊이 임계값 설정
    float cur_hor = map_cons_hor_thr2;
    float cur_ver = map_cons_ver_thr2;
    float cur_depth = max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + map_cons_depth_thr2;
    
    // 수평 및 수직 인덱스를 기준으로 탐색
    for (int ind_hor = -map_cons_hor_num2; ind_hor <= map_cons_hor_num2; ind_hor ++)
    {
        for (int ind_ver = -map_cons_ver_num2; ind_ver <= map_cons_ver_num2; ind_ver ++)  
        {
            // 새로운 위치 계산
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);
            
            // 위치가 유효하지 않다면 건너뜀
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            
            // 해당 픽셀의 포인트 벡터를 가져옴
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];                         
            
            // 깊이 값이 기준 범위 내에 있지 않다면 건너뜀
            if (map_info.max_depth_all[pos_new] > p.vec(2) + cur_depth && \
                map_info.min_depth_all[pos_new] < p.vec(2) - cur_depth)
            {
                continue;
            }   
            
            // 포인트를 차례로 검사하여 일관성 확인
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j];
                
                // 정적 객체이며 시간 차이가 크고 깊이 및 위치가 기준 범위 내에 있으면 참 반환
                if (point->dyn == STATIC && \
                    fabs(p.time-point->time) > frame_dur && \
                    fabs(p.vec(2)-point->vec(2)) <  cur_depth && \
                    fabs(p.vec(0)-point->vec(0)) < map_cons_hor_thr2 && \
                    fabs(p.vec(1)-point->vec(1)) < map_cons_ver_thr2)
                {
                    return true;
                }               
            }         
        }
    }
    
    // 보간(interpolation) 옵션이 활성화되어 있고 경계값을 벗어나면 보간 처리
    if(interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r) )
    {
        // 보간 임계값 계산
        float cur_interp = interp_thr2*(depth_map_list.back()->map_index - map_info.map_index + 1);
        // 전체 깊이 보간 값 계산
        float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
        
        // 깊이 차이가 보간 임계값보다 작으면 참 반환, 그렇지 않으면 거짓 반환
        if( fabs(p.vec(2) - depth_all)  < cur_interp) 
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool  DynObjFilter::Case2SearchPointOccludingP(point_soph & p, const DepthMap &map_info)
{ 
    // 가로 방향 인덱스를 -occ_hor_num2부터 occ_hor_num2까지 반복합니다
    for (int ind_hor = -occ_hor_num2; ind_hor <= occ_hor_num2; ind_hor ++)
    {
        // 세로 방향 인덱스를 -occ_ver_num2부터 occ_ver_num2까지 반복합니다
        for (int ind_ver = -occ_ver_num2; ind_ver <= occ_ver_num2; ind_ver ++)
        {   
            // 새로운 위치를 계산합니다
            int pos_new = ((p.hor_ind + ind_hor)%MAX_1D) * MAX_1D_HALF + ((p.ver_ind +ind_ver)%MAX_1D_HALF);         
            // 위치가 범위를 벗어나면 계속 진행합니다
            if (pos_new < 0 || pos_new >= MAX_2D_N)  continue;
            // 해당 픽셀에 있는 점 목록을 가져옵니다
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];            
            // 최소 깊이가 점 p의 깊이보다 크면 계속 진행합니다
            if (map_info.min_depth_all[pos_new] > p.vec(2))
            {
                continue;
            }   
            // 각 픽셀 안의 점을 순회하며 확인합니다
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                // 조건에 맞는 점을 가져옵니다
                const point_soph* p_cond = points_in_pixel[j];
                // 점이 가려져 있고 깊이 일관성이 있는지 확인합니다
                if (Case2IsOccluded(p, *p_cond) && Case2DepthConsistencyCheck(*p_cond, map_info)) 
                {
                    // 가려진 점의 인덱스와 벡터를 설정합니다
                    p.occu_index[0] = map_info.map_index;
                    p.occu_index[1] = pos_new;
                    p.occu_index[2] = j;
                    p.occ_vec = p.vec;
                    // 가려진 점을 찾았으면 true를 반환합니다
                    return true;
                }
            }        
        }
    }
    // 가려진 점을 찾지 못했으면 false를 반환합니다
    return false;
}

bool  DynObjFilter::Case2IsOccluded(const point_soph & p, const point_soph & p_occ)
{
    // 데이터셋이 0이고 왜곡된 경우나 유효하지 않은 경우 false를 반환합니다
    if((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID) return false;
    // 점이 자기 자신의 범위 내에 있으면 false를 반환합니다
    if((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) || \
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }
    // 시간 차이를 계산합니다
    float delta_t = p.time - p_occ.time;
    // 현재 가로와 세로 가림 임계값을 설정합니다
    float cur_occ_hor = occ_hor_thr2; 
    float cur_occ_ver = occ_ver_thr2; 
    // 시간 차이가 양수일 때만 처리합니다
    if(delta_t > 0)
    {
        // 깊이 임계값을 계산합니다
        float depth_thr2_final = min(max(cutoff_value, k_depth_max_thr2*(p.vec(2) - d_depth_max_thr2)) + occ_depth_thr2, v_min_thr2*delta_t);
        // 깊이 및 가로/세로 위치 차이를 확인하여 가려진 점인지 판단합니다
        if (p.vec(2) >  p_occ.vec(2) + depth_thr2_final && \
            fabs(p.vec(0)-p_occ.vec(0)) < cur_occ_hor && \
            fabs(p.vec(1)-p_occ.vec(1)) < cur_occ_ver )
        {
            return true;
        }              
    }
    // 가려진 점이 아니면 false를 반환합니다
    return false;
}

float DynObjFilter::DepthInterpolationAll(point_soph & p, int map_index, const DepthMap2D &depth_map)
{
    // 깊이를 초기화합니다
    V3F p_1 = V3F::Zero();
    V3F p_2 = V3F::Zero();
    V3F p_3 = V3F::Zero();
    // 주변 점들을 저장할 벡터를 만듭니다
    vector<V3F> p_neighbors;
    int all_num = 0;
    
    // 가로 및 세로 방향 인덱스를 탐색합니다
    for (int ind_hor = -interp_hor_num; ind_hor <= interp_hor_num; ind_hor++)
    {
        for (int ind_ver = -interp_ver_num; ind_ver <= interp_ver_num; ind_ver++)
        {   
            // 새 위치를 계산합니다
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);          
            // 위치가 유효하지 않으면 계속 진행합니다
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
            // 깊이 맵에서 픽셀의 점들을 가져옵니다
            const vector<point_soph*> & points_in_pixel = depth_map[pos_new];
            
            // 픽셀 안의 각 점을 탐색합니다
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j]; 
                // 시간 차이가 프레임 지속 시간보다 작으면 건너뜁니다
                if (fabs(point->time - p.time) < frame_dur)
                {
                    continue;
                }
                // 가로 및 세로 차이를 계산합니다
                float hor_minus = point->vec(0) - p.vec(0);
                float ver_minus = point->vec(1) - p.vec(1);
                // 임계값 내에 있는지 확인합니다
                if (fabs(hor_minus) < interp_hor_thr && fabs(ver_minus) < interp_ver_thr)
                {
                    all_num++;
                    p_neighbors.push_back(point->vec);
                    // 가장 가까운 점을 업데이트합니다
                    if (p_1(2) < 0.000001 || fabs(hor_minus) + fabs(ver_minus) < fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1)))
                    {
                        p_1 = point->vec;
                    }                   
                }
            }
        }
    }
    
    // 주변 점의 수를 확인합니다
    int cur_size = p_neighbors.size();
    if (p_1(2) < 10E-5 || cur_size < 3)
    {
        // 점이 없으면 -1을 반환합니다
        return -1;
    }
    
    // 삼각형을 구성할 점들을 찾습니다
    for (int t_i = 0; t_i < cur_size - 2; t_i++)
    {
        p_1 = p_neighbors[t_i];
        p_2 = V3F::Zero();
        p_3 = V3F::Zero();
        float min_fabs = 2 * (interp_hor_thr + interp_ver_thr);
        float x = p.vec(0) - p_1(0);
        float y = p.vec(1) - p_1(1);
        float alpha = 0, beta = 0;
        
        // 두 번째 점을 찾습니다
        for (int i = t_i + 1; i < cur_size - 1; i++)
        {
            if (fabs(p_neighborsvec(0)) + fabs(p_neighbors  - p)) < min_fabs)
            {
                p_2 = p_neighbors[i];
                float single_fabs = fabs(p_neighbors  - p.vec(fabs(p_neighbors  - p.vec(1));
          if (single_fabs >= min_fabs) continue;
                
                // 세 번째 점을 찾습니다
                for (int ii = i + 1; ii < cur_size; ii++)
                {
                    float cur_fabs = fabs(p_neighbors  - p.vec(0)) + fabsghbors  - p.vec(1)) + \
                              fabs(p_neighbors  - p.vec(0)) + fabs(p_neighbo.vec(1));
                    ifabs < min_fabs)
                    {
                        float x1 = p_neighbors  - p_1(0);
                        float _neighbors  - p_1(0);
                        float y1 = bors  - p_1(1);
                        float y2 = p_neig - p_1(1);
                        float lower = x1 * y2 y1;
                        
                        // 삼각형의 면적이 충분히 큰지 확인합니다
                        if (fabs(lower) > 10E-5)
                        {
                            alpha = (x * y2 - y * x2) / lower;
                            beta = -(x * y1 - y * x1) / lower;
                            if (alpha > 0 && alpha < 1 && beta > 0 && beta < 1 && (alpha + beta) > 0 && (alpha + beta) < 1)
                            {
                                p_3 = p_neighbors[ii];
                                min_fabs = cur_fabs; 
                            }                        
                        }
                    }
                }
            }
        }
        
        // 깊이를 계산할 점이 없으면 계속 진행합니다
        if (p_2(2) < 10E-5 || p_3(2) < 10E-5)
        {
            continue;
        }
        
        // 깊이를 보간하여 반환합니다
        float depth_cal = (1 - alpha - beta) * p_1(2) + alpha * p_2(2) + beta * p_3(2);
        return depth_cal;
    }
    
    // 삼각형이 없으면 -2를 반환합니다
    return -2;
} // -1은 점이 없음을 나타내고, -2는 삼각형이 없음을 나타냅니다 > 1000은 가우스 보간을 나타냅니다


bool DynObjFilter::Case2DepthConsistencyCheck(const point_soph & p, const DepthMap &map_info)
{
    // 깊이 차이 누적 값을 초기화합니다
    float all_minus = 0;
    // 점 개수를 초기화합니다
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;
    
    // 가로 및 세로 방향으로 깊이 일관성 검사를 수행합니다
    for (int ind_hor = -depth_cons_hor_num2; ind_hor <= depth_cons_hor_num2; ind_hor++)
    {
        for (int ind_ver = -depth_cons_ver_num2; ind_ver <= depth_cons_ver_num2; ind_ver++)
        {   
            // 새로운 위치를 계산합니다
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF); 
            // 위치가 유효하지 않으면 건너뜁니다
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
            
            // 깊이 맵에서 픽셀의 점들을 가져옵니다
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];
            
            // 각 점에 대해 깊이 일관성을 검사합니다
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j]; 
                // 시간 및 위치 차이가 임계값 내에 있는 경우에만 처리합니다
                if (fabs(point->time - p.time) < frame_dur && fabs(point->vec(0) - p.vec(0)) < depth_cons_hor_thr2 && \
                    fabs(point->vec(1) - p.vec(1)) < depth_cons_ver_thr2)
                {
                    all_num++;
                    if (point->dyn == STATIC) 
                    {
                        // 깊이 차이를 계산합니다
                        float cur_minus = p.vec(2) - point->vec(2);
                        if (fabs(cur_minus) < depth_cons_depth_max_thr2)
                        {
                            num++;
                            all_minus += fabs(point->vec(2) - p.vec(2));
                        }
                        else if (cur_minus > 0)
                        {
                            smaller_num++;
                        }
                        else
                        {
                            greater_num++;
                        }
                    }
                }
            }
        }
    }
    
    // 점이 하나 이상 있는 경우 검사를 진행합니다
    if (all_num > 0)
    {
        // 일관성 점이 두 개 이상일 때 평균 깊이 차이를 확인합니다
        if (num > 1)
        {           
            float cur_depth_thr = max(depth_cons_depth_thr2, k_depth2 * p.vec(2));
            if (all_minus / (num - 1) > cur_depth_thr)
            {
                return false;
            }      
        }
        // 깊이 차이가 모두 한쪽 방향일 경우 true를 반환합니다
        if (greater_num == 0 || smaller_num == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // 점이 없으면 false를 반환합니다
        return false;
    }  
}



bool DynObjFilter::Case2VelCheck(float v1, float v2, double delta_t)
{
    // 두 속도 차이가 주어진 시간 차이와 가속 임계값을 곱한 값보다 작으면 true를 반환합니다
    if (fabs(v1 - v2) < delta_t * acc_thr2)
    {
        return true;
    }
    // 그렇지 않으면 false를 반환합니다
    return false;
}

bool DynObjFilter::Case3VelCheck(float v1, float v2, double delta_t)
{
    // 두 속도 차이가 주어진 시간 차이와 가속 임계값을 곱한 값보다 작으면 true를 반환합니다
    if (fabs(v1 - v2) < delta_t * acc_thr3)
    {
        return true;
    }
    // 그렇지 않으면 false를 반환합니다
    return false;
}

bool DynObjFilter::Case3(point_soph & p)
{
    // 데이터셋이 0이고 점이 왜곡된 경우는 처리하지 않으며 false를 반환합니다
    if (dataset == 0 && p.is_distort) return false;

    // 깊이 맵 리스트에서 가장 최신 항목의 인덱스를 설정합니다
    int first_i = depth_map_list.size();
    first_i -= 1;
    // 깊이 맵이 비어 있으면 false를 반환합니다
    if (first_i < 0) return false;

    // 점을 구면 좌표로 변환합니다
    point_soph p_spherical = p;
    SphericalProjection(p, depth_map_list[first_i]->map_index, depth_map_list[first_i]->project_R, depth_map_list[first_i]->project_T, p_spherical);

    // 구면 좌표에서 인덱스와 깊이가 유효하지 않으면 점을 INVALID로 설정하고 false를 반환합니다
    if (fabs(p_spherical.hor_ind) >= MAX_1D || fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f ||
        p_spherical.position < 0 || p_spherical.position >= MAX_2D_N)
    {
        p.dyn = INVALID;
        return false;
    }

    int cur_occ_times = 0; // 현재 가림 횟수를 초기화합니다

    // Case3의 조건에 맞는지 확인합니다
    if (Case3Enter(p_spherical, *depth_map_list[first_i]))
    {
        // 깊이 맵 일관성 검사에 실패한 경우 가림 점 탐색을 시작합니다
        if (!Case3MapConsistencyCheck(p_spherical, *depth_map_list[first_i], case3_interp_en))
        {
            double ti = 0; // 시간 차이를 초기화합니다
            float vi = 0;  // 속도를 초기화합니다
            float min_hor = occ_hor_thr3, min_ver = occ_ver_thr3;
            bool map_cons = true; // 맵 일관성 플래그를 설정합니다

            // 가로 방향과 세로 방향을 탐색합니다
            for (int ind_hor = -occ_hor_num3; ind_hor <= occ_hor_num3; ind_hor++)
            {
                for (int ind_ver = -occ_ver_num3; ind_ver <= occ_ver_num3; ind_ver++)
                {
                    // 새 위치를 계산합니다
                    int pos_new = ((p_spherical.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p_spherical.ver_ind + ind_ver) % MAX_1D_HALF);
                    // 위치가 유효하지 않으면 건너뜁니다
                    if (pos_new < 0 || pos_new >= MAX_2D_N) continue;

                    // 새 위치의 픽셀에 있는 점들을 가져옵니다
                    const vector<point_soph*> & points_in_pixel = depth_map_list[first_i]->depth_map[pos_new];
                    // 현재 깊이 값이 점의 깊이보다 작으면 건너뜁니다
                    if (depth_map_list[first_i]->max_depth_all[pos_new] < p_spherical.vec(2)) continue;

                    // 픽셀 내의 각 점을 검사합니다
                    for (int k = 0; k < points_in_pixel.size() && map_cons; k++)
                    {
                        const point_soph* p_occ = points_in_pixel[k];
                        // 가림 조건과 깊이 일관성을 모두 만족하는 경우
                        if (Case3IsOccluding(p_spherical, *p_occ) && Case3DepthConsistencyCheck(*p_occ, *depth_map_list[first_i]))
                        {
                            // 가림 횟수를 증가시키고 시간 및 속도를 업데이트합니다
                            cur_occ_times = 1;
                            ti = (p_occ->time + p.time) / 2;
                            vi = (p_occ->vec(2) - p_spherical.vec(2)) / (p.time - p_occ->time);

                            // 가림 인덱스와 벡터를 설정합니다
                            p.is_occu_index[0] = depth_map_list[first_i]->map_index;
                            p.is_occu_index[1] = pos_new;
                            p.is_occu_index[2] = k;
                            p.is_occ_vec = p_spherical.vec;
                            p.is_occu_times = cur_occ_times;

                            // 깊이 맵을 역순으로 탐색하며 일관성을 확인합니다
                            point_soph p0 = p;
                            point_soph p1 = *points_in_pixel[k];
                            int i = depth_map_list.size() - 2;

                            while (i >= 0)
                            {
                                // 가림 점의 인덱스가 유효하지 않거나 범위를 벗어나면 새로 프로젝션합니다
                                if (p1.is_occu_index[0] == -1 || p1.is_occu_index[0] < depth_map_list.front()->map_index)
                                {
                                    SphericalProjection(p1, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p1);
                                    // 가림된 점을 탐색합니다
                                    if (Case3SearchPointOccludedbyP(p1, *depth_map_list[i]))
                                    {
                                        p1.is_occ_vec = p1.vec;
                                    }
                                    else
                                    {
                                        break; // 찾지 못하면 종료합니다
                                    }
                                }

                                // 깊이 맵 인덱스를 업데이트합니다
                                i = p1.is_occu_index[0] - depth_map_list.front()->map_index;
                                point_soph* p2 = depth_map_list[i]->depth_map[p1.is_occu_index[1]][p1.is_occu_index[2]];
                                SphericalProjection(p, depth_map_list[i]->map_index, depth_map_list[i]->project_R, depth_map_list[i]->project_T, p);

                                // 맵 일관성 검사를 수행합니다
                                if (Case3MapConsistencyCheck(p, *depth_map_list[i], case3_interp_en))
                                {
                                    map_cons = false; // 일관성이 없으면 루프를 종료합니다
                                    break;
                                }

                                // 속도를 계산합니다
                                float vc = -(p1.is_occ_vec(2) - p2->vec(2)) / (p1.time - p2->time);
                                double tc = (p2->time + p1.time) / 2;

                                // 가림 및 속도 일관성을 확인합니다
                                if (Case3IsOccluding(p, *p2) &&
                                    Case3DepthConsistencyCheck(*p2, *depth_map_list[i]) && Case3VelCheck(vi, vc, ti - tc))
                                {
                                    cur_occ_times += 1;
                                    if (cur_occ_times >= occluding_times_thr3)
                                    {
                                        p.is_occu_times = cur_occ_times;
                                        return true; // 가림 횟수가 임계값 이상이면 true를 반환합니다
                                    }
                                    p1 = *p2; // 다음 점으로 업데이트합니다
                                    vi = vc;
                                    ti = tc;
                                }
                                else
                                {
                                    break; // 조건을 만족하지 않으면 종료합니다
                                }
                                i--; // 이전 깊이 맵으로 이동합니다
                            }
                        }
                        if (cur_occ_times >= occluding_times_thr3) break; // 가림 횟수가 임계값 이상이면 종료합니다
                    }
                    if (cur_occ_times >= occluding_times_thr3) break; // 루프를 종료합니다
                }
                if (cur_occ_times >= occluding_times_thr3) break; // 외부 루프를 종료합니다
            }
        }
    }
    if (cur_occ_times >= occluding_times_thr3)
    {
        p.is_occu_times = cur_occ_times;
        return true; // 최종적으로 가림 횟수가 임계값 이상이면 true를 반환합니다
    }
    return false; // 조건을 만족하지 않으면 false를 반환합니다
}



bool DynObjFilter::Case3Enter(point_soph & p, const DepthMap &map_info)
{
    // 점이 STATIC이 아니면 false를 반환합니다
    if (p.dyn != STATIC)
    {
        return false;
    }
    
    // 최소 깊이와 깊이 임계값을 초기화합니다
    float min_depth = 0;
    float depth_thr3_final = max(cutoff_value, k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) + occ_depth_thr3;
    
    // 현재 위치에 점이 존재하는 경우
    if (map_info.depth_map[p.position].size() > 0)
    {
        // 최소 깊이를 가져옵니다
        const point_soph* min_point = map_info.depth_map[p.position][map_info.min_depth_index_all[p.position]];
        min_depth = min_point->vec(2);
        
        // 시간 차이를 계산하여 깊이 임계값을 수정합니다
        float delta_t = (p.time - min_point->time);
        depth_thr3_final = min(depth_thr3_final, v_min_thr3 * delta_t);
    }
    
    // 데이터셋이 0이고 왜곡된 경우 깊이 임계값을 확대합니다
    if (dataset == 0 && p.is_distort)
    {
        depth_thr3_final = enlarge_distort * depth_thr3_final;
    }
    
    // 점의 깊이가 최소 깊이보다 크고 깊이 임계값보다 작으면 true를 반환합니다
    if (p.vec(2) < min_depth - depth_thr3_final)
    {
        case3_num++;
        return true;
    }
    else
    {
        return false;
    }
}

bool DynObjFilter::Case3MapConsistencyCheck(point_soph & p, const DepthMap &map_info, bool interp)
{
    // 속도, 가로, 세로, 깊이 임계값을 설정합니다
    float cur_v_min = v_min_thr3;
    float cur_hor = map_cons_hor_thr3;
    float cur_ver = map_cons_ver_thr3;
    float cur_depth = max(cutoff_value, k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) + map_cons_depth_thr3;
    
    // 데이터셋이 0이고 왜곡된 경우 속도 임계값을 확대합니다
    if (dataset == 0 && p.is_distort) cur_v_min = enlarge_distort * cur_v_min;
    
    // 가로 및 세로 방향으로 일관성 검사를 수행합니다
    for (int ind_hor = -map_cons_hor_num3; ind_hor <= map_cons_hor_num3; ind_hor++)
    {
        for (int ind_ver = -map_cons_ver_num3; ind_ver <= map_cons_ver_num3; ind_ver++)
        {
            // 새 위치를 계산합니다
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            // 위치가 유효하지 않으면 건너뜁니다
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
            
            // 해당 위치의 점들을 가져옵니다
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];
            
            // 최대 깊이 및 최소 깊이 조건을 확인합니다
            if (map_info.max_depth_all[pos_new] > p.vec(2) + cur_depth &&
                map_info.min_depth_all[pos_new] < p.vec(2) - cur_depth)
            {
                continue;
            }
            
            // 각 점에 대해 일관성을 확인합니다
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j];
                // 점이 STATIC이고 시간 차이가 프레임 지속 시간보다 크며 깊이 및 위치 차이가 임계값 내에 있는지 확인합니다
                if (point->dyn == STATIC &&
                    fabs(p.time - point->time) > frame_dur &&
                    (point->vec(2) - p.vec(2)) < cur_depth && 
                    fabs(p.vec(0) - point->vec(0)) < cur_hor &&
                    fabs(p.vec(1) - point->vec(1)) < cur_ver)
                {
                    return true;
                }
            }
        }
    }
    
    // 보간이 필요한 경우 좌표 범위와 보간 깊이 차이를 확인합니다
    if (interp && (p.local(0) < self_x_b || p.local(0) > self_x_f || p.local(1) > self_y_l || p.local(1) < self_y_r))
    {
        float cur_interp = interp_thr3 * (depth_map_list.back()->map_index - map_info.map_index + 1);
        float depth_all = DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
        if (fabs(p.vec(2) - depth_all) < cur_interp)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool DynObjFilter::Case3SearchPointOccludedbyP(point_soph & p, const DepthMap &map_info)
{
    // 가로 및 세로 방향으로 탐색합니다
    for (int ind_hor = -occ_hor_num3; ind_hor <= occ_hor_num3; ind_hor++)
    {
        for (int ind_ver = -occ_ver_num3; ind_ver <= occ_ver_num3; ind_ver++)
        {
            // 새 위치를 계산합니다
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            // 위치가 유효하지 않으면 건너뜁니다
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
            
            // 해당 위치의 점들을 가져옵니다
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];
            
            // 최소 깊이 조건을 확인합니다
            if (map_info.min_depth_all[pos_new] > p.vec(2))
            {
                continue;
            }
            
            // 각 점에 대해 가림 조건과 깊이 일관성을 확인합니다
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* p_cond = points_in_pixel[j];
                if (Case3IsOccluding(p, *p_cond) && Case3DepthConsistencyCheck(*p_cond, map_info))
                {
                    // 가림 인덱스와 벡터를 설정하고 true를 반환합니다
                    p.is_occu_index[0] = map_info.map_index;
                    p.is_occu_index[1] = pos_new;
                    p.is_occu_index[2] = j;
                    p.occ_vec = p.vec;
                    return true;
                }
            }
        }
    }
    // 조건을 만족하지 않으면 false를 반환합니다
    return false;
}


bool DynObjFilter::Case3IsOccluding(const point_soph & p, const point_soph & p_occ)
{
    // 데이터셋이 0이고 점이 왜곡된 경우나 유효하지 않은 경우 false를 반환합니다
    if ((dataset == 0 && p_occ.is_distort) || (dataset == 0 && p.is_distort) || p_occ.dyn == INVALID) return false;

    // 점이 자신의 범위 내에 있으면 false를 반환합니다
    if ((p.local(0) > self_x_b && p.local(0) < self_x_f && p.local(1) < self_y_l && p.local(1) > self_y_r) || 
        (p_occ.local(0) > self_x_b && p_occ.local(0) < self_x_f && p_occ.local(1) < self_y_l && p_occ.local(1) > self_y_r))
    {
        return false;
    }

    // 시간 차이를 계산합니다
    float delta_t = p.time - p_occ.time;
    if (delta_t > 0)
    {
        // 깊이 임계값을 설정합니다
        float depth_thr3_final = min(max(cutoff_value, k_depth_max_thr3 * (p.vec(2) - d_depth_max_thr3)) + map_cons_depth_thr3, v_min_thr3 * delta_t);
        if (dataset == 0 && p.is_distort) depth_thr3_final = enlarge_distort * depth_thr3_final;

        // 깊이와 가로/세로 위치 차이가 임계값 내에 있는지 확인합니다
        if (p_occ.vec(2) > p.vec(2) + depth_thr3_final &&
            fabs(p.vec(0) - p_occ.vec(0)) < occ_hor_thr3 &&
            fabs(p.vec(1) - p_occ.vec(1)) < occ_ver_thr3)
        {
            return true;
        }            
    }
    // 조건을 만족하지 않으면 false를 반환합니다
    return false;
}

bool DynObjFilter::Case3DepthConsistencyCheck(const point_soph & p, const DepthMap &map_info)
{
    // 깊이 차이 누적 값과 개수를 초기화합니다
    float all_minus = 0;
    int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;

    // 가로 및 세로 방향으로 깊이 일관성 검사를 수행합니다
    for (int ind_hor = -depth_cons_hor_num3; ind_hor <= depth_cons_hor_num3; ind_hor++)
    {
        for (int ind_ver = -depth_cons_ver_num3; ind_ver <= depth_cons_ver_num3; ind_ver++)
        {
            // 새 위치를 계산합니다
            int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF + ((p.ver_ind + ind_ver) % MAX_1D_HALF);
            // 위치가 유효하지 않으면 건너뜁니다
            if (pos_new < 0 || pos_new >= MAX_2D_N) continue;

            // 해당 위치의 점들을 가져옵니다
            const vector<point_soph*> & points_in_pixel = map_info.depth_map[pos_new];

            // 각 점에 대해 일관성을 검사합니다
            for (int j = 0; j < points_in_pixel.size(); j++)
            {
                const point_soph* point = points_in_pixel[j];
                // 시간 및 위치 차이가 임계값 내에 있는 경우에만 처리합니다
                if (fabs(point->time - p.time) < frame_dur && fabs(point->vec(0) - p.vec(0)) < depth_cons_hor_thr3 &&
                    fabs(point->vec(1) - p.vec(1)) < depth_cons_ver_thr3)
                {
                    all_num++;
                    // 점이 STATIC이면 깊이 차이를 확인합니다
                    if (point->dyn == STATIC)
                    {
                        float cur_minus = p.vec(2) - point->vec(2);
                        // 깊이 차이가 임계값 내에 있으면 누적합니다
                        if (fabs(cur_minus) < depth_cons_depth_max_thr3)
                        {
                            num++;
                            all_minus += fabs(point->vec(2) - p.vec(2));
                        }
                        // 깊이 차이가 양수이면 smaller_num을 증가시킵니다
                        else if (cur_minus > 0)
                        {
                            smaller_num++;
                        }
                        // 깊이 차이가 음수이면 greater_num을 증가시킵니다
                        else
                        {
                            greater_num++;
                        }
                    }
                }
            }
        }
    }

    // 검사한 점이 하나 이상 있으면 처리합니다
    if (all_num > 0)
    {
        // 일관성 점이 두 개 이상일 때 평균 깊이 차이를 확인합니다
        if (num > 1)
        {
            float cur_depth_thr = max(depth_cons_depth_thr3, k_depth3 * p.vec(2));
            if (all_minus / (num - 1) > cur_depth_thr)
            {
                return false;
            }
        }
        // 모든 깊이 차이가 같은 방향인 경우 true를 반환합니다
        if (greater_num == 0 || smaller_num == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        // 점이 없으면 false를 반환합니다
        return false;
    }
}


void DynObjFilter::publish_dyn(const ros::Publisher & pub_point_out, const ros::Publisher & pub_frame_out, const ros::Publisher & pub_steady_points, const double & scan_end_time)
{
    // 동적 객체가 발견되었을 때 로그를 출력합니다
    if (cluster_coupled) // pubLaserCloudEffect, pub_pcl_dyn_extend, pubLaserCloudEffect_depth에 대한 조건입니다
    {
        cout << "Found Dynamic Objects, numbers: " << laserCloudDynObj_clus->points.size() 
             << " Total time: " << time_total 
             << " Average total time: " << time_total_avr << endl;
    }
    else
    {
        cout << "Found Dynamic Objects, numbers: " << laserCloudDynObj->points.size() 
             << " Total time: " << time_total 
             << " Average total time: " << time_total_avr << endl;
    }

    // case1, case2, case3 개수를 출력합니다
    cout << "case1 num: " << case1_num << " case2 num: " << case2_num << " case3 num: " << case3_num << endl;
    case1_num = 0;
    case2_num = 0;
    case3_num = 0;

    // ROS 메시지로 변환하여 동적 객체를 퍼블리시합니다
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudDynObj_world, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes3.header.frame_id = frame_id;
    pub_point_out.publish(laserCloudFullRes3);

    // 클러스터링이 활성화된 경우 추가 퍼블리시를 수행합니다
    if (cluster_coupled || cluster_future)
    {
        sensor_msgs::PointCloud2 laserCloudFullRes4;
        pcl::toROSMsg(*laserCloudDynObj_clus, laserCloudFullRes4);
        laserCloudFullRes4.header.stamp = ros::Time().fromSec(scan_end_time);
        laserCloudFullRes4.header.frame_id = frame_id;
        pub_frame_out.publish(laserCloudFullRes4);
    }

    // 정적 객체에 대한 PointCloud를 생성합니다
    sensor_msgs::PointCloud2 laserCloudFullRes2;
    PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);

    // 클러스터링이 활성화된 경우 정적 객체를 누적하고 다운샘플링합니다
    if (cluster_coupled)
    {
        // 누적된 점이 제한보다 적으면 계속 누적합니다
        if (laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for (int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {   
            // 제한에 도달하면 오래된 점을 제거하고 새 점을 추가합니다
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj_clus);
            for (int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }

        // 다운샘플링 필터를 설정합니다
        pcl::VoxelGrid<PointType> downSizeFiltermap;
        downSizeFiltermap.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
        
        // 다운샘플링된 점을 필터링하고 ROS 메시지로 변환합니다
        PointCloudXYZI laserCloudSteadObj_down;
        downSizeFiltermap.filter(laserCloudSteadObj_down);
        pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
    }
    else
    {
        // 클러스터링이 비활성화된 경우 정적 객체 로그를 출력합니다
        cout << "Found Steady Objects, numbers: " << laserCloudSteadObj->points.size() << endl;
        
        // 정적 객체를 누적하고 퍼블리시할 데이터를 준비합니다
        if (laserCloudSteadObj_accu_times < laserCloudSteadObj_accu_limit)
        {
            laserCloudSteadObj_accu_times++;
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for (int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        else
        {   
            // 제한에 도달하면 오래된 점을 제거하고 새 점을 추가합니다
            laserCloudSteadObj_accu.pop_front();
            laserCloudSteadObj_accu.push_back(laserCloudSteadObj);
            for (int i = 0; i < laserCloudSteadObj_accu.size(); i++)
            {
                *laserCloudSteadObj_pub += *laserCloudSteadObj_accu[i];
            }
        }
        // ROS 메시지로 변환합니다
        pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
    }

    // 정적 객체 메시지를 퍼블리시합니다
    laserCloudFullRes2.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes2.header.frame_id = frame_id;
    pub_steady_points.publish(laserCloudFullRes2);
}

void DynObjFilter::set_path(string file_path, string file_path_origin)
{
    // 경로가 설정되었음을 표시하고 파일 경로를 저장합니다
    is_set_path = true;
    out_file = file_path;
    out_file_origin = file_path_origin;
}



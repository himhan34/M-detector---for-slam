#include <omp.h> // OpenMP 라이브러리를 포함하여 병렬 처리를 지원합니다
#include <mutex> // 뮤텍스 라이브러리를 포함하여 상호 배제를 구현합니다
#include <math.h> // 수학 함수를 사용하기 위한 헤더를 포함합니다
#include <thread> // 멀티스레딩을 위한 헤더를 포함합니다
#include <fstream> // 파일 입출력을 위한 헤더를 포함합니다
#include <iostream> // 표준 입출력을 위한 헤더를 포함합니다
#include <csignal> // 신호 처리를 위한 헤더를 포함합니다
#include <unistd.h> // 유닉스 시스템 호출을 위한 헤더를 포함합니다

#include <m-detector/DynObjFilter.h> // 동적 객체 필터 헤더를 포함합니다
#include <visualization_msgs/Marker.h> // 시각화 마커 메시지를 포함합니다
#include <visualization_msgs/MarkerArray.h> // 시각화 마커 배열 메시지를 포함합니다
#include <pcl_conversions/pcl_conversions.h> // PCL과 ROS 메시지 간 변환을 위한 헤더를 포함합니다
#include <pcl/point_cloud.h> // PCL 포인트 클라우드 클래스를 사용하기 위한 헤더를 포함합니다
#include <pcl/point_types.h> // PCL 포인트 타입을 정의하기 위한 헤더를 포함합니다
#include <pcl/io/pcd_io.h> // PCD 파일 입출력을 위한 헤더를 포함합니다
#include <sensor_msgs/PointCloud2.h> // 포인트 클라우드 메시지를 포함합니다
#include <tf/transform_datatypes.h> // TF 변환 데이터 타입을 포함합니다
#include <tf/transform_broadcaster.h> // TF 브로드캐스터를 포함합니다
#include <geometry_msgs/Vector3.h> // 3D 벡터 메시지를 포함합니다
#include <unistd.h> // 유닉스 시스템 호출을 위한 헤더를 포함합니다 (중복 포함됨)
#include <dirent.h> // 디렉토리 탐색을 위한 헤더를 포함합니다
#include <iomanip> // 입출력 설정을 위한 헤더를 포함합니다
#include <Eigen/Core> // Eigen 코어 라이브러리를 포함합니다
#include <Eigen/Geometry> // Eigen 기하학 라이브러리를 포함합니다
#include <Eigen/Dense> // Eigen 밀집 행렬 라이브러리를 포함합니다

using namespace std; // 표준 네임스페이스를 사용합니다

typedef pcl::PointXYZINormal PointType; // 포인트 타입을 pcl::PointXYZINormal로 정의합니다
typedef pcl::PointCloud<PointType> PointCloudXYZI; // 포인트 클라우드 타입을 정의합니다

pcl::PointCloud<pcl::PointXYZINormal> lastcloud; // 마지막 포인트 클라우드를 저장할 객체를 선언합니다
PointCloudXYZI::Ptr last_pc(new PointCloudXYZI()); // 마지막 포인트 클라우드를 포인터로 선언합니다
ros::Publisher pub_pointcloud, pub_marker, pub_iou_view; // ROS 퍼블리셔를 선언합니다

// 여러 파일 경로 및 이름을 저장할 문자열 변수를 선언합니다
string dataset_folder, pred_folder, pred_origin_folder, recall_folder, recall_origin_folder, label_folder, recall_file, recall_origin_file;
string semantic_folder, out_folder;

// 파일 출력을 위한 스트림 객체를 선언합니다
ofstream recall_rec, recall_origin_rec;

// 전체 통계 변수를 초기화합니다
int total_tp_origin = 0, total_fn_origin = 0, total_fp_origin = 0, total_op_origin = 0, total_tn_origin = 0.0;

// KITTI 데이터셋의 객체 타입을 매핑하는 해시 맵을 선언합니다
std::unordered_map<int, std::string> objects_types_map_kitti;

// NuScenes 데이터셋의 객체 타입 및 클래스 매핑을 위한 해시 맵을 선언합니다
std::unordered_map<int, std::string> objects_types_map_nuscenes;
std::unordered_map<int, int> objects_class_map_nuscenes;

// Waymo 데이터셋의 객체 타입을 매핑하는 해시 맵을 선언합니다
std::unordered_map<int, std::string> objects_types_map_waymo;

void Init() // 객체 타입과 클래스 매핑을 초기화하는 함수입니다
{
    // KITTI 데이터셋의 객체 타입을 매핑합니다
    objects_types_map_kitti[0] = "Person"; // 0번 타입: 사람
    objects_types_map_kitti[1] = "Truck"; // 1번 타입: 트럭
    objects_types_map_kitti[2] = "Car"; // 2번 타입: 자동차
    objects_types_map_kitti[3] = "Tram"; // 3번 타입: 트램
    objects_types_map_kitti[4] = "Pedestrain"; // 4번 타입: 보행자
    objects_types_map_kitti[5] = "Cyclist"; // 5번 타입: 자전거 운전자
    objects_types_map_kitti[6] = "Van"; // 6번 타입: 밴

    // NuScenes 데이터셋의 객체 타입을 매핑합니다
    objects_types_map_nuscenes[0] = "Animal"; // 0번 타입: 동물
    objects_types_map_nuscenes[1] = "Pedestrian"; // 1번 타입: 보행자
    objects_types_map_nuscenes[2] = "Movable_object"; // 2번 타입: 움직일 수 있는 객체
    objects_types_map_nuscenes[3] = "Bicycle"; // 3번 타입: 자전거
    objects_types_map_nuscenes[4] = "Bus"; // 4번 타입: 버스
    objects_types_map_nuscenes[5] = "Car"; // 5번 타입: 자동차
    objects_types_map_nuscenes[6] = "Emergency"; // 6번 타입: 긴급 차량
    objects_types_map_nuscenes[7] = "Motorcycle"; // 7번 타입: 오토바이
    objects_types_map_nuscenes[8] = "Trailer"; // 8번 타입: 트레일러
    objects_types_map_nuscenes[9] = "Truck"; // 9번 타입: 트럭
    objects_types_map_nuscenes[10] = "Ego"; // 10번 타입: 자차

    // NuScenes 데이터셋의 클래스 매핑을 설정합니다
    objects_class_map_nuscenes[1] = 0; // 1번 클래스: 보행자
    objects_class_map_nuscenes[2] = 1; // 2번 클래스: 차량
    objects_class_map_nuscenes[3] = 1; // 3번 클래스: 차량
    objects_class_map_nuscenes[4] = 1; // 4번 클래스: 차량
    objects_class_map_nuscenes[5] = 1; // 5번 클래스: 차량
    objects_class_map_nuscenes[6] = 1; // 6번 클래스: 차량
    objects_class_map_nuscenes[7] = 1; // 7번 클래스: 차량
    objects_class_map_nuscenes[8] = 1; // 8번 클래스: 차량
    objects_class_map_nuscenes[9] = 2; // 9번 클래스: 트럭
    objects_class_map_nuscenes[10] = 2; // 10번 클래스: 트럭
    objects_class_map_nuscenes[11] = 2; // 11번 클래스: 트럭
    objects_class_map_nuscenes[12] = 2; // 12번 클래스: 트럭
    objects_class_map_nuscenes[14] = 3; // 14번 클래스: 트레일러
    objects_class_map_nuscenes[15] = 4; // 15번 클래스: 버스
    objects_class_map_nuscenes[16] = 4; // 16번 클래스: 버스
    objects_class_map_nuscenes[17] = 5; // 17번 클래스: 오토바이
    objects_class_map_nuscenes[18] = 6; // 18번 클래스: 자전거
    objects_class_map_nuscenes[19] = 6; // 19번 클래스: 자전거
    objects_class_map_nuscenes[20] = 6; // 20번 클래스: 자전거
    objects_class_map_nuscenes[21] = 7; // 21번 클래스: 동물
    objects_class_map_nuscenes[22] = 8; // 22번 클래스: 움직일 수 있는 객체
    objects_class_map_nuscenes[23] = 9; // 23번 클래스: 기타
    objects_class_map_nuscenes[31] = 10; // 31번 클래스: 자차

    // Waymo 데이터셋의 객체 타입을 매핑합니다
    objects_types_map_waymo[0] = "Vehicle"; // 0번 타입: 차량
    objects_types_map_waymo[1] = "Pedestrian"; // 1번 타입: 보행자
    objects_types_map_waymo[2] = "Cyclist"; // 2번 타입: 자전거 운전자
}


void NuscenesCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    int label_file_num = 0; // 레이블 파일 개수를 저장하는 변수
    
    if(label_folder != "") // 레이블 폴더가 비어있지 않은지 확인
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str()); // 레이블 폴더 열기
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL) // 폴더 내 파일 읽기
        {
            if(label_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            label_file_num++; // 파일 개수 증가
        }
        closedir(label_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl; // 레이블 폴더가 없을 경우 경고 메시지 출력
        return;
    }

    int pred_file_num = 0; // 예측 파일 개수를 저장하는 변수
    
    if(pred_folder != "") // 예측 폴더가 비어있지 않은지 확인
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str()); // 예측 폴더 열기
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL) // 폴더 내 파일 읽기
        {
            if(pred_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            pred_file_num++; // 파일 개수 증가
        }
        closedir(pred_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl; // 예측 폴더가 없을 경우 경고 메시지 출력
        return;
    }

    if(label_file_num != pred_file_num) // 레이블 파일과 예측 파일의 개수가 일치하지 않으면
    {
        cout<<"file num error "<<label_folder<<endl; // 파일 개수 오류 메시지 출력
    }

    // 클래스별 정보를 저장할 벡터
    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    
    // 총 TP, FN, FP, TN, OP 카운터 초기화
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    
    // NuScenes의 객체 수와 평균 리콜 정보를 저장할 벡터 초기화
    std::vector<int> objects_numbers_nuscenes(11, 0), objects_numbers_nuscenes_origin(11, 0);
    std::vector<float> average_recalls_nuscenes(11, 0.0), average_recalls_nuscenes_origin(11, 0.0);
    std::vector<float> average_recalls2_nuscenes(11, 0.0);

    for(int frames = 0; frames < label_file_num; frames++) // 모든 프레임에 대해 반복
    {   
        cout << "frame: " << frames << endl; // 현재 프레임 번호 출력
        
        stringstream ss;
        ss << setw(6) << setfill('0') << frames ; // 프레임 번호를 형식에 맞게 변환
        
        string label_file = label_folder; 
        label_file += ss.str(); 
        label_file.append(".label"); // 레이블 파일 이름 생성
        
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label"); // 예측 파일 이름 생성

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary); // 레이블 파일 열기
        if(!label_input.good()) // 파일을 제대로 읽지 못했을 경우
        {
            std::cerr << "Could not read label file: " << label_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }
        label_input.seekg(0, std::ios::beg); // 파일 포인터를 시작 위치로 이동

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary); // 예측 파일 열기
        if(!pred_input.good()) // 파일을 제대로 읽지 못했을 경우
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }

        std::fstream test(label_file.c_str(), std::ios::in | std::ios::binary); // 레이블 파일 테스트로 열기
        if(!test.good()) // 파일을 제대로 읽지 못했을 경우
        {
            std::cerr << "Could not read label file: " << label_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }
        test.seekg(0, std::ios::beg); // 파일 포인터를 시작 위치로 이동
        
        int class_id;
        test.read((char *) &class_id, sizeof(int)); // 클래스 ID 읽기
        test.read((char *) &class_id, sizeof(int)); // 클래스 ID 읽기

        // TP, FN, FP, OP, TN 초기화 및 변수 선언
        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;
        float cur_self_vel;

        // 레이블과 예측 데이터를 저장할 해시맵 초기화
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        if(test.eof()) // 파일이 비어있는 경우
        {
            cout<<" empty"<<endl;
        }
        else
        {
            while(pred_input.good() && !pred_input.eof()) // 예측 파일이 유효한 동안 반복
            {
                int pred_num = -1;
                pred_input.read((char *) &pred_num, sizeof(int)); // 예측 데이터 읽기
                pred_num = pred_num & 0xFFFF; // 16비트 마스크 적용

                int label_num = -1, id = -1;
                label_input.read((char *) &label_num, sizeof(int)); // 레이블 데이터 읽기
                label_num = label_num & 0xFFFF; // 16비트 마스크 적용

                if(label_num >= 1000 && label_num < 65535) // 특정 클래스 범위 내의 레이블
                {
                    if(!labels_map.count(label_num)) labels_map[label_num] = 1; // 레이블 맵에 추가
                    else labels_map[label_num] += 1;
                    
                    if(pred_num >= 251 && pred_num < 65535) // 특정 클래스 범위 내의 예측
                    {
                        if(!predicts_map.count(label_num)) predicts_map[label_num] = 1; // 예측 맵에 추가
                        else predicts_map[label_num] += 1;
                        tp += 1; // True Positive 증가
                    }
                    else
                    {
                        fn += 1; // False Negative 증가
                    }
                    count += 1; // 총 객체 수 증가
                }
                else if(label_num < 251) // 다른 클래스 범위
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        fp += 1; // False Positive 증가
                    }
                    else
                    {
                        tn += 1; // True Negative 증가
                    }
                }
                else if (label_num >= 65535) // 클래스 ID가 범위를 벗어난 경우
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        op += 1; // Other Positive 증가
                    }
                }
                else if (label_num >= 251 && label_num < 1000) // 다른 클래스 범위
                {
                    tn += 1; // True Negative 증가
                }
            }
        }

        if(tp + fn + fp > 10e-5) iou = ((float)tp) / (float)(tp + fn + fp); // IoU 계산
        total_tp += tp; // 총 TP 누적
        total_fn += fn; // 총 FN 누적
        total_fp += fp; // 총 FP 누적
        total_tn += tn; // 총 TN 누적
        total_op += op; // 총 OP 누적
        label_input.close(); // 레이블 파일 닫기
        pred_input.close(); // 예측 파일 닫기

        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<<" count: "<<count<<" iou: "<<iou<<endl; // 결과 출력

        for(auto it = labels_map.begin(); it != labels_map.end(); it++) // 레이블 맵 반복
        {   
            int class_num = it->first / 1000; // 클래스 번호 계산
            if(objects_class_map_nuscenes.count(class_num) == 0) // 맵에서 클래스 번호 확인
            {
                cout<<"we do not find "<< class_num << "  " << it->first <<endl; // 클래스 번호를 찾지 못한 경우 출력
                continue;
            } 
            cout<<"remap class "<< class_num;  
            class_num = objects_class_map_nuscenes[class_num]; // 클래스 번호 재매핑
            cout<<" to "<< class_num << endl;

            objects_numbers_nuscenes[class_num]++; // 객체 수 증가
            
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; // 리콜 출력

            // 평균 리콜 계산
            average_recalls_nuscenes[class_num] = average_recalls_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1) / objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first] / (float)it->second / objects_numbers_nuscenes[class_num];
            average_recalls2_nuscenes[class_num] = average_recalls2_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1) / objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first] / (float)it->second * (float)predicts_map[it->first] / (float)it->second / objects_numbers_nuscenes[class_num];
            
            // 클래스 번호에 따라 리콜을 다른 벡터에 추가
            if(class_num == 4 || class_num == 5 || class_num == 7 || class_num == 9) recalls_vehicle.push_back((float)predicts_map[it->first] / (float)it->second);
            if(class_num == 1) recalls_pedestrian.push_back((float)predicts_map[it->first] / (float)it->second);
            if(class_num == 3) recalls_cyclist.push_back((float)predicts_map[it->first] / (float)it->second);
        }

        for(int i = 0; i < objects_numbers_nuscenes.size(); i++) // 모든 객체에 대해 반복
        {
            cout << "average_recall of " << objects_types_map_nuscenes[i] << " is: " << average_recalls_nuscenes[i] << " objects number: " << objects_numbers_nuscenes[i] << " variance: " << average_recalls2_nuscenes[i] - average_recalls_nuscenes[i] * average_recalls_nuscenes[i] << endl; // 평균 리콜과 분산 출력
        }
        
        cout << "average_suppress: " << (float)(total_fn + total_tn) / (float)(total_tp + total_fp + total_tn + total_fn) << endl; // 평균 억제율 출력
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp + total_fp + total_fn + total_tn <<endl; // 총 카운터 출력
        cout << "Average iou: " << ((float)total_tp) / (float)(total_tp + total_fn + total_fp) << endl; // 평균 IoU 출력

        if((frames + 1) == label_file_num) // 모든 프레임이 처리된 후
        {
            class_nums.clear(); // 클래스 번호 초기화
            class_recalls.clear(); // 클래스 리콜 초기화
            class_var.clear(); // 클래스 분산 초기화

            for(int i = 0; i < objects_numbers_nuscenes.size(); i++) // 모든 객체에 대해 반복
            {
                class_nums.push_back(objects_numbers_nuscenes[i]); // 클래스 번호 추가
                class_recalls.push_back(average_recalls_nuscenes[i]); // 클래스 리콜 추가
                class_var.push_back(average_recalls2_nuscenes[i] - average_recalls_nuscenes[i] * average_recalls_nuscenes[i]); // 클래스 분산 추가
            }
        }
    }

    // 리콜 결과 파일에 출력
    recall_rec << total_tp << " " << total_fp << " " << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
    // 예측 파일 포인터를 시작 위치로 이동
    // pred_input.seekg(0, std::ios::beg);
}


void NuscenesCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    int label_file_num = 0; // 레이블 파일 개수를 저장하는 변수
    
    if(label_folder != "") // 레이블 폴더가 비어있지 않은지 확인
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str()); // 레이블 폴더 열기
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL) // 폴더 내 파일 읽기
        {
            if(label_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            label_file_num++; // 파일 개수 증가
        }
        closedir(label_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl; // 레이블 폴더가 없을 경우 경고 메시지 출력
        return;
    }

    int pred_file_num = 0; // 예측 파일 개수를 저장하는 변수
    
    if(pred_folder != "") // 예측 폴더가 비어있지 않은지 확인
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str()); // 예측 폴더 열기
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL) // 폴더 내 파일 읽기
        {
            if(pred_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            pred_file_num++; // 파일 개수 증가
        }
        closedir(pred_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl; // 예측 폴더가 없을 경우 경고 메시지 출력
        return;
    }

    if(label_file_num != pred_file_num) // 레이블 파일과 예측 파일의 개수가 일치하지 않으면
    {
        cout<<"file num error "<<label_folder<<endl; // 파일 개수 오류 메시지 출력
    }

    // 클래스별 정보를 저장할 벡터
    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    
    // 총 TP, FN, FP, TN, OP 카운터 초기화
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    
    // NuScenes의 객체 수와 평균 리콜 정보를 저장할 벡터 초기화
    std::vector<int> objects_numbers_nuscenes(11, 0), objects_numbers_nuscenes_origin(11, 0);
    std::vector<float> average_recalls_nuscenes(11, 0.0), average_recalls_nuscenes_origin(11, 0.0);
    std::vector<float> average_recalls2_nuscenes(11, 0.0);

    for(int frames = 0; frames < label_file_num; frames++) // 모든 프레임에 대해 반복
    {   
        cout << "frame: " << frames << endl; // 현재 프레임 번호 출력
        
        stringstream ss;
        ss << setw(6) << setfill('0') << frames ; // 프레임 번호를 형식에 맞게 변환
        
        string label_file = label_folder; 
        label_file += ss.str(); 
        label_file.append(".label"); // 레이블 파일 이름 생성
        
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label"); // 예측 파일 이름 생성

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary); // 레이블 파일 열기
        if(!label_input.good()) // 파일을 제대로 읽지 못했을 경우
        {
            std::cerr << "Could not read label file: " << label_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }
        label_input.seekg(0, std::ios::beg); // 파일 포인터를 시작 위치로 이동

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary); // 예측 파일 열기
        if(!pred_input.good()) // 파일을 제대로 읽지 못했을 경우
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }

        std::fstream test(label_file.c_str(), std::ios::in | std::ios::binary); // 레이블 파일 테스트로 열기
        if(!test.good()) // 파일을 제대로 읽지 못했을 경우
        {
            std::cerr << "Could not read label file: " << label_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }
        test.seekg(0, std::ios::beg); // 파일 포인터를 시작 위치로 이동
        
        int class_id;
        test.read((char *) &class_id, sizeof(int)); // 클래스 ID 읽기
        test.read((char *) &class_id, sizeof(int)); // 클래스 ID 읽기

        // TP, FN, FP, OP, TN 초기화 및 변수 선언
        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;
        float cur_self_vel;

        // 레이블과 예측 데이터를 저장할 해시맵 초기화
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        if(test.eof()) // 파일이 비어있는 경우
        {
            cout<<" empty"<<endl;
        }
        else
        {
            while(pred_input.good() && !pred_input.eof()) // 예측 파일이 유효한 동안 반복
            {
                int pred_num = -1;
                pred_input.read((char *) &pred_num, sizeof(int)); // 예측 데이터 읽기
                pred_num = pred_num & 0xFFFF; // 16비트 마스크 적용

                int label_num = -1, id = -1;
                label_input.read((char *) &label_num, sizeof(int)); // 레이블 데이터 읽기
                label_num = label_num & 0xFFFF; // 16비트 마스크 적용

                if(label_num >= 1000 && label_num < 65535) // 특정 클래스 범위 내의 레이블
                {
                    if(!labels_map.count(label_num)) labels_map[label_num] = 1; // 레이블 맵에 추가
                    else labels_map[label_num] += 1;
                    
                    if(pred_num >= 251 && pred_num < 65535) // 특정 클래스 범위 내의 예측
                    {
                        if(!predicts_map.count(label_num)) predicts_map[label_num] = 1; // 예측 맵에 추가
                        else predicts_map[label_num] += 1;
                        tp += 1; // True Positive 증가
                    }
                    else
                    {
                        fn += 1; // False Negative 증가
                    }
                    count += 1; // 총 객체 수 증가
                }
                else if(label_num < 251) // 다른 클래스 범위
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        fp += 1; // False Positive 증가
                    }
                    else
                    {
                        tn += 1; // True Negative 증가
                    }
                }
                else if (label_num >= 65535) // 클래스 ID가 범위를 벗어난 경우
                {
                    if(pred_num >= 251 && pred_num < 65535)
                    {
                        op += 1; // Other Positive 증가
                    }
                }
                else if (label_num >= 251 && label_num < 1000) // 다른 클래스 범위
                {
                    tn += 1; // True Negative 증가
                }
            }
        }

        if(tp + fn + fp > 10e-5) iou = ((float)tp) / (float)(tp + fn + fp); // IoU 계산
        total_tp += tp; // 총 TP 누적
        total_fn += fn; // 총 FN 누적
        total_fp += fp; // 총 FP 누적
        total_tn += tn; // 총 TN 누적
        total_op += op; // 총 OP 누적
        label_input.close(); // 레이블 파일 닫기
        pred_input.close(); // 예측 파일 닫기

        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<<" count: "<<count<<" iou: "<<iou<<endl; // 결과 출력

        for(auto it = labels_map.begin(); it != labels_map.end(); it++) // 레이블 맵 반복
        {   
            int class_num = it->first / 1000; // 클래스 번호 계산
            if(objects_class_map_nuscenes.count(class_num) == 0) // 맵에서 클래스 번호 확인
            {
                cout<<"we do not find "<< class_num << "  " << it->first <<endl; // 클래스 번호를 찾지 못한 경우 출력
                continue;
            } 
            cout<<"remap class "<< class_num;  
            class_num = objects_class_map_nuscenes[class_num]; // 클래스 번호 재매핑
            cout<<" to "<< class_num << endl;

            objects_numbers_nuscenes[class_num]++; // 객체 수 증가
            
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; // 리콜 출력

            // 평균 리콜 계산
            average_recalls_nuscenes[class_num] = average_recalls_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1) / objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first] / (float)it->second / objects_numbers_nuscenes[class_num];
            average_recalls2_nuscenes[class_num] = average_recalls2_nuscenes[class_num] * (objects_numbers_nuscenes[class_num] - 1) / objects_numbers_nuscenes[class_num] + (float)predicts_map[it->first] / (float)it->second * (float)predicts_map[it->first] / (float)it->second / objects_numbers_nuscenes[class_num];
            
            // 클래스 번호에 따라 리콜을 다른 벡터에 추가
            if(class_num == 4 || class_num == 5 || class_num == 7 || class_num == 9) recalls_vehicle.push_back((float)predicts_map[it->first] / (float)it->second);
            if(class_num == 1) recalls_pedestrian.push_back((float)predicts_map[it->first] / (float)it->second);
            if(class_num == 3) recalls_cyclist.push_back((float)predicts_map[it->first] / (float)it->second);
        }

        for(int i = 0; i < objects_numbers_nuscenes.size(); i++) // 모든 객체에 대해 반복
        {
            cout << "average_recall of " << objects_types_map_nuscenes[i] << " is: " << average_recalls_nuscenes[i] << " objects number: " << objects_numbers_nuscenes[i] << " variance: " << average_recalls2_nuscenes[i] - average_recalls_nuscenes[i] * average_recalls_nuscenes[i] << endl; // 평균 리콜과 분산 출력
        }
        
        cout << "average_suppress: " << (float)(total_fn + total_tn) / (float)(total_tp + total_fp + total_tn + total_fn) << endl; // 평균 억제율 출력
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp + total_fp + total_fn + total_tn <<endl; // 총 카운터 출력
        cout << "Average iou: " << ((float)total_tp) / (float)(total_tp + total_fn + total_fp) << endl; // 평균 IoU 출력

        if((frames + 1) == label_file_num) // 모든 프레임이 처리된 후
        {
            class_nums.clear(); // 클래스 번호 초기화
            class_recalls.clear(); // 클래스 리콜 초기화
            class_var.clear(); // 클래스 분산 초기화

            for(int i = 0; i < objects_numbers_nuscenes.size(); i++) // 모든 객체에 대해 반복
            {
                class_nums.push_back(objects_numbers_nuscenes[i]); // 클래스 번호 추가
                class_recalls.push_back(average_recalls_nuscenes[i]); // 클래스 리콜 추가
                class_var.push_back(average_recalls2_nuscenes[i] - average_recalls_nuscenes[i] * average_recalls_nuscenes[i]); // 클래스 분산 추가
            }
        }
    }

    // 리콜 결과 파일에 출력
    recall_rec << total_tp << " " << total_fp << " " << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
    // 예측 파일 포인터를 시작 위치로 이동
    // pred_input.seekg(0, std::ios::beg);
}

void WaymoCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    int label_file_num = 0; // 레이블 파일 수를 저장하는 변수
    
    if(label_folder != "") // 레이블 폴더가 비어있지 않은 경우
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str()); // 레이블 폴더 열기
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL) // 폴더의 파일들을 읽기
        {
            if(label_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            label_file_num++; // 파일 수 증가
        }
        closedir(label_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl; // 레이블 폴더가 없으면 경고 출력
        return;
    }

    int pred_file_num = 0; // 예측 파일 수를 저장하는 변수
    
    if(pred_folder != "") // 예측 폴더가 비어있지 않은 경우
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str()); // 예측 폴더 열기
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL) // 폴더의 파일들을 읽기
        {
            if(pred_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            pred_file_num++; // 파일 수 증가
        }
        closedir(pred_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl; // 예측 폴더가 없으면 경고 출력
        return;
    }

    if(label_file_num != pred_file_num) // 레이블 파일 수와 예측 파일 수가 다르면
    {
        cout<<"file num error "<<label_folder<<endl; // 파일 수 오류 출력
    }

    // 클래스별 정보를 저장할 벡터
    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    
    // 총 TP, FN, FP, OP, TN 카운터 초기화
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    
    // Waymo 객체 수와 평균 리콜 정보를 저장할 벡터 초기화
    std::vector<int> objects_numbers_waymo(3, 0);
    std::vector<float> average_recalls_waymo(3, 0.0);
    std::vector<float> average_recalls2_waymo(3, 0.0);

    for(int frames = 0; frames < label_file_num; frames++) // 모든 프레임에 대해 반복
    {   
        cout << "frame: " << frames << endl; // 현재 프레임 번호 출력
        stringstream ss;
        ss << setw(6) << setfill('0') << frames; // 프레임 번호를 형식에 맞게 변환
        
        string label_file = label_folder;
        label_file += ss.str(); 
        label_file.append(".label"); // 레이블 파일 이름 생성
        
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label"); // 예측 파일 이름 생성

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary); // 레이블 파일 열기
        if(!label_input.good()) // 파일을 제대로 읽지 못한 경우
        {
            std::cerr << "Could not read label file: " << label_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }
        label_input.seekg(0, std::ios::beg); // 파일 포인터를 시작 위치로 이동

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary); // 예측 파일 열기
        if(!pred_input.good()) // 파일을 제대로 읽지 못한 경우
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }

        // TP, FN, FP, OP, TN 초기화 및 변수 선언
        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;

        // 레이블과 예측 데이터를 저장할 해시맵 초기화
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        while(pred_input.good() && !pred_input.eof()) // 예측 파일이 유효한 동안 반복
        {
            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int)); // 예측 데이터 읽기
            pred_num = pred_num & 0xFFFF; // 16비트 마스크 적용

            int label_num = -1, id = -1;
            label_input.read((char *) &label_num, sizeof(int)); // 레이블 데이터 읽기
            label_num = label_num & 0xFFFF; // 16비트 마스크 적용

            if(label_num >= 251 && label_num < 65535) // 특정 클래스 범위 내의 레이블
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1; // 레이블 맵에 추가
                else labels_map[label_num] += 1;

                if(pred_num >= 251 && pred_num < 65535) // 특정 클래스 범위 내의 예측
                {
                    tp += 1; // True Positive 증가
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1; // 예측 맵에 추가
                    else predicts_map[label_num] += 1;
                }
                else
                {
                    fn += 1; // False Negative 증가
                }
                count += 1; // 총 객체 수 증가
            }
            else if (label_num < 251 && label_num < 65535) // 다른 클래스 범위
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1; // False Positive 증가
                }
                else
                {
                    tn += 1; // True Negative 증가
                }
            }
            else if (label_num >= 65535) // 클래스 ID가 범위를 벗어난 경우
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    op += 1; // Other Positive 증가
                }
            }
        }
        
        iou = ((float)tp) / (float)(tp + fn + fp); // IoU 계산
        total_tp += tp; // 총 TP 누적
        total_fn += fn; // 총 FN 누적
        total_fp += fp; // 총 FP 누적
        total_tn += tn; // 총 TN 누적
        total_op += op; // 총 OP 누적
        label_input.close(); // 레이블 파일 닫기
        pred_input.close(); // 예측 파일 닫기

        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<< " op: " <<op << " tn: " << tn << " count: "<<count<<" iou: "<<iou<<endl; // 결과 출력

        for(auto it = labels_map.begin(); it != labels_map.end(); it++) // 레이블 맵 반복
        {   
            int class_num = it->first / 1000; // 클래스 번호 계산
            objects_numbers_waymo[class_num]++; // 객체 수 증가
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; // 리콜 출력

            // 평균 리콜 계산
            average_recalls_waymo[class_num] = average_recalls_waymo[class_num] * (objects_numbers_waymo[class_num] - 1) / objects_numbers_waymo[class_num] + (float)predicts_map[it->first] / (float)it->second / objects_numbers_waymo[class_num];
            average_recalls2_waymo[class_num] = average_recalls2_waymo[class_num] * (objects_numbers_waymo[class_num] - 1) / objects_numbers_waymo[class_num] + (float)predicts_map[it->first] / (float)it->second * (float)predicts_map[it->first] / (float)it->second / objects_numbers_waymo[class_num];
        }

        for(int i = 0; i < objects_numbers_waymo.size(); i++) // 모든 객체에 대해 반복
        {
            cout << "average_recall of " << objects_types_map_waymo[i] << " is: " << average_recalls_waymo[i] << " objects number: " << objects_numbers_waymo[i] << " variance: " << average_recalls2_waymo[i] - average_recalls_waymo[i] * average_recalls_waymo[i] << endl; // 평균 리콜과 분산 출력
        }

        cout << "average_suppress: " << (float)(total_fn + total_tn) / (float)(total_tp + total_fp + total_tn + total_fn) << endl; // 평균 억제율 출력
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp + total_fp + total_fn + total_tn <<endl; // 총 카운터 출력
        cout << "Average iou: " << ((float)total_tp) / (float)(total_tp + total_fn + total_fp) << endl; // 평균 IoU 출력

        if((frames + 1) == label_file_num) // 모든 프레임이 처리된 후
        {
            class_nums.clear(); // 클래스 번호 초기화
            class_recalls.clear(); // 클래스 리콜 초기화
            class_var.clear(); // 클래스 분산 초기화

            for(int i = 0; i < objects_numbers_waymo.size(); i++) // 모든 객체에 대해 반복
            {
                class_nums.push_back(objects_numbers_waymo[i]); // 클래스 번호 추가
                class_recalls.push_back(average_recalls_waymo[i]); // 클래스 리콜 추가
                class_var.push_back(average_recalls2_waymo[i] - average_recalls_waymo[i] * average_recalls_waymo[i]); // 클래스 분산 추가
            }
        }
    }
    
    // 리콜 결과 파일에 출력
    recall_rec << total_tp << " " << total_fp << " " << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
    // 예측 파일 포인터를 시작 위치로 이동
    // pred_input.seekg(0, std::ios::beg);
}

void KittiCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian, std::vector<float> &recalls_cyclist)
{
    int label_file_num = 0; // 레이블 파일 수를 저장하는 변수

    if(label_folder != "") // 레이블 폴더가 비어있지 않은 경우
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str()); // 레이블 폴더 열기
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL) // 폴더의 파일들을 읽기
        {
            if(label_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            label_file_num++; // 파일 수 증가
        }
        closedir(label_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl; // 레이블 폴더가 없으면 경고 출력
        return;
    }

    int pred_file_num = 0; // 예측 파일 수를 저장하는 변수

    if(pred_folder != "") // 예측 폴더가 비어있지 않은 경우
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str()); // 예측 폴더 열기
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL) // 폴더의 파일들을 읽기
        {
            if(pred_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            pred_file_num++; // 파일 수 증가
        }
        closedir(pred_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl; // 예측 폴더가 없으면 경고 출력
        return;
    }

    if(label_file_num != pred_file_num) // 레이블 파일 수와 예측 파일 수가 다르면
    {
        cout<<"file num error "<<label_folder<<endl; // 파일 수 오류 출력
    }

    // 클래스별 정보를 저장할 벡터
    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    
    // 총 TP, FN, FP, OP, TN 카운터 초기화
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    
    // Kitti 객체 수와 평균 리콜 정보를 저장할 벡터 초기화
    std::vector<int> objects_numbers_kitti(7, 0);
    std::vector<float> average_recalls_kitti(7, 0.0);
    std::vector<float> average_recalls2_kitti(7, 0.0);

    for(int frames = 0; frames < label_file_num; frames++) // 모든 프레임에 대해 반복
    {   
        cout << "frame: " << frames << endl; // 현재 프레임 번호 출력
        stringstream ss;
        ss << setw(6) << setfill('0') << frames; // 프레임 번호를 형식에 맞게 변환
        
        string label_file = label_folder;
        label_file += ss.str(); 
        label_file.append(".label"); // 레이블 파일 이름 생성
        
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label"); // 예측 파일 이름 생성

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary); // 레이블 파일 열기
        if(!label_input.good()) // 파일을 제대로 읽지 못한 경우
        {
            std::cerr << "Could not read label file: " << label_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }
        label_input.seekg(0, std::ios::beg); // 파일 포인터를 시작 위치로 이동

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary); // 예측 파일 열기
        if(!pred_input.good()) // 파일을 제대로 읽지 못한 경우
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }

        // TP, FN, FP, OP, TN 초기화 및 변수 선언
        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;

        // 레이블과 예측 데이터를 저장할 해시맵 초기화
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        while(pred_input.good() && !pred_input.eof()) // 예측 파일이 유효한 동안 반복
        {
            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int)); // 예측 데이터 읽기
            pred_num = pred_num & 0xFFFF; // 16비트 마스크 적용

            int label_num = -1, id = -1;
            label_input.read((char *) &label_num, sizeof(int)); // 레이블 데이터 읽기
            label_num = label_num & 0xFFFF; // 16비트 마스크 적용

            if(label_num >= 251 && label_num < 65535) // 특정 클래스 범위 내의 레이블
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1; // 레이블 맵에 추가
                else labels_map[label_num] += 1;

                if(pred_num >= 251 && pred_num < 65535) // 특정 클래스 범위 내의 예측
                {
                    tp += 1; // True Positive 증가
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1; // 예측 맵에 추가
                    else predicts_map[label_num] += 1;
                }
                else if (pred_num == 9) // 예측 번호가 9인 경우
                {
                    fn += 1; // False Negative 증가
                }
                count += 1; // 총 객체 수 증가
            }
            else if (label_num < 251 && label_num < 65535) // 다른 클래스 범위
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1; // False Positive 증가
                }
                else if (pred_num == 9) // 예측 번호가 9인 경우
                {
                    tn += 1; // True Negative 증가
                }
            }
            else if (label_num >= 65535) // 클래스 ID가 범위를 벗어난 경우
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    op += 1; // Other Positive 증가
                }
            }
        }
        
        iou = ((float)tp) / (float)(tp + fn + fp); // IoU 계산
        total_tp += tp; // 총 TP 누적
        total_fn += fn; // 총 FN 누적
        total_fp += fp; // 총 FP 누적
        total_tn += tn; // 총 TN 누적
        total_op += op; // 총 OP 누적
        label_input.close(); // 레이블 파일 닫기
        pred_input.close(); // 예측 파일 닫기

        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<< " op: " <<op << " tn: " << tn << " count: "<<count<<" iou: "<<iou<<endl; // 결과 출력

        for(auto it = labels_map.begin(); it != labels_map.end(); it++) // 레이블 맵 반복
        {   
            int class_num = it->first / 1000; // 클래스 번호 계산
            objects_numbers_kitti[class_num]++; // 객체 수 증가
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; // 리콜 출력

            // 평균 리콜 계산
            average_recalls_kitti[class_num] = average_recalls_kitti[class_num] * (objects_numbers_kitti[class_num] - 1) / objects_numbers_kitti[class_num] + (float)predicts_map[it->first] / (float)it->second / objects_numbers_kitti[class_num];
            average_recalls2_kitti[class_num] = average_recalls2_kitti[class_num] * (objects_numbers_kitti[class_num] - 1) / objects_numbers_kitti[class_num] + (float)predicts_map[it->first] / (float)it->second * (float)predicts_map[it->first] / (float)it->second / objects_numbers_kitti[class_num];

            // 클래스 번호에 따라 리콜을 다른 벡터에 추가
            if(class_num == 1 || class_num == 2 || class_num == 3 || class_num == 6) recalls_vehicle.push_back((float)predicts_map[it->first] / (float)it->second);
            if(class_num == 4) recalls_pedestrian.push_back((float)predicts_map[it->first] / (float)it->second);
            if(class_num == 6) recalls_cyclist.push_back((float)predicts_map[it->first] / (float)it->second);
        }

        for(int i = 0; i < objects_numbers_kitti.size(); i++) // 모든 객체에 대해 반복
        {
            cout << "average_recall of " << objects_types_map_kitti[i] << " is: " << average_recalls_kitti[i] << " objects number: " << objects_numbers_kitti[i] << " variance: " << average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i] << endl; // 평균 리콜과 분산 출력
        }
        
        cout << "average_suppress: " << (float)(total_fn + total_tn) / (float)(total_tp + total_fp + total_tn + total_fn) << endl; // 평균 억제율 출력
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp + total_fp + total_fn + total_tn <<endl; // 총 카운터 출력
        cout << "Average iou: " << ((float)total_tp) / (float)(total_tp + total_fn + total_fp) << " with frames: " << frames << " , " << label_file_num << endl; // 평균 IoU 출력

        if((frames + 1) == label_file_num) // 모든 프레임이 처리된 후
        {
            class_nums.clear(); // 클래스 번호 초기화
            class_recalls.clear(); // 클래스 리콜 초기화
            class_var.clear(); // 클래스 분산 초기화

            for(int i = 0; i < objects_numbers_kitti.size(); i++) // 모든 객체에 대해 반복
            {
                class_nums.push_back(objects_numbers_kitti[i]); // 클래스 번호 추가
                class_recalls.push_back(average_recalls_kitti[i]); // 클래스 리콜 추가
                class_var.push_back(average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i]); // 클래스 분산 추가
            }
        }
    }
    
    // 리콜 결과 파일에 출력
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " " << ((float)total_tp) / (float)(total_tp + total_fn + total_fp) << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
}


void AviaCalRecall(std::vector<float> &recalls_vehicle, std::vector<float> &recalls_pedestrian)
{
    int label_file_num = 0; // 레이블 파일 수를 저장하는 변수

    if(label_folder != "") // 레이블 폴더가 비어있지 않은 경우
    {
        DIR* label_dir;	
        label_dir = opendir(label_folder.c_str()); // 레이블 폴더 열기
        struct dirent* label_ptr;
        while((label_ptr = readdir(label_dir)) != NULL) // 폴더의 파일들을 읽기
        {
            if(label_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            label_file_num++; // 파일 수 증가
        }
        closedir(label_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less label "<<label_folder<<endl; // 레이블 폴더가 없으면 경고 출력
        return;
    }

    int pred_file_num = 0; // 예측 파일 수를 저장하는 변수

    if(pred_folder != "") // 예측 폴더가 비어있지 않은 경우
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str()); // 예측 폴더 열기
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL) // 폴더의 파일들을 읽기
        {
            if(pred_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            pred_file_num++; // 파일 수 증가
        }
        closedir(pred_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl; // 예측 폴더가 없으면 경고 출력
        return;
    }

    if(label_file_num != pred_file_num) // 레이블 파일 수와 예측 파일 수가 다르면
    {
        cout<<"file num error "<<label_folder<<endl; // 파일 수 오류 출력
    }

    // 클래스별 정보를 저장할 벡터
    vector<int> class_nums;
    vector<float> class_recalls;
    vector<float> class_var;
    
    // 총 TP, FN, FP, OP, TN 카운터 초기화
    int total_tp = 0, total_fn = 0, total_fp = 0, total_op = 0, total_tn = 0.0;
    
    // Kitti 객체 수와 평균 리콜 정보를 저장할 벡터 초기화
    std::vector<int> objects_numbers_kitti(7, 0);
    std::vector<float> average_recalls_kitti(7, 0.0);
    std::vector<float> average_recalls2_kitti(7, 0.0);

    for(int frames = 0; frames < label_file_num; frames++) // 모든 프레임에 대해 반복
    {   
        cout << "frame: " << frames << endl; // 현재 프레임 번호 출력
        stringstream ss;
        ss << setw(6) << setfill('0') << frames; // 프레임 번호를 형식에 맞게 변환
        
        string label_file = label_folder;
        label_file += ss.str(); 
        label_file.append(".label"); // 레이블 파일 이름 생성
        
        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label"); // 예측 파일 이름 생성

        std::fstream label_input(label_file.c_str(), std::ios::in | std::ios::binary); // 레이블 파일 열기
        if(!label_input.good()) // 파일을 제대로 읽지 못한 경우
        {
            std::cerr << "Could not read label file: " << label_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }
        label_input.seekg(0, std::ios::beg); // 파일 포인터를 시작 위치로 이동

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary); // 예측 파일 열기
        if(!pred_input.good()) // 파일을 제대로 읽지 못한 경우
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }

        // TP, FN, FP, OP, TN 초기화 및 변수 선언
        int tp = 0, fn = 0, fp = 0, op = 0, tn = 0, count = 0;
        float iou = 0.0f;

        // 레이블과 예측 데이터를 저장할 해시맵 초기화
        std::unordered_map<int, int> labels_map;
        std::unordered_map<int, int> predicts_map;

        while(pred_input.good() && !pred_input.eof()) // 예측 파일이 유효한 동안 반복
        {
            int pred_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int)); // 예측 데이터 읽기
            pred_num = pred_num & 0xFFFF; // 16비트 마스크 적용

            int label_num = -1, id = -1;
            label_input.read((char *) &label_num, sizeof(int)); // 레이블 데이터 읽기
            label_num = label_num & 0xFFFF; // 16비트 마스크 적용

            if(label_num >= 251 && label_num < 65535) // 특정 클래스 범위 내의 레이블
            {   
                if(!labels_map.count(label_num)) labels_map[label_num] = 1; // 레이블 맵에 추가
                else labels_map[label_num] += 1;

                if(pred_num >= 251 && pred_num < 65535) // 특정 클래스 범위 내의 예측
                {
                    tp += 1; // True Positive 증가
                    if(!predicts_map.count(label_num)) predicts_map[label_num] = 1; // 예측 맵에 추가
                    else predicts_map[label_num] += 1;
                }
                else if (pred_num == 9) // 예측 번호가 9인 경우
                {
                    fn += 1; // False Negative 증가
                }
                count += 1; // 총 객체 수 증가
            }
            else if (label_num < 251 && label_num < 65535) // 다른 클래스 범위
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    fp += 1; // False Positive 증가
                }
                else if (pred_num == 9) // 예측 번호가 9인 경우
                {
                    tn += 1; // True Negative 증가
                }
            }
            else if (label_num >= 65535) // 클래스 ID가 범위를 벗어난 경우
            {
                if(pred_num >= 251 && pred_num < 65535)
                {
                    op += 1; // Other Positive 증가
                }
            }
        }
        
        iou = ((float)tp) / (float)(tp + fn + fp); // IoU 계산
        total_tp += tp; // 총 TP 누적
        total_fn += fn; // 총 FN 누적
        total_fp += fp; // 총 FP 누적
        total_tn += tn; // 총 TN 누적
        total_op += op; // 총 OP 누적
        label_input.close(); // 레이블 파일 닫기
        pred_input.close(); // 예측 파일 닫기

        cout<<"tp: "<<tp<<"  fp: "<<fp<<" fn: "<<fn<< " op: " <<op << " tn: " << tn << " count: "<<count<<" iou: "<<iou<<endl; // 결과 출력

        for(auto it = labels_map.begin(); it != labels_map.end(); it++) // 레이블 맵 반복
        {   
            int class_num = it->first / 1000; // 클래스 번호 계산
            objects_numbers_kitti[class_num]++; // 객체 수 증가
            std::cout << "id: " << it->first << " labels: " << it->second << " predict: " << predicts_map[it->first] << " recall: " << (float)predicts_map[it->first]/(float)it->second << std::endl; // 리콜 출력

            // 평균 리콜 계산
            average_recalls_kitti[class_num] = average_recalls_kitti[class_num] * (objects_numbers_kitti[class_num] - 1) / objects_numbers_kitti[class_num] + (float)predicts_map[it->first] / (float)it->second / objects_numbers_kitti[class_num];
            average_recalls2_kitti[class_num] = average_recalls2_kitti[class_num] * (objects_numbers_kitti[class_num] - 1) / objects_numbers_kitti[class_num] + (float)predicts_map[it->first] / (float)it->second * (float)predicts_map[it->first] / (float)it->second / objects_numbers_kitti[class_num];
        }

        for(int i = 0; i < objects_numbers_kitti.size(); i++) // 모든 객체에 대해 반복
        {
            cout << "average_recall of " << objects_types_map_kitti[i] << " is: " << average_recalls_kitti[i] << " objects number: " << objects_numbers_kitti[i] << " variance: " << average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i] << endl; // 평균 리콜과 분산 출력
        }
        
        cout << "average_suppress: " << (float)(total_fn + total_tn) / (float)(total_tp + total_fp + total_tn + total_fn) << endl; // 평균 억제율 출력
        cout<<"total_tp: "<<total_tp<<"  total_fp: "<<total_fp<<" total_fn: " <<total_fn << " total_op: " <<total_op << " total_tn: " << total_tn  << " total_points: " << total_tp + total_fp + total_fn + total_tn <<endl; // 총 카운터 출력
        cout << "Average iou: " << ((float)total_tp) / (float)(total_tp + total_fn + total_fp) << endl; // 평균 IoU 출력

        if((frames + 1) == label_file_num) // 모든 프레임이 처리된 후
        {
            class_nums.clear(); // 클래스 번호 초기화
            class_recalls.clear(); // 클래스 리콜 초기화
            class_var.clear(); // 클래스 분산 초기화

            for(int i = 0; i < objects_numbers_kitti.size(); i++) // 모든 객체에 대해 반복
            {
                class_nums.push_back(objects_numbers_kitti[i]); // 클래스 번호 추가
                class_recalls.push_back(average_recalls_kitti[i]); // 클래스 리콜 추가
                class_var.push_back(average_recalls2_kitti[i] - average_recalls_kitti[i] * average_recalls_kitti[i]); // 클래스 분산 추가
            }
        }
    }
    
    // 리콜 결과 파일에 출력
    recall_rec << total_tp << " " << total_fp << " "  << total_fn << " " << total_tn << " ";
    for(int i = 0; i < class_recalls.size(); i++)
    {
        recall_rec << class_recalls[i] << " " << class_nums[i] << " " << class_var[i] << " ";
    }
    recall_rec << endl;
}

void SemanticCombine()
{
    int semantic_file_num = 0; // 시맨틱 파일 수를 저장하는 변수

    if(semantic_folder != "") // 시맨틱 폴더가 비어있지 않은 경우
    {
        DIR* semantic_dir;	
        semantic_dir = opendir(semantic_folder.c_str()); // 시맨틱 폴더 열기
        struct dirent* semantic_ptr;
        while((semantic_ptr = readdir(semantic_dir)) != NULL) // 폴더의 파일들을 읽기
        {
            if(semantic_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            semantic_file_num++; // 파일 수 증가
        }
        closedir(semantic_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less semantic "<<semantic_folder<<endl; // 시맨틱 폴더가 없으면 경고 출력
        return;
    }

    int pred_file_num = 0; // 예측 파일 수를 저장하는 변수

    if(pred_folder != "") // 예측 폴더가 비어있지 않은 경우
    {
        DIR* pred_dir;	
        pred_dir = opendir(pred_folder.c_str()); // 예측 폴더 열기
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL) // 폴더의 파일들을 읽기
        {
            if(pred_ptr->d_name[0] == '.') {continue;} // 숨김 파일 무시
            pred_file_num++; // 파일 수 증가
        }
        closedir(pred_dir); // 폴더 닫기
    } 
    else
    {
        cout<<"less pred "<<pred_folder<<endl; // 예측 폴더가 없으면 경고 출력
        return;
    }

    if(semantic_file_num != pred_file_num) // 시맨틱 파일 수와 예측 파일 수가 다르면
    {
        cout<<"file num error "<<label_folder<<endl; // 파일 수 오류 출력
    }

    for(int frames = 0; frames < semantic_file_num; frames++) // 모든 프레임에 대해 반복
    {
        stringstream ss;
        ss << setw(6) << setfill('0') << frames; // 프레임 번호를 형식에 맞게 변환

        string pred_file = pred_folder;
        pred_file += ss.str(); 
        pred_file.append(".label"); // 예측 파일 이름 생성

        string semantic_file = semantic_folder;
        semantic_file += ss.str(); 
        semantic_file.append(".label"); // 시맨틱 파일 이름 생성
        
        string out_file = out_folder;
        out_file += ss.str(); 
        out_file.append(".label"); // 출력 파일 이름 생성

        ofstream out;
        out.open(out_file, ios::out  | ios::binary); // 출력 파일 열기 (바이너리 모드)

        std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary); // 예측 파일 열기
        if(!pred_input.good()) // 파일을 제대로 읽지 못한 경우
        {
            std::cerr << "Could not read prediction file: " << pred_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }

        std::fstream semantic_input(semantic_file.c_str(), std::ios::in | std::ios::binary); // 시맨틱 파일 열기
        if(!semantic_input.good()) // 파일을 제대로 읽지 못한 경우
        {
            std::cerr << "Could not read semantic file: " << semantic_file << std::endl; // 오류 메시지 출력
            exit(EXIT_FAILURE); // 프로그램 종료
        }

        while(pred_input.good() && !pred_input.eof()) // 예측 파일이 유효한 동안 반복
        {
            int pred_num = -1, semantic_num = -1;
            pred_input.read((char *) &pred_num, sizeof(int)); // 예측 데이터 읽기
            pred_num = pred_num & 0xFFFF; // 16비트 마스크 적용

            semantic_input.read((char *) &semantic_num, sizeof(int)); // 시맨틱 데이터 읽기
            semantic_num = semantic_num & 0xFFFF; // 16비트 마스크 적용

            if(pred_num == 251) // 예측 번호가 251인 경우
            {
                if (semantic_num < 40) // 시맨틱 번호가 40보다 작은 경우
                {
                    int tmp = 251;
                    out.write((char*)&tmp, sizeof(int)); // 251을 출력 파일에 쓰기
                }
                else // 시맨틱 번호가 40 이상인 경우
                {
                    int tmp = 9;
                    out.write((char*)&tmp, sizeof(int)); // 9를 출력 파일에 쓰기
                }
            }
            else // 예측 번호가 251이 아닌 경우
            {
                int tmp = 9;
                out.write((char*)&tmp, sizeof(int)); // 9를 출력 파일에 쓰기
            }
        }
        out.close(); // 출력 파일 닫기
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_dynamic"); // ROS 노드 초기화
    ros::NodeHandle nh; // ROS 노드 핸들 생성
    Init(); // 초기화 함수 호출

    // 변수 선언 및 초기화
    int dataset = -1, start_param = 0, end_param = 0, start_se = 0, end_se = 0;
    bool is_origin = false;

    // ROS 매개변수 읽기
    nh.param<int>("dyn_obj/dataset", dataset, -1); // 데이터셋 매개변수
    nh.param<bool>("dyn_obj/is_origin", is_origin, false); // 원본 여부 매개변수
    nh.param<string>("dyn_obj/dataset_folder", dataset_folder, "/"); // 데이터셋 폴더 매개변수
    nh.param<int>("dyn_obj/start_param", start_param, -1); // 시작 매개변수
    nh.param<int>("dyn_obj/end_param", end_param, 0); // 끝 매개변수
    nh.param<int>("dyn_obj/start_se", start_se, -1); // 시작 SE 매개변수
    nh.param<int>("dyn_obj/end_se", end_se, 0); // 끝 SE 매개변수

    if(dataset == 0) // 데이터셋이 0인 경우
    {
        recall_folder = dataset_folder + "recall/"; // 리콜 폴더 경로 설정
        recall_origin_folder = dataset_folder + "recall_origin/"; // 원본 리콜 폴더 경로 설정
        string command;
        
        // 리콜 폴더와 원본 리콜 폴더 생성
        command = "mkdir -p " + recall_folder;
        system(command.c_str());
        command = "mkdir -p " + recall_origin_folder;
        system(command.c_str());

        for(int i = start_param; i < end_param; i++) // 시작 매개변수부터 끝 매개변수까지 반복
        {
            if(is_origin) // 원본 여부에 따라 리콜 파일 설정
            {
                recall_file = recall_origin_folder + "kitti" + to_string(i) + "_origin" + ".txt";
            }
            else
            {   
                recall_file = recall_folder + "kitti" + to_string(i) + ".txt";
            }
            
            recall_rec.open(recall_file, std::ios::out); // 리콜 결과 파일 열기
            std::vector<float> recalls_vehicle; // 차량 리콜 벡터
            std::vector<float> recalls_pedestrian; // 보행자 리콜 벡터
            std::vector<float> recalls_cyclist; // 자전거 리콜 벡터

            for(int j = start_se; j < end_se; j++) // 시작 SE부터 끝 SE까지 반복
            {   
                stringstream ss;
                ss << setw(4) << setfill('0') << j; // SE 번호를 형식에 맞게 변환
                
                if(is_origin) // 원본 여부에 따라 예측 폴더 경로 설정
                {
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "_origin/";
                }
                else
                {   
                    pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "/";
                }
                
                label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/"; // 레이블 폴더 경로 설정
                cout << "pred: " << pred_folder << endl; // 예측 폴더 출력
                cout << label_folder << endl; // 레이블 폴더 출력
                
                KittiCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist); // 리콜 계산 함수 호출
            }

            // 차량 리콜 값의 1/8, 1/2, 7/8 위치를 계산
            int q1_vehicle = round(recalls_vehicle.size() / 8);
            int q2_vehicle = round(recalls_vehicle.size() / 2);
            int q3_vehicle = round(recalls_vehicle.size() * 7 / 8);

            // 보행자 리콜 값의 1/8, 1/2, 7/8 위치를 계산
            int q1_pedestrian = round(recalls_pedestrian.size() / 8);
            int q2_pedestrian = round(recalls_pedestrian.size() / 2);
            int q3_pedestrian = round(recalls_pedestrian.size() * 7 / 8);

            // 자전거 리콜 값의 1/8, 1/2, 7/8 위치를 계산
            int q1_cyclist = round(recalls_cyclist.size() / 8);
            int q2_cyclist = round(recalls_cyclist.size() / 2);
            int q3_cyclist = round(recalls_cyclist.size() * 7 / 8);

            // 리콜 벡터 정렬
            std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
            std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
            std::sort(recalls_cyclist.begin(), recalls_cyclist.end());

            // 차량 리콜 값 출력
            if(recalls_vehicle.size() > 0)
            {
                recall_rec << recalls_vehicle.at(q1_vehicle) << " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
            }
            else
            {
                recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
            }

            // 보행자 리콜 값 출력
            if(recalls_pedestrian.size() > 0)
            {
                recall_rec << recalls_pedestrian.at(q1_pedestrian) << " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
            }
            else
            {
                recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
            }

            // 자전거 리콜 값 출력
            if(recalls_cyclist.size() > 0)
            {
                recall_rec << recalls_cyclist.at(q1_cyclist) << " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl;  
            }
            else
            {
                recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
            }

            recall_rec.close(); // 리콜 결과 파일 닫기
        }
    }

   if(dataset == 1) // 데이터셋이 1인 경우
{
    recall_folder = dataset_folder + "recall/"; // 리콜 폴더 경로 설정
    recall_origin_folder = dataset_folder + "recall_origin/"; // 원본 리콜 폴더 경로 설정
    string command;

    // 리콜 폴더와 원본 리콜 폴더 생성
    command = "mkdir -p " + recall_folder;
    system(command.c_str());
    command = "mkdir -p " + recall_origin_folder;
    system(command.c_str());

    for(int i = start_param; i < end_param; i++) // 시작 매개변수부터 끝 매개변수까지 반복
    {
        if(is_origin) // 원본 여부에 따라 리콜 파일 설정
        {
            recall_file = recall_origin_folder + "nuscenes" + to_string(i) + "_origin" + ".txt";
        }
        else
        {
            recall_file = recall_folder + "nuscenes" + to_string(i) + ".txt";
        }

        recall_rec.open(recall_file, std::ios::out); // 리콜 결과 파일 열기
        std::vector<float> recalls_vehicle; // 차량 리콜 벡터
        std::vector<float> recalls_pedestrian; // 보행자 리콜 벡터
        std::vector<float> recalls_cyclist; // 자전거 리콜 벡터

        for(int j = start_se; j < end_se; j++) // 시작 SE부터 끝 SE까지 반복
        {
            stringstream ss;
            ss << setw(4) << setfill('0') << j; // SE 번호를 형식에 맞게 변환
            
            if(is_origin) // 원본 여부에 따라 예측 폴더 경로 설정
            {
                pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "_origin/";
            }
            else
            {
                pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "/";
            }
            
            label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/"; // 레이블 폴더 경로 설정
            cout << "pred: " << pred_folder << endl; // 예측 폴더 출력
            cout << label_folder << endl; // 레이블 폴더 출력

            NuscenesCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist); // 리콜 계산 함수 호출
        }

        // 차량 리콜 값의 1/4, 1/2, 3/4 위치를 계산
        int q1_vehicle = round(recalls_vehicle.size() / 4);
        int q2_vehicle = round(recalls_vehicle.size() / 2);
        int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

        // 보행자 리콜 값의 1/4, 1/2, 3/4 위치를 계산
        int q1_pedestrian = round(recalls_pedestrian.size() / 4);
        int q2_pedestrian = round(recalls_pedestrian.size() / 2);
        int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

        // 자전거 리콜 값의 1/4, 1/2, 3/4 위치를 계산
        int q1_cyclist = round(recalls_cyclist.size() / 4);
        int q2_cyclist = round(recalls_cyclist.size() / 2);
        int q3_cyclist = round(recalls_cyclist.size() * 3 / 4);

        // 리콜 벡터 정렬
        std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
        std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
        std::sort(recalls_cyclist.begin(), recalls_cyclist.end());

        // 차량 리콜 값 출력
        if(recalls_vehicle.size() > 0)
        {
            recall_rec << recalls_vehicle.at(q1_vehicle) << " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
        }
        else
        {
            recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
        }

        // 보행자 리콜 값 출력
        if(recalls_pedestrian.size() > 0)
        {
            recall_rec << recalls_pedestrian.at(q1_pedestrian) << " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
        }
        else
        {
            recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
        }

        // 자전거 리콜 값 출력
        if(recalls_cyclist.size() > 0)
        {
            recall_rec << recalls_cyclist.at(q1_cyclist) << " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl;  
        }
        else
        {
            recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
        }

        recall_rec.close(); // 리콜 결과 파일 닫기
    }
}


   if(dataset == 2) // 데이터셋이 2인 경우
{
    recall_folder = dataset_folder + "recall/"; // 리콜 폴더 경로 설정
    recall_origin_folder = dataset_folder + "recall_origin/"; // 원본 리콜 폴더 경로 설정
    string command;

    // 리콜 폴더와 원본 리콜 폴더 생성
    command = "mkdir -p " + recall_folder;
    system(command.c_str());
    command = "mkdir -p " + recall_origin_folder;
    system(command.c_str());

    for(int i = start_param; i < end_param; i++) // 시작 매개변수부터 끝 매개변수까지 반복
    {
        if(is_origin) // 원본 여부에 따라 리콜 파일 설정
        {
            recall_file = recall_origin_folder + "waymo" + to_string(i) + "_origin" + ".txt";
        }
        else
        {
            recall_file = recall_folder + "waymo" + to_string(i) + ".txt";
        }

        recall_rec.open(recall_file, std::ios::out); // 리콜 결과 파일 열기
        std::vector<float> recalls_vehicle; // 차량 리콜 벡터
        std::vector<float> recalls_pedestrian; // 보행자 리콜 벡터
        std::vector<float> recalls_cyclist; // 자전거 리콜 벡터

        for(int j = start_se; j < end_se; j++) // 시작 SE부터 끝 SE까지 반복
        {
            stringstream ss;
            ss << setw(4) << setfill('0') << j; // SE 번호를 형식에 맞게 변환
            
            if(is_origin) // 원본 여부에 따라 예측 폴더 경로 설정
            {
                pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "_origin/";
            }
            else
            {
                pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "/";
            }
            
            label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/"; // 레이블 폴더 경로 설정
            cout << "pred: " << pred_folder << endl; // 예측 폴더 출력
            cout << label_folder << endl; // 레이블 폴더 출력

            WaymoCalRecall(recalls_vehicle, recalls_pedestrian, recalls_cyclist); // 리콜 계산 함수 호출
        }

        // 차량 리콜 값의 1/4, 1/2, 3/4 위치를 계산
        int q1_vehicle = round(recalls_vehicle.size() / 4);
        int q2_vehicle = round(recalls_vehicle.size() / 2);
        int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

        // 보행자 리콜 값의 1/4, 1/2, 3/4 위치를 계산
        int q1_pedestrian = round(recalls_pedestrian.size() / 4);
        int q2_pedestrian = round(recalls_pedestrian.size() / 2);
        int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

        // 자전거 리콜 값의 1/4, 1/2, 3/4 위치를 계산
        int q1_cyclist = round(recalls_cyclist.size() / 4);
        int q2_cyclist = round(recalls_cyclist.size() / 2);
        int q3_cyclist = round(recalls_cyclist.size() * 3 / 4);

        // 리콜 벡터 정렬
        std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
        std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());
        std::sort(recalls_cyclist.begin(), recalls_cyclist.end());

        // 차량 리콜 값 출력
        if(recalls_vehicle.size() > 0)
        {
            recall_rec << recalls_vehicle.at(q1_vehicle) << " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
        }
        else
        {
            recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
        }

        // 보행자 리콜 값 출력
        if(recalls_pedestrian.size() > 0)
        {
            recall_rec << recalls_pedestrian.at(q1_pedestrian) << " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
        }
        else
        {
            recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
        }

        // 자전거 리콜 값 출력
        if(recalls_cyclist.size() > 0)
        {
            recall_rec << recalls_cyclist.at(q1_cyclist) << " " << recalls_cyclist.at(q2_cyclist) << " " << recalls_cyclist.at(q3_cyclist) << endl;  
        }
        else
        {
            recall_rec << -1 << " " << -1 << " " << -1 << endl; // 리콜 값이 없을 경우 -1 출력
        }

        recall_rec.close(); // 리콜 결과 파일 닫기
    }
}
    // dataset이 3일 때 실행
if(dataset == 3)
{
    // recall과 recall_origin 폴더 경로 설정
    recall_folder = dataset_folder + "recall/";
    recall_origin_folder = dataset_folder + "recall_origin/";
    string command;

    // recall 폴더 생성
    command = "mkdir -p " + recall_folder;
    system(command.c_str());
    // recall_origin 폴더 생성
    command = "mkdir -p " + recall_origin_folder;
    system(command.c_str());

    // 주어진 범위의 파라미터에 대해 반복
    for(int i = start_param; i < end_param; i++ )
    {
        // origin 여부에 따라 recall 파일 경로 설정
        if(is_origin)
        {
            recall_file = recall_origin_folder + "avia" + to_string(i) + "_origin" + ".txt";
        }
        else
        {
            recall_file = recall_folder + "avia" + to_string(i) + ".txt";
        }
        recall_rec.open(recall_file, std::ios::out); // recall 파일 열기
        std::vector<float> recalls_vehicle; // 차량 recall 벡터
        std::vector<float> recalls_pedestrian; // 보행자 recall 벡터

        // 주어진 범위의 시퀀스에 대해 반복
        for(int j = start_se; j < end_se; j++)
        {   
            stringstream ss;
            ss << setw(4) << setfill('0') << j ;
            // origin 여부에 따라 예측 폴더 경로 설정
            if(is_origin)
            {
                pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "_origin/";
            }
            else
            {
                pred_folder = dataset_folder + "sequences/" + ss.str() + "/predictions" + to_string(i) + "/";
            }
            label_folder = dataset_folder + "sequences/" + ss.str() + "/labels/";

            cout<<"pred: "<<pred_folder<<endl;
            cout<<label_folder<<endl;

            // AviaCalRecall 함수 호출
            AviaCalRecall(recalls_vehicle, recalls_pedestrian);
        }

        // recall 벡터의 1사분위수, 중앙값, 3사분위수 계산
        int q1_vehicle = round(recalls_vehicle.size() / 4);
        int q2_vehicle = round(recalls_vehicle.size() / 2);
        int q3_vehicle = round(recalls_vehicle.size() * 3 / 4);

        int q1_pedestrian = round(recalls_pedestrian.size() / 4);
        int q2_pedestrian = round(recalls_pedestrian.size() / 2);
        int q3_pedestrian = round(recalls_pedestrian.size() * 3 / 4);

        // 차량과 보행자 recall 벡터를 정렬
        std::sort(recalls_vehicle.begin(), recalls_vehicle.end());
        std::sort(recalls_pedestrian.begin(), recalls_pedestrian.end());

        // 차량 recall 벡터에 대해 값을 파일에 기록
        if(recalls_vehicle.size() > 0)
        {
            recall_rec << recalls_vehicle.at(q1_vehicle) <<  " " << recalls_vehicle.at(q2_vehicle) << " " << recalls_vehicle.at(q3_vehicle) << endl;
        }
        else
        {
            recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
        }

        // 보행자 recall 벡터에 대해 값을 파일에 기록
        if(recalls_pedestrian.size() > 0)
        {
            recall_rec << recalls_pedestrian.at(q1_pedestrian) <<  " " << recalls_pedestrian.at(q2_pedestrian) << " " << recalls_pedestrian.at(q3_pedestrian) << endl;
        }
        else
        {
            recall_rec << -1 <<  " " << -1 << " " << -1 << endl;
        }

        recall_rec.close(); // recall 파일 닫기
    }
}

// dataset이 -1일 때 실행
if(dataset == -1)
{
    string all_pred = dataset_folder + "residual_1/";
    string all_semantic = dataset_folder + "semantic/";
    string all_out = dataset_folder + "residual_1_semantic/";
    vector<string> seq_names;

    // dataset_folder가 비어있지 않으면 시퀀스 이름을 seq_names에 추가
    if(dataset_folder != "")
    {
        DIR* pred_dir;    
        pred_dir = opendir(all_pred.c_str());
        struct dirent* pred_ptr;
        while((pred_ptr = readdir(pred_dir)) != NULL)
        {
            if(pred_ptr->d_name[0] == '.') {continue;} // 숨김 파일은 건너뛰기
            string cur_folder(pred_ptr->d_name);
            seq_names.push_back(cur_folder); // 시퀀스 이름 추가
        }
        closedir(pred_dir); // 디렉토리 닫기
    }

    // 각 시퀀스에 대해 반복
    for(int i = 0; i < seq_names.size(); i++ )
    {       
        // 예측, 의미론적, 출력 폴더 경로 설정
        pred_folder = all_pred + seq_names[i] + "/predictions/";
        semantic_folder = all_semantic + seq_names[i] + "/predictions/";
        out_folder = all_out + seq_names[i] + "/predictions/";

        cout<<pred_folder<<endl;
        cout<<semantic_folder<<endl;
        cout<<out_folder<< " " <<i <<endl;

        // SemanticCombine 함수 호출
        SemanticCombine();
    }
}
    
    // ROS 이벤트 처리
    ros::spin();
    return 0;
}

    
   

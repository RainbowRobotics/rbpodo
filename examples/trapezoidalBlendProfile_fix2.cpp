// 전체 코드: Waypoint 기반 블렌딩 이동 제어 (사다리꼴 속도 프로파일 포함)

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <cmath>
#include "rbpodo/rbpodo.hpp"

using namespace std;
using namespace Eigen;
using namespace rb;

typedef Matrix<double, 6, 6> Matrix6d;

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr int LOOP_HZ = 25;
constexpr double LOOP_DT = 1.0 / LOOP_HZ;

// --- 로봇 모델 클래스는 기존 Robot9Link 코드 그대로 붙여넣으세요 ---
// 여기에 forwardKinematics(), computeJacobian6x6(), 등 포함된 Robot9Link 클래스 정의 붙여넣기
struct LinkDH {
    double a;       // 링크 길이 (mm)
    double alpha;   // twist 각도 (rad)
    double d;       // 링크 오프셋 (mm)
    double theta;   // 관절각 (rad) + 오프셋 포함 (변수 or 고정)
    bool isRevolute; // true면 관절 변수(회전), false면 고정
};

// degrees <-> radians 변환
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad / M_PI * 180.0; }

// 9개 링크를 포함하는 6축 로봇 예제
class Robot9Link {
public:
    // D-H 파라미터 (초기값: 오프셋 포함)
    // alpha, theta는 rad 단위로 관리
    vector<LinkDH> links;

    Robot9Link() {
        links.resize(9);

        // Link 1
        links[0].a = 0.0;
        links[0].alpha = deg2rad(-90.0);
        links[0].d = 169.2;    // mm
        links[0].theta = 0.0;      // 관절 변수(θ1), 초기 0
        links[0].isRevolute = true;

        // Link 2 (θ2 - 90° 오프셋 -> theta에 -90° 더해놓고 시작)
        links[1].a = 0.0;
        links[1].alpha = deg2rad(0.0);
        links[1].d = -148.4;    // mm
        links[1].theta = deg2rad(-90.0); // 초기 오프셋
        links[1].isRevolute = true;

        // Link 3 (고정 링크, a1=425, d3=148.4)
        links[2].a = 425.0;
        links[2].alpha = deg2rad(0.0);
        links[2].d = 0.0;    // mm
        links[2].theta = 0.0;      // 고정
        links[2].isRevolute = false;

        // Link 4 (θ3)
        links[3].a = 0.0;
        links[3].alpha = deg2rad(0.0);
        links[3].d = 148.4;    // mm
        links[3].theta = 0.0;      // 관절 변수
        links[3].isRevolute = true;

        // Link 5 (θ4, a2=392, d5=110.7)
        links[4].a = 392.0;
        links[4].alpha = deg2rad(0.0);
        links[4].d = 0.0;    // mm
        links[4].theta = 0.0;
        links[4].isRevolute = false;

        // Link 6 (θ5 + 90° 오프셋)
        links[5].a = 0.0;
        links[5].alpha = deg2rad(0.0);
        links[5].d = -110.7;     // mm
        links[5].theta = deg2rad(90.0);  // 초기 오프셋
        links[5].isRevolute = true;

        // Link 7 (고정 링크, 예: 손목 오프셋)
        links[6].a = 0.0;
        links[6].alpha = deg2rad(90.0);
        links[6].d = 0.0;      // mm
        links[6].theta = 0.0;
        links[6].isRevolute = false;

        // Link 8 (θ6)
        links[7].a = 0.0;
        links[7].alpha = deg2rad(-90.0);
        links[7].d = 110.7;      // mm
        links[7].theta = 0.0;
        links[7].isRevolute = true;

        // Link 9 (툴링크)
        links[8].a = 0.0;
        links[8].alpha = deg2rad(90.0);
        links[8].d = -96.7;
        links[8].theta = 0.0;
        links[8].isRevolute = true;
    }

    vector<double> joint_offset = { 0, -M_PI / 2, 0, M_PI / 2, 0, 0 };

    // 단일 D-H 변환행렬 (theta, alpha는 rad, d, a는 mm 단위)
    Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha) {
        double ct = cos(theta);
        double st = sin(theta);
        double ca = cos(alpha);
        double sa = sin(alpha);

        Eigen::Matrix4d T;
        T << ct, -st * ca, st* sa, a* ct,
            st, ct* ca, -ct * sa, a* st,
            0, sa, ca, d,
            0, 0, 0, 1;
        return T;
    }

    // 현재 links[] 상태(θ 포함)로부터 end-effector(마지막 Link 9)까지의 forward kinematics
    // 반환: 4x4 변환행렬 (mm, rad)
    Eigen::Matrix4d forwardKinematics(const vector<double>& jointVals) {
        // jointVals에는 실제 '회전'이 일어나는 6개의 관절각(θ1~θ6)이 들어있다고 가정
        // links[] 중 isRevolute=true 인 곳만 업데이트
        int jointIndex = 0;
        for (size_t i = 0; i < links.size(); i++) {
            if (links[i].isRevolute) {
                links[i].theta = jointVals[jointIndex] + joint_offset[jointIndex];
                jointIndex++;
            }
        }

        // base -> link9까지 변환행렬 계산
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 9; i++) {
            double th = links[i].theta;
            double dd = links[i].d;
            double aa = links[i].a;
            double al = links[i].alpha;
            T = T * dhTransform(th, dd, aa, al);
        }

        return T;
    }

    // 6x6 자코비언 (위치+자세)
    // 상단 3행: z_i x (p_end - p_i), 하단 3행: z_i
    Matrix<double, 6, 6> computeJacobian6x6(const vector<double>& jointVals) {
        // 우선 forwardKinematics 계산과 유사하게, 중간 프레임들을 모두 구해야 함
        // (주의) 자코비언을 구할 때도 links[].theta에 jointVals를 반영해야 하지만,
        //        여기서는 편의상 별도 계산 루틴을 만들겠습니다.

        // link0(base) ~ link9 까지 10개 프레임
        int jointIndex = 0;
        // links[].theta에 jointVals 반영
        for (size_t i = 0; i < links.size(); i++) {
            if (links[i].isRevolute) {
                links[i].theta = jointVals[jointIndex] + joint_offset[jointIndex];
                jointIndex++;
            }
        }

        // 2) 각 프레임별 누적 변환행렬 T_list
        vector<Matrix4d> T_list(10, Matrix4d::Identity());
        for (int i = 0; i < 9; i++) {
            T_list[i + 1] = T_list[i] * dhTransform(links[i].theta, links[i].d, links[i].a, links[i].alpha);
        }
        Vector3d p_end = T_list[9].block<3, 1>(0, 3);

        // 3) 자코비언 계산
        Matrix<double, 6, 6> J;
        J.setZero();

        jointIndex = 0;
        for (int i = 0; i < 9; i++) {
            if (!links[i].isRevolute) continue; // 고정 링크는 스킵

            // z_i, p_i
            Vector3d z_i = T_list[i].block<3, 1>(0, 2);
            Vector3d p_i = T_list[i].block<3, 1>(0, 3);

            // 위치 미분: z_i x (p_end - p_i)
            Vector3d Jv = z_i.cross(p_end - p_i);
            // 자세 미분: z_i
            Vector3d Jw = z_i;

            // 상단 3행은 위치, 하단 3행은 회전
            J.block<3, 1>(0, jointIndex) = Jv;
            J.block<3, 1>(3, jointIndex) = Jw;

            jointIndex++;
        }

        return J;
    }

    // 4x4 변환행렬로부터 (roll, pitch, yaw) [rad] 추출 (Z-Y-X 순 등 여러 정의가 있으니 주의)
    // 여기서는 roll=pitch=yaw 순서를 X->Y->Z 로 가정한 예시
    // 실제 로봇 소프트웨어와 일치하는 방식으로 구현해야 함
    Vector3d matrixToRPY(const Matrix4d& T) {
        // 회전행렬
        Matrix3d R = T.block<3, 3>(0, 0);

        // roll(y), pitch(x), yaw(z) 등 여러 convention이 있으므로 주의
        // 여기서는 R = Rz(yaw)*Ry(pitch)*Rx(roll) 형태라 가정한 예시
        // 일반적 ZYX 순서: yaw->pitch->roll
        // roll = atan2(R32, R33)
        // pitch = -asin(R31)
        // yaw = atan2(R21, R11)
        // (행렬에서 항목 추출)

        double roll, pitch, yaw;
        // Z-Y-X convention (yaw-pitch-roll) 예시:
        pitch = -asin(R(2, 0));
        double cPitch = cos(pitch);

        if (fabs(cPitch) > 1e-6) {
            roll = atan2(R(2, 1), R(2, 2));
            yaw = atan2(R(1, 0), R(0, 0));
        }
        else {
            // pitch ~ ±90도 부근 특이
            roll = 0.0;
            yaw = atan2(-R(0, 1), R(1, 1));
        }
        return Vector3d(roll, pitch, yaw);
    }

    // 수치적 IK (위치+자세)
    // target_position: (x, y, z) [mm]
    // target_orientation: (roll, pitch, yaw) [rad], ZYX 순서 가정
    // initialQ: 초기 관절값 (6개)
    // maxIter, eps: 반복 제한 / 오차 한계
    vector<double> inverseKinematics6D(const Vector3d& target_position,
        const Vector3d& target_orientation,
        const vector<double>& initialQ,
        int maxIter = 100,
        double eps = 1e-3)
    {
        vector<double> q = initialQ; // 현재 추정값 (6개)

        for (int iter = 0; iter < maxIter; iter++) {
            // 1) 현재 FK
            Matrix4d T_cur = forwardKinematics(q);
            Vector3d p_cur = T_cur.block<3, 1>(0, 3);
            Vector3d rpy_cur = matrixToRPY(T_cur);

            // 2) 위치 오차
            Vector3d pos_err = target_position - p_cur;
            // 3) 자세 오차 (단순 차)
            //   주의: rpy_des - rpy_cur 는 ±π 근처에서 불연속 가능
            Vector3d ori_err = target_orientation - rpy_cur;

            // 4) 종합 오차 (6x1)
            Vector<double, 6> e;
            e << pos_err(0), pos_err(1), pos_err(2),
                ori_err(0), ori_err(1), ori_err(2);

            if (e.norm() < eps) {
                cout << "[IK] Converged at iteration " << iter
                    << ", error norm = " << e.norm() << endl;
                return q;
            }

            // 5) 자코비언(6x6)
            Matrix<double, 6, 6> J = computeJacobian6x6(q);

            // 6) 의사역행렬
            //    (J * J^T)가 역가역이 안 되는 특이점에서 문제 발생 가능
            Matrix<double, 6, 6> JJt = J * J.transpose();
            if (fabs(JJt.determinant()) < 1e-12) {
                cout << "[IK] Near-singular or singular Jacobian. Stop.\n";
                return q;
            }
            Matrix<double, 6, 6> Jinv = J.transpose() * JJt.inverse();

            // 7) 관절 업데이트 Δq = J^+ * e
            Vector<double, 6> dq = Jinv * e;
            for (int i = 0; i < 6; i++) {
                q[i] += dq(i);
            }
        }

        cout << "[IK] Failed to converge after " << maxIter << " iterations.\n";
        return q;
    }
};
// ---- 사다리꼴 속도 프로파일 클래스 (시작/종료 속도 고려) ----
class TrapezoidalBlendProfile {
public:
    double s_total;
    double v_start, v_end, v_max, a_max;
    double t_total;
    double t_accel, t_const, t_decel;

    TrapezoidalBlendProfile(double s_total_, double v_start_, double v_end_, double v_max_, double a_max_)
        : s_total(s_total_), v_start(v_start_), v_end(v_end_), v_max(v_max_), a_max(a_max_) {

        double t_ramp_up = (v_max - v_start) / a_max;
        double t_ramp_down = (v_max - v_end) / a_max;

        double s_ramp_up = (v_start + v_max) * t_ramp_up * 0.5;
        double s_ramp_down = (v_end + v_max) * t_ramp_down * 0.5;

        if (s_ramp_up + s_ramp_down > s_total) {
            v_max = sqrt((a_max * s_total) + 0.5 * (v_start * v_start + v_end * v_end));
            t_accel = (v_max - v_start) / a_max;
            t_decel = (v_max - v_end) / a_max;
            t_const = 0.0;
        }
        else {
            t_accel = t_ramp_up;
            t_decel = t_ramp_down;
            double s_const = s_total - (s_ramp_up + s_ramp_down);
            t_const = s_const / v_max;
        }
        t_total = t_accel + t_const + t_decel;
    }

    double get_velocity(double t) const {
        if (t < t_accel) return v_start + a_max * t;
        else if (t < t_accel + t_const) return v_max;
        else if (t < t_total) return v_max - a_max * (t - t_accel - t_const);
        else return v_end;
    }

    double get_total_time() const { return t_total; }
};

int main() {
    podo::Cobot robot("10.0.2.7");
    podo::ResponseCollector rc;
    robot.set_operation_mode(rc, podo::OperationMode::Real);
    robot.set_speed_bar(rc, 0.5);
    robot.flush(rc);

    Robot9Link model;

    // 현재 관절각도 읽기
    vector<double> joint_deg(6);
    for (int i = 0; i < 6; ++i)
        robot.get_system_variable(rc, static_cast<podo::SystemVariable>(static_cast<int>(podo::SystemVariable::SD_J0_ANG) + i), joint_deg[i]);


    vector<double> q_rad(6);
    for (int i = 0; i < 6; ++i)
        q_rad[i] = joint_deg[i] * DEG2RAD;

    // 사용자에게 waypoint 입력 받기
    vector<Vector3d> waypoints;
    int N;
    cout << "Enter number of waypoints: ";
    cin >> N;

    for (int i = 0; i < N; ++i) {
        Vector3d pt;
        cout << "Waypoint " << i << " (x y z): ";
        for (int j = 0; j < 3; ++j) cin >> pt[j];
        waypoints.push_back(pt);
    }

    Vector3d p_start = model.forwardKinematics(q_rad).block<3, 1>(0, 3);
    Vector3d p_prev = p_start;

    double max_vel = 100.0;  // mm/s
    double max_acc = 100.0; // mm/s^2



    for (size_t wp = 0; wp < waypoints.size(); ++wp) {
        Vector3d p_target = waypoints[wp];
        Vector3d dir = p_target - p_prev;
        double dist = dir.norm();
        if (dist < 1e-6) continue;
        dir.normalize();

        double v0 = (wp == 0) ? 0.0 : max_vel;
        double v1 = (wp == waypoints.size() - 1) ? 0.0 : max_vel;

        TrapezoidalBlendProfile profile(dist, v0, v1, max_vel, max_acc);
        double t = 0.0;
        double t_total = profile.get_total_time();
        int step = 0;

        while (t < t_total) {
            auto t_start = chrono::steady_clock::now();

            double v_mag = profile.get_velocity(t);
            Vector3d v_tcp_linear = dir * v_mag;

            Matrix4d T_cur = model.forwardKinematics(q_rad);
            Vector3d p_cur = T_cur.block<3, 1>(0, 3);
            Vector3d rpy_cur = model.matrixToRPY(T_cur);

            Matrix<double, 6, 6> J = model.computeJacobian6x6(q_rad);
            Matrix<double, 6, 6> Jinv = J.transpose() *
                (J * J.transpose() + 0.01 * Matrix6d::Identity()).inverse();

            Vector<double, 6> v_tcp;
            v_tcp.head<3>() = v_tcp_linear;
            v_tcp.tail<3>().setZero();

            Vector<double, 6> dq = Jinv * v_tcp;
            array<double, 6> dq_deg;
            for (int i = 0; i < 6; ++i) {
                q_rad[i] += dq[i] * LOOP_DT;
                dq_deg[i] = dq[i] * RAD2DEG;
            }

            // 10 스텝마다 디버깅 출력
            if (step % 10 == 0) {
                cout << "\n[Waypoint " << wp << " - STEP " << step << "] t = " << t << " sec\n";
                cout << "  ? TCP 위치 [mm]:   " << p_cur.transpose() << "\n";
                cout << "  ? TCP 자세 [deg]:  " << (rpy_cur * RAD2DEG).transpose() << "\n";
                cout << "  ? TCP 속도 [mm/s]: " << v_tcp_linear.transpose() << "\n";
                cout << "  ? dq (deg/s):      ";
                for (double val : dq_deg) cout << val << " ";
                cout << "\n  ? q (deg):         ";
                for (double q : q_rad) cout << rad2deg(q) << " ";
                cout << endl;
            }
            constexpr double MAX_JOINT_SPEED = 30.0;  // deg/s 제한
            dq_deg[i] = std::clamp(dq_deg[i], -MAX_JOINT_SPEED, MAX_JOINT_SPEED);

            robot.move_speed_j(rc, dq_deg, LOOP_DT, LOOP_DT * 2, 0.5, 0.5);

            t += LOOP_DT;
            step++;
            this_thread::sleep_until(t_start + chrono::microseconds(int(LOOP_DT * 1e6)));
        }

        p_prev = p_target;
    }

    cout << "\n? 모든 waypoint 이동 완료\n";
    return 0;
}
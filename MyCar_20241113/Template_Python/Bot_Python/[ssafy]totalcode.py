# DrivingInterface의 drive_controller 모듈에서 DrivingController 클래스 임포트
from DrivingInterface.drive_controller import DrivingController
import math

previous_value = 0

class DrivingClient(DrivingController):

    def __init__(self):
        # =========================================================== #
        #  멤버 변수 초기화 영역                                      #
        # =========================================================== #
        # 여기부터 코드 수정/추가 영역
        
        self.debug_mode = False  # 디버그 모드 활성화 여부

        # 제어 모드: True(API 기반 제어), False(키보드 기반 제어)
        self.enable_api_control = True
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99  # 트랙 타입 식별용(사용자 정의)

        # 사고 발생 및 복구 관련 변수
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        self.accident_step = 0
        
        # 유턴 관련 변수
        self.uturn_step = 0
        self.uturn_count = 0

        # 여기까지 코드 수정/추가 영역
        # =========================================================== #
        
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):
        # =========================================================== #
        #  주행 로직 구현 영역                                         #
        # =========================================================== #
        # 여기부터 코드 수정/추가 영역
        #
        # sensing_info: 차량의 현재 상태, 트랙 정보, 장애물 정보 등을 포함
        # car_controls: 반환할 차량 제어값(조향, 스로틀, 브레이크)
        
        if self.debug_mode:
            # 디버그 모드일 때 각종 정보 출력
            print("=========================================================")
            print("[MyCar] to_middle: {}".format(sensing_info.to_middle))
            print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] speed: {} km/h".format(sensing_info.speed))
            print("[MyCar] moving_forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving_angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))
            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        # 도로 반 너비를 구하고 차선 유지에 참고 (도로 폭/2 - 3)
        half_lane_width = self.half_road_limit - 3
        
        # 기본적으로 스로틀은 최대로, 브레이크는 0으로 설정
        car_controls.throttle = 1
        car_controls.brake = 0

        # 현재 차량 상태 변수
        current_middle = sensing_info.to_middle   # 차량이 중앙으로부터 어느 정도 떨어져있는지
        current_speed = sensing_info.speed         # 현재 차량 속도
        forward_angles = sensing_info.track_forward_angles  # 전방 waypoint 각도 정보

        # 장애물 회피를 위한 변수
        obstacle_detect_start = 0
        obstacle_detect_end = 150

        # 가능한 주행 라인 후보를 생성 (도로 폭 내에서 0.1 간격)
        possible_lines = [round(i * 0.1, 1) for i in range(-int(half_lane_width)*10, int(half_lane_width)*10)]
        backup_lines = []
        distance_checker = 0
        obstacle_counter = 0

        # 장애물이 있는 위치를 피해갈 수 있는 라인만 필터링
        for obstacle in sensing_info.track_forward_obstacles:
            obs_dist, obs_middle = obstacle['dist'], obstacle['to_middle']

            # 특정 거리 안에 들어온 장애물에 대해 회피 선별
            if obstacle_detect_start <= obs_dist <= obstacle_detect_end:
                # 장애물 주변 패딩 거리 설정 (차량 속도 및 각도 반영)
                if abs(forward_angles[int(obs_dist/10)]) > 5:
                    padding = 3.5
                elif 30 < obs_dist < 80:
                    padding = 2.5
                else:
                    padding = 2.25

                # 장애물 주변 라인은 가능한 라인에서 제외
                possible_lines = [i for i in possible_lines if not (obs_middle - padding <= i <= obs_middle + padding)]
                obstacle_counter += 1
            
            # 가능한 라인이 없으면 백업 라인으로 복원
            if not possible_lines:
                possible_lines = backup_lines[:]
                break
            else:
                backup_lines = possible_lines[:]
            
            # 너무 많은 장애물을 처리하는 경우를 회피하기 위해 한 번에 너무 멀리 가지 않도록 제어
            if obs_dist - distance_checker > 50 and obstacle_counter >= 2:
                break
            else:
                distance_checker = obs_dist

        # 현재 차량의 middle 위치에서 가장 가까운 라인을 목표 라인으로 결정
        target_line = min(possible_lines, key=lambda x: abs(x - current_middle))

        # PID 컨트롤을 흉내낸 간단한 조향 보정 계산
        proportional = -(current_middle - target_line) * 0.1
        integral = proportional ** 2 * 0.05 if proportional >= 0 else -proportional ** 2 * 0.05
        middle_correction = 0.5 * proportional + 0.4 * integral

        # 속도 영역에 따른 forward_angles 사용 인덱스 결정
        if current_speed < 50 and sensing_info.lap_progress > 1:
            target_index = 0
        elif current_speed < 120:
            target_index = 1
        elif current_speed < 180:
            target_index = 2
        else:
            target_index = 4

        # 기본 조향 계산
        # 작은 각도일 경우 상대적으로 단순 보정
        if (forward_angles[target_index]) < 45:
            # 장애물이 없고 속도가 낮다면 기민한 조향
            if len(sensing_info.track_forward_obstacles) == 0 and current_speed < 160:
                car_controls.steering = (forward_angles[target_index] - sensing_info.moving_angle) / 90 - current_middle / 80
            else:
                # 속도에 따라 조향 보정 비율 변경
                if current_speed < 70:
                    steering_base = (forward_angles[target_index] - sensing_info.moving_angle) / 60
                else:
                    steering_base = (forward_angles[target_index] - sensing_info.moving_angle) / 90
                car_controls.steering = steering_base
                car_controls.steering += middle_correction
        else:
            # 큰 각도의 코너 진입 시, 곡률 반경을 이용한 조향 계산
            adjusted_speed = current_speed if current_speed >= 60 else 60
            if forward_angles[target_index] < 0:
                # 좌측 코너 회전 반경
                radius = self.half_road_limit - 1.25 + current_middle
                beta = - math.pi * adjusted_speed * 0.1 / radius
                car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if forward_angles[target_index] > -60 else -1
            else:
                # 우측 코너 회전 반경
                radius = self.half_road_limit - 1.25 - current_middle
                beta = math.pi * adjusted_speed * 0.1 / radius
                car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if forward_angles[target_index] < 60 else 1
            
            # 속도가 너무 높으면 감속
            if current_speed > 80:
                car_controls.throttle = -1
                car_controls.brake = 1

        # 안전 제어: 급커브나 차선 이탈에 가까우면 감속
        if (abs(forward_angles[int(current_speed//20)]) > 40 or abs(current_middle) > 9 or abs(car_controls.steering) >= 0.5) and current_speed > 100:
            car_controls.throttle = 0
            if current_middle > 9:
                car_controls.steering -= 0.1
            elif current_middle < -9:
                car_controls.steering += 0.1
            car_controls.brake = 0.3 if current_speed < 110 else 1

        # 고속에서 커브 진입 시 강제 감속
        if current_speed > 170 and abs(forward_angles[-1]) > 10:
            car_controls.throttle = -0.5
            car_controls.brake = 1
        
        # 차량이 거의 멈춘 상태라면 조향 중립
        if current_speed < 5:
            car_controls.steering = 0

        # 사고 발생 상황 복구 로직
        if current_speed > 10:
            # 차량이 움직이면 사고 단계 초기화
            self.accident_step = 0
            self.recovery_count = 0
            self.accident_count = 0

        # lap_progress가 조금 진행된 상황에서 속도가 거의 0이면 사고 카운트 증가
        if sensing_info.lap_progress > 0.5 and self.accident_step == 0 and abs(current_speed) < 1.0:
            self.accident_count += 1
        
        # 일정 횟수 이상 사고 상태면 사고 단계 진입
        if self.accident_count > 8:
            self.accident_step = 1

        # 사고 단계 1: 후진으로 빠져나오기
        if self.accident_step == 1:
            self.recovery_count += 1
            car_controls.steering = 0
            car_controls.throttle = -1
            car_controls.brake = 0

        # 일정 시간 후진 후 다음 단계로 전환
        if self.recovery_count > 20:
            self.accident_step = 2
            self.recovery_count = 0
            self.accident_count = 0

        # 사고 단계 2: 전진하면서 자세 재정렬
        if self.accident_step == 2:
            car_controls.steering = 0
            car_controls.throttle = 1
            car_controls.brake = 1
            if sensing_info.speed > -1:
                self.accident_step = 0
                car_controls.throttle = 1
                car_controls.brake = 0

        # 유턴(역주행) 상황 복구 로직
        # 전진이 아니라 후진 상태이고, 사고나 회복 상태가 아니라면 U턴 카운트 증가
        if not sensing_info.moving_forward and not (self.accident_count + self.accident_step + self.recovery_count) and current_speed > 0:
            self.uturn_count += 1
            if not self.uturn_step:
                if current_middle >= 0:
                    self.uturn_step = 1
                else:
                    self.uturn_step = -1
        
        # 다시 전진 상태라면 U턴 상태 초기화
        if sensing_info.moving_forward:
            self.uturn_count = 0
            self.uturn_step = 0

        # U턴 카운트가 일정 이상이면 조향을 한쪽으로 고정하고 조금 전진해서 자세 바꾸기
        if self.uturn_count > 5:
            car_controls.steering = self.uturn_step
            car_controls.throttle = 0.5
        
        if self.debug_mode:
            # 디버그용 현재 제어값 출력
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        # 여기까지 코드 수정/추가 영역
        # =========================================================== #
        
        # 계산한 제어값 반환
        return car_controls

    def set_player_name(self):
        # 플레이어 이름 설정(필요하면)
        player_name = ""
        return player_name

if __name__ == '__main__':
    # 메인 함수 실행 시 봇 시작
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)

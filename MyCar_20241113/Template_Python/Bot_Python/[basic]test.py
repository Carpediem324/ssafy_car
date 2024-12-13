import math
from DrivingInterface.drive_controller import DrivingController

before = 0

class DrivingClient(DrivingController):

    def __init__(self):
        # =========================================================== #
        # 멤버 변수 초기화 영역
        # =========================================================== #
        
        # 디버그 모드 출력 여부 설정
        self.is_debug = False

        # 차량 제어 모드 설정 (True: 코드로 제어 / False: 키보드 제어)
        self.enable_api_control = True
        super().set_enable_api_control(self.enable_api_control)

        # 트랙 타입 (기본값 99)
        self.track_type = 99

        # 사고 및 복구 관련 플래그/카운트
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

        # 상위 클래스 초기화
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):
        # =========================================================== #
        # 차량 주행 제어 로직 작성 영역
        # =========================================================== #

        # 디버그 정보 출력 (옵션)
        if self.is_debug:
            print("=========================================================")
            print("[MyCar] 차의 중앙 기준 오프셋: {}".format(sensing_info.to_middle))
            print("[MyCar] 충돌 여부: {}".format(sensing_info.collided))
            print("[MyCar] 현재 속도: {} km/h".format(sensing_info.speed))
            print("[MyCar] 전진 중 여부: {}".format(sensing_info.moving_forward))
            print("[MyCar] 주행 각도: {}".format(sensing_info.moving_angle))
            print("[MyCar] 랩 진행률: {}".format(sensing_info.lap_progress))
            print("[MyCar] 앞으로의 트랙 각도 정보: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] 앞쪽 장애물 정보: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] 상대 차량 정보: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] 웨이포인트까지의 거리: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        # 기본 가속 및 브레이크 값 설정
        car_controls.throttle = 1
        car_controls.brake = 0

        # 차량 중심 오프셋 값
        current_middle = sensing_info.to_middle
        # 현재 속도
        current_speed = sensing_info.speed

        # middle값의 부호에 따라 방향 플래그 설정
        direction_flag = 1 if current_middle >= 0 else -1

        # 웨이 포인트, 각도 리스트 생성
        # points: 웨이포인트까지의 거리 정보를 기반으로 좌표 변환에 활용
        points = [current_middle * direction_flag] + sensing_info.distance_to_way_points
        # angles: 방향 플래그를 고려한 트랙 각도 리스트
        angles = [0] + [a * direction_flag for a in sensing_info.track_forward_angles]

        # 'bo' 리스트와 'ts' 리스트는 경로 보정에 사용되는 각도들을 담는다.
        bo = [90, ]
        ts = []
        
        # 웨이포인트 기반 경로 계산 로직
        for i in range(20):
            C = 180 - bo[i] - (angles[i+1] - angles[i])
            temp_val = points[i] * math.sin(C * math.pi / 180) / points[i+1]

            # asin 범위 체크
            if temp_val > 1:
                temp_val = 1
            elif temp_val < -1:
                temp_val = -1
            
            A = math.asin(temp_val) * 180 / math.pi
            bo.append(A)
            ts.append(180 - C - A)

        # ways: 앞으로의 경로를 x,y 좌표계 형태로 저장
        ways = []
        for j in range(20):
            # sum(ts[:j+1]) : 누적 각도로부터 웨이포인트 좌표 변환
            ways.append([
                points[j+1] * math.sin(sum(ts[:j+1]) * math.pi / 180),
                -points[j+1] * direction_flag * math.cos(sum(ts[:j+1]) * math.pi / 180)
            ])

        # 속도에 따라 타겟 웨이포인트 인덱스(tg) 결정
        if current_speed < 120:
            tg = 5
        elif current_speed < 140:
            tg = 7
        else:
            tg = 9

        # 목표 각도(theta) 계산
        # ways[tg]: 목표 지점, moving_angle: 현재 진행 각도
        theta = math.atan(ways[tg][1] / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle

        # 각도 조건에 따라 조향 결정
        if abs(angles[tg+2]) < 49 or abs(angles[tg+1]) < 47:
            # 트랙 끝쪽(angles[-1])이 큰 각도이며 속도가 높다면 조향 보정
            if abs(angles[-1]) > 90 and current_speed > 165:
                if angles[-1] * direction_flag >= 0:
                    theta = math.atan((ways[tg][1] + 9) / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle
                else:
                    theta = math.atan((ways[tg][1] - 9) / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle
                car_controls.steering = theta / 180
            else:
                # 속도가 낮으면 좀 더 직접적인 조향, 속도가 높으면 덜 민감하게 조향
                if current_speed < 120:
                    car_controls.steering = theta / 85
                else:
                    car_controls.steering = theta / (current_speed * 0.95)

        else:
            # 커브가 큰 경우
            r = max(abs(ways[tg][0]), abs(ways[tg][1]))
            alpha = math.asin(math.sqrt(ways[tg][0] ** 2 + ways[tg][1] ** 2) / (2 * r)) * 2
            beta = alpha * current_speed * 0.1 / r
            beta = beta if theta >= 0 else -beta
            car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) * 1.15

        # 디버그 모드에서 최종 제어 값 출력
        if self.is_debug:
            print("[MyCar] 최종 조향값: {}, 스로틀: {}, 브레이크: {}".format(
                car_controls.steering, car_controls.throttle, car_controls.brake))

        return car_controls

    def set_player_name(self):
        # settings.json에서 플레이어 이름이 지정되지 않았다면 빈 문자열 반환
        player_name = ""
        return player_name

if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)

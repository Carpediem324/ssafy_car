import math
from DrivingInterface.drive_controller import DrivingController

class DrivingClient(DrivingController):

    def __init__(self):
        # 멤버 변수 초기화
        self.is_debug = False
        self.enable_api_control = True
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

        # 이전 스티어링 값 저장 변수 추가
        self.before_steering = 0.0

        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):
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

        # 기본 가속 및 브레이크
        car_controls.throttle = 1
        car_controls.brake = 0

        current_middle = sensing_info.to_middle
        current_speed = sensing_info.speed
        direction_flag = 1 if current_middle >= 0 else -1

        # 최고 속도 제한: 180km/h 근처에서 throttle 감소
        if current_speed > 180:
            car_controls.throttle = 0.5
            # 필요하다면 약간의 브레이크도 사용 가능
            car_controls.brake = 0.1

        points = [current_middle * direction_flag] + sensing_info.distance_to_way_points
        angles = [0] + [a * direction_flag for a in sensing_info.track_forward_angles]

        bo = [90, ]
        ts = []
        
        # 0으로 나누는 상황 회피를 위한 분모 체크
        for i in range(20):
            C = 180 - bo[i] - (angles[i+1] - angles[i])
            denominator = points[i+1] if points[i+1] != 0 else 1e-6
            temp_val = (points[i] * math.sin(C * math.pi / 180)) / denominator
            if temp_val > 1:
                temp_val = 1
            elif temp_val < -1:
                temp_val = -1
            
            A = math.asin(temp_val) * 180 / math.pi
            bo.append(A)
            ts.append(180 - C - A)

        ways = []
        for j in range(20):
            ways.append([
                points[j+1] * math.sin(sum(ts[:j+1]) * math.pi / 180),
                -points[j+1] * direction_flag * math.cos(sum(ts[:j+1]) * math.pi / 180)
            ])

        if current_speed < 120:
            tg = 5
        elif current_speed < 140:
            tg = 7
        else:
            tg = 9

        if ways[tg][0] == 0:
            ways[tg][0] = 1e-6  # 0으로 나누는 상황 회피
        theta = math.atan(ways[tg][1] / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle

        # 조향값 결정 로직
        if abs(angles[tg+2]) < 49 or abs(angles[tg+1]) < 47:
            if abs(angles[-1]) > 90 and current_speed > 165:
                # 코너 보정값 축소
                correction_factor = 3
                if angles[-1] * direction_flag >= 0:
                    theta = math.atan((ways[tg][1] + correction_factor) / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle
                else:
                    theta = math.atan((ways[tg][1] - correction_factor) / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle

                desired_steering = theta / 180
            else:
                if current_speed < 120:
                    desired_steering = theta / 85
                else:
                    # 고속에서는 민감도 축소
                    desired_steering = theta / (current_speed * 1.2)
        else:
            # 큰 커브에서는 기존 로직 유지
            r = max(abs(ways[tg][0]), abs(ways[tg][1]))
            if r == 0:
                r = 1e-6
            alpha = math.asin(math.sqrt(ways[tg][0] ** 2 + ways[tg][1] ** 2) / (2 * r)) * 2
            beta = alpha * current_speed * 0.1 / r
            beta = beta if theta >= 0 else -beta
            desired_steering = (beta - sensing_info.moving_angle * math.pi / 180) * 1.15

        # 조향값 클램핑
        max_steering = 0.5
        if desired_steering > max_steering:
            desired_steering = max_steering
        elif desired_steering < -max_steering:
            desired_steering = -max_steering

        # 조향 변화량 제한
        max_steering_change = 0.05
        steering_diff = desired_steering - self.before_steering
        if steering_diff > max_steering_change:
            desired_steering = self.before_steering + max_steering_change
        elif steering_diff < -max_steering_change:
            desired_steering = self.before_steering - max_steering_change

        car_controls.steering = desired_steering
        self.before_steering = desired_steering

        if self.is_debug:
            print("[MyCar] 최종 조향값: {}, 스로틀: {}, 브레이크: {}".format(
                car_controls.steering, car_controls.throttle, car_controls.brake))

        return car_controls

    def set_player_name(self):
        player_name = ""
        return player_name

if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)

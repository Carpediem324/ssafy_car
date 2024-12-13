import math
from bcrypt import kdf

from numpy import False_
from sklearn.cluster import k_means
from DrivingInterface.drive_controller import DrivingController

before = 0
accident_step = 0
recovery_count = 0
accident_count = 0
uturn_step = 0
uturn_count = 0

class DrivingClient(DrivingController):

    def __init__(self):
        # =========================================================== #
        # 이 영역에서 멤버 변수를 설정할 수 있다.                      #
        # =========================================================== #
        # 이 아래는 수정 구역
        #

        self.is_debug = False

        # 차량 제어 방식(api 또는 키보드)
        # True(코드로 제어), False(키보드로 제어)
        self.enable_api_control = True
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

        #
        # 수정 구역 끝
        # ==========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):
        global accident_count, recovery_count, accident_step, uturn_count, uturn_step

        # =========================================================== #
        # 주행 로직 작성 구역                                         #
        # 이 영역에 주행 규칙을 추가/수정할 수 있다.                  #
        # =========================================================== #
        # 수정 구역 시작
        #

        if self.is_debug:
            print("=========================================================")
            print("[MyCar] 중앙선까지 거리: {}".format(sensing_info.to_middle))
            print("[MyCar] 충돌 여부: {}".format(sensing_info.collided))
            print("[MyCar] 차량 속도: {} km/h".format(sensing_info.speed))
            print("[MyCar] 전진 중 여부: {}".format(sensing_info.moving_forward))
            print("[MyCar] 이동 각도: {}".format(sensing_info.moving_angle))
            print("[MyCar] 랩 진행률: {}".format(sensing_info.lap_progress))
            print("[MyCar] 전방 트랙 각도 정보: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] 전방 장애물 정보: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] 상대 차량 정보: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] 웨이포인트까지 거리: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        # 가속/브레이크 기본값 설정
        car_controls.throttle = 1
        car_controls.brake = 0

        # 웨이포인트 좌표 계산 시작
        middle = sensing_info.to_middle
        spd = sensing_info.speed

        # 차량이 오른쪽(+)인지 왼쪽(-)인지 판단
        plag = 1 if middle >= 0 else -1

        points = [middle * plag,] + sensing_info.distance_to_way_points
        angles = [0,] + [angle * plag for angle in sensing_info.track_forward_angles]
        bo = [90, ]
        ts = []
        for i in range(20):
            C = 180 - bo[i] - (angles[i+1] - angles[i])
            temp = points[i] * math.sin(C * math.pi / 180) / points[i+1]
            if temp > 1:
                temp = 1
            elif temp < -1:
                temp = -1
            A =  math.asin(temp) * 180 / math.pi
            bo.append(A)
            ts.append(180 - C - A)

        # 웨이포인트 좌표 계산
        ways = []
        for j in range(20): 
            ways.append([
                points[j+1] * math.sin(sum(ts[:j+1]) * math.pi / 180), 
                - points[j+1] * plag * math.cos(sum(ts[:j+1]) * math.pi / 180)
            ])

        # 장애물 좌표 계산
        obs = []
        near = abs(points[0] * math.cos((90 - angles[1]) * math.pi / 180)) + points[1] * math.cos(bo[1] * math.pi / 180)
        for obj in sensing_info.track_forward_obstacles:
            d, m = obj['dist'] - near, obj['to_middle']
            if d <= 0:
                n, k = -1, obj['dist']
                ang = (90 - angles[n+1] * plag) * math.pi / 180
                obs.append([k * math.sin(ang) - m * math.cos(ang), -middle + k * math.cos(ang) + m * math.sin(ang)])
    
            else:
                n, k = int(d // 10), d % 10
                if n+2 > 10:
                    break
                ang = (90 - angles[n+1] * plag) * math.pi / 180
                obs.append([
                    ways[n][0] + k * math.sin(ang) - m * math.cos(ang),
                    ways[n][1] + k * math.cos(ang) + m * math.sin(ang)
                ])

        
        # 속도별 목표 웨이포인트 인덱스 설정
        if spd < 120:
            tg = 4
        elif spd < 155:
            tg = 5
        else:
            tg = 7

        theta = math.atan(ways[tg][1] / ways[tg][0]) * 180 / math.pi - sensing_info.moving_angle

        # 조향값 설정
        if abs(theta) < 50:
            if spd < 120:
                car_controls.steering = theta / 120
            else:
                car_controls.steering = theta / 100
        else:
            r = max(abs(ways[tg][0]), abs(ways[tg][1]))
            alpha = math.asin(math.sqrt(ways[tg][0] ** 2 + ways[tg][1] ** 2) / (2 * r)) * 2
            beta = alpha * spd * 0.12 / r
            beta = beta if theta >= 0 else -beta
            car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) * 1

        # 복잡한 코너 상황에서 속도 제어
        if abs(sum(sensing_info.track_forward_angles[10:])) > 1000 and angles[-1] < 130 and spd > 110:
            car_controls.throttle = -0.2
            if abs(angles[1]) > 5:
                car_controls.steering = 1 if angles[1] >= 0 else -1

        # 충돌 발생 시 탈출 로직 (추후 수정 필요)

        if spd > 10:
            accident_step = 0
            recovery_count = 0
            accident_count = 0

        if sensing_info.lap_progress > 0.5 and accident_step == 0 and abs(spd) < 1.0:
            accident_count += 1
        
        if accident_count > 8:
            accident_step = 1

        if accident_step == 1:
            recovery_count += 1
            car_controls.steering = 0
            car_controls.throttle = -1
            car_controls.brake = 0

        if recovery_count > 20:
            accident_step = 2
            recovery_count = 0
            accident_count = 0

        if accident_step == 2:
            car_controls.steering = 0
            car_controls.throttle = 1
            car_controls.brake = 1
            if sensing_info.speed > -1:
                accident_step = 0
                car_controls.throttle = 1
                car_controls.brake = 0

        # 역방향 주행 시 유턴 제어 로직
        if not sensing_info.moving_forward and not (accident_count + accident_step + recovery_count) and spd > 0:
            uturn_count += 1
            if middle >= 0:
                uturn_step = 1
            else:
                uturn_step = -1
        
        if sensing_info.moving_forward:
            uturn_count = 0

        if uturn_count > 5:
            car_controls.steering = uturn_step
            car_controls.throttle = 0.5

        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # 수정 구역 끝
        # ==========================================================#
        return car_controls


    def set_player_name(self):
        player_name = ""
        return player_name

if __name__ == '__main__':
    print("[MyCar] 주행 봇 시작! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] 주행 봇 종료! (PYTHON)")

    exit(return_code)

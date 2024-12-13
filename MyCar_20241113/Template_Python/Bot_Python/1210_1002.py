import math
from DrivingInterface.drive_controller import DrivingController

class DrivingClient(DrivingController):

    def __init__(self):
        # 기본 설정
        self.is_debug = False
        self.enable_api_control = True
        super().set_enable_api_control(self.enable_api_control)

        # 기본 트랙 타입
        self.track_type = 99

        # 사고 및 복구 관련 변수
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0

        # 상위 클래스 초기화
        super().__init__()

    def control_driving(self, car_controls, sensing_info):
        # 디버그 출력
        if self.is_debug:
            print("=========================================================")
            print("[MyCar] 오프셋: {}".format(sensing_info.to_middle))
            print("[MyCar] 속도: {} km/h".format(sensing_info.speed))
            print("[MyCar] 차량 각도: {}".format(sensing_info.moving_angle))
            print("[MyCar] 트랙 앞 각도: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] 웨이포인트 거리: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        # 기본값 설정
        car_controls.throttle = 1.0
        car_controls.brake = 0.0

        current_speed = sensing_info.speed
        current_angle = sensing_info.moving_angle
        to_middle = sensing_info.to_middle
        forward_angles = sensing_info.track_forward_angles

        # 앞으로의 트랙 각도 정보 (최대 20개 정도 제공)
        # 예: forward_angles[0], forward_angles[1], ...
        # 0번 인덱스: 현재 차 앞의 기준각과의 상대적 차이
        # 양수: 왼쪽 커브, 음수: 오른쪽 커브(혹은 그 반대 환경에 따라 다를 수 있음)
        # 여기서는 절대값이 클수록 커브가 심한 것으로 가정

        # 커브 강도를 판단하기 위해 앞쪽 몇 개의 각도를 확인
        # 직선 구간: 평균각도 변화가 작음
        # 커브 구간: 평균각도 변화가 큼
        # 예를 들어 처음 두세 개 각도를 보고 커브 예상
        angle_ahead = forward_angles[0] if len(forward_angles) > 0 else 0
        angle_second = forward_angles[1] if len(forward_angles) > 1 else angle_ahead
        angle_third = forward_angles[2] if len(forward_angles) > 2 else angle_second

        # 전체적으로 앞으로의 꺾임 정도를 파악 (단순 평균)
        upcoming_curve = (angle_ahead + angle_second + angle_third) / 3.0

        # 목표 스티어링 계산: 단순히 첫번째 포인트를 기준으로 조향
        # 여기서는 좀 더 안정적인 계산을 위해 현재 각도와 트랙 각도의 차이를 활용
        target_steer = (upcoming_curve - current_angle) / 60.0  # 분모를 조절하여 감도 변경
        # steering 범위를 -1 ~ 1로 제한
        if target_steer > 1.0:
            target_steer = 1.0
        elif target_steer < -1.0:
            target_steer = -1.0
        car_controls.steering = target_steer

        # 속도 제어 로직: 커브가 심할수록 감속
        # 앞으로 올 커브 각도가 클수록 차를 안정적으로 컨트롤하기 위해 감속
        # 절대값이 클수록 큰 커브
        curve_intensity = abs(upcoming_curve)

        # 일정 속도 이상에서 큰 커브 진입 시 감속
        if curve_intensity > 30:  # 커브 각도가 30도 이상이면 꽤 큰 커브로 가정
            # 속도를 제어
            if current_speed > 140:
                # 급커브에서는 빠르게 브레이크를 사용해 속도 조절
                car_controls.brake = 0.3
                car_controls.throttle = 0.7
            elif current_speed > 120:
                # 중간 정도 커브에서는 약간의 감속
                car_controls.brake = 0.1
                car_controls.throttle = 0.9
            else:
                # 현재속도가 낮다면 굳이 감속할 필요 없이 가속 유지
                car_controls.brake = 0.0
                car_controls.throttle = 1.0
        else:
            # 직선 구간 혹은 완만한 커브
            # 속도를 최대한 끌어올림
            if current_speed < 200:
                car_controls.throttle = 1.0
                car_controls.brake = 0.0
            else:
                # 너무 속도가 빠르면 코너 제어가 어렵기 때문에 약간 감속
                car_controls.throttle = 0.9
                car_controls.brake = 0.0

        # 트랙 중앙으로 돌아가려는 노력
        # to_middle이 양수면 트랙 중앙에서 오른쪽, 음수면 왼쪽
        # 지나치게 한쪽으로 치우친 경우 약간 조향으로 중앙 복귀
        if abs(to_middle) > 2.0:
            correction = (0 - to_middle) / 10.0
            # 기존 스티어링에 중앙 복귀를 위한 추가 조정
            car_controls.steering += correction
            # 스티어링 보정 범위 제한
            if car_controls.steering > 1.0:
                car_controls.steering = 1.0
            elif car_controls.steering < -1.0:
                car_controls.steering = -1.0

        if self.is_debug:
            print("[MyCar] 최종 조향값: {}, 스로틀: {}, 브레이크: {}".format(
                car_controls.steering, car_controls.throttle, car_controls.brake))

        return car_controls

    def set_player_name(self):
        return ""

if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")
    exit(return_code)

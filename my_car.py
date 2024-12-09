from DrivingInterface.drive_controller import DrivingController
import math

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  멤버 변수 설정 영역 ======================================= #
        # =========================================================== #
        # 여기서부터 수정 가능 영역
        # 디버그 모드 설정
        self.is_debug = False

        # API 제어 또는 키보드 제어 선택
        self.enable_api_control = True  # True: 코드로 제어 / False: 키보드로 제어
        super().set_enable_api_control(self.enable_api_control)

        # 여기까지 수정 가능 영역
        # =========================================================== #
        super().__init__()

    def tr_steer_count(self, car_controls, sensing_info):
        # 전방 트랙 각도 가져오기
        ft_angs = sensing_info.track_forward_angles

        # 현재 자동차와 전방 트랙 각도 계산
        ft_ang = ft_angs[0]
        t_val_plus = ft_angs[2] - ft_angs[1]
        t_plus = 50

        # 기본 조향값 계산
        c_st = ft_ang / 90
        co_th_dscp = 7 / 9
        co_br_ascp = 1 / 9

        # 특정 조건에서 각도 보정
        if 1 / 12 < c_st < 2 / 3:
            if ft_ang >= 0:
                ft_ang += 16
            elif ft_ang < 0:
                ft_ang -= 16

            if ft_ang >= 90:
                ft_ang = 90
            elif ft_ang < -90:
                ft_ang = -90

            c_st = ft_ang / 90

        # 조향값 및 속도 제어
        car_controls.steering = c_st
        car_controls.throttle = 1 - abs(c_st) * co_th_dscp

    def control_driving(self, car_controls, sensing_info):
        # =========================================================== #
        # 주행 제어 규칙을 작성하는 영역 ============================= #
        # =========================================================== #
        # 여기서부터 수정 가능 영역
        # 디버그 모드에서 상태 출력
        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))
            print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))
            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))
            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################
        # 초기 설정
        car_controls.steering = 0
        car_controls.throttle = 0.8
        car_controls.brake = 0

        # 현재 자동차와 트랙 사이의 각도 계산
        ct_ang = sensing_info.moving_angle

        # 트랙 각도 보정을 위한 계산
        ft_angs = sensing_info.track_forward_angles
        ft_dists = sensing_info.distance_to_way_points
        td_hs = [0] * 10
        td_ws = [0] * 10
        ft_tr_angs = [0] * 10
        up_ft_tr_angs = [0] * 10

        ft_ang1 = ft_angs[0]
        ft_ang2 = ft_angs[1]
        ft_dist1 = ft_dists[0]
        ft_dist2 = ft_dists[1]
        ft_tr_per = 1

        for i in range(8):
            td_hs[i] = ft_dists[i + 1] - ft_dists[i]
            td_ws[i] = round(math.tan(math.radians(ft_angs[i + 1])) * ft_dists[i + 1], 2) - round(
                math.tan(math.radians(ft_angs[i])) * ft_dists[i], 2
            )
            if i == 1:
                ft_tr_per = 1 / 2
            else:
                ft_tr_per = 1 / 3

            ft_tr_angs[i] = round(math.degrees(round(math.atan(td_ws[i] / td_hs[i]), 2)), 2) * ft_tr_per
            up_ft_tr_angs[i] = ft_tr_angs[i] * (6 / 5)

        td_h = ft_dist2 - ft_dist1
        td_w = round(math.tan(math.radians(ft_ang2)) * ft_dist2, 2) - round(
            math.tan(math.radians(ft_ang1)) * ft_dist1, 2
        )
        ft_tr12_ang = round(math.degrees(round(math.atan(td_w / td_h), 2)), 2)

        # 회전 각도 보정 및 속도 설정
        # ...

        # 디버그 출력
        print("현재 차 각도: ", sensing_info.moving_angle)
        print("전방 트랙사이 각도: ", ft_tr12_ang)
        print("전방 트랙1 각도: ", ft_ang1)
        print("half road: ", self.half_road_limit)
        print("중간 거리: ", sensing_info.to_middle)
        print("c_st: ", car_controls.steering)
        print("c_th: ", car_controls.throttle)
        print("c_br: ", car_controls.brake)
        print()

        if self.is_debug:
            print(
                "[MyCar] steering:{}, throttle:{}, brake:{}".format(
                    car_controls.steering, car_controls.throttle, car_controls.brake
                )
            )

        # 여기까지 수정 가능 영역
        # =========================================================== #
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

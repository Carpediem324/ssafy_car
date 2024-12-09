from DrivingInterface.drive_controller import DrivingController

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        # 추가적으로 사용할 멤버 변수 초기화 영역
        # =========================================================== #
        
        # 디버그 모드: True일 경우 각종 정보를 출력한다.
        self.debug_mode = False
        
        # 충돌 상태 관리 변수
        self.crash_happened = False     # 충돌로 차량이 멈춘 상태 여부
        self.crash_timer = 0            # 차량 속도가 매우 낮은 상태(=사고 상태)가 계속되는 시간 카운트
        self.crash_recover_counter = 0  # 후진 등을 통해 충돌 상태에서 복구하는 시간 카운트
        
        # 트랙 특성 파악(필요시 사용, 여기서는 임의로 활용)
        self.track_type = None
        
        super().__init__()

    def control_driving(self, car_controls, sensing_info):
        # sensing_info를 이용해 주행 로직을 제어하는 부분
        
        # 디버그용 정보 출력
        if self.debug_mode:
            print("-----------------------------------------------------------")
            print(f"to_middle: {sensing_info.to_middle}")
            print(f"speed: {sensing_info.speed}")
            print(f"moving_angle: {sensing_info.moving_angle}")
            print(f"collided: {sensing_info.collided}")
            print(f"forward_angles: {sensing_info.track_forward_angles}")
            print(f"obstacles: {sensing_info.track_forward_obstacles}")
            print(f"lap_progress: {sensing_info.lap_progress}")
            print("-----------------------------------------------------------")

        # --------------------------
        # 기본 파라미터 및 초기값 설정
        # --------------------------
        current_speed = sensing_info.speed
        mid_offset = sensing_info.to_middle
        forward_angles = sensing_info.track_forward_angles
        
        # 제한된 도로 폭 내에서 차량이 중앙선에 가깝게 유지하기 위한 보정값
        # 도로 폭 대비 안전 범위를 고려하여 약간의 여유를 둔다.
        lane_width_half = self.half_road_limit - 1.0

        # --------------------------
        # 사고(충돌) 상태 판정 및 복구 로직
        # --------------------------
        # 일정 속도 미만에서 장시간 머무르면 충돌 상태라고 판단
        if current_speed > 30.0:  
            # 정상 주행 중 -> 충돌 상태 초기화
            self.crash_happened = False
            self.crash_timer = 0
            self.crash_recover_counter = 0
        else:
            # 속도가 매우 낮은 상태가 지속되면 충돌 상태라고 판단
            if sensing_info.lap_progress > 1.0 and abs(current_speed) < 1.0:
                self.crash_timer += 1
            if self.crash_timer > 6:  # 약 0.6초 이상 속도 정체 시 충돌로 간주
                self.crash_happened = True

        # 충돌 상태면 후진으로 복구 시도
        if self.crash_happened:
            # 약간 핸들을 꺾고 후진
            car_controls.steering = 0.05
            car_controls.throttle = -1.0
            car_controls.brake = 0.0
            self.crash_recover_counter += 1
            
            # 일정 시간 후진 후 복구 완료 처리
            if self.crash_recover_counter > 20:
                self.crash_happened = False
                self.crash_timer = 0
                self.crash_recover_counter = 0
            return car_controls

        # --------------------------
        # 주행 방향 및 속도 제어 로직
        # --------------------------

        # 전방 커브 각도 분석을 통한 주행 방향 참고:
        # 속도에 따라 참조할 각도 인덱스 결정(속도가 높을수록 앞쪽 각도 참조)
        angle_index = min(int(current_speed / 50), len(forward_angles)-1) 
        target_angle = forward_angles[angle_index]

        # 차량이 중앙에서 얼마나 벗어났는지, 그리고 트랙 진행 방향 대비 현재 주행 각도를 반영
        # 중앙 정렬을 위한 미세 조정: 중앙선에서 멀어질수록 반대 방향으로 미세 조정
        center_adjust = - (mid_offset / 80.0)  # 중앙에서 최대 ±6m 정도; 이를 80으로 나눠 0.075 내외로 변환

        # 커브 각도 대비 현재 진행 각도 차이를 통해 핸들 조정값 계산
        # 속도가 높을수록 민감도(steer_factor) 감소
        if current_speed <= 70:
            steer_factor = 1.3
        elif current_speed <= 100:
            steer_factor = 0.9
        else:
            steer_factor = 0.7
        steering_input = (target_angle - sensing_info.moving_angle) / (current_speed * steer_factor + 0.001)
        
        # 중앙 정렬값 추가
        steering_input += center_adjust

        # 속도 제어: 커브가 클수록 속도를 약간 줄임
        curve_intensity = abs(target_angle)
        # 기본 스로틀
        throttle_input = 0.75
        # 커브 강도에 따라 스로틀 조정
        if curve_intensity > 30:
            throttle_input = 0.6
        if curve_intensity > 60:
            throttle_input = 0.4
        
        # 저속구간일 경우 초기 가속
        if current_speed < 50:
            throttle_input = 0.9

        # 장애물 회피 로직
        # 가장 가까운 장애물을 파악하고, 일정 거리 이하이면 회피 핸들 조정
        brake_input = 0.0
        if sensing_info.track_forward_obstacles:
            nearest_obs = sensing_info.track_forward_obstacles[0]
            obs_dist = nearest_obs['dist']
            obs_pos = nearest_obs['to_middle']

            # 장애물이 가까울 경우 스티어링 보정 
            if obs_dist < 30:
                # 장애물과 차의 상대 위치 확인
                diff_pos = obs_pos - mid_offset
                safe_margin = 2.2  # 차량 폭 및 여유 포함한 최소 회피거리
                
                # 장애물이 차로와 겹칠 경우 회피 필요
                if abs(diff_pos) < safe_margin:
                    # 어느 방향으로 피할지 결정: 여유 공간이 넓은 쪽으로 회피
                    if mid_offset >= 0:
                        # 차량이 오른쪽에 치우쳐 있으면 왼쪽 여유 확인
                        left_space = lane_width_half + min(mid_offset, obs_pos)
                        right_space = lane_width_half - max(mid_offset, obs_pos)
                        if right_space > safe_margin:
                            # 오른쪽으로 회피
                            steering_input += (safe_margin - abs(diff_pos)) * 0.5
                        else:
                            # 왼쪽으로 회피
                            steering_input -= (safe_margin - abs(diff_pos)) * 0.5
                    else:
                        # 차량이 왼쪽에 치우쳐 있을 경우
                        left_space = lane_width_half + min(mid_offset, obs_pos)
                        right_space = lane_width_half - max(mid_offset, obs_pos)
                        if left_space > safe_margin:
                            # 왼쪽으로 회피
                            steering_input -= (safe_margin - abs(diff_pos)) * 0.5
                        else:
                            # 오른쪽으로 회피
                            steering_input += (safe_margin - abs(diff_pos)) * 0.5

                    # 회피 중 속도 조정(감속)
                    if current_speed > 80:
                        throttle_input = 0.5
                        brake_input = 0.3

        # 스티어링 값 범위 제한
        if steering_input > 1:
            steering_input = 1
        elif steering_input < -1:
            steering_input = -1

        # 최종 결정값 반영
        car_controls.steering = steering_input
        car_controls.throttle = throttle_input
        car_controls.brake = brake_input

        if self.debug_mode:
            print(f"Steering: {car_controls.steering}, Throttle: {car_controls.throttle}, Brake: {car_controls.brake}")

        return car_controls

    def set_player_name(self):
        # 별도로 플레이어 이름을 지정하지 않음
        player_name = ""
        return player_name

if __name__ == '__main__':
    print("[MyCar] Start Bot!")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot!")
    exit(return_code)

from DrivingInterface.drive_controller import DrivingController
import math

previous_value = 0

class DrivingClient(DrivingController):

    def __init__(self):
        # =========================================================== #
        #  Member variables                                           #
        # =========================================================== #
        # Start editing from here
        #
        
        self.debug_mode = False

        # Control mode: True(API-based) or False(Keyboard-based)
        self.enable_api_control = True
        super().set_enable_api_control(self.enable_api_control)

        self.track_type = 99

        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        self.accident_step = 0
        self.uturn_step = 0
        self.uturn_count = 0

        #
        # End editing here
        # ===========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        #  Area for implementing driving logic                        #
        # =========================================================== #
        # Start editing from here
        #

        if self.debug_mode:
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

        half_lane_width = self.half_road_limit - 3
        car_controls.throttle = 1
        car_controls.brake = 0

        # Extracting main variables
        current_middle = sensing_info.to_middle
        current_speed = sensing_info.speed
        forward_angles = sensing_info.track_forward_angles

        # Obstacle avoidance logic
        obstacle_detect_start = 0
        obstacle_detect_end = 150

        # Generate a line of possible middles within road limits
        possible_lines = [round(i * 0.1, 1) for i in range(-int(half_lane_width)*10, int(half_lane_width)*10)]
        backup_lines = []
        distance_checker = 0
        obstacle_counter = 0

        # Filter possible lines by removing ranges blocked by obstacles
        for obstacle in sensing_info.track_forward_obstacles:
            obs_dist, obs_middle = obstacle['dist'], obstacle['to_middle']

            if obstacle_detect_start <= obs_dist <= obstacle_detect_end:
                if abs(forward_angles[int(obs_dist/10)]) > 5:
                    padding = 3.5
                elif 30 < obs_dist < 80:
                    padding = 2.5
                else:
                    padding = 2.25

                possible_lines = [i for i in possible_lines if not obs_middle - padding <= i <= obs_middle + padding]
                obstacle_counter += 1
            
            # If no line remains, revert to backup
            if not possible_lines:
                possible_lines = backup_lines[:]
                break
            else:
                backup_lines = possible_lines[:]
            
            # If the next obstacle is too far and we already removed multiple obstacles
            if obs_dist - distance_checker > 50 and obstacle_counter >= 2:
                break
            else:
                distance_checker = obs_dist

        target_line = min(possible_lines, key=lambda x: abs(x - current_middle))
        proportional = -(current_middle - target_line) * 0.1
        integral = proportional ** 2 * 0.05 if proportional >= 0 else -proportional ** 2 * 0.05
        middle_correction = 0.5 * proportional + 0.4 * integral

        # Driving logic based on speed and angle
        if current_speed < 50 and sensing_info.lap_progress > 1:
            target_index = 0
        elif current_speed < 120:
            target_index = 1
        elif current_speed < 180:
            target_index = 2
        else:
            target_index = 4

        # Steering calculation
        if (forward_angles[target_index]) < 45:
            if len(sensing_info.track_forward_obstacles) == 0 and current_speed < 160:
                car_controls.steering = (forward_angles[target_index] - sensing_info.moving_angle) / 90 - current_middle / 80
            else:
                if current_speed < 70:
                    steering_base = (forward_angles[target_index] - sensing_info.moving_angle) / 60
                else:
                    steering_base = (forward_angles[target_index] - sensing_info.moving_angle) / 90
                car_controls.steering = steering_base
                car_controls.steering += middle_correction
        else:
            adjusted_speed = current_speed if current_speed >= 60 else 60
            if forward_angles[target_index] < 0:
                radius = self.half_road_limit - 1.25 + current_middle
                beta = - math.pi * adjusted_speed * 0.1 / radius
                car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if forward_angles[target_index] > -60 else -1
            else:    
                radius = self.half_road_limit - 1.25 - current_middle
                beta = math.pi * adjusted_speed * 0.1 / radius
                car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if forward_angles[target_index] < 60 else 1
            if current_speed > 80:
                car_controls.throttle = -1
                car_controls.brake = 1

        # Safety control in curves or off-center conditions
        if (abs(forward_angles[int(current_speed//20)]) > 40 or abs(current_middle) > 9 or abs(car_controls.steering) >= 0.5) and current_speed > 100:
            car_controls.throttle = 0
            if current_middle > 9:
                car_controls.steering -= 0.1
            elif current_middle < -9:
                car_controls.steering += 0.1
            car_controls.brake = 0.3 if current_speed < 110 else 1

        # Hard braking if speed is too high approaching a curve
        if current_speed > 170 and abs(forward_angles[-1]) > 10:
            car_controls.throttle = -0.5
            car_controls.brake = 1
        
        # Reset steering if almost stopped
        if current_speed < 5:
            car_controls.steering = 0

        # Collision recovery logic
        if current_speed > 10:
            self.accident_step = 0
            self.recovery_count = 0
            self.accident_count = 0

        if sensing_info.lap_progress > 0.5 and self.accident_step == 0 and abs(current_speed) < 1.0:
            self.accident_count += 1
        
        if self.accident_count > 8:
            self.accident_step = 1

        if self.accident_step == 1:
            self.recovery_count += 1
            car_controls.steering = 0
            car_controls.throttle = -1
            car_controls.brake = 0

        if self.recovery_count > 20:
            self.accident_step = 2
            self.recovery_count = 0
            self.accident_count = 0

        if self.accident_step == 2:
            car_controls.steering = 0
            car_controls.throttle = 1
            car_controls.brake = 1
            if sensing_info.speed > -1:
                self.accident_step = 0
                car_controls.throttle = 1
                car_controls.brake = 0

        # U-turn recovery logic if moving backward
        if not sensing_info.moving_forward and not (self.accident_count + self.accident_step + self.recovery_count) and current_speed > 0:
            self.uturn_count += 1
            if not self.uturn_step:
                if current_middle >= 0:
                    self.uturn_step = 1
                else:
                    self.uturn_step = -1
        
        if sensing_info.moving_forward:
            self.uturn_count = 0
            self.uturn_step = 0

        if self.uturn_count > 5:
            car_controls.steering = self.uturn_step
            car_controls.throttle = 0.5
        
        if self.debug_mode:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # End editing here
        # ===========================================================#
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

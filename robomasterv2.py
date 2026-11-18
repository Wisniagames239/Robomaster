###############################################################################################
#  Skrypt sterujący Dji RoboMaster S1 (Śledzenie szkieletu całego ciała - MediaPipe Pose)
#  Dominik Wiśniewski aka wisniagames239
###############################################################################################
from robomaster import robot, camera
import keyboard
import mouse
import time
import cv2
import threading
import mediapipe as mp

# --- Stałe konfiguracyjne sterowania ---
CENTER_X, CENTER_Y = 600, 400       # Centralna pozycja kursora myszy
INITIAL_ROBOT_SPEED = 100        # Początkowa prędkość robota (max 150 dla bezpieczeństwa)
GIMBAL_SENSITIVITY = 1           # Czułość myszy do sterowania gimbalem
CAMERA_RESOLUTION = camera.STREAM_540P
CMD_INTERVAL = 0.05              # Minimalny odstęp między komendami ruchu (20 Hz)

# --- Stałe regulacji odległości (TUTAJ REGULUJESZ DYSTANS) ---
TARGET_DISTANCE_M = 0.80         # Docelowa odległość w metrach (zmniejsz = podejdzie bliżej)
CALIBRATION_BODY_K = 0.78        # Stała kalibracji (Dystans = K / H)
STOP_HEIGHT_MIN = 0.85           # Próg zatrzymania (im bliżej 1.0, tym bliżej Ciebie stanie)
STOP_HEIGHT_MAX = 0.94           # Próg cofania (gdy podejdziesz zbyt blisko)
STOP_WIDTH_MAX = 0.72            # Próg cofania dla szerokości sylwetki
DETECTION_DELAY_SEC = 0.8        # Wymagany czas ciągłej detekcji przed ruszeniem


class PDController:
    def __init__(self, kp, kd, output_limits=(-100, 100), t=1):
        self.kp = kp
        self.kd = kd
        self.output_min, self.output_max = output_limits
        self.t = t

        self.previous_error = 0
        self.last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return 0

        error = (0 - current_value) * self.t
        p_term = self.kp * error
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative

        output = max(self.output_min, min(p_term + d_term, self.output_max))

        self.previous_error = error
        self.last_time = current_time

        return output


# --- Globalne zmienne stanu ---
ep_robot = None
ep_chassis = None
ep_gimbal = None
ep_camera = None
ep_blaster = None
ep_battery = None

connection_active = False
running = True
speed = INITIAL_ROBOT_SPEED
prev_mouse_x, prev_mouse_y = 0, 0
gimbal_angles = [0.0, 0.0, 0.0, 0.0]
battery = 100

mode = 1  # 1 - manual, 2 - auto tracking
modesir = {1: "1 - Manualny", 2: "2 - Auto (Szkielet)"}

# MediaPipe Pose
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose_detector = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=0,
    min_detection_confidence=0.55,
    min_tracking_confidence=0.55
)

latest_pose_landmarks = None
skeleton_debug_info = ""
person_detected_start_time = None
auto_drive_wheel_speed = 0
filtered_dist = None
is_moving_dist = False

pd_track_yaw = None
pd_track_dist = None

is_firing = False
window_initialized = False
last_gimbal_exec_time = 0
last_chassis_exec_time = 0


# --- Funkcje callback ---
def gimbal_angle_callback(*args):
    global gimbal_angles
    gimbal_angles = list(args[0])


def battery_callback(*args):
    global battery
    battery = args[0]


# --- Inicjalizacja i zamykanie robota ---
def initialize_robot():
    global ep_robot, ep_chassis, ep_gimbal, ep_camera, ep_blaster, ep_battery, \
        connection_active, prev_mouse_x, prev_mouse_y, pd_track_yaw, pd_track_dist

    ep_robot = robot.Robot()
    try:
        ep_robot.initialize(conn_type="ap")
        print("✅ Połączono z RoboMasterem!")
        connection_active = True

        pd_track_yaw = PDController(kp=350, kd=5, output_limits=(-200, 200), t=0.5)
        pd_track_dist = PDController(kp=95, kd=18, output_limits=(-75, 75), t=1.0)

        ep_chassis = ep_robot.chassis
        ep_gimbal = ep_robot.gimbal
        ep_camera = ep_robot.camera
        ep_blaster = ep_robot.blaster
        ep_battery = ep_robot.battery

        ep_blaster.set_led(brightness=0, effect="off")

        ep_gimbal.recenter().wait_for_completed()
        print("✅ Gimbal wyśrodkowany.")
        ep_robot.set_robot_mode(mode="free")

        if ep_camera.start_video_stream(display=False, resolution=CAMERA_RESOLUTION):
            print("✅ Strumień wideo uruchomiony.")
        else:
            print("❌ Nie udało się uruchomić strumienia wideo.")
            return False

        try:
            mouse.move(CENTER_X, CENTER_Y)
            time.sleep(0.05)
            prev_mouse_x, prev_mouse_y = CENTER_X, CENTER_Y
            print(f"✅ Kursor myszy ustawiony na ({CENTER_X}, {CENTER_Y})")
        except Exception as e:
            print(f"❌ Nie można pobrać pozycji myszy: {e}")
            return False

        ep_gimbal.sub_angle(freq=8, callback=gimbal_angle_callback)
        time.sleep(0.5)
        ep_battery.sub_battery_info(freq=1, callback=battery_callback)
        time.sleep(0.5)

        print("✅ Inicjalizacja zakończona pomyślnie!")
        return True

    except Exception as e:
        print(f"❌ Błąd połączenia lub inicjalizacji: {e}")
        close_robot_connection()
        return False


def close_robot_connection():
    global ep_robot, connection_active
    if pose_detector:
        pose_detector.close()
    if ep_robot:
        try:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            if ep_blaster:
                ep_blaster.set_led(brightness=0, effect="off")
            if ep_gimbal:
                ep_gimbal.unsub_angle()
            if ep_battery:
                ep_battery.unsub_battery_info()
            if ep_camera:
                ep_camera.stop_video_stream()
            cv2.destroyAllWindows()
            ep_robot.close()
            print("🛑 Połączenie z RoboMasterem zamknięte.")
        except Exception as e:
            print(f"❌ Błąd podczas zamykania połączenia: {e}")
    connection_active = False


# --- Obsługa strzału w osobnym wątku ---
def fire_blaster_async():
    global is_firing
    try:
        ep_blaster.set_led(brightness=100, effect="on")
        ep_blaster.fire()
        time.sleep(0.25)
        ep_blaster.set_led(brightness=0, effect="off")
        time.sleep(0.15)
    except Exception as e:
        print(f"⚠️ Błąd podczas strzału: {e}")
    finally:
        is_firing = False


# --- Obsługa klawiatury i myszy ---
def handle_input(pid_yaw_gimbal):
    global running, speed, prev_mouse_x, prev_mouse_y, mode, latest_pose_landmarks, \
        skeleton_debug_info, is_firing, person_detected_start_time, auto_drive_wheel_speed, \
        filtered_dist, is_moving_dist

    wheels = [0, 0, 0, 0]
    yaw_gimbal_input = 0
    pitch_gimbal_input = 0

    if keyboard.is_pressed('w'):
        wheels = [1, 1, 1, 1]
    if keyboard.is_pressed('s'):
        wheels = [-1, -1, -1, -1]
    if keyboard.is_pressed('a'):
        wheels = [-1, 1, 1, -1]
    if keyboard.is_pressed('d'):
        wheels = [1, -1, -1, 1]
    if keyboard.is_pressed('q'):
        wheels = [1, 1, -1, -1]
    if keyboard.is_pressed('e'):
        wheels = [-1, -1, 1, 1]
    if keyboard.is_pressed('x'):
        wheels = [0, 0, 0, 0]

    if keyboard.is_pressed('r'):
        speed = (speed % 200) + 50
        time.sleep(0.15)
        print(f"⚙️ Ustawiona prędkość robota: {speed}")

    if keyboard.is_pressed('f') and not is_firing:
        is_firing = True
        threading.Thread(target=fire_blaster_async, daemon=True).start()

    if keyboard.is_pressed('m'):
        mode = (mode % 2) + 1
        print(f"⚙️ Zmiana trybu na: {modesir[mode]}")

        latest_pose_landmarks = None
        skeleton_debug_info = ""
        person_detected_start_time = None
        auto_drive_wheel_speed = 0
        filtered_dist = None
        is_moving_dist = False

        ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        time.sleep(0.3)

    if keyboard.is_pressed("esc"):
        print("🛑 ESC wciśnięty. Zatrzymywanie programu.")
        running = False

    current_mouse_x, current_mouse_y = mouse.get_position()
    delta_x = current_mouse_x - prev_mouse_x
    delta_y = current_mouse_y - prev_mouse_y
    try:
        mouse.move(CENTER_X, CENTER_Y)
        prev_mouse_x, prev_mouse_y = CENTER_X, CENTER_Y
    except Exception as e:
        print(f"⚠️ Nie można ustawić kursora myszy: {e}")
        prev_mouse_x, prev_mouse_y = current_mouse_x, current_mouse_y

    if abs(delta_x) > 0:
        yaw_gimbal_input += delta_x * GIMBAL_SENSITIVITY
    if abs(delta_y) > 0:
        pitch_gimbal_input -= delta_y * GIMBAL_SENSITIVITY

    return wheels, yaw_gimbal_input, pitch_gimbal_input, pid_yaw_gimbal.update(gimbal_angles[1])


# --- Analiza szkieletu całego ciała (MediaPipe Pose) ---
def process_pose_tracking(img):
    global latest_pose_landmarks, skeleton_debug_info, last_gimbal_exec_time, \
        person_detected_start_time, auto_drive_wheel_speed, filtered_dist, is_moving_dist

    current_time = time.time()
    results = pose_detector.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    if results.pose_landmarks:
        latest_pose_landmarks = results.pose_landmarks
        lm = results.pose_landmarks.landmark

        visible_pts = [
            pt for pt in lm
            if pt.visibility > 0.65 and 0.01 < pt.x < 0.99 and 0.01 < pt.y < 0.99
        ]

        upper_body_visible = any(
            lm[idx].visibility > 0.65 and 0.02 < lm[idx].y < 0.95
            for idx in (0, 11, 12)
        )

        if len(visible_pts) >= 4:
            xs = [pt.x for pt in visible_pts]
            ys = [pt.y for pt in visible_pts]

            body_w = max(xs) - min(xs)
            body_h = max(ys) - min(ys)
            center_x = (min(xs) + max(xs)) / 2.0
            center_y = (min(ys) + max(ys)) / 2.0

            if person_detected_start_time is None:
                person_detected_start_time = current_time
                pd_track_dist.previous_error = 0
                pd_track_dist.last_time = current_time

            raw_dist = CALIBRATION_BODY_K / max(body_h, 0.05)
            filtered_dist = raw_dist if filtered_dist is None else (0.3 * raw_dist + 0.7 * filtered_dist)

            skeleton_debug_info = f" | H:{body_h:.2f} W:{body_w:.2f} (~{filtered_dist:.2f}m)"

            if (current_time - person_detected_start_time) >= DETECTION_DELAY_SEC:
                if not upper_body_visible:
                    auto_drive_wheel_speed = 0
                    is_moving_dist = False
                elif body_h >= STOP_HEIGHT_MAX or body_w >= STOP_WIDTH_MAX:
                    auto_drive_wheel_speed = -25
                    is_moving_dist = True
                elif body_h >= STOP_HEIGHT_MIN:
                    auto_drive_wheel_speed = 0
                    is_moving_dist = False
                    pd_track_dist.previous_error = 0
                    pd_track_dist.last_time = current_time
                else:
                    dist_error = filtered_dist - TARGET_DISTANCE_M
                    threshold = 0.06 if is_moving_dist else 0.12

                    if abs(dist_error) > threshold:
                        is_moving_dist = True
                        auto_drive_wheel_speed = int(pd_track_dist.update(-dist_error))
                    else:
                        is_moving_dist = False
                        auto_drive_wheel_speed = 0
                        pd_track_dist.previous_error = 0
                        pd_track_dist.last_time = current_time
            else:
                auto_drive_wheel_speed = 0

            yaw_speed = pd_track_yaw.update(0.5 - center_x)
            pitch_speed = 20 if not upper_body_visible else (0.48 - center_y) * 180

            if ep_gimbal and (current_time - last_gimbal_exec_time >= CMD_INTERVAL):
                try:
                    ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
                    last_gimbal_exec_time = current_time
                except Exception:
                    pass
        else:
            auto_drive_wheel_speed = 0
            skeleton_debug_info = " | Za malo punktow"
    else:
        latest_pose_landmarks = None
        skeleton_debug_info = ""
        person_detected_start_time = None
        auto_drive_wheel_speed = 0
        filtered_dist = None
        is_moving_dist = False

        if ep_gimbal and (current_time - last_gimbal_exec_time >= CMD_INTERVAL):
            try:
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                last_gimbal_exec_time = current_time
            except Exception:
                pass


# --- Wyświetlanie obrazu i HUD ---
def display_camera_feed(img):
    global running, connection_active, window_initialized

    if img is not None:
        if not connection_active:
            print("✅ Połączenie z RoboMasterem przywrócone.")
            connection_active = True

        img_h, img_w = img.shape[:2]

        if latest_pose_landmarks and mode == 2:
            mp_drawing.draw_landmarks(
                img,
                latest_pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=3),
                mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
            )

        cv2.putText(img, f"P: {gimbal_angles[0]:.2f} | Y: {gimbal_angles[1]:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, f"P_ground: {gimbal_angles[2]:.2f} | Y_ground: {gimbal_angles[3]:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, f"Battery: {battery}%", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, f"MODE: {modesir[mode]}{skeleton_debug_info}", (10, img_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2, cv2.LINE_AA)

        controls_help = [
            "STEROWANIE:",
            "WASD - Jazda | QE - Obrot",
            f"R - Predkosc ({speed}) | X - Stop",
            "F - Strzal | M - Zmiana trybu",
            "Mysz - Gimbal | ESC - Wyjscie"
        ]

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        font_thickness = 1
        line_spacing = 18
        margin_right = 15
        margin_bottom = 15

        max_text_width = max(cv2.getTextSize(line, font, font_scale, font_thickness)[0][0] for line in controls_help)
        total_height = len(controls_help) * line_spacing

        box_x1 = img_w - max_text_width - margin_right - 8
        box_y1 = img_h - total_height - margin_bottom
        box_x2 = img_w - margin_right + 8
        box_y2 = img_h - margin_bottom + 8

        overlay = img.copy()
        cv2.rectangle(overlay, (box_x1, box_y1), (box_x2, box_y2), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)

        for i, line in enumerate(controls_help):
            text_w = cv2.getTextSize(line, font, font_scale, font_thickness)[0][0]
            x_pos = img_w - text_w - margin_right
            y_pos = box_y1 + 15 + (i * line_spacing)
            color = (0, 255, 255) if i == 0 else (255, 255, 255)
            cv2.putText(img, line, (x_pos, y_pos), font, font_scale, color, font_thickness, cv2.LINE_AA)

        window_name = "RoboMaster S1 Camera"
        cv2.imshow(window_name, img)

        if not window_initialized:
            cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)
            cv2.waitKey(1)
            cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 0)
            window_initialized = True

        if cv2.waitKey(1) == 27:  # ESC
            print("🛑 ESC wciśnięty w oknie kamery. Zamykanie.")
            running = False
    else:
        if connection_active:
            print("⚠️ Utrata łączności z RoboMasterem (podczas odczytu kamery). Zatrzymywanie ruchu.")
            connection_active = False
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)


# --- Główna pętla sterowania ---
def main_control_loop():
    global running, last_chassis_exec_time

    pid_yaw_gimbal = PDController(kp=3, kd=0.05, output_limits=(-100, 100))

    try:
        mouse.move(CENTER_X, CENTER_Y)
        time.sleep(0.05)
        print(f"✅ Kursor myszy ustawiony na ({CENTER_X}, {CENTER_Y})")
    except Exception as e:
        print(f"❌ Nie można ustawić pozycji myszy: {e}")
        running = False
        return

    print("\n--- Instrukcje sterowania RoboMaster S1 ---")
    print("  Klawiatura:")
    print("    W A S D - ruch (przód, lewo, tył, prawo)")
    print("    Q E     - obrót podwozia (lewo, prawo)")
    print("    R       - zmiana prędkości (50, 100, 150, 200)")
    print("    F       - strzał")
    print("    X       - zatrzymanie")
    print("    M       - tryb (1 - Manual, 2 - Auto Tracking)")
    print("    ESC     - wyjście")
    print("  Mysz/Touchpad:")
    print("    Poruszaj myszką, aby sterować gimbalem (obrót i pochylenie)")
    print("-------------------------------------------\n")

    while running:
        try:
            img = None
            if ep_camera and connection_active:
                try:
                    img = ep_camera.read_cv2_image(strategy="newest", timeout=1)
                except Exception:
                    img = None

            if mode == 2 and img is not None:
                process_pose_tracking(img)

            display_camera_feed(img)

            if not running:
                break

            wheels, yaw_gimbal_speed, pitch_gimbal_speed, rotation_correction = handle_input(pid_yaw_gimbal)

            if connection_active:
                curr_time = time.time()

                if mode == 1 and (curr_time - last_chassis_exec_time >= CMD_INTERVAL):
                    final_w1 = wheels[0] * speed + int(rotation_correction)
                    final_w2 = wheels[1] * speed + int(rotation_correction)
                    final_w3 = wheels[2] * speed - int(rotation_correction)
                    final_w4 = wheels[3] * speed - int(rotation_correction)

                    try:
                        ep_chassis.drive_wheels(
                            w1=int(final_w1),
                            w2=int(-final_w2),
                            w3=int(final_w3),
                            w4=int(-final_w4)
                        )
                        ep_gimbal.drive_speed(
                            pitch_speed=pitch_gimbal_speed,
                            yaw_speed=yaw_gimbal_speed
                        )
                        last_chassis_exec_time = curr_time
                    except Exception as e:
                        if "timeout" not in str(e):
                            print(f"⚠️ Błąd sterowania: {e}")

                elif mode == 2:
                    correction_val = int(rotation_correction)
                    drive_offset = auto_drive_wheel_speed
                    interval = CMD_INTERVAL if (abs(correction_val) > 1 or abs(drive_offset) > 0) else 0.2

                    if curr_time - last_chassis_exec_time > interval:
                        final_w1 = drive_offset + correction_val
                        final_w2 = drive_offset + correction_val
                        final_w3 = drive_offset - correction_val
                        final_w4 = drive_offset - correction_val

                        try:
                            ep_chassis.drive_wheels(
                                w1=int(final_w1),
                                w2=int(-final_w2),
                                w3=int(final_w3),
                                w4=int(-final_w4)
                            )
                            last_chassis_exec_time = curr_time
                        except Exception:
                            pass
                    time.sleep(0.01)

            if battery < 2 and connection_active:
                print(f"⚠️ Niski poziom baterii ({battery}%). Zalecane podładowanie. Zatrzymywanie programu.")
                running = False
                break

        except Exception as e:
            print(f"⚠️ Błąd w pętli sterowania: {e}")
            if "timeout" in str(e):
                continue


# --- Główna punkt startowy ---
if __name__ == "__main__":
    if initialize_robot():
        main_control_loop()
    else:
        print("Program nie mógł zostać uruchomiony z powodu błędów inicjalizacji.")

    close_robot_connection()
    print("Program zakończył działanie.")
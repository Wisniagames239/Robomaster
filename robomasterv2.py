###############################################################################################
#  Skrypt sterujący Dji RoboMaster S1
#  Dominik Wiśniewski aka wisniagames239
###############################################################################################
from robomaster import robot, camera
import keyboard
import mouse
import time
import cv2

# --- Stałe konfiguracyjne ---
CENTER_X, CENTER_Y = 600, 400  # Centralna pozycja kursora myszy
INITIAL_ROBOT_SPEED = 100  # Początkowa prędkość robota (max 150 dla bezpieczeństwa)
GIMBAL_SENSITIVITY = 1  # Czułość myszy do sterowania gimbalem
CAMERA_RESOLUTION = camera.STREAM_540P


class PDController:
    def __init__(self, kp, kd, output_limits=(-100, 100), t=0.5):
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

        error = 0 - current_value

        p_term = self.kp * error

        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative

        output = p_term + d_term

        output = max(self.output_min, min(output, self.output_max))

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
ep_vision = None

connection_active = False
speed = INITIAL_ROBOT_SPEED
prev_mouse_x, prev_mouse_y = 0, 0
gimbal_angles = [0.0, 0.0, 0.0, 0.0]
battery = 100
last_pitch = 0
last_yaw = 0
running = True
mode = 1  # 1 - manual, 2 - auto tracking
modesir = {1: "1 - Manualny", 2: "2 - Automatyczny"}
latest_person_info = None
last_target_pos = None
pd_track_yaw = None

# Zmienne do limitowania częstotliwości komend (Fix timeout error)
last_gimbal_exec_time = 0
last_chassis_exec_time = 0
CMD_INTERVAL = 0.05


# --- Funkcje callback ---


def gimbal_angle_callback(*args):
    global gimbal_angles
    gimbal_angles = list(args[0])


def battery_callback(*args):
    global battery
    battery = args[0]


# --- Inicjalizacja robota ---


def initialize_robot():
    global ep_robot, ep_chassis, ep_gimbal, ep_camera, ep_blaster, ep_battery, ep_vision, connection_active, \
        prev_mouse_x, prev_mouse_y, pd_track_yaw
    ep_robot = robot.Robot()
    try:
        ep_robot.initialize(conn_type="ap")
        print("✅ Połączono z RoboMasterem!")
        connection_active = True
        pd_track_yaw = PDController(kp=350, kd=5, output_limits=(-200, 200), t=5)
        ep_chassis = ep_robot.chassis
        ep_gimbal = ep_robot.gimbal
        ep_camera = ep_robot.camera
        ep_blaster = ep_robot.blaster
        ep_battery = ep_robot.battery
        ep_vision = ep_robot.vision

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
            print(f"❌Nie można pobrać pozycji myszy: {e}")
            return False

        ep_gimbal.sub_angle(freq=8, callback=gimbal_angle_callback)
        time.sleep(0.5)
        ep_battery.sub_battery_info(freq=1, callback=battery_callback)
        time.sleep(0.5)

        print("✅ Połączono z RoboMasterem!")
        return True

    except Exception as e:
        print(f"❌ Błąd połączenia lub inicjalizacji: {e}")
        close_robot_connection()
        return False


def close_robot_connection():
    global ep_robot, connection_active
    if ep_robot:
        try:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            if ep_blaster:
                ep_blaster.set_led(brightness=0, effect="off")
            if ep_gimbal:
                ep_gimbal.unsub_angle()
            if ep_battery:
                ep_battery.unsub_battery_info()
            if ep_vision:
                ep_vision.unsub_detect_info(name="person")
            if ep_camera:
                ep_camera.stop_video_stream()
            cv2.destroyAllWindows()
            ep_robot.close()
            print("🛑 Połączenie z RoboMasterem zamknięte.")
        except Exception as e:
            print(f"❌ Błąd podczas zamykania połączenia: {e}")
    connection_active = False


def handle_input(pid_yaw_gimbal):
    global running, speed, prev_mouse_x, prev_mouse_y, mode, latest_person_info
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
        time.sleep(0.05)
        print(f"⚙️ Ustawiona prędkość robota: {speed}")
    if keyboard.is_pressed('f'):
        ep_blaster.set_led(brightness=100, effect="on")
        ep_robot.play_sound(1, 1)
        ep_blaster.fire()
        time.sleep(0.2)
        ep_blaster.set_led(brightness=100, effect="off")
        time.sleep(0.2)

    if keyboard.is_pressed('m'):
        mode = (mode % 2) + 1
        print(f"⚙️ Zmiana trybu na: {modesir[mode]}")

        if mode == 2:
            print("👁️ Włączanie wizji...")
            if ep_vision:
                ep_vision.sub_detect_info(name="person", callback=on_detect_person)
        else:
            print("👁️ Wyłączanie wizji...")
            if ep_vision:
                ep_vision.unsub_detect_info(name="person")
            latest_person_info = None
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

    pid_correction_val = pid_yaw_gimbal.update(gimbal_angles[1])
    return wheels, yaw_gimbal_input, pitch_gimbal_input, pid_correction_val


def display_camera_feed(img):
    global running, connection_active, latest_person_info
    if img is not None:
        if not connection_active:
            print("✅ Połączenie z RoboMasterem przywrócone.")
            connection_active = True

        # --- RYSOWANIE PROSTOKĄTA WOKÓŁ CZŁOWIEKA ---
        if latest_person_info and mode == 2:
            img_h, img_w = img.shape[:2]

            x_center, y_center, w_norm, h_norm = latest_person_info[0]

            w_pixel = int(w_norm * img_w)
            h_pixel = int(h_norm * img_h)
            x_pixel = int(x_center * img_w)
            y_pixel = int(y_center * img_h)

            top_left_x = int(x_pixel - (w_pixel / 2))
            top_left_y = int(y_pixel - (h_pixel / 2))
            bottom_right_x = int(x_pixel + (w_pixel / 2))
            bottom_right_y = int(y_pixel + (h_pixel / 2))

            cv2.rectangle(img, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (255, 0, 0), 3)
            cv2.circle(img, (x_pixel, y_pixel), 5, (0, 0, 255), -1)

        text_gimbal = f"P: {gimbal_angles[0]:.2f} | Y: {gimbal_angles[1]:.2f}"
        text_gimbal_g = f"P_ground: {gimbal_angles[2]:.2f} | Y_ground: {gimbal_angles[3]:.2f}"
        text_battery = f"Battery: {battery}%"

        cv2.putText(img, text_gimbal, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, text_gimbal_g, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, text_battery, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        text_mode = f"MODE: {modesir[mode]}"
        cv2.putText(img, text_mode, (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("RoboMaster S1 Camera", img)
        key = cv2.waitKey(1)
        if key == 27:  # ESC
            print("🛑 ESC wciśnięty w oknie kamery. Zamykanie.")
            running = False
    else:
        if connection_active:
            print("⚠️ Utrata łączności z RoboMasterem (podczas odczytu kamery). Zatrzymywanie ruchu.")
            connection_active = False
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)


def on_detect_person(person_info):
    global mode, latest_person_info, last_target_pos, last_gimbal_exec_time

    if mode != 2:
        return

    current_time = time.time()
    if current_time - last_gimbal_exec_time < CMD_INTERVAL:
        return

    if person_info:
        if last_target_pos is None:
            selected_person = person_info[0]
        else:
            last_x, last_y = last_target_pos
            min_dist = 10000
            best_match = None

            for p in person_info:
                px, py, pw, ph = p
                dist = (px - last_x) ** 2 + (py - last_y) ** 2

                if dist < min_dist:
                    min_dist = dist
                    best_match = p

            selected_person = best_match

        if selected_person:
            latest_person_info = [selected_person]

            x, y, w, h = selected_person

            last_target_pos = (x, y)

            yaw_speed = pd_track_yaw.update(0.5 - x)
            pitch_speed = (0.5 - y) * 180

            if ep_gimbal:
                try:
                    ep_gimbal.drive_speed(pitch_speed=pitch_speed, yaw_speed=yaw_speed)
                    last_gimbal_exec_time = current_time
                except:
                    pass
    else:
        latest_person_info = None
        last_target_pos = None

        if ep_gimbal:
            try:
                ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
                last_gimbal_exec_time = current_time
            except:
                pass


# --- Główna funkcja sterowania ---


def main_control_loop():
    global running, connection_active, speed, last_chassis_exec_time

    pid_yaw_gimbal = PDController(kp=3, kd=0.05, output_limits=(-100, 100), t=5)
    final_w1, final_w2, final_w3, final_w4 = 0, 0, 0, 0
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
                except Exception as e:
                    img = None

            display_camera_feed(img)

            if not running:
                break

            wheels, yaw_gimbal_speed, pitch_gimbal_speed, rotation_correction = handle_input(pid_yaw_gimbal)

            if mode == 1:
                if connection_active:
                    final_w1 = wheels[0] * speed + int(rotation_correction)
                    final_w2 = wheels[1] * speed + int(rotation_correction)
                    final_w3 = wheels[2] * speed - int(rotation_correction)
                    final_w4 = wheels[3] * speed - int(rotation_correction)

                ep_chassis.drive_wheels(w1=int(final_w1), w2=int(-final_w2), w3=int(final_w3), w4=int(-final_w4))
                ep_gimbal.drive_speed(pitch_speed=pitch_gimbal_speed, yaw_speed=yaw_gimbal_speed)

            elif mode == 2:
                if connection_active:
                    curr_time = time.time()
                    correction_val = int(rotation_correction)
                    should_send = False
                    if abs(correction_val) > 1:
                        if curr_time - last_chassis_exec_time > CMD_INTERVAL:
                            should_send = True
                    else:
                        # Jeśli stoimy, wysyłaj rzadziej (np. co 200ms) tylko żeby podtrzymać połączenie
                        if curr_time - last_chassis_exec_time > 0.2:
                            should_send = True

                    if should_send:
                        final_w1 = correction_val
                        final_w2 = correction_val
                        final_w3 = -correction_val
                        final_w4 = -correction_val
                        try:
                            ep_chassis.drive_wheels(w1=int(final_w1), w2=int(-final_w2), w3=int(final_w3),
                                                    w4=int(-final_w4))
                            last_chassis_exec_time = curr_time
                        except:
                            pass

                time.sleep(0.02)
            else:
                time.sleep(0.01)

            if battery < 5 and connection_active:
                print(f"⚠️ Niski poziom baterii ({battery}%). Zalecane podładowanie. Zatrzymywanie programu.")
                running = False
                break

        except Exception as e:
            print(f"⚠️ Błąd w pętli sterowania: {e}")
            if "timeout" in str(e):
                continue


# --- Główny punkt startowy ---
if __name__ == "__main__":
    if initialize_robot():
        main_control_loop()
    else:
        print("Program nie mógł zostać uruchomiony z powodu błędów inicjalizacji.")

    close_robot_connection()
    print("Program zakończył działanie.")
###############################################################################################
#  Skrypt sterujący Dji RoboMaster S1
#  Dominik Wiśniewski aka wisniagames239
###############################################################################################
from robomaster import robot, camera
import keyboard
import mouse
import time
import cv2
import threading

# --- Stałe konfiguracyjne ---
CENTER_X, CENTER_Y = 600, 400  # Centralna pozycja kursora myszy
INITIAL_ROBOT_SPEED = 100      # Początkowa prędkość robota (max 150 dla bezpieczeństwa)
GIMBAL_SENSITIVITY = 0.5       # Czułość myszy do sterowania gimbalem
GIMBAL_SUB_FREQ = 8            # Częstotliwość subskrypcji kątów gimbala
BATTERY_SUB_FREQ = 1           # Częstotliwość subskrypcji baterii
CAMERA_RESOLUTION = camera.STREAM_540P
LOOP_DELAY = 0.01              # Opóźnienie pętli głównej

# PD Controller dla stabilizacji gimbala
class PDController:
    def __init__(self, Kp, Kd, output_limits=(-100, 100), t=0.5):
        self.Kp = Kp
        self.Kd = Kd
        self.output_min, self.output_max = output_limits
        self.t = t

        self.previous_error = 0
        self.last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return 0

        error = (0 - current_value)

        p_term = self.Kp * error

        derivative = (error - self.previous_error) / dt
        d_term = self.Kd * derivative

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

connection_active = False
running = True
robot_speed = INITIAL_ROBOT_SPEED
gimbal_angles = [0.0, 0.0, 0.0, 0.0]
battery_level = 100

prev_mouse_x, prev_mouse_y = CENTER_X, CENTER_Y

# --- Funkcje callback ---
def gimbal_angle_callback(*args):
    global gimbal_angles
    gimbal_angles = list(args[0])

def battery_callback(*args):
    global battery_level
    battery_level = args[0]

# --- Inicjalizacja robota ---
def initialize_robot():
    global ep_robot, ep_chassis, ep_gimbal, ep_camera, ep_blaster, ep_battery, connection_active

    ep_robot = robot.Robot()
    try:
        ep_robot.initialize(conn_type="ap")
        print("✅ Połączono z RoboMasterem!")
        connection_active = True

        ep_chassis = ep_robot.chassis
        ep_gimbal = ep_robot.gimbal
        ep_camera = ep_robot.camera
        ep_blaster = ep_robot.blaster
        ep_battery = ep_robot.battery

        ep_blaster.set_led(brightness=255, effect="off")
        ep_gimbal.recenter().wait_for_completed()
        print("✅ Gimbal wyśrodkowany.")

        ep_robot.reset_robot_mode()
        ep_robot.set_robot_mode(mode="free")
        print(f"✅ Aktualny tryb robota: {ep_robot.get_robot_mode()}")

        if not ep_camera.start_video_stream(display=False, resolution=CAMERA_RESOLUTION):
            print("❌ Nie udało się uruchomić strumienia wideo.")
            return False
        print("✅ Strumień wideo uruchomiony.")

        ep_gimbal.sub_angle(freq=GIMBAL_SUB_FREQ, callback=gimbal_angle_callback)
        ep_battery.sub_battery_info(freq=BATTERY_SUB_FREQ, callback=battery_callback)
        time.sleep(0.5)

        return True
    except Exception as e:
        print(f"❌ Błąd połączenia lub inicjalizacji: {e}")
        close_robot_connection()
        return False

# --- Zamykanie połączenia ---
def close_robot_connection():
    global ep_robot, connection_active
    if ep_robot:
        try:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            if ep_blaster:
                ep_blaster.set_led(brightness=255, effect="off")
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

# --- Obsługa wejścia użytkownika ---
def handle_input(pid_yaw_gimbal):
    global running, robot_speed, prev_mouse_x, prev_mouse_y

    # Obsługa klawiatury
    wheel_speeds = [0, 0, 0, 0] # w1, w2, w3, w4
    rotation_direction = 0 # -1 dla lewo, 1 dla prawo

    if keyboard.is_pressed('w'):
        wheel_speeds = [1, 1, 1, 1]
    elif keyboard.is_pressed('s'):
        wheel_speeds = [-1, -1, -1, -1]
    elif keyboard.is_pressed('a'):
        wheel_speeds = [-1, 1, 1, -1]
    elif keyboard.is_pressed('d'):
        wheel_speeds = [1, -1, -1, 1]

    if keyboard.is_pressed('q'):
        rotation_direction = 1
    elif keyboard.is_pressed('e'):
        rotation_direction = -1

    if keyboard.is_pressed('r'):
        robot_speed = (robot_speed % 200) + 50
        time.sleep(0.1)
        print(f"⚙️ Ustawiona prędkość robota: {robot_speed}")

    if keyboard.is_pressed('x'):
        wheel_speeds = [0, 0, 0, 0]
        rotation_direction = 0

    if keyboard.is_pressed('f'):
        if connection_active:
            ep_blaster.set_led(brightness=100, effect="on")
            ep_robot.play_sound(1,1)
            ep_blaster.fire()
            time.sleep(0.5)
            ep_blaster.set_led(brightness=100, effect="off")
            time.sleep(0.5)

    if keyboard.is_pressed("esc"):
        print("🛑 ESC wciśnięty. Zatrzymywanie programu.")
        running = False

    # Obsługa myszy (gimbal)
    current_mouse_x, current_mouse_y = mouse.get_position()
    delta_x = current_mouse_x - prev_mouse_x
    delta_y = current_mouse_y - prev_mouse_y

    try:
        mouse.move(CENTER_X, CENTER_Y)
        prev_mouse_x, prev_mouse_y = CENTER_X, CENTER_Y
    except Exception as e:
        print(f"⚠️ Nie można ustawić kursora myszy: {e}")
        prev_mouse_x, prev_mouse_y = current_mouse_x, current_mouse_y

    gimbal_yaw_input = delta_x * GIMBAL_SENSITIVITY
    gimbal_pitch_input = -delta_y * GIMBAL_SENSITIVITY # Odwrócenie osi Y

    # Zastosowanie PD do stabilizacji yaw (jeśli nie ma aktywnego ruchu myszy)
    # Możesz to zmodyfikować, aby PD działało zawsze, albo tylko gdy nie ma ruchu myszy
    if abs(gimbal_yaw_input) < 1.0: # Jeśli ruch myszy jest minimalny, włącz PD
        pid_yaw_output = pid_yaw_gimbal.update(gimbal_angles[1])
        gimbal_yaw_input += pid_yaw_output # Dodaj korektę z PID
        gimbal_yaw_input = max(-100, min(gimbal_yaw_input, 100)) # Ograniczenie prędkości
    else:
        pid_yaw_gimbal.previous_error = 0 # Reset PID przy ręcznym sterowaniu
        pid_yaw_gimbal.last_time = time.time()

    return wheel_speeds, rotation_direction, gimbal_yaw_input, gimbal_pitch_input

# --- Rysowanie obrazu z kamery ---
def display_camera_feed(img):
    global running, connection_active
    if img is not None:
        if not connection_active:
            print("✅ Połączenie z RoboMasterem przywrócone.")
            connection_active = True

        # Dodawanie tekstu do obrazu
        text_gimbal = f"P: {gimbal_angles[0]:.2f} | Y: {gimbal_angles[1]:.2f}"
        text_gimbal_g = f"P_ground: {gimbal_angles[2]:.2f} | Y_ground: {gimbal_angles[3]:.2f}"
        text_battery = f"Battery: {battery_level}%"
        text_speed = f"Speed: {robot_speed}"

        cv2.putText(img, text_gimbal, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, text_gimbal_g, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, text_battery, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(img, text_speed, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow("RoboMaster S1 Camera", img)
        key = cv2.waitKey(1)
        if key == 27: # ESC
            print("🛑 ESC wciśnięty w oknie kamery. Zamykanie.")
            running = False
    else:
        if connection_active:
            print("⚠️ Utrata łączności z RoboMasterem (podczas odczytu kamery). Zatrzymywanie ruchu.")
            connection_active = False
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(1) # Daj czas na stabilizację po utracie połączenia

# --- Główna funkcja sterowania ---
def main_control_loop():
    global running, connection_active, robot_speed

    pid_yaw_gimbal = PDController(Kp=2.0, Kd=0.05, output_limits=(-100, 100)) # Dostosuj Kp i Kd

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
    print("    R       - zmiana prędkości (50, 100, 150)")
    print("    F       - strzał")
    print("    X       - zatrzymanie")
    print("    ESC     - wyjście")
    print("  Mysz/Touchpad:")
    print("    Poruszaj myszką, aby sterować gimbalem (obrót i pochylenie)")
    print("-------------------------------------------\n")

    while running:
        try:
            # 1. Odczyt i wyświetlanie obrazu
            img = None
            if ep_camera and connection_active:
                try:
                    img = ep_camera.read_cv2_image(strategy="newest", timeout=1)
                except Exception as e:
                    print(f"❌ Błąd odczytu klatki kamery: {e}")
                    img = None # Ustawiamy na None, żeby wyzwolić logikę utraty połączenia

            display_camera_feed(img)

            if not running: # Sprawdź, czy display_camera_feed nie ustawił running na False
                break

            # 2. Obsługa wejścia użytkownika
            wheel_speeds_input, rotation_direction_input, gimbal_yaw_speed, gimbal_pitch_speed = handle_input(pid_yaw_gimbal)

            # 3. Sterowanie robotem
            if connection_active:
                # Sterowanie podwoziem
                final_w1 = wheel_speeds_input[0] * robot_speed + pid_yaw_gimbal
                final_w2 = wheel_speeds_input[1] * robot_speed + pid_yaw_gimbal
                final_w3 = wheel_speeds_input[2] * robot_speed - pid_yaw_gimbal
                final_w4 = wheel_speeds_input[3] * robot_speed - pid_yaw_gimbal

                ep_chassis.drive_wheels(w1=int(final_w1), w2=int(-final_w2),
                                        w3=int(final_w3), w4=int(-final_w4))

                # Sterowanie gimbalem
                ep_gimbal.drive_speed(pitch_speed=gimbal_pitch_speed, yaw_speed=gimbal_yaw_speed)

            # 4. Sprawdzenie baterii
            if battery_level < 10 and connection_active:
                print(f"⚠️ Niski poziom baterii ({battery_level}%). Zalecane podładowanie. Zatrzymywanie programu.")
                running = False
                break

            time.sleep(LOOP_DELAY) # Ograniczenie częstotliwości pętli

        except Exception as e:
            print(f"❌ Krytyczny błąd w pętli sterowania: {e}")
            running = False
            break

# --- Główny punkt startowy ---
if __name__ == "__main__":
    if initialize_robot():
        main_control_loop()
    else:
        print("Program nie mógł zostać uruchomiony z powodu błędów inicjalizacji.")

    close_robot_connection()
    print("Program zakończył działanie.")
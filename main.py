###############################################################################################
#  Skrypt sterujący Dji RoboMaster S1
#  Dominik Wiśniewski aka wisniagames239
###############################################################################################
from robomaster import robot, camera
import keyboard
import mouse
import time
import cv2
#from PDcontroller import PDController
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

        error = 0 - current_value

        p_term = self.Kp * error

        derivative = (error - self.previous_error) / dt
        d_term = self.Kd * derivative

        output = p_term + d_term

        output = max(self.output_min, min(output, self.output_max))

        self.previous_error = error
        self.last_time = current_time

        return output

#Główny program
CENTER_X, CENTER_Y = 600, 400
ep_robot = robot.Robot()
ep_camera = None
try:
    ep_robot.initialize(conn_type="ap")
    print(" Połączono z RoboMasterem!")

    # Pobranie dostępu do podwozia, gimbala i kamery
    ep_chassis = ep_robot.chassis
    ep_gimbal = ep_robot.gimbal
    ep_camera = ep_robot.camera
    ep_blaster = ep_robot.blaster
    ep_blaster.set_led(brightness=255, effect="off")
    ep_gimbal.recenter().wait_for_completed()
    print("Gimbal wyśrodkowany.")
    ep_robot.reset_robot_mode()
    ep_robot.set_robot_mode(mode="gimbal_lead")
    current_mode = ep_robot.get_robot_mode()
    print(f"Aktualny tryb robota po ustawieniu: {current_mode}")

    # Uruchomienie strumienia wideo - TYLKO RAZ
    if ep_camera.start_video_stream(display=False, resolution=camera.STREAM_540P):
        print("Strumień wideo uruchomiony.")
    else:
        print("Nie udało się uruchomić strumienia wideo.")
        ep_robot.close()
        exit()

except Exception as e:
    print(f" Błąd połączenia lub inicjalizacji: {e}")
    ep_robot.close()
    exit()

# Prędkość robota
speed = 100  # max 150 dla bezpieczeństwa

# Czułość myszy do sterowania gimbalem
gimbal_sensitivity = 1

try:
    mouse.move(CENTER_X, CENTER_Y)
    time.sleep(0.05)
    prev_mouse_x, prev_mouse_y = CENTER_X, CENTER_Y
    print(f"Kursor myszy ustawiony na ({CENTER_X}, {CENTER_Y})")
except Exception as e:
    print(f"Nie można pobrać pozycji myszy: {e}")
    if ep_camera:
        ep_camera.stop_video_stream()
    ep_robot.close()
    exit()

# Funkcja callback dla subskrypcji kątów gimbala
gimbal_angles = [0.0, 0.0, 0.0, 0.0]
def gimbal_angle_callback(*args):
    global gimbal_angles
    gimbal_angles = list(args[0])

ok = ep_gimbal.sub_angle(freq=10, callback=gimbal_angle_callback)
print("sub_angle started:", ok)
time.sleep(0.5)

# Instrukcje
print(" Sterowanie robotem (klawiatura):")
print("   W - przód | S - tył | A - w lewo | D - w prawo")
print("   Q - obrót w lewo | E - obrót w prawo | F - strzał")
print("   X - stop | ESC - wyjście")
print(" Gimbal: poruszaj touchpadem / myszką w lewo-prawo i góra-dół")

last_pitch = 0
last_yaw = 0
# Initializacja PDControllera
pid_yaw_gimbal = PDController(Kp=3, Kd=0.05, output_limits=(-100, 100), t=5)
# Pętla główna
running = True
while running:
    try:
        current_pitch = gimbal_angles[0]  # Wychylenie względem podwozia
        current_yaw = gimbal_angles[1]  # Obrót względem podwozia
        current_pitch_g = gimbal_angles[2]  # Wychylenie bezwzględne (względem ziemi)
        current_yaw_g = gimbal_angles[3]  # Obrót bezwzględny (względem ziemi)
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            if img is not None:
                text = f"P: {current_pitch:.2f} | Y: {current_yaw:.2f}"
                text2 = f"P_ground: {current_pitch_g:.2f} | Y_ground: {current_yaw_g:.2f}"
                cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(img, text2, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("RoboMaster S1 Camera", img)
                key = cv2.waitKey(1)
                if key == 27:
                    print("ESC wciśnięty w oknie kamery. Zamykanie.")
                    running = False
                    break
            else:
                pass
        except Exception as e:
            print(f"Błąd odczytu/wyświetlania klatki: {e}")
        motion_x = 0
        motion_y = 0
        motion_z = 0
        wheel1 = 0  # koło prawe tył
        wheel2 = 0  # koło prawe przód    odwrócone
        wheel3 = 0  # koło lewe tył
        wheel4 = 0  # koło lewe przód     odwrócone
        yaw_gimbal_input = 0
        pitch_gimbal_input = 0

        if keyboard.is_pressed('w'):
            wheel1 += 1
            wheel2 += 1
            wheel3 += 1
            wheel4 += 1
        if keyboard.is_pressed('s'):
            wheel1 -= 1
            wheel2 -= 1
            wheel3 -= 1
            wheel4 -= 1
        if keyboard.is_pressed('a'):
            wheel1 -= 1
            wheel2 += 1
            wheel3 += 1
            wheel4 -= 1
        if keyboard.is_pressed('d'):
            wheel1 += 1
            wheel2 -= 1
            wheel3 -= 1
            wheel4 += 1
        if keyboard.is_pressed('q'):
            wheel1 += 1
            wheel2 += 1
            wheel3 -= 1
            wheel4 -= 1
        if keyboard.is_pressed('e'):
            wheel1 -= 1
            wheel2 -= 1
            wheel3 += 1
            wheel4 += 1
        if keyboard.is_pressed('x'):
            wheel1 = wheel2 = wheel3 = wheel4 = 0
        if keyboard.is_pressed('f'):
            ep_blaster.set_led(brightness=100, effect="on")
            ep_robot.play_sound(1,1)
            ep_blaster.fire()
            time.sleep(0.5)
            ep_blaster.set_led(brightness=100, effect="off")
            time.sleep(0.5)

        current_mouse_x, current_mouse_y = mouse.get_position()
        delta_x = current_mouse_x - prev_mouse_x
        delta_y = current_mouse_y - prev_mouse_y
        try:
            mouse.move(CENTER_X, CENTER_Y)
            prev_mouse_x, prev_mouse_y = CENTER_X, CENTER_Y
        except Exception as e:
            prev_mouse_x, prev_mouse_y = current_mouse_x, current_mouse_y

        if abs(delta_x) > 0:
            yaw_gimbal_input += delta_x * gimbal_sensitivity
        if abs(delta_y) > 0:
            pitch_gimbal_input -= delta_y * gimbal_sensitivity
        #Logika PD
        pid_output_yaw = pid_yaw_gimbal.update(current_yaw)
        if (yaw_gimbal_input!= last_yaw or pitch_gimbal_input != last_pitch):
            ep_gimbal.drive_speed(pitch_speed=pitch_gimbal_input, yaw_speed=yaw_gimbal_input)
            last_pitch = pitch_gimbal_input
            last_yaw = yaw_gimbal_input
        effective_rotation_input = pid_output_yaw
        final_w1 = wheel1 * speed + effective_rotation_input
        final_w2 = wheel2 * speed + effective_rotation_input
        final_w3 = wheel3 * speed - effective_rotation_input
        final_w4 = wheel4 * speed - effective_rotation_input
        ep_chassis.drive_wheels(w1=int(final_w1), w2=int(-final_w2),
                                w3=int(final_w3), w4=int(-final_w4))
        if keyboard.is_pressed("esc"):
            print(" Zatrzymanie robota i zakończenie programu.")
            break

        time.sleep(0.01)

    except Exception as e:
        print(f" Błąd sterowania: {e}")
        break

# Zatrzymanie i zamknięcie
ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
ep_gimbal.unsub_angle()
ep_camera.stop_video_stream()
cv2.destroyAllWindows()
ep_robot.close()
print("Połączenie z RoboMasterem zamknięte.")
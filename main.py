###############################################################################################
#  Skrypt sterujący Dji RoboMaster S1
#  Dominik Wiśniewski aka wisniagames239
###############################################################################################
from robomaster import robot, camera
import keyboard
import mouse
import time
import cv2

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

    ep_gimbal.recenter().wait_for_completed()
    print("Gimbal wyśrodkowany.")

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
speed = 100  # max 100 dla bezpieczeństwa

# Większa czułość myszy do sterowania gimbalem
gimbal_sensitivity = 0.7

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

# Instrukcje
print(" Sterowanie robotem (klawiatura):")
print("   W - przód | S - tył | Q - w lewo | E - w prawo")
print("   A - obrót w lewo | D - obrót w prawo | X - stop | ESC - wyjście")
print(" Gimbal: poruszaj touchpadem / myszką w lewo-prawo i góra-dół")

last_pitch = 0
last_yaw = 0
# Pętla główna
running = True
while running:
    try:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            if img is not None:
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
        yaw = 0
        pitch = 0

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
        if keyboard.is_pressed('q'):
            wheel1 -= 1
            wheel2 += 1
            wheel3 += 1
            wheel4 -= 1
        if keyboard.is_pressed('e'):
            wheel1 += 1
            wheel2 -= 1
            wheel3 -= 1
            wheel4 += 1
        if keyboard.is_pressed('a'):
            wheel1 += 1
            wheel2 += 1
            wheel3 -= 1
            wheel4 -= 1
        if keyboard.is_pressed('d'):
            wheel1 -= 1
            wheel2 -= 1
            wheel3 += 1
            wheel4 += 1
        if keyboard.is_pressed('x'):
            wheel1 = wheel2 = wheel3 = wheel4 = 0

        current_mouse_x, current_mouse_y = mouse.get_position()
        delta_x = current_mouse_x - prev_mouse_x
        delta_y = current_mouse_y - prev_mouse_y
        try:
            mouse.move(CENTER_X, CENTER_Y)
            prev_mouse_x, prev_mouse_y = CENTER_X, CENTER_Y
        except Exception as e:
            prev_mouse_x, prev_mouse_y = current_mouse_x, current_mouse_y

        if abs(delta_x) > 0:
            yaw = delta_x * gimbal_sensitivity
        if abs(delta_y) > 0:
            pitch = -delta_y * gimbal_sensitivity
        if (yaw != last_yaw or pitch != last_pitch):
            ep_gimbal.drive_speed(pitch_speed=pitch, yaw_speed=yaw)
            last_pitch = pitch
            last_yaw = yaw

        # Obliczanie o ile musi się obrócić by wyrównać z gimbalem
        effective_rotation_input = last_yaw * 0.53
        final_w1 = wheel1 * speed - effective_rotation_input
        final_w2 = wheel2 * speed - effective_rotation_input
        final_w3 = wheel3 * speed + effective_rotation_input
        final_w4 = wheel4 * speed + effective_rotation_input
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
ep_camera.stop_video_stream()
cv2.destroyAllWindows()
ep_robot.close()
print("Połączenie z RoboMasterem zamknięte.")

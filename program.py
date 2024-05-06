import cv2
from cv2 import aruco
import numpy as np
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading
import math
import datetime
#takeoff
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
    time.sleep(5)

    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude")
            break
        time.sleep(1)

#maju
def pitch(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
        0b0000111111000111, # type_mask
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # m/s
        0, 0, 0, # x, y, z acceleration
        0, 0)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#fungsi memanggil pitch
#pitch(velocity_x, velocity_y, velocity_z, duration)
#pitch(1, 0, 0,4)

def yaw1(heading, relative=True):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an abs
        # olute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        30,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def yaw2(heading, relative=True):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an abs
        # olute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        30,          # param 2, yaw speed deg/s
        -1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def kanan():
    yaw1(82)
    time.sleep(4)

def kiri():
    yaw2(85)
    time.sleep(4)

def move_drone():
    global id,x,y,z,aTargetAltitude, angle
    while True:
        global a
        a = 0
        if id == 21:
            ct = datetime.datetime.now()
            print("altitude =", vehicle.location.global_relative_frame.alt,"(",ct,")")
            if y > 13:
                print("gerak kekiri")
                pitch(0,-0.1,0,1)
            elif y < 7:
                print("gerak kekanan")
                pitch(0, 0.1, 0, 1)
            elif y > 7 and y < 13:
                print("gerak maju")
                print("Angle =", angle)
                pitch(1,0,0,1)
        elif id == 7:
            ct = datetime.datetime.now()
            print("altitude =", vehicle.location.global_relative_frame.alt, "(", ct, ")")
            if y > 13:
                print("gerak kekiri")
                pitch(0, -0.1, 0, 1)
            elif y < 7:
                print("gerak kekanan")
                pitch(0, 0.1, 0, 1)
            elif y > 7 and y < 13:
                print("gerak maju")
                print("Angle =", angle)
                pitch(1, 0, 0, 1)

        elif (id == 20) :
            if (angle < 280) and (angle > 260):
                if x > 12:
                    print("gerak mundur")
                    pitch(-0.1, 0, 0, 1)
                elif x < 5:
                    print("gerak maju")
                    pitch(0.1, 0, 0, 1)
                elif x > 5 and x < 12:
                    print("putar kanan")
                    kiri()
            if (angle <= 200) and (angle >= 160):
                if y > 13:
                    print("gerak kekiri")
                    pitch(0, -0.15, 0, 1)
                elif y < 7:
                    print("gerak kekanan")
                    pitch(0, 0.15, 0, 1)
                elif y > 7 and y < 13:
                    print("gerak maju")
                    print("Angle =", angle)
                    pitch(1, 0, 0, 1)

        elif id == 1:
            ct = datetime.datetime.now()
            print("altitude =", vehicle.location.global_relative_frame.alt, "(", ct, ")")
            if (angle < 100) and (angle > 80):
                if x > 12:
                    print("gerak mundur")
                    pitch(-0.1, 0, 0, 1)
                elif x < 5:
                    print("gerak maju")
                    pitch(0.1, 0, 0, 1)
                elif x > 5 and x < 12:
                    print("putar kanan")
                    kanan()
            if (angle < 200) and (angle > 160):
                if y > 13:
                    print("gerak kekiri")
                    pitch(0, -0.15, 0, 1)
                elif y < 7:
                    print("gerak kekanan")
                    pitch(0, 0.15, 0, 1)
                elif y > 7 and y < 13:
                    print("gerak maju")
                    print("Angle =", angle)
                    pitch(1, 0, 0, 1)




        elif id == 5:
            ct = datetime.datetime.now()
            print("altitude =", vehicle.location.global_relative_frame.alt, "(", ct, ")")

            if y > 13:
                print("gerak kekiri")
                pitch(0, -0.1, 0, 1)
            elif y < 7:
                print("gerak kekanan")
                pitch(0, 0.1, 0, 1)
            elif x > 10:
                print("gerak mundur")
                pitch(-0.1,0, 0, 1)
            elif x < 5:
                print("gerak maju")
                pitch(0.1,0, 0, 1)
            elif y > 7 and y < 13 and x > 5 and x < 10:
                pitch(0,0,0.1,1)
            if vehicle.location.global_relative_frame.alt<=1.8:
                if y > 8:
                    print("gerak kekiri")
                    pitch(0, -0.1, 0, 1)
                elif y <6:
                    print("gerak kekanan")
                    pitch(0, 0.1, 0, 1)
                elif x >7:
                    print("gerak mundur")
                    pitch(-0.1, 0, 0, 1)
                elif x <4:
                    print("gerak kekanan")
                    pitch(0.1, 0, 0, 1)
                elif y >6 and y <8 and x >4 and x <7:
                    print("LAND")
                    vehicle.mode = VehicleMode("LAND")
                    print("vehicle close")
                    vehicle.close()
                    exit(1)


        for a in range(40):
            if id == 0:
                print(id)
                print("SISTEM COUNTER SAFETY BERJALAN")
                pitch( 0.3,0,0,1)
                a += 1
                print(a)
                while a == 40:
                    time.sleep(2)
                    print("LAND")
                    vehicle.mode = VehicleMode("LAND")
                    print("vehicle close")
                    vehicle.close()
                    exit(1)

def detect_aruco():
    calib_data_path = "MultiMatrix.npz"
    calib_data = np.load(calib_data_path)
    print(calib_data.files)

    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    r_vectors = calib_data["rVector"]
    t_vectors = calib_data["tVector"]

    MARKER_SIZE = 8  # CM
    marker_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_6X6_100)
    param_markers = cv2.aruco.DetectorParameters()
    cap = cv2.VideoCapture(0)
    while True:

        ret, frame = cap.read()
        if not ret:
            break
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers

        )
        if marker_corners:

            rVec, tVec, _ = cv2.aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)

            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                cX = int((top_right[0] + bottom_right[0]) / 2.0)
                cY = int((top_right[1] + bottom_right[1]) / 2.0)

                dX = int((top_left[0] + top_right[0]) / 2.0)
                dY = int((top_left[1] + top_right[1]) / 2.0)

                aX = int((bottom_left[0] + bottom_right[0]) / 2.0)
                aY = int((bottom_left[1] + bottom_right[1]) / 2.0)

                top = (top_left[0] + top_right[0]) / 2, -((top_left[1] + top_right[1]) / 2)
                centre = (top_left[0] + top_right[0] + bottom_left[0] + bottom_right[0]) / 4, -(
                            (top_left[1] + top_right[1] + bottom_left[1] + bottom_right[1]) / 4)
                global angle
                try:
                    angle = round(math.degrees(np.arctan((top[1] - centre[1]) / (top[0] - centre[0]))))
                except:
                    # add some conditions for 90 and 270
                    if (top[1] > centre[1]):
                        angle = 90
                    elif (top[1] < centre[1]):
                        angle = 270
                if (top[0] >= centre[0] and top[1] < centre[1]):
                    angle = 360 + angle
                elif (top[0] < centre[0]):
                    angle = 180 + angle

                # Draw the pose of the marker
                #poir = cv2.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv2.putText(
                    frame,
                    f"id: {ids[0]}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    3,
                    (200, 100, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"Sudut :{angle}",
                    bottom_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    3,
                    (200, 100, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.line(frame, (dX, dY), (cX, cY), (255, 0, 0), 4)


                # print(ids, "  ", corners)
            global x,y,z
            x = round(tVec[i][0][0], 1)
            y = round(tVec[i][0][1], 1)
            z = round(tVec[i][0][2], 2)
            #print(y)

            #current_time = millis()
            global id
            if ids == 1:
                id = 1
                cv2.putText(frame,
                            f"ID = {ids}",
                            (15, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            "Perintah Belok Kanan",
                            (125, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            f"Angle = {angle}",
                            (370, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
            elif ids == 20:
                id = 20
                cv2.putText(frame,
                            f"ID = {ids}",
                            (15, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            "Perintah Belok Kiri",
                            (125, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            f"Angle = {angle}",
                            (370, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
            elif ids == 21:
                id = 21
                cv2.putText(frame,
                            f"ID = {ids}",
                            (15, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            "Perintah Maju",
                            (125, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            f"Angle = {angle}",
                            (370, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
            elif ids == 7:
                id = 7
                cv2.putText(frame,
                            f"ID = {ids}",
                            (15, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            "Perintah Maju",
                            (125, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            f"Angle = {angle}",
                            (370, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
            elif ids == 5:
                id = 5
                cv2.putText(frame,
                            f"ID = {ids}",
                            (15, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            "Perintah Landing",
                            (125, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
                cv2.putText(frame,
                            f"Angle = {angle}",
                            (370, 30),
                            cv2.FONT_HERSHEY_PLAIN, 1,
                            (0, 0, 255),
                            1,
                            cv2.LINE_4)
            else:
                cv2.putText(
                    frame,
                    f"Maju",
                    bottom_left,
                    cv2.FONT_HERSHEY_PLAIN,
                    2.5,
                    (0, 0, 0),
                    2,
                    cv2.LINE_AA,
                )
        if marker_IDs == None:
            id = 0
            angle = None
            cv2.putText(frame,
                        f"ID = {[id]}",
                        (15, 30),
                        cv2.FONT_HERSHEY_PLAIN, 1,
                        (0, 0, 255),
                        1,
                        cv2.LINE_4)
            cv2.putText(frame,
                        "Sistem Safety Berjalan",
                        (125, 30),
                        cv2.FONT_HERSHEY_PLAIN, 1,
                        (0, 0, 255),
                        1,
                        cv2.LINE_4)
            cv2.putText(frame,
                        f"Angle = Tidak Terdeteksi ID",
                        (370, 30),
                        cv2.FONT_HERSHEY_PLAIN, 1,
                        (0, 0, 255),
                        1,
                        cv2.LINE_4)

        cv2.line(frame, (0, 240), (700, 240), (0, 0, 255), 4)

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)



    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    vehicle = connect("COM4", baud=57600)
    vehicle.mode = VehicleMode("GUIDED")
    arm_and_takeoff(3)

    t1 = threading.Thread(target=detect_aruco)
    t2 = threading.Thread(target=move_drone)

    t1.start()
    t2.start()

    t1.join()
    t2.join()








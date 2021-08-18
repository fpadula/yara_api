import numpy as np
import cv2
import sys, time, math
import dt_apriltags

# Print line between two points with floating type
def draw_line(frame, point_A, point_B, color, width):
    point_Ai = (int(point_A[0]), int(point_A[1]))
    point_Bi = (int(point_B[0]), int(point_B[1]))
    cv2.line(frame, point_Ai, point_Bi, color, width)

# Print tag id
def print_tag_info(frame, tag, font, color, scale, thickness):
    point_Ai = (int(tag.corners[0][0]), int(tag.corners[0][1]))
    text = "ID:" + str(tag.tag_id)
    cv2.putText(frame, text, (point_Ai[0], point_Ai[1] - 15),
                font, scale, color, thickness)
    (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
    cv2.circle(frame, (cX, cY), 5,(0, 0, 255), -1)

def draw_rectangle(frame, points, color, width, diagonals=False):
    for i in range(len(points) -1):
        draw_line(frame, points[i], points[i+1], color, width)
    draw_line(frame, points[-1], points[0], color, width)
    if diagonals:
        draw_line(frame, points[0], points[2], color, width)
        draw_line(frame, points[1], points[3], color, width)

def warp_point(H, point):
    warped_point = np.matmul(H, np.r_[point, [1]])
    warped_point /= warped_point[2]
    return warped_point[:-1]

def draw_axis(img, origin, imgpts):
    # corner = tuple(corners[0].ravel())
    origini = (int(origin[0]), int(origin[1]))
    cv2.line(img, origini, tuple(imgpts[0].astype(int).ravel()), (0,0,255), 5)
    cv2.line(img, origini, tuple(imgpts[1].astype(int).ravel()), (0,255,0), 5)
    cv2.line(img, origini, tuple(imgpts[2].astype(int).ravel()), (255,0,0), 5)

"""
Transform coords from pixels in workspace to robot coords:
    point:          Point coordinate in workspace pixel coords.
    robot_pos:      Robot position in workspace coords (m). This is obtained
                    by multiplying the robot coordinate in pixel coords
                    by the pixel-to-meters ratio (pix2m_ratio).
    robot_rot_mat:  The robot rotation matrix.
    pix2m_ratio:    The ratio between pixels in workspace view and robot
                    coords.

    Returns:
        Robot world coordinates
"""
def wspix2robot_coord(point, robot_pos, robot_rot_mat, ws_pix_to_world, world_to_ws_pix):
    robot_pos_meters = np.matmul(world_to_ws_pix, np.r_[robot_pos, [1]]) # Getting robot position in ws pixels
    ret = point - robot_pos_meters # Subtracting the robot origin
    ret = np.matmul(robot_rot_mat, ret)
    ret = np.matmul(ws_pix_to_world, np.r_[ret, [1]])
    return ret

"""
Calculates a homography transformation from image pixels to workspace
pixels.
    ws_corners:     Workspace corners in camera pixel coordinates.

    Returns:
        H:      Homography transform
        size:   Workspace size (width, height)
"""
def calculate_H(ws_corners):
    ws_view_height = np.linalg.norm(np.array(ws_corners[0])-np.array(ws_corners[1]))
    ws_view_width = np.linalg.norm(np.array(ws_corners[2])-np.array(ws_corners[1]))

    ws_view_corners = np.float32([
        [ws_view_width, 0],
        [ws_view_width, ws_view_height],
        [0, ws_view_height],
        [0, 0]
    ])
    H, _ = cv2.findHomography(np.array(ws_corners), ws_view_corners, cv2.RANSAC,5.0)
    return H, (int(ws_view_width), int(ws_view_height))

tag_detector = dt_apriltags.Detector(families='tag36h11')
camera_matrix = np.loadtxt('camera_params.txt', delimiter=',')
camera_distortion = np.loadtxt('dist_coefs.txt', delimiter=',')
cap = cv2.VideoCapture(0)
# Same dimensions as calibrated
size = (1920, 1080)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
correct_frames = False

newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, camera_distortion, size, 1, size)

tags_to_id = [0, 1, 2, 3]
ws_corners = [
    [0,0],
    [0,0],
    [0,0],
    [0,0]
]

detecting_ws_corners = True
print("1. Setting workspace corners:")
print("\tSet the tags in the workspace corners and press Enter or press Esc to skip and load from file.")
while detecting_ws_corners:
    ret, frame = cap.read()
    if correct_frames:
        frame = cv2.undistort(frame, camera_matrix, camera_distortion, None, newcameramtx)
        x, y, w, h = roi
        frame = frame[y:y+h, x:x+w]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue,
    detected_tags = tag_detector.detect(gray)
    # if ids is not None and ids[0] == id_to_find:
    for tag in detected_tags:
        if tag.tag_id not in tags_to_id:
            continue
        (cX, cY) = (tag.center[0], tag.center[1])
        ws_corners[tag.tag_id] = [cX, cY]
        draw_rectangle(frame, tag.corners, (0, 255, 0), 2)
        print_tag_info(frame, tag, cv2.FONT_HERSHEY_SIMPLEX, (0, 0, 255), 0.5, 2)

    if len(detected_tags)>=4:
        draw_rectangle(frame, ws_corners, (0, 255, 0), 2, diagonals=True)
        camera_to_ws_H, ws_view_size = calculate_H(ws_corners)
        ws_view = cv2.warpPerspective(frame , camera_to_ws_H, ws_view_size)

        frame = cv2.resize(frame, (int(size[0]/2), int(size[1]/2)))
        ws_view = cv2.resize(ws_view, (int(size[0]/2), int(size[1]/2)))
        frame = np.hstack((frame, ws_view))
    cv2.imshow('frame', frame)

    key = cv2.waitKey(1) & 0xFF
    # Save to file:
    if key == 13:
        detecting_ws_corners = False
        print("\tSaving workspace specs to file...")
        ws_corners = np.array(ws_corners)
        np.savetxt('ws_corners.txt', ws_corners, delimiter=',')
    # Load from file:
    elif key==27:
        detecting_ws_corners = False
        print("\tLoading workspace specs from file...")
        ws_corners = np.loadtxt('ws_corners.txt', delimiter=',')

camera_to_ws_H, ws_view_size = calculate_H(ws_corners)
ws_to_camera_H = np.linalg.inv(camera_to_ws_H)

measuring_px2m_ratio = True
horizontal_d = True
vertical_d = False
tags_to_id = [0, 3]
measuring_pts = {}
for tag_no in tags_to_id:
    measuring_pts[tag_no] = ws_corners[tag_no]
print("2. Setting pixel to meters ratio:")
# print("\tSet tag 0 and tag 1 25cm apart and press Enter or press Esc to skip and load from file.")
print("\tPress enter and enter the distance between tags 0 and 1 in meters.")

# input("Insert distance between tag 0 and 1:")
while measuring_px2m_ratio:
    ret, frame = cap.read()
    if correct_frames:
        frame = cv2.undistort(frame, camera_matrix, camera_distortion, None, newcameramtx)
        x, y, w, h = roi
        frame = frame[y:y+h, x:x+w]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue,
    draw_rectangle(frame, ws_corners, (0, 255, 0), 2, diagonals=True)
    detected_tags = tag_detector.detect(gray)
    # if ids is not None and ids[0] == id_to_find:
    for tag in detected_tags:
        if tag.tag_id not in tags_to_id:
            continue
        measuring_pts[tag.tag_id] = [tag.center[0], tag.center[1]]
        draw_rectangle(frame, tag.corners, (0, 255, 0), 2)
        print_tag_info(frame, tag, cv2.FONT_HERSHEY_SIMPLEX, (0, 0, 255), 0.5, 2)

    draw_line(frame, measuring_pts[tags_to_id[0]], measuring_pts[tags_to_id[1]], (0, 0, 255), 3)
    measuring_pts_vector = warp_point(camera_to_ws_H, np.array(measuring_pts[tags_to_id[0]]))\
                            -warp_point(camera_to_ws_H, np.array(measuring_pts[tags_to_id[1]]))
    pixel_dist = np.linalg.norm(measuring_pts_vector)
    dist_text_anchor = (np.array(measuring_pts[tags_to_id[1]]) + measuring_pts_vector/2).astype(int)
    cv2.putText(frame, "D=" + str(int(pixel_dist)) + "px", dist_text_anchor,
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)

    ws_view = cv2.warpPerspective(frame , camera_to_ws_H, ws_view_size)
    frame = cv2.resize(frame, (int(size[0]/2), int(size[1]/2)))
    ws_view = cv2.resize(ws_view, (int(size[0]/2), int(size[1]/2)))
    frame = np.hstack((frame, ws_view))
    cv2.imshow('frame', frame)

    key = cv2.waitKey(1) & 0xFF
    # Save to file:
    if key == 13:
        if horizontal_d:
            horizontal_d = False
            vertical_d = True
            d0d3_d = input("Enter distance between tag 0 and 3:")
            px2m_ratio_h = float(d0d3_d)/pixel_dist
            tags_to_id = [0, 1]
            measuring_pts = {}
            for tag_no in tags_to_id:
                measuring_pts[tag_no] = ws_corners[tag_no]
        elif vertical_d:
            vertical_d = False
            measuring_px2m_ratio = False
            d0d1_d = input("Enter distance between tag 0 and 1:")
            px2m_ratio_v = float(d0d1_d)/pixel_dist
            print("\tSaving workspace specs to file...")
            np.savetxt('px2m_ratio_h.txt', np.array([px2m_ratio_h]), delimiter=',')
            np.savetxt('px2m_ratio_v.txt', np.array([px2m_ratio_v]), delimiter=',')
    # Load from file:
    elif key==27:
        measuring_px2m_ratio = False
        print("\tLoading workspace specs from file...")
        px2m_ratio_h = np.loadtxt('px2m_ratio_h.txt', delimiter=',')
        px2m_ratio_v = np.loadtxt('px2m_ratio_v.txt', delimiter=',')

print("\tWorkspace dimensions in real world: %.4fm x %.4fm" % (ws_view_size[0]*px2m_ratio_h, ws_view_size[1]*px2m_ratio_v))

src_points = np.array([    
    [0, ws_view_size[1]],
    [ws_view_size[0], ws_view_size[1]],
    [ws_view_size[0], 0]
]).astype(np.float32)
dest_points = np.array([    
    [0, ws_view_size[1]*px2m_ratio_v],
    [ws_view_size[0]*px2m_ratio_h, ws_view_size[1]*px2m_ratio_v],
    [ws_view_size[0]*px2m_ratio_h, 0]
]).astype(np.float32)
ws_pix_to_world = cv2.getAffineTransform(src_points, dest_points)
world_to_ws_pix = cv2.getAffineTransform(dest_points, src_points)
# print(ws_pix_to_world)
# quit()
setting_robot_orig = True
tags_to_id = [0]
measuring_pts = {}
for tag_no in tags_to_id:
    measuring_pts[tag_no] = ws_corners[tag_no]
fx, fy, cx, cy = camera_matrix[0, 0], camera_matrix[1, 1], \
                    camera_matrix[0, 2], camera_matrix[1, 2]
axis = np.float32([[1,0,0], [0,1,0], [0,0,1]]).reshape(-1,3)
# Accounting for the Z flip in camera coords:
tag_to_world = np.array([
    [1, 0, 0],
    [0, -1, 0],
    [0, 0, -1]
])
axis*= 0.15 # Axis size in m
axis = np.matmul(tag_to_world, axis)

print("3. Setting robot origin:")
print("\tSet tag 0 in the desired robot origin position/orientation and press Enter or press Esc to skip and load from file.")
while setting_robot_orig:
    ret, frame = cap.read()
    if correct_frames:
        frame = cv2.undistort(frame, camera_matrix, camera_distortion, None, newcameramtx)
        x, y, w, h = roi
        frame = frame[y:y+h, x:x+w]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue,
    draw_rectangle(frame, ws_corners, (0, 255, 0), 2, diagonals=True)
    detected_tags = tag_detector.detect(gray, estimate_tag_pose=True,
                                        camera_params=[fx, fy, cx, cy], tag_size=0.04)
    for tag in detected_tags:
        if tag.tag_id not in tags_to_id:
            continue
        robot_frame_ci_orig = (tag.center[0], tag.center[1])
        draw_rectangle(frame, tag.corners, (0, 255, 0), 2)
        print_tag_info(frame, tag, cv2.FONT_HERSHEY_SIMPLEX, (0, 0, 255), 0.5, 2)
        robot_rvec, _ = cv2.Rodrigues(tag.pose_R) # We just want the rot. vector, not the jacobian
        robot_tvec = tag.pose_t
        robot_axis_image, _ = cv2.projectPoints(axis, robot_rvec, robot_tvec,
                                                camera_matrix, camera_distortion)
        draw_axis(frame, robot_frame_ci_orig, robot_axis_image)

    ws_view = cv2.warpPerspective(frame , camera_to_ws_H, ws_view_size)
    frame = cv2.resize(frame, (int(size[0]/2), int(size[1]/2)))
    ws_view = cv2.resize(ws_view, (int(size[0]/2), int(size[1]/2)))
    frame = np.hstack((frame, ws_view))
    cv2.imshow('frame', frame)

    key = cv2.waitKey(1) & 0xFF
    # Wait for enter key:
    # Save to file:
    if key == 13:
        setting_robot_orig = False
        print("\tSaving workspace specs to file...")
        # robot_frame_orig = warp_point(camera_to_ws_H, np.array(robot_frame_ci_orig)) * px2m_ratio
        robot_frame_orig = warp_point(camera_to_ws_H, np.array(robot_frame_ci_orig))
        robot_frame_orig = np.matmul(ws_pix_to_world, np.r_[robot_frame_orig, [1]])
        np.savetxt('robot_frame_orig.txt', robot_frame_orig, delimiter=',')
        np.savetxt('robot_rvec.txt', robot_rvec, delimiter=',')
        np.savetxt('robot_tvec.txt', robot_tvec, delimiter=',')
    # Load from file:
    elif key==27:
        setting_robot_orig = False
        print("\tLoading workspace specs from file...")
        robot_frame_orig = np.loadtxt('robot_frame_orig.txt', delimiter=',')
        robot_rvec = np.loadtxt('robot_rvec.txt', delimiter=',')
        robot_tvec = np.loadtxt('robot_tvec.txt', delimiter=',')

# Robot frame axis projected in camera image
robot_axis_in_ci, _ = cv2.projectPoints(axis, robot_rvec, robot_tvec,
                                        camera_matrix,
                                        camera_distortion)
# Robot frame x and y direction in workspace pixels
robot_frame_x_ws = warp_point(camera_to_ws_H, robot_axis_in_ci[0][0])
robot_frame_y_ws = warp_point(camera_to_ws_H, robot_axis_in_ci[1][0])
# Robot frame position in workspace pixels
robot_frame_orig_ws = np.matmul(world_to_ws_pix, np.r_[robot_frame_orig, [1]])
new_x = robot_frame_x_ws - robot_frame_orig_ws
new_y = robot_frame_y_ws - robot_frame_orig_ws
new_x /= np.linalg.norm(new_x)
new_y /= np.linalg.norm(new_y)
robot_rot_mat = np.c_[new_x.T, new_y.T]

query_tag_position = True
tags_to_id = [1]
measuring_pts = {}
for tag_no in tags_to_id:
    measuring_pts[tag_no] = ws_corners[tag_no]
print("4. Query tag position:")
print("\tDisplaying tag 1 position relative to the robot frame. Press ESC or Enter to exit.")
while query_tag_position:
    ret, frame = cap.read()
    if correct_frames:
        frame = cv2.undistort(frame, camera_matrix, camera_distortion, None, newcameramtx)
        x, y, w, h = roi
        frame = frame[y:y+h, x:x+w]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue,
    draw_rectangle(frame, ws_corners, (0, 255, 0), 2, diagonals=True)
    detected_tags = tag_detector.detect(gray, estimate_tag_pose=True,
                                        camera_params=[fx, fy, cx, cy], tag_size=0.04)
    # if ids is not None and ids[0] == id_to_find:
    for tag in detected_tags:
        if tag.tag_id not in tags_to_id:
            continue
        (cX, cY) = (tag.center[0], tag.center[1])
        draw_rectangle(frame, tag.corners, (0, 255, 0), 2)
        print_tag_info(frame, tag, cv2.FONT_HERSHEY_SIMPLEX, (0, 0, 255), 0.5, 2)
        tag_rvec, _ = cv2.Rodrigues(tag.pose_R) # We just want the rot. vector, not the jacobian
        tag_tvec = tag.pose_t
        tag_axis_image, _ = cv2.projectPoints(axis, tag_rvec, tag_tvec, camera_matrix, camera_distortion)
        draw_axis(frame, (cX, cY), tag_axis_image)
        text_ancor = (int(tag.corners[1][0]), int(tag.corners[1][1]))
        tag_in_robot_coords = wspix2robot_coord(
                                            warp_point(camera_to_ws_H, np.array([cX, cY])),
                                            robot_frame_orig, robot_rot_mat,ws_pix_to_world, world_to_ws_pix)
        cv2.putText(frame, "("+str(tag_in_robot_coords[0].round(4)) +","+str(tag_in_robot_coords[1].round(4))+")", (text_ancor[0], text_ancor[1]+25),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

    # robot_axis_image, _ = cv2.projectPoints(axis, robot_rvec, robot_tvec, camera_matrix, camera_distortion)
    draw_axis(frame, warp_point(ws_to_camera_H, robot_frame_orig_ws), robot_axis_image)
    # robot_frame_c_i = warp_point(ws_to_camera_H, robot_frame_orig_ws)
    # cv2.circle(frame, (int(robot_frame_c_i[0]), int(robot_frame_c_i[1])), 5,(0, 0, 255), -1)

    ws_view = cv2.warpPerspective(frame , camera_to_ws_H, ws_view_size)

    frame = cv2.resize(frame, (int(size[0]/2), int(size[1]/2)))
    ws_view = cv2.resize(ws_view, (int(size[0]/2), int(size[1]/2)))
    frame = np.hstack((frame, ws_view))
    cv2.imshow('frame', frame)

    key = cv2.waitKey(1) & 0xFF
    # Wait for enter key:
    # Save to file:
    if key == 13:
        query_tag_position = False

    # Load from file:
    elif key==27:
        query_tag_position = False

cap.release()
cv2.destroyAllWindows()
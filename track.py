import cv2
from nanpy import (ArduinoApi, SerialManager, Servo)
from pivideo import VideoStream
import RPi.GPIO as GPIO
import signal
import sys
import time

FACE_DETECTION_STATUS_PIN = 18
SERVO_X_ARDUINO_PIN = 8
SERVO_Y_ARDUINO_PIN = 9
CAMERA_RESOLUTION = (320, 240)

def extract_area_size(face):
  x, y, w, h = face
  area = w * h
  return (area, face)

def keep_biggest(previous_face, current_face):
  area, face = current_face
  p_area, p_face = previous_face
  return current_face if (area > p_area) else previous_face

def get_biggest_face(faces):
  face_with_areas = map(extract_area_size, faces)
  biggest_face_with_area = reduce(keep_biggest, face_with_areas)
  _, biggest_face = biggest_face_with_area
  return biggest_face

def get_compensation_angle(face, camera_center, resolution):

  x, y, w, h = face

  # Position of the face
  face_x = x + w / 2
  face_y = y + h / 2

  # Camera parameters and mappings
  x0, y0 = camera_center
  cam_width, cam_heigh = resolution
  x_op = float(50/2) # half opening of the cam in deg
  y_op = float(40/2) # half opening of the cam in deg

  # face position ratio
  face_ratio_x = float(face_x) / cam_width
  face_ratio_y = float(face_y) / cam_heigh

  # ratio to angle
  face_x_angle = int(x_op * (face_ratio_x - 0.5)) # deg
  face_y_angle = int(y_op * (face_ratio_y - 0.5)) # deg

  # Compute angle compensation
  compensation_angle_x = 0
  compensation_angle_y = 0

  if (face_x > x0) or (face_x < x0):
    compensation_angle_x = -1 * face_x_angle

  if (face_y > y0) or (face_y < y0) :
    compensation_angle_y = face_y_angle

  return (compensation_angle_x, compensation_angle_y)

def create_video_stream(resolution):
  # flip picture as camera is mounted upside-down
  rotation = 180
  # create a threaded video stream
  vs = VideoStream(resolution=resolution, rotation=rotation).start()
  time.sleep(1) # warmup
  return vs

def main():

  # GPIO setup
  GPIO.setmode(GPIO.BCM)
  GPIO.setwarnings(False)
  GPIO.setup(FACE_DETECTION_STATUS_PIN, GPIO.OUT)

  # Serial connection setup (for the servos)
  connection = SerialManager()
  ArduinoApi(connection=connection)
  servo_x = Servo(SERVO_X_ARDUINO_PIN)
  servo_y = Servo(SERVO_Y_ARDUINO_PIN)

  # Threaded Video stream setup
  vs = create_video_stream(CAMERA_RESOLUTION)
  camera_center = map(lambda x: x/2, CAMERA_RESOLUTION) # from camera resolution

  # Face detection Haar cascade file
  face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml')

  # Initial pan-tilt angles
  angle_x = 90
  angle_y = 90
  servo_x.write(angle_x)
  servo_y.write(angle_y)

  def stop():
    vs.stop()
    GPIO.output(FACE_DETECTION_STATUS_PIN, False)
    GPIO.cleanup()
    connection.close()

  def exit_handler(signum, frame):
    stop()
    print "\nBye!"
    sys.exit(0)

  # Clean manual stop
  signal.signal(signal.SIGINT, exit_handler)

  while True:

    try:
      # grab the frame from the threaded video stream
      frame = vs.read()

      # Convert to grayscale
      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      # Look for faces in the image
      faces = face_cascade.detectMultiScale(gray, 1.5, 2)

      if faces is not ():
        # Display face detection on a led
        GPIO.output(FACE_DETECTION_STATUS_PIN, True)

        # Look for the biggest face
        biggest_face = get_biggest_face(faces)

        # Compute pan-tilt angle to adjust centring
        ax, ay = get_compensation_angle(biggest_face, camera_center, CAMERA_RESOLUTION)

        # Update angles values
        angle_x = angle_x + ax
        angle_y = angle_y + ay
        servo_x.write(angle_x)
        servo_y.write(angle_y)

      else:
        GPIO.output(FACE_DETECTION_STATUS_PIN, False)

    except Exception as e:
      print e
      stop()
      sys.exit(1)


if __name__ == "__main__":
    main()

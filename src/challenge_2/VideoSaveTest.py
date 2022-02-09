import argparse
import cv2
from tqdm import tqdm


if __name__ == "__main__":
    from timeit import default_timer as timer

    start = timer()
    # ...
    parser = argparse.ArgumentParser(description="Test Far Field Logo detection")
    parser.add_argument('--video', default="test_videos/2021_11_12_10_58_15.mp4", nargs='?')
    parser.add_argument('--video2', default="test_videos/multiscale_markers_720p.mp4", nargs='?')
    parser.add_argument('--codec', default="MJPG")
    # parser.add_argument('template', default="logo_template_2.png",  nargs='?')
    args = parser.parse_args()
    draw_centroids = True
    ext = "avi"
    cap = cv2.VideoCapture(args.video)
    cap2 = cv2.VideoCapture(args.video2)
    size = (426, 240)
    
    result = cv2.VideoWriter(f'test_rec.{ext}', cv2.VideoWriter_fourcc(*args.codec), 30, size)
    r2 = cv2.VideoWriter(f'test_rec2.{ext}', cv2.VideoWriter_fourcc(*args.codec), 30, size)
    cv2.VideoWriter()
    for i in tqdm(range(30*30)):
        success, img = cap.read()
        if not success:
            break
        result.write(cv2.resize(img, size, interpolation=cv2.INTER_AREA))
        r2.write(cv2.resize(cap2.read()[1], size, interpolation=cv2.INTER_AREA))

    end = timer()
    print(end - start) # Time in seconds, e.g. 5.38091952400282
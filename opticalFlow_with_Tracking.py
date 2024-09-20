# 优化后的光流追踪—Lucas-Kanade tracker
# （当不见检查下一个关键点的正确程度时，即使图像中的任何特征点消失，光流也有可能找到下一个看起来可能靠近它的点。实际上对于稳健的跟踪，角点应该在特定的时间间隔内检测点。
# 找到特征点后，每 30 帧对光流点的向后检查，只选择好的。）
# Lucas Kanade稀疏光流演示。使用GoodFeatures跟踪用于跟踪初始化和匹配验证的回溯帧之间。
# Lucas-Kanade sparse optical flow demo. Uses goodFeaturesToTrack for track initialization and back-tracking for match verification between frames.
# Usage
# pyhton lk_track.py images/slow_traffic_small.mp4
# 按 ESC键退出
from __future__ import print_function
import imutils
import numpy as np
import cv2

def draw_str(dst, target, s):
 x, y = target
 cv2.putText(dst, s, (x + 1, y + 1), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
 cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)

lk_params = dict(winSize=(15, 15),
  maxLevel=2,
  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
feature_params = dict(maxCorners=500,
 qualityLevel=0.3,
 minDistance=7,
 blockSize=7)

class App:
 def __init__(self, video_src):
  self.track_len = 10
  self.detect_interval = 30
  self.tracks = []
  self.cam = cv2.VideoCapture(video_src)
  self.frame_idx = 0
 def run(self):
  while True:
    _ret, frame = self.cam.read()
    if not _ret:
        break
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    vis = frame.copy()
    if len(self.tracks) > 0:
    img0, img1 = self.prev_gray, frame_gray
    p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
    p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
    p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
d = abs(p0 - p0r).reshape(-1, 2).max(-1)
 good = d < 1
 new_tracks = []
 for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
  if not good_flag:continue
  tr.append((x, y))
  if len(tr) > self.track_len:del tr[0]
  new_tracks.append(tr)
  cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
 self.tracks = new_tracks
 cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
 draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))
if self.frame_idx % self.detect_interval == 0:
 mask = np.zeros_like(frame_gray)
 mask[:] = 255
 for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
  cv2.circle(mask, (x, y), 5, 0, -1)
 p = cv2.goodFeaturesToTrack(frame_gray, mask=mask, **feature_params)
 if p is not None:
  for x, y in np.float32(p).reshape(-1, 2):self.tracks.append([(x, y)])
self.prev_gray = frame_gray
cv2.imshow('lk_track', vis)
print(self.frame_idx)
cv2.imwrite('videoOof-imgs/' + str(self.frame_idx) + '.jpg', imutils.resize(vis, 500))
self.frame_idx += 1
ch = cv2.waitKey(1)
if ch == 27:
 break
def main():
 import sys
 try:
  video_src = sys.argv[1]
 except:
  video_src = 0
 App(video_src).run()
 print('Done')

if __name__ == '__main__':
 print(__doc__)
 main()
 cv2.destroyAllWindows()
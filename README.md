### 실행 환경

---

python 3.9.12

numpy 1.21.5

 matplotlib 3.5.1

### 내용

---

gps, rtk rosbag 파일을 실행하게 되면 시각적으로 실시간 확인이 가능하다.

빨간색이 gps, 파란색이 rtk 이다.

### 실행 방법

---

1. roscore 실행
2. rosbag play <해당 rosbag파일 경로, 이름>
3. python3 gps_view.py

### Truble Shooting

---

### 실행 오류 : attributeerror 'list' 개체에 'ndim' 속성이 없습니다.  
파이썬 3.9 이상 버전을 사용해야한다.
